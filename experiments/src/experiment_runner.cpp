/** @brief Implementation for Experiment Runner
 * @author Dan McGann
 */
#include "experiments/experiment_runner.h"

#include <gtsam/sam/RangeFactor.h>
#include <jrl/IOMeasurements.h>
#include <jrl/Writer.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <nlohmann/json.hpp>

#include "baselines/baselines.h"

using json = nlohmann::json;

namespace experiments {

/*********************************************************************************************************************/
void ExperimentRunner::runExperiment(const jrl::Dataset& dataset, const ExperimentRunner::Agents& agents,
                                     const CommunicationModel& comm_model, const std::string& output_dir,
                                     size_t iter_result_output_rate) {
  jrl::Writer result_writer;
  // Get the names of each agent in the dataset
  std::vector<RobotId> robots = dataset.robots();

  // Setup Marker for each robot
  std::map<RobotId, size_t> robot_next_entry;
  for (auto& rid : robots) robot_next_entry[rid] = 0;
  std::map<RobotId, size_t> robot_final_entry;
  for (auto& rid : robots) robot_final_entry[rid] = dataset.measurements(rid).size();

  // Setup tracker for tracking runtimes
  std::map<RobotId, std::vector<double>> robot_iteration_runtime;
  for (auto& rid : robots) robot_iteration_runtime[rid] = std::vector<double>();

  // Loop over all timesteps in the dataset
  jrl::Results iteration_results("", "", {});
  bool run_dataset = true;
  size_t num_iters = 0;
  while (run_dataset) {
    // Get next timestamps
    size_t min_stamp = std::numeric_limits<size_t>::max();
    for (auto& rid : robots) {
      if (robot_next_entry[rid] < robot_final_entry[rid]) {
        size_t robot_next_stamp = dataset.measurements(rid)[robot_next_entry[rid]].stamp;
        min_stamp = robot_next_stamp < min_stamp ? robot_next_stamp : min_stamp;
      }
    }

    std::cout << "\tRunning Iteration: " << num_iters << std::endl;
    // Mark the Robots to run updates on
    std::vector<std::pair<RobotId, size_t>> robots_to_update;
    std::map<RobotId, std::optional<size_t>> robots_current_pose_idxes;
    for (auto& rid : robots) {
      if (robot_next_entry[rid] < robot_final_entry[rid] &&
          dataset.measurements(rid)[robot_next_entry[rid]].stamp <= min_stamp) {
        robots_to_update.push_back(std::make_pair(rid, robot_next_entry[rid]));
        std::cout << "\t\tUpdating Robot: " << rid << " with entry " << robot_next_entry[rid] << std::endl;
        robot_next_entry[rid]++;
      }
      if (robot_next_entry[rid] > 0) {
        robots_current_pose_idxes[rid] = robot_next_entry[rid] - 1;
      } else {
        robots_current_pose_idxes[rid] = std::nullopt;
      }
    }

    iteration_results = runIteration(num_iters, robots_to_update, robots_current_pose_idxes, dataset, agents,
                                     comm_model, robot_iteration_runtime);

    if (num_iters % iter_result_output_rate == 0) {
      std::string prefix = (boost::format("%012d") % num_iters).str();
      result_writer.writeResults(iteration_results, output_dir + "/iterations/" + prefix + ".jrr", true);
      computeAndSaveMetrics(robots_current_pose_idxes, result_writer,
                            output_dir + "/iterations/" + prefix + "_metrics.jrm", dataset, iteration_results);
    }
    num_iters++;

    // Check If we have completed the dataset
    bool complete = true;
    for (auto& rid : robots) complete = complete && robot_next_entry[rid] == robot_final_entry[rid];
    run_dataset = !complete;
  }

  // Output the final results
  result_writer.writeResults(iteration_results, output_dir + "/final_results.jrr", true);
  computeAndSaveMetrics(std::nullopt, result_writer, output_dir + "/final_metrics.jrm", dataset, iteration_results);

  // Output the timing results
  json iteration_runtime_json;
  for (RobotId rid : robots) iteration_runtime_json[std::string(1, rid)] = robot_iteration_runtime[rid];
  std::ofstream output_stream(output_dir + "/robot_iteration_runtimes.json");
  output_stream << iteration_runtime_json;
  output_stream.close();
}

/*********************************************************************************************************************/
jrl::Results ExperimentRunner::runIteration(size_t iteration_count,
                                            const std::vector<std::pair<RobotId, size_t>>& robots_to_update,
                                            const std::map<RobotId, std::optional<size_t>>& robots_current_pose_idx,
                                            const jrl::Dataset& dataset, const ExperimentRunner::Agents& agents,
                                            const CommunicationModel& comm_model,
                                            std::map<RobotId, std::vector<double>>& robot_iteration_runtime) {
  jrl::Results iter_results(dataset.name(), agents.begin()->second->name(), dataset.robots());

  // Run update on each robot
  for (size_t i = 0; i < robots_to_update.size(); i++) {
    auto& [rid, step_idx] = robots_to_update[i];
    jrl::Entry entry = dataset.measurements(rid)[step_idx];
    gtsam::KeySet robot_directly_affected_variables = entry.measurements.keys();

    // Compute initialization from the new variables
    gtsam::Values robot_new_theta;
    for (size_t i = 0; i < entry.measurement_types.size(); i++) {
      // Special case for range factors grab initialization from dataset's initialization
      auto type = entry.measurement_types.at(i);
      auto factor = entry.measurements.at(i);
      if (type == jrl::RangeFactorPose2Tag) {
        gtsam::Key key_to_init = factor->keys().back();
        gtsam::Pose2 p = dataset.initialization(rid).at<gtsam::Pose2>(key_to_init);
        robot_new_theta.insert(key_to_init, gtsam::Pose2(gtsam::Rot2(), p.translation()));
      } else if (type == jrl::RangeFactorPose3Tag) {
        gtsam::Key key_to_init = factor->keys().back();
        gtsam::Pose3 p = dataset.initialization(rid).at<gtsam::Pose3>(key_to_init);
        robot_new_theta.insert(key_to_init, gtsam::Pose3(gtsam::Rot3(), p.translation()));
      }
    }

    // Use initialization module to initialize any variables not initialized from range
    gtsam::Values current_est_copy(agents.at(rid)->getEstimate());
    current_est_copy.insert(robot_new_theta);
    robot_new_theta.insert(initializer_.initialization(entry, current_est_copy));

    // Perform the update
    auto exe_time_start = std::chrono::high_resolution_clock::now();
    if (i == robots_to_update.size() - 1 && std::dynamic_pointer_cast<baselines::CentralizedAgent>(agents.at(rid))) {
      auto cent_ptr = std::dynamic_pointer_cast<baselines::CentralizedAgent>(agents.at(rid));
      cent_ptr->forceUpdate(entry.measurements, robot_new_theta);
    } else {
      agents.at(rid)->update(entry.measurements, robot_new_theta);
    }
    auto exe_time_end = std::chrono::high_resolution_clock::now();

    // Accumulate the runtime information
    robot_iteration_runtime[rid].push_back(
        std::chrono::duration_cast<std::chrono::nanoseconds>(exe_time_end - exe_time_start).count());
  }

  // Do any communication on this step
  for (size_t i = 0; i < comm_model.roundsPerCommStep(); i++) {
    std::vector<std::pair<RobotId, RobotId>> communications =
        comm_model.getStepCommunication(iteration_count, robots_current_pose_idx);
    for (const std::pair<RobotId, RobotId>& comm_pair : communications) {
      std::cout << "\t\tSimulating Communication between robot " << comm_pair.first << ", and robot "
                << comm_pair.second << std::endl;

      // Handshake Step 1: Communicate new shared variables so each robot can update bookkeeping
      DeclaredVariables a_new_declared_shared_vars =
          agents.at(comm_pair.first)->declareNewSharedVariables(comm_pair.second);
      DeclaredVariables b_new_declared_shared_vars =
          agents.at(comm_pair.second)->declareNewSharedVariables(comm_pair.first);
      agents.at(comm_pair.first)->receiveNewSharedVariables(comm_pair.second, b_new_declared_shared_vars);
      agents.at(comm_pair.second)->receiveNewSharedVariables(comm_pair.first, a_new_declared_shared_vars);

      // Handshake Step 2: Communicate actual data for all variables shared between the robots
      CommunicationData::shared_ptr a_data = agents.at(comm_pair.first)->sendCommunication(comm_pair.second);
      CommunicationData::shared_ptr b_data = agents.at(comm_pair.second)->sendCommunication(comm_pair.first);
      // Get timing for A receive communication
      auto a_time_start = std::chrono::high_resolution_clock::now();
      agents.at(comm_pair.first)->receiveCommunication(comm_pair.second, b_data);
      if (i != comm_model.roundsPerCommStep() - 1) agents.at(comm_pair.first)->update();
      auto a_time_end = std::chrono::high_resolution_clock::now();
      // Get timing for A receive communication
      auto b_time_start = std::chrono::high_resolution_clock::now();
      agents.at(comm_pair.second)->receiveCommunication(comm_pair.first, a_data);
      if (i != comm_model.roundsPerCommStep() - 1) agents.at(comm_pair.second)->update();
      auto b_time_end = std::chrono::high_resolution_clock::now();

      // Include the receive communication time with this iteration's runtime since that is where most work happens
      robot_iteration_runtime[comm_pair.first].back() +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(a_time_end - a_time_start).count();

      robot_iteration_runtime[comm_pair.second].back() +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(b_time_end - b_time_start).count();
    }
  }

  // Aggregate results from each robot
  for (auto& rid : dataset.robots()) {
    // Aggregate the results
    gtsam::Values estimate = agents.at(rid)->getEstimate();
    jrl::ValueTypes all_types = dataset.groundTruthWithTypes(rid).types;
    jrl::ValueTypes estimate_types;
    for (gtsam::Key key : estimate.keys()) {
      estimate_types[key] = all_types[key];
    }
    iter_results.robot_solutions[rid] = jrl::TypedValues(estimate, estimate_types);
  }

  return iter_results;
}

/*********************************************************************************************************************/
void ExperimentRunner::computeAndSaveMetrics(const std::optional<std::map<RobotId, std::optional<size_t>>> step_idxes,
                                             const jrl::Writer& writer, const std::string& output_file,
                                             const jrl::Dataset& dataset, const jrl::Results& results) const {
  jrl::MetricSummary metrics;
  if (pose_type_ == "POSE3") {
    metrics = jrl::metrics::computeMetricSummary<gtsam::Pose3>(dataset, results, align_, false, true, step_idxes);
  } else if (pose_type_ == "POSE2") {
    metrics = jrl::metrics::computeMetricSummary<gtsam::Pose2>(dataset, results, align_, false, true, step_idxes);
  } else if (pose_type_ == "VEC2") {
    metrics = jrl::metrics::computeMetricSummary<gtsam::Point2>(dataset, results, align_, false, true, step_idxes);
  } else if (pose_type_ == "VEC3") {
    metrics = jrl::metrics::computeMetricSummary<gtsam::Point3>(dataset, results, align_, false, true, step_idxes);
  } else {
    throw std::runtime_error("ExperimentRunner: invalid pose_type when computing metrics");
  }

  writer.writeMetricSummary(metrics, output_file, true);
}

}  // namespace experiments

#pragma once
/** @brief Main interface for running incremental distributed SLAM experiments.
 * The purpose of the experiment runner module is to feed data from a JRL dataset
 * to an instance of a Incremental SAM Agent and module communications between agent.
 * @author Dan McGann
 */
#include <jrl/Dataset.h>
#include <jrl/Initialization.h>
#include <jrl/Results.h>
#include <jrl/Writer.h>

#include "experiments/communication_model.h"
#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace experiments {

class ExperimentRunner {
  /** TYPES **/
 public:
  typedef std::map<RobotId, std::shared_ptr<IncrementalSAMAgent>> Agents;

  /** FIELDS **/
 protected:
  /// @brief Class for computing initialization
  jrl::Initializer initializer_;
  /// @brief Pose type to use for computing metrics
  std::string pose_type_;
  /// @brief Whether or not to align trajectories in computing metrics
  bool align_;

  /** INTERFACE **/
 public:
  /// @brief Constructor
  ExperimentRunner(std::string pose_type, bool align) : pose_type_(pose_type), align_(align) {}

  /** @brief Main entry point for running an experiment
   * @param dataset: The dataset that we are simulating
   * @param agents: The agents (corresponding to each robot in the dataset) that implement the distributed SLAM algo
   * @param output_dir: Location to save out intermediate results
   * @param iter_result_output_rate: The rate at which to save intermediate results
   */
  void runExperiment(const jrl::Dataset& dataset, const Agents& agents, const CommunicationModel& comm_model,
                     const std::string& output_dir, size_t iter_result_output_rate);

  void simulateBatch(const jrl::Dataset& dataset, const Agents& agents, const CommunicationModel& comm_model,
                     const std::string& output_dir, size_t iter_result_output_rate);

  /** HELPERS **/
 protected:
  /** @brief Simulates a single timestep of the experiment
   * @param iteration_count: The iteration of the simulation being run
   * @param step_idxes: The entry indices for active robots to be run on this iteration
   * @param robots_current_pose_idx: The current pose indicies of each robot used to limit range of communications
   * @param dataset: The dataset that we are simulating
   * @param agents: The agents (corresponding to each robot in the dataset) that implement the distributed SLAM algo
   * @param comm_model: The communication model used to simulate communication at this iteration
   * @param robot_iteration_runtime: Accumulator for robot iteration runtimes [MUTATED]
   */
  jrl::Results runIteration(size_t iteration_count, const std::vector<std::pair<RobotId, size_t>>& step_idxes,
                            const std::map<RobotId, std::optional<size_t>>& robots_current_pose_idx,
                            const jrl::Dataset& dataset, const Agents& agents, const CommunicationModel& comm_model,
                            std::map<RobotId, std::vector<double>>& robot_iteration_runtime);

  /// @brief Computes and saves metrics for the given results
  void computeAndSaveMetrics(const std::optional<std::map<RobotId, std::optional<size_t>>> step_idxes,
                             const jrl::Writer& writer, const std::string& output_file, const jrl::Dataset& dataset,
                             const jrl::Results& results) const;
};

}  // namespace experiments
/** @brief Implementation for Communication Model
 * @author Dan McGann
 */
#include "experiments/communication_model.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>

namespace experiments {

/*********************************************************************************************************************/
CommunicationModel::CommunicationModel(const CommunicationModelParams& params, jrl::Dataset& dataset,
                                       const std::optional<size_t> rand_seed)
    : params_(params), dataset_(dataset) {
  if (rand_seed) {
    rand_gen_ = std::make_unique<std::mt19937>(*rand_seed);
  } else {
    rand_gen_ = std::make_unique<std::mt19937>();
  }
}

/*********************************************************************************************************************/
std::vector<std::pair<RobotId, RobotId>> CommunicationModel::getStepCommunication(
    size_t step_idx, const std::map<RobotId, std::optional<size_t>>& robots_current_pose_idx) const {
  // Get the agents in a random order
  std::vector<RobotId> agents = dataset_.robots();
  std::shuffle(agents.begin(), agents.end(), *rand_gen_);

  // Setup accumulators
  std::map<RobotId, std::vector<RobotId>> agent_communications;  // The communications for each agent
  for (const RobotId& agent : agents) agent_communications[agent] = {};
  std::vector<std::pair<RobotId, RobotId>> communications;  // Aggregation of all communications

  // Cond 0: (early exit) check if the robots communicate on this iter
  if ((step_idx % params_.communication_rate) != 0) {
    return communications;
  }

  // Loop over all agents and generate communications for each
  for (size_t i = 0; i < agents.size(); i++) {
    RobotId agent_i = agents[i];
    // Loop over all other agents that have not generated communications
    for (size_t j = i + 1; j < agents.size(); j++) {
      RobotId agent_j = agents[j];
      // Determine if a communication is possible between agent i and agent j
      // Cond 1: Both robots have bandwidth for another communication
      if (robots_current_pose_idx.at(agent_i).has_value() && robots_current_pose_idx.at(agent_j).has_value() &&
          agent_communications[agent_i].size() < params_.max_communication_partners &&
          agent_communications[agent_j].size() < params_.max_communication_partners) {
        // Cond 2: Robots are in communication range
        double rng =
            range(dataset_.groundTruth(agent_i).at(gtsam::symbol(agent_i, *(robots_current_pose_idx.at(agent_i)))),
                  dataset_.groundTruth(agent_j).at(gtsam::symbol(agent_j, *(robots_current_pose_idx.at(agent_j)))));
        if (rng < params_.communication_range) {
          communications.push_back(std::make_pair(agent_i, agent_j));
          agent_communications[agent_i].push_back(agent_j);
          agent_communications[agent_j].push_back(agent_i);
        }
      }
    }
  }

  // Filter out communications that randomly fail
  std::vector<std::pair<RobotId, RobotId>> successful_communications;  // Aggregation of all communications
  std::binomial_distribution<int> comm_success_dist(1, params_.success_prob);
  for (size_t i = 0; i < communications.size(); i++) {
    if (comm_success_dist(*rand_gen_)) {
      successful_communications.push_back(communications[i]);
    }
  }

  return successful_communications;
}
/*********************************************************************************************************************/
size_t CommunicationModel::roundsPerCommStep() const { return params_.rounds_per_comm_step; }

/*********************************************************************************************************************/
double CommunicationModel::range(const gtsam::Value& a, const gtsam::Value& b) const {
  if (typeid(a) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
    return (a.cast<gtsam::Pose2>().translation() - b.cast<gtsam::Pose2>().translation()).norm();
  } else if (typeid(a) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
    return (a.cast<gtsam::Pose3>().translation() - b.cast<gtsam::Pose3>().translation()).norm();
  } else {
    throw std::runtime_error("CommunicationModel::range received a non-pose value");
  }
}

}  // namespace experiments
#pragma once
/** @brief Defines the pattern of communication for an experiment.
 * There are many ways to implement the practical communication between robots
 * depending on the network structure, limitations on SWAP, etc.
 * This module defines a unified interface for all communication models so that
 * we can explore their affect on the incremental distributed SAM methods.
 * @author Dan McGann
 */
#include <jrl/Dataset.h>

#include <random>

#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace experiments {
/// @brief Parameters that define the behavior of a CommunicationModel
struct CommunicationModelParams {
  /// @brief The max range each robot can communicate
  double communication_range{std::numeric_limits<double>::max()};
  /// @brief The maximum number of agents each robot can communicate with in a single timestep
  size_t max_communication_partners{std::numeric_limits<size_t>::max()};
  /// @brief The rate (in measurements) that robots can communicate
  size_t communication_rate{1};
  /// @brief The number of rounds of communication to run per communications step
  size_t rounds_per_comm_step{1};
  /// @brief The probability that a communication is successful
  double success_prob{0.95};
};

/// @brief Implementation of generic communication model
class CommunicationModel {
  /** FIELDS **/
 protected:
  /// @brief The parameters defining the behavior of the communication model
  CommunicationModelParams params_;
  /// @brief The dataset for which we are modeling communications
  jrl::Dataset dataset_;
  /// @brief The random generator used by the communication model
  std::unique_ptr<std::mt19937> rand_gen_;

  /** INTERFACE **/
 public:
  /** @brief CommunicationModel constructor
   * @param params: The parameters that define the behavior of the communication model
   * @param dataset: The dataset for which we are simulating communications
   * @param rand_seed: The optional seed for the random number generator used to generate communications
   *
   */
  CommunicationModel(const CommunicationModelParams& params, jrl::Dataset& dataset,
                     const std::optional<size_t> rand_seed = std::nullopt);

  /** @brief Determines the current communications between robots in the contained dataset
   * @param The index of the entry that was just executed by the robots
   * @returns vector of robot pairs that communicate in this time step
   */
  std::vector<std::pair<RobotId, RobotId>> getStepCommunication(
      size_t step_idx, const std::map<RobotId, std::optional<size_t>>& robots_current_pose_idx) const;

  /// @brief The number of communication rounds for each step of the algorithm
  size_t roundsPerCommStep() const;

  /** @brief Computes the range between two poses
   * @param a: The first pose as a generic value
   * @param b: The second pose as a generic value
   */
  double range(const gtsam::Value& a, const gtsam::Value& b) const;
};

}  // namespace experiments
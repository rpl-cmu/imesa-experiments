#pragma once
/** @brief Implementation of DDF-SAM2 algorithm via the Incremental SAM Agent Interface
 * Notes: We use the naive bayes approximation for map summarization, and for simplicity maintain
 * two bayes trees one for the augmented system and one for the local system
 * 
 * @author Dan McGann
 */
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/AntiFactor.h>

#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace ddfsam2 {

/// @brief The data communicated between two DDFSAM2 agents
struct DDFSAM2CommunicationData : public CommunicationData {
  /// @brief The map summaries for each variable shared between the robots
  std::map<gtsam::Key, gtsam::NonlinearFactor::shared_ptr> summarized_map;
};

/// @brief Configuration parameters for the DDFSAM2 algo, all agents should use the same params
struct DDFSAM2AgentParams {
  /// @brief  Flag indicating to add a weak prior to all shared variables locally to ensure they are fully determine
  bool prior_shared_variables{false};
  /// @brief The standard deviation used for the weak prior when prior_shared_variables_is true
  double shared_variable_prior_sigma{1e2};
  /// @brief When we get a new inter-robot measurement we need to add a temporary prior on
  // new shared variables until we communicate with that robot. for said prior we use this sigma
  double map_summary_initial_sigma{10};
};

/// @brief Implementation of the DDFSAM2 Algorithm
class DDFSAM2Agent : public IncrementalSAMAgent {
  /** STATIC MEMBERS **/
 public:
  inline static const std::string METHOD_NAME = "ddfsam2";

  /** FIELDS **/
 protected:
  /// @brief The hyper-parameters for the DDFSAM2 algorithm
  DDFSAM2AgentParams params_;

  /// @brief The underlying incremental smoother instance incorporating summarized maps from other robots
  gtsam::ISAM2 augmented_smoother_;
  /// @brief The local map used for computing map summaries
  gtsam::ISAM2 local_smoother_;
  /// @brief The local estimate used for computing map summaries
  gtsam::Values current_local_estimate_;

  /// @brief List of all global variables observed
  gtsam::KeySet observed_global_variables_;
  /// @brief Mapping from shared variables to the agents with which they are shared
  std::map<gtsam::Key, std::set<RobotId>> shared_var_robots_;
  /// @brief Mapping from other agents to all variables shared between this and the other agent
  std::map<RobotId, gtsam::KeySet> robot_shared_vars_;
  /// @brief Mapping from other agents to all vars that need to be declared as shared with the other agent
  std::map<RobotId, gtsam::KeySet> undeclared_robot_shared_vars_;

  /// @brief The map summaries from other robots
  std::map<RobotId, std::map<gtsam::Key, gtsam::NonlinearFactor::shared_ptr>> map_summaries_;
  /// @brief Accumulator of map summaries from the last communication to add to the system
  gtsam::NonlinearFactorGraph map_summaries_to_add_;

  /** INTERFACE **/
 public:
  /// @brief Constructor
  DDFSAM2Agent(const RobotId& rid, DDFSAM2AgentParams params = DDFSAM2AgentParams())
      : IncrementalSAMAgent(rid),
        params_(params),
        augmented_smoother_(gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 0.1, 1)),
        local_smoother_(gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 0.1, 1)) {}

  /// @brief @see IncrementalSAMAgent
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                            const gtsam::Values& new_theta = gtsam::Values()) override;

  /// @brief @see IncrementalSAMAgent
  void receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) override;

  /// @brief @see IncrementalSAMAgent
  void receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) override;

  /// @brief @see IncrementalSAMAgent
  DeclaredVariables declareNewSharedVariables(const RobotId& other_robot) override;

  /// @brief @see IncrementalSAMAgent
  CommunicationData::shared_ptr sendCommunication(const RobotId& other_robot) override;

  /// @brief @see IncrementalSAMAgent
  std::string name() const;

  /** HELPERS **/
 private:
  /** @brief Adds new shared variables updating the internal book keeping and constructing map summary priors
   * @param New shared variables and the robot they are shared with induced by a inter-robot update
   * @param Values containing initial estimates for all new shared variables
   * @param Undeclared: Flag indicating that these shared variables are "undeclared" and not yet know to the other robot
   * @returns Map summary priors for each new shared variable
   */
  gtsam::NonlinearFactorGraph addNewSharedVariables(const std::vector<std::pair<RobotId, gtsam::Key>>& new_shared_vars,
                                                    const gtsam::Values& new_theta, bool undeclared);
};

}  // namespace ddfsam2
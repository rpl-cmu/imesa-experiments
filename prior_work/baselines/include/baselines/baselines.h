#pragma once
/** @brief Implementation of iMESA algorithm via the Incremental SAM Agent Interface
 * 
 * @author Dan McGann
 */
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace baselines {

/// @brief The data communicated between two iMESA agents
struct EmptyCommunicationData : public CommunicationData {};

/**
 *  ######  ######## ##    ## ######## ########     ###    ##       #### ######## ######## ########
 * ##    ## ##       ###   ##    ##    ##     ##   ## ##   ##        ##       ##  ##       ##     ##
 * ##       ##       ####  ##    ##    ##     ##  ##   ##  ##        ##      ##   ##       ##     ##
 * ##       ######   ## ## ##    ##    ########  ##     ## ##        ##     ##    ######   ##     ##
 * ##       ##       ##  ####    ##    ##   ##   ######### ##        ##    ##     ##       ##     ##
 * ##    ## ##       ##   ###    ##    ##    ##  ##     ## ##        ##   ##      ##       ##     ##
 *  ######  ######## ##    ##    ##    ##     ## ##     ## ######## #### ######## ######## ########
 */
/// @brief Implementation of the iMESA Algorithm
class CentralizedAgent : public IncrementalSAMAgent {
  /** STATIC MEMBERS **/
 public:
  inline static const std::string METHOD_NAME = "centralized";

  /** FIELDS **/
 public:
  /** Below are all fields for the centralized solver
   * They are static as a centralized solver aggregates info from all agents and we are working within the
   * IncrementalSAMAgent interface. These fields must be public so they can be initialized in the `.cpp` file.
   */
  /// @brief The number of robots in the team
  static size_t num_agents_;
  /// @brief The params to use for the underlying iSAM2 Solver
  static gtsam::ISAM2Params isam2_params_;
  /// @brief The underlying iSAM2 Solver
  static gtsam::ISAM2 smoother_;
  /// @brief The underlying solution to the global C-SLAM problem
  static gtsam::Values global_estimate_;
  /// @brief The update for the current timestep. Aggregated during each agents update
  static gtsam::NonlinearFactorGraph aggregated_update_;
  /// @brief The new values associated with the aggregated_update_
  static gtsam::Values aggregated_new_values_;
  /// @brief The agents from which we have aggregated new factors and values from
  static std::set<RobotId> aggregated_agents_;

  /// @brief The keys allocated to the local robot [NON-static since unique to each agent]
  gtsam::KeySet local_keys_;

  /** INTERFACE **/
 public:
  /// @brief Constructor
  CentralizedAgent(const RobotId& rid) : IncrementalSAMAgent(rid) { num_agents_++; }

  /// @brief @see IncrementalSAMAgent
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                            const gtsam::Values& new_theta = gtsam::Values()) override;

  /// @brief Forces update using all aggregated data, does not continue to aggregate
  gtsam::ISAM2Result forceUpdate(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                                 const gtsam::Values& new_theta = gtsam::Values());

  /// @brief @see IncrementalSAMAgent
  void receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) override;

  /// @brief @see IncrementalSAMAgent
  void receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) override;

  /// @brief @see IncrementalSAMAgent
  DeclaredVariables declareNewSharedVariables(const RobotId& other_robot) override;

  /// @brief @see IncrementalSAMAgent
  CommunicationData::shared_ptr sendCommunication(const RobotId& other_robot) override;

  /// @brief Returns the current solution of the distributed incremental smoother
  gtsam::Values getEstimate() override;

  /// @brief @see IncrementalSAMAgent
  std::string name() const;
};

/**
 * #### ##    ## ########  ######## ########  ######## ##    ## ########  ######## ##    ## ########
 *  ##  ###   ## ##     ## ##       ##     ## ##       ###   ## ##     ## ##       ###   ##    ##
 *  ##  ####  ## ##     ## ##       ##     ## ##       ####  ## ##     ## ##       ####  ##    ##
 *  ##  ## ## ## ##     ## ######   ########  ######   ## ## ## ##     ## ######   ## ## ##    ##
 *  ##  ##  #### ##     ## ##       ##        ##       ##  #### ##     ## ##       ##  ####    ##
 *  ##  ##   ### ##     ## ##       ##        ##       ##   ### ##     ## ##       ##   ###    ##
 * #### ##    ## ########  ######## ##        ######## ##    ## ########  ######## ##    ##    ##
 */

/// @brief Parameters for the independent baseline agent
struct IndependentAgentParams {
  /// @brief  Flag indicating to add a weak prior to all shared variables to ensure they are fully determine
  bool prior_shared_variables{false};
  /// @brief The standard deviation used for the weak prior when prior_shared_variables_is true
  double shared_variable_prior_sigma{1e2};
};

/// @brief Implementation of the iMESA Algorithm
class IndependentAgent : public IncrementalSAMAgent {
  /** STATIC MEMBERS **/
 public:
  inline static const std::string METHOD_NAME = "independent";

  /** FIELDS **/
 private:
  /// @brief The configuration parameters for the agent
  IndependentAgentParams params_;
  /// @brief The underlying incremental smoother instance
  gtsam::ISAM2 smoother_;

  /** INTERFACE **/
 public:
  /// @brief Constructor
  IndependentAgent(const RobotId& rid, const IndependentAgentParams& params)
      : IncrementalSAMAgent(rid),
        params_(params),
        smoother_(gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 0.1, 1)) {}

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
};

/** @brief Helper function to generically generate a prior used in Independent baseline
 * Implemented as a factory so that we can do so generically
 * For params @see gtsam::PriorFactor
 */
gtsam::NonlinearFactor::shared_ptr priorFactory(const gtsam::Key& key, const gtsam::Value& prior,
                                                const gtsam::SharedNoiseModel& noise_model);

}  // namespace baselines
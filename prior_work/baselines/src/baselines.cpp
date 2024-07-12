/** @brief Implementation for Baselines
 * @author Dan McGann
 */
#include "baselines/baselines.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

using namespace incremental_sam_agent;

namespace baselines {

/**
 *  ######  ######## ##    ## ######## ########     ###    ##       #### ######## ######## ########
 * ##    ## ##       ###   ##    ##    ##     ##   ## ##   ##        ##       ##  ##       ##     ##
 * ##       ##       ####  ##    ##    ##     ##  ##   ##  ##        ##      ##   ##       ##     ##
 * ##       ######   ## ## ##    ##    ########  ##     ## ##        ##     ##    ######   ##     ##
 * ##       ##       ##  ####    ##    ##   ##   ######### ##        ##    ##     ##       ##     ##
 * ##    ## ##       ##   ###    ##    ##    ##  ##     ## ##        ##   ##      ##       ##     ##
 *  ######  ######## ##    ##    ##    ##     ## ##     ## ######## #### ######## ######## ########
 */

// Initialize the static members of the centralized baseline class
size_t CentralizedAgent::num_agents_ = 0;
gtsam::ISAM2Params CentralizedAgent::isam2_params_ = gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 0.1, 1);
gtsam::ISAM2 CentralizedAgent::smoother_ = gtsam::ISAM2(CentralizedAgent::isam2_params_);
gtsam::Values CentralizedAgent::global_estimate_ = gtsam::Values();
gtsam::NonlinearFactorGraph CentralizedAgent::aggregated_update_ = gtsam::NonlinearFactorGraph();
gtsam::Values CentralizedAgent::aggregated_new_values_ = gtsam::Values();
std::set<RobotId> CentralizedAgent::aggregated_agents_ = std::set<RobotId>();

/*********************************************************************************************************************/
gtsam::ISAM2Result CentralizedAgent::update(const gtsam::NonlinearFactorGraph& new_factors,
                                            const gtsam::Values& new_theta) {
  if (new_factors.nrFactors() > 0) {
    // Aggregate the keys assigned to this agent
    auto new_keys = new_theta.keys();
    local_keys_.insert(new_keys.begin(), new_keys.end());

    // Filter new theta for any variables that are already initialized globally
    gtsam::Values filtered_new_theta;
    for (const auto& kvp : new_theta) {
      gtsam::Symbol sym(kvp.key);
      if ((sym.chr() == robot_id_ || sym.chr() == GLOBAL_ID) && !aggregated_new_values_.exists(kvp.key) &&
          !global_estimate_.exists(kvp.key)) {
        filtered_new_theta.insert(kvp.key, kvp.value);
      }
    }

    // Aggregate the new measurements
    aggregated_update_.push_back(new_factors);
    aggregated_new_values_.insert(filtered_new_theta);
    aggregated_agents_.insert(robot_id_);

    // If we have the update for all agents run the update
    if (aggregated_agents_.size() == num_agents_) {
      auto result = smoother_.update(aggregated_update_, aggregated_new_values_);
      global_estimate_ = smoother_.calculateEstimate();

      // Reset the accumulators
      aggregated_update_ = gtsam::NonlinearFactorGraph();
      aggregated_new_values_ = gtsam::Values();
      aggregated_agents_ = std::set<RobotId>();

      return result;
    } else {
      return gtsam::ISAM2Result();
    }
  } else {
    // Special Case for empty update
    auto result = smoother_.update();
    global_estimate_ = smoother_.calculateEstimate();
    return result;
  }
}

/*********************************************************************************************************************/
gtsam::ISAM2Result CentralizedAgent::forceUpdate(const gtsam::NonlinearFactorGraph& new_factors,
                                                 const gtsam::Values& new_theta) {
  // Aggregate the keys assigned to this agent
  auto new_keys = new_theta.keys();
  local_keys_.insert(new_keys.begin(), new_keys.end());

  // Filter new theta for any variables that are already initialized globally
  gtsam::Values filtered_new_theta;
  for (const auto& kvp : new_theta) {
    gtsam::Symbol sym(kvp.key);
    if ((sym.chr() == robot_id_ || sym.chr() == GLOBAL_ID) && !aggregated_new_values_.exists(kvp.key) &&
        !global_estimate_.exists(kvp.key)) {
      filtered_new_theta.insert(kvp.key, kvp.value);
    }
  }

  // Aggregate the new measurements
  aggregated_update_.push_back(new_factors);
  aggregated_new_values_.insert(filtered_new_theta);
  aggregated_agents_.insert(robot_id_);

  auto result = smoother_.update(aggregated_update_, aggregated_new_values_);
  global_estimate_ = smoother_.calculateEstimate();

  // Reset the accumulators
  aggregated_update_ = gtsam::NonlinearFactorGraph();
  aggregated_new_values_ = gtsam::Values();
  aggregated_agents_ = std::set<RobotId>();

  return result;
}

/*********************************************************************************************************************/
void CentralizedAgent::receiveNewSharedVariables(const RobotId& other_robot,
                                                 const DeclaredVariables& declared_variables) {
  return;  // Does nothing
}

/*********************************************************************************************************************/
void CentralizedAgent::receiveCommunication(const RobotId& other_robot,
                                            const CommunicationData::shared_ptr& comm_data) {
  return;  // Does nothing
}
/*********************************************************************************************************************/
DeclaredVariables CentralizedAgent::declareNewSharedVariables(const RobotId& other_robot) {
  return DeclaredVariables();  // Does nothing
}

/*********************************************************************************************************************/
CommunicationData::shared_ptr CentralizedAgent::sendCommunication(const RobotId& other_robot) {
  return std::make_shared<EmptyCommunicationData>();  // Does Nothing
}

gtsam::Values CentralizedAgent::getEstimate() {
  current_estimate_ = global_estimate_.filter([this](gtsam::Key key) { return local_keys_.count(key) != 0; });
  return current_estimate_;
}

/*********************************************************************************************************************/
std::string CentralizedAgent::name() const { return CentralizedAgent::METHOD_NAME; }

/**
 * #### ##    ## ########  ######## ########  ######## ##    ## ########  ######## ##    ## ########
 *  ##  ###   ## ##     ## ##       ##     ## ##       ###   ## ##     ## ##       ###   ##    ##
 *  ##  ####  ## ##     ## ##       ##     ## ##       ####  ## ##     ## ##       ####  ##    ##
 *  ##  ## ## ## ##     ## ######   ########  ######   ## ## ## ##     ## ######   ## ## ##    ##
 *  ##  ##  #### ##     ## ##       ##        ##       ##  #### ##     ## ##       ##  ####    ##
 *  ##  ##   ### ##     ## ##       ##        ##       ##   ### ##     ## ##       ##   ###    ##
 * #### ##    ## ########  ######## ##        ######## ##    ## ########  ######## ##    ##    ##
 */

/*********************************************************************************************************************/
gtsam::ISAM2Result IndependentAgent::update(const gtsam::NonlinearFactorGraph& new_factors,
                                            const gtsam::Values& new_theta) {
  gtsam::NonlinearFactorGraph determined_factors(new_factors);

  // Add priors if configured
  if (params_.prior_shared_variables) {
    for (auto& key : new_theta.keys()) {
      if (gtsam::Symbol(key).chr() != robot_id_) {
        determined_factors.push_back(priorFactory(
            key, new_theta.at(key),
            gtsam::noiseModel::Isotropic::Sigma(new_theta.at(key).dim(), params_.shared_variable_prior_sigma)));
      }
    }
  }

  // Run the local update
  auto result = smoother_.update(determined_factors, new_theta);
  current_estimate_ = smoother_.calculateEstimate();
  return result;
}

/*********************************************************************************************************************/
void IndependentAgent::receiveNewSharedVariables(const RobotId& other_robot,
                                                 const DeclaredVariables& declared_variables) {
  return;  // Does nothing
}

/*********************************************************************************************************************/
void IndependentAgent::receiveCommunication(const RobotId& other_robot,
                                            const CommunicationData::shared_ptr& comm_data) {
  return;  // Does nothing
}
/*********************************************************************************************************************/
DeclaredVariables IndependentAgent::declareNewSharedVariables(const RobotId& other_robot) {
  return DeclaredVariables();  // Does nothing
}

/*********************************************************************************************************************/
CommunicationData::shared_ptr IndependentAgent::sendCommunication(const RobotId& other_robot) {
  return std::make_shared<EmptyCommunicationData>();  // Does Nothing
}

/*********************************************************************************************************************/
std::string IndependentAgent::name() const { return IndependentAgent::METHOD_NAME; }

/*********************************************************************************************************************/
gtsam::NonlinearFactor::shared_ptr priorFactory(const gtsam::Key& key, const gtsam::Value& prior,
                                                const gtsam::SharedNoiseModel& noise_model) {
  if (typeid(prior) == typeid(gtsam::GenericValue<gtsam::Vector>)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Vector>>(key, prior.cast<gtsam::Vector>(), noise_model);
  } else if (typeid(prior) == typeid(gtsam::GenericValue<gtsam::Vector2>)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Vector2>>(key, prior.cast<gtsam::Vector2>(), noise_model);
  } else if (typeid(prior) == typeid(gtsam::GenericValue<gtsam::Vector3>)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(key, prior.cast<gtsam::Vector3>(), noise_model);
  } else if (typeid(prior) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Pose2>>(key, prior.cast<gtsam::Pose2>(), noise_model);
  } else if (typeid(prior) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, prior.cast<gtsam::Pose3>(), noise_model);
  } else {
    throw std::runtime_error("Invalid prior type passed to Prior factory");
  }
}

}  // namespace baselines
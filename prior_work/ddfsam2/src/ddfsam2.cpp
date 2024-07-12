/** @brief Implementation for DDFSAM2
 * @author Dan McGann
 */

#include "ddfsam2/ddfsam2.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include "baselines/baselines.h"

using namespace incremental_sam_agent;

namespace ddfsam2 {
/**
 * #### ##    ## ######## ######## ########  ########    ###     ######  ########
 *  ##  ###   ##    ##    ##       ##     ## ##         ## ##   ##    ## ##
 *  ##  ####  ##    ##    ##       ##     ## ##        ##   ##  ##       ##
 *  ##  ## ## ##    ##    ######   ########  ######   ##     ## ##       ######
 *  ##  ##  ####    ##    ##       ##   ##   ##       ######### ##       ##
 *  ##  ##   ###    ##    ##       ##    ##  ##       ##     ## ##    ## ##
 * #### ##    ##    ##    ######## ##     ## ##       ##     ##  ######  ########
 */
/*********************************************************************************************************************/
gtsam::ISAM2Result DDFSAM2Agent::update(const gtsam::NonlinearFactorGraph& new_factors,
                                        const gtsam::Values& new_theta) {
  gtsam::NonlinearFactorGraph determined_factors(new_factors);

  // Add priors to shared variables if configured
  if (params_.prior_shared_variables) {
    for (auto& key : new_theta.keys()) {
      if (gtsam::Symbol(key).chr() != robot_id_) {
        determined_factors.push_back(baselines::priorFactory(
            key, new_theta.at(key),
            gtsam::noiseModel::Isotropic::Sigma(new_theta.at(key).dim(), params_.shared_variable_prior_sigma)));
      }
    }
  }

  // Setup container for all new factors we are updating the system with
  gtsam::NonlinearFactorGraph augmented_new_factors(determined_factors);

  // Determine if we observed new variables of other robots
  std::vector<std::pair<RobotId, gtsam::Key>> new_shared_variables;
  for (const auto& key : new_factors.keys()) {
    gtsam::Symbol sym(key);
    if (sym.chr() != robot_id_ && !current_estimate_.exists(key)) {
      if (sym.chr() == GLOBAL_ID) {
        observed_global_variables_.insert(key);
      } else {
        new_shared_variables.push_back(std::make_pair(sym.chr(), key));
      }
    }
  }
  // Add new dummy summaries for all newly observed other robot variables
  augmented_new_factors.push_back(addNewSharedVariables(new_shared_variables, new_theta, true));

  // Add any summaries received since last update
  augmented_new_factors.push_back(map_summaries_to_add_);
  map_summaries_to_add_ = gtsam::NonlinearFactorGraph();

  // Update the Augmented System
  gtsam::ISAM2Result result = augmented_smoother_.update(augmented_new_factors, new_theta);
  // Update the Local System
  local_smoother_.update(determined_factors, new_theta);

  // Update the estimates
  current_estimate_ = augmented_smoother_.calculateEstimate();
  current_local_estimate_ = local_smoother_.calculateEstimate();

  return result;
}

/*********************************************************************************************************************/
void DDFSAM2Agent::receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) {
  std::vector<std::pair<RobotId, gtsam::Key>> new_shared_variables;
  for (const auto& key : declared_variables.new_shared_variables) {
    new_shared_variables.push_back(std::make_pair(other_robot, key));
  }

  for (const auto& key : declared_variables.observed_global_variables) {
    // If we have observed the key, but it is not marked as shared with the other_robot add it
    if (observed_global_variables_.count(key) != 0 && robot_shared_vars_[other_robot].count(key) == 0) {
      new_shared_variables.push_back(std::make_pair(other_robot, key));
    }
  }

  // Update Book keeping and construct new map summary priors for any new shared variables declared by the other robot
  // These are local variables that the other robot observed, so we can use current estimate to initialize.
  // These summaries will also be overwritten immediately when the agent receives communication from other_robot
  map_summaries_to_add_.push_back(addNewSharedVariables(new_shared_variables, current_estimate_, false));
}

/*********************************************************************************************************************/
void DDFSAM2Agent::receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) {
  std::shared_ptr<DDFSAM2CommunicationData> ddfsam2_comm_data =
      std::dynamic_pointer_cast<DDFSAM2CommunicationData>(comm_data);

  gtsam::NonlinearFactorGraph map_summary_update;
  for (const gtsam::Key& key : robot_shared_vars_[other_robot]) {
    // Add an anti-factor for the curent summary
    map_summary_update.emplace_shared<gtsam::AntiFactor>(map_summaries_[other_robot][key]);
    // Add the new summary we just got from the other robot
    map_summary_update.push_back(ddfsam2_comm_data->summarized_map[key]);
    // Update the bookkeeping for this summary
    map_summaries_[other_robot][key] = ddfsam2_comm_data->summarized_map[key];
  }

  // Update the Augmented system
  map_summaries_to_add_.push_back(map_summary_update);
}

/*********************************************************************************************************************/
DeclaredVariables DDFSAM2Agent::declareNewSharedVariables(const RobotId& other_robot) {
  DeclaredVariables result;
  result.new_shared_variables = undeclared_robot_shared_vars_[other_robot];
  result.observed_global_variables = observed_global_variables_;

  undeclared_robot_shared_vars_[other_robot] = gtsam::KeySet();
  return result;
}

/*********************************************************************************************************************/
CommunicationData::shared_ptr DDFSAM2Agent::sendCommunication(const RobotId& other_robot) {
  std::shared_ptr<DDFSAM2CommunicationData> comm_data = std::make_shared<DDFSAM2CommunicationData>();

  // Summarize map from local smoother using Naive bayes approximation
  for (const gtsam::Key& shared_key : robot_shared_vars_[other_robot]) {
    comm_data->summarized_map[shared_key] = baselines::priorFactory(
        shared_key, current_local_estimate_.at(shared_key),
        gtsam::noiseModel::Gaussian::Covariance(local_smoother_.marginalCovariance(shared_key)));
  }
  return comm_data;
}

/*********************************************************************************************************************/
std::string DDFSAM2Agent::name() const { return DDFSAM2Agent::METHOD_NAME; }

/**
 * ##     ## ######## ##       ########  ######## ########   ######
 * ##     ## ##       ##       ##     ## ##       ##     ## ##    ##
 * ##     ## ##       ##       ##     ## ##       ##     ## ##
 * ######### ######   ##       ########  ######   ########   ######
 * ##     ## ##       ##       ##        ##       ##   ##         ##
 * ##     ## ##       ##       ##        ##       ##    ##  ##    ##
 * ##     ## ######## ######## ##        ######## ##     ##  ######
 */
/*********************************************************************************************************************/
gtsam::NonlinearFactorGraph DDFSAM2Agent::addNewSharedVariables(
    const std::vector<std::pair<RobotId, gtsam::Key>>& new_shared_vars, const gtsam::Values& new_theta,
    bool undeclared) {
  // Setup container for the constructed biased priors
  gtsam::NonlinearFactorGraph new_map_summary_priors;

  // Iterate over all new shared variables (either new variables, or new shares of existing vars with new robots)
  for (const std::pair<RobotId, gtsam::Key>& pair : new_shared_vars) {
    RobotId rid = pair.first;
    gtsam::Key key = pair.second;

    // If this is a new variable add a new entry else it an existing variable shared with a new robot so update
    if (shared_var_robots_.count(key) == 0) {
      shared_var_robots_[key] = {rid};
    } else {
      shared_var_robots_[key].insert(rid);
    }

    // If this is the first share with a robot add an entry else extend the current shared
    if (robot_shared_vars_.count(rid) == 0) {
      robot_shared_vars_[rid] = gtsam::KeySet();
      map_summaries_[rid] = std::map<gtsam::Key, gtsam::NonlinearFactor::shared_ptr>();
    }
    robot_shared_vars_[rid].insert(key);

    // If these new shared variables are undeclared we need to mark them for communication to other robots
    if (undeclared) {
      if (undeclared_robot_shared_vars_.count(rid) == 0) {
        undeclared_robot_shared_vars_[rid] = gtsam::KeySet();
      }
      undeclared_robot_shared_vars_[rid].insert(key);
    }

    // Construct a temporary prior to be used until we have communication
    gtsam::NonlinearFactor::shared_ptr new_var_prior = baselines::priorFactory(
        pair.second, new_theta.at(pair.second),
        gtsam::noiseModel::Isotropic::Sigma(new_theta.at(pair.second).dim(), params_.map_summary_initial_sigma));
    map_summaries_[pair.first][pair.second] = new_var_prior;
    new_map_summary_priors.push_back(new_var_prior);
  }

  return new_map_summary_priors;
}

}  // namespace ddfsam2

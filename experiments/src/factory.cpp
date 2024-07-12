/** @brief Implementation for Method Factory
 * @author Dan McGann
 */
#include "experiments/factory.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include "baselines/baselines.h"
#include "ddfsam2/ddfsam2.h"
#include "experiments/config.h"
#include "imesa/imesa.h"

namespace experiments {

std::shared_ptr<IncrementalSAMAgent> agent_factory(const RobotId& rid, const std::string& method_name,
                                                   const std::string& method_param_file, const std::string& pose_type) {
  // Baselines
  /*********************************************************************************************************************/
  if (method_name == baselines::CentralizedAgent::METHOD_NAME) {
    return std::make_shared<baselines::CentralizedAgent>(rid);
  }
  /*******************************************************/
  else if (method_name == baselines::IndependentAgent::METHOD_NAME) {
    baselines::IndependentAgentParams params;
    if (method_param_file != "") params = parseIndependentAgentParams(method_param_file);
    return std::make_shared<baselines::IndependentAgent>(rid, params);
  }
  // IMESA
  /*********************************************************************************************************************/
  else if (method_name == imesa::IMESAAgent::METHOD_NAME) {
    imesa::IMESAAgentParams params;
    if (method_param_file != "") params = parseIMESAAgentParams(method_param_file);
    params.biased_prior_noise_model_sigmas = std::make_pair(0.1, 1);
    return std::make_shared<imesa::IMESAAgent>(rid, params);
  }
  // PriorWork
  /*********************************************************************************************************************/
  else if (method_name == ddfsam2::DDFSAM2Agent::METHOD_NAME) {
    ddfsam2::DDFSAM2AgentParams params;
    if (method_param_file != "") params = parseDDFSAM2AgentParams(method_param_file);
    return std::make_shared<ddfsam2::DDFSAM2Agent>(rid, params);
  }
  // Default
  /*********************************************************************************************************************/
  else {
    throw std::runtime_error("agent_factory: Invalid method_name provided");
  }
}

}  // namespace experiments
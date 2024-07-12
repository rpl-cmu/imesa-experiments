/** @brief Implementation for Config
 * @author Dan McGann
 */
#include "experiments/config.h"

namespace experiments {

/**
 *    ###     ######   ######## ##    ## ########  ######
 *   ## ##   ##    ##  ##       ###   ##    ##    ##    ##
 *  ##   ##  ##        ##       ####  ##    ##    ##
 * ##     ## ##   #### ######   ## ## ##    ##     ######
 * ######### ##    ##  ##       ##  ####    ##          ##
 * ##     ## ##    ##  ##       ##   ###    ##    ##    ##
 * ##     ##  ######   ######## ##    ##    ##     ######
 */
/*********************************************************************************************************************/
imesa::IMESAAgentParams parseIMESAAgentParams(const std::string& params_file) {
  // Parse the file
  std::ifstream ifs(params_file);
  json parsed_json = json::parse(ifs);

  // Construct the params
  imesa::IMESAAgentParams params;
  params.pose_biased_prior_type =
      biased_priors::BPTypeFromString(parsed_json["pose_biased_prior_type"].get<std::string>());
  params.initial_penalty_param = parsed_json["initial_penalty_param"].get<double>();
  params.penalty_param = parsed_json["penalty_param"].get<double>();
  params.shared_var_wildfire_threshold = parsed_json["shared_var_wildfire_threshold"].get<double>();
  return params;
}

/*********************************************************************************************************************/
baselines::IndependentAgentParams parseIndependentAgentParams(const std::string& params_file) {
  // Parse the file
  std::ifstream ifs(params_file);
  json parsed_json = json::parse(ifs);

  // Construct the params
  baselines::IndependentAgentParams params;
  params.prior_shared_variables = parsed_json["prior_shared_variables"].get<bool>();
  params.shared_variable_prior_sigma = parsed_json["shared_variable_prior_sigma"].get<double>();
  return params;
}

/*********************************************************************************************************************/
ddfsam2::DDFSAM2AgentParams parseDDFSAM2AgentParams(const std::string& params_file) {
  // Parse the file
  std::ifstream ifs(params_file);
  json parsed_json = json::parse(ifs);

  // Construct the params
  ddfsam2::DDFSAM2AgentParams params;
  params.prior_shared_variables = parsed_json["prior_shared_variables"].get<bool>();
  params.shared_variable_prior_sigma = parsed_json["shared_variable_prior_sigma"].get<double>();
  params.map_summary_initial_sigma = parsed_json["map_summary_initial_sigma"].get<double>();
  return params;
}

/**
 *  ######   #######  ##     ## ##     ## ##     ## ##    ## ####  ######     ###    ######## ####  #######  ##    ##
 * ##    ## ##     ## ###   ### ###   ### ##     ## ###   ##  ##  ##    ##   ## ##      ##     ##  ##     ## ###   ##
 * ##       ##     ## #### #### #### #### ##     ## ####  ##  ##  ##        ##   ##     ##     ##  ##     ## ####  ##
 * ##       ##     ## ## ### ## ## ### ## ##     ## ## ## ##  ##  ##       ##     ##    ##     ##  ##     ## ## ## ##
 * ##       ##     ## ##     ## ##     ## ##     ## ##  ####  ##  ##       #########    ##     ##  ##     ## ##  ####
 * ##    ## ##     ## ##     ## ##     ## ##     ## ##   ###  ##  ##    ## ##     ##    ##     ##  ##     ## ##   ###
 *  ######   #######  ##     ## ##     ##  #######  ##    ## ####  ######  ##     ##    ##    ####  #######  ##    ##
 */
/*********************************************************************************************************************/
CommunicationModelParams parseCommunicationModelParams(const std::string& params_file) {
  // Parse the file
  std::ifstream ifs(params_file);
  json parsed_json = json::parse(ifs);

  // Construct the params
  CommunicationModelParams params;
  params.communication_range = parsed_json["communication_range"].get<size_t>();
  params.max_communication_partners = parsed_json["max_communication_partners"].get<size_t>();
  params.communication_rate = parsed_json["communication_rate"].get<size_t>();
  params.rounds_per_comm_step = parsed_json["rounds_per_comm_step"].get<size_t>();
  params.success_prob = parsed_json["success_prob"].get<double>();

  return params;
}

}  // namespace experiments
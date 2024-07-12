#pragma once
/** @brief Facilitates json file based config of communications models and agents
 *  This module implements parsing of JSON files used to configure the agents and communication models used in these
 * experiments
 * @author Dan McGann
 */
#include <nlohmann/json.hpp>

#include "ddfsam2/ddfsam2.h"
#include "experiments/communication_model.h"
#include "imesa/imesa.h"
#include "baselines/baselines.h"

using json = nlohmann::json;

using namespace incremental_sam_agent;

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

/// @brief Parses parameters for iMESA
imesa::IMESAAgentParams parseIMESAAgentParams(const std::string& params_file);

/// @brief Parses parameters for IndependentAgent
baselines::IndependentAgentParams parseIndependentAgentParams(const std::string& params_file);

/// @brief Parses parameters for DDFSAM2
ddfsam2::DDFSAM2AgentParams parseDDFSAM2AgentParams(const std::string& params_file);

/**
 *  ######   #######  ##     ## ##     ## ##     ## ##    ## ####  ######     ###    ######## ####  #######  ##    ##
 * ##    ## ##     ## ###   ### ###   ### ##     ## ###   ##  ##  ##    ##   ## ##      ##     ##  ##     ## ###   ##
 * ##       ##     ## #### #### #### #### ##     ## ####  ##  ##  ##        ##   ##     ##     ##  ##     ## ####  ##
 * ##       ##     ## ## ### ## ## ### ## ##     ## ## ## ##  ##  ##       ##     ##    ##     ##  ##     ## ## ## ##
 * ##       ##     ## ##     ## ##     ## ##     ## ##  ####  ##  ##       #########    ##     ##  ##     ## ##  ####
 * ##    ## ##     ## ##     ## ##     ## ##     ## ##   ###  ##  ##    ## ##     ##    ##     ##  ##     ## ##   ###
 *  ######   #######  ##     ## ##     ##  #######  ##    ## ####  ######  ##     ##    ##    ####  #######  ##    ##
 */

/// @brief Parses parameters for an experiment's communication model
CommunicationModelParams parseCommunicationModelParams(const std::string& params_file);

}  // namespace experiments
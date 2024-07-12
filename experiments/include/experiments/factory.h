#pragma once
/// @brief The purpose of the factor module is to construct the agents for various methods explored in these experiments
/// @author Dan McGann
#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace experiments {

/// @brief Constructs an agent for the given method name
std::shared_ptr<IncrementalSAMAgent> agent_factory(const RobotId& rid, const std::string& method_name,
                                                   const std::string& method_param_file, const std::string& pose_type);

}  // namespace experiments

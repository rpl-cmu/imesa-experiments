/** @brief Run Trial Entry Point
 * @author Dan McGann
 */
#include <jrl/Dataset.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <ctime>
#include <iomanip>

#include "experiments/communication_model.h"
#include "experiments/config.h"
#include "experiments/experiment_runner.h"
#include "experiments/factory.h"
#include "imesa/imesa.h"

using namespace experiments;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;

/**
 *    ###    ########   ######    ######
 *   ## ##   ##     ## ##    ##  ##    ##
 *  ##   ##  ##     ## ##        ##
 * ##     ## ########  ##   ####  ######
 * ######### ##   ##   ##    ##        ##
 * ##     ## ##    ##  ##    ##  ##    ##
 * ##     ## ##     ##  ######    ######
 */

po::variables_map handle_args(int argc, const char *argv[]) {
  // Define the options
  po::options_description options("Allowed options");
  // clang-format off
  options.add_options()
      ("help,h",                                                      "Produce help message")
      ("input_dataset,d", po::value<std::string>()->required(),       "(Required) Path to JSON Robot Log dataset file.")
      ("method,m",        po::value<std::string>()->required(),       "(Required) The name of the method to run (see factory.h for list of method names).")
      ("method_params,p", po::value<std::string>()->default_value(""),"(Optional) Path to method parameters file.")
      ("comm_params,c",   po::value<std::string>()->default_value(""),"(Optional) Path to communication model parameters file.")
      ("output_dir,o",    po::value<std::string>()->required(),       "(Required) Directory to which the results will be saved.")
      ("output_rate,r",   po::value<std::size_t>()->required(),       "(Required) Rate that intermediate results will be saved in iterations.")
      ("random_seed,s",   po::value<std::size_t>()->required(),       "(Required) The random seed to use for this trial.")
      ("pose_type,t",     po::value<std::string>()->required(),       "(Required) The type of the poses in the dataset (for metric computation) [POSE2, POSE3, VEC2, VEC3].")
      ("no_align",                                                    "(Optional) Flag indicating we should not align trajectories in metric computation.");
  // clang-format on

  // Parse and return the options
  po::variables_map var_map;
  po::store(po::parse_command_line(argc, argv, options), var_map);

  // Handle help special case
  if (var_map.count("help") || argc == 1) {
    std::cout << "run-trial: Main entry point to run incremental distributed SLAM methods on JRL datasets. Please "
                 "provide "
                 "required arguments: "
              << std::endl;
    std::cout << options << "\n";
    exit(1);
  }

  // Handle all other arguments
  po::notify(var_map);

  return var_map;
}

/**
 * ##     ##    ###    #### ##    ##
 * ###   ###   ## ##    ##  ###   ##
 * #### ####  ##   ##   ##  ####  ##
 * ## ### ## ##     ##  ##  ## ## ##
 * ##     ## #########  ##  ##  ####
 * ##     ## ##     ##  ##  ##   ###
 * ##     ## ##     ## #### ##    ##
 */
int main(int argc, const char *argv[]) {
  auto args = handle_args(argc, argv);

  // Get the Dataset
  jrl::Parser parser;
  jrl::Dataset dataset = parser.parseDataset(args["input_dataset"].as<std::string>());

  // Build the Agents
  ExperimentRunner::Agents agents;
  for (auto &rid : dataset.robots()) {
    agents[rid] =
        experiments::agent_factory(rid, args["method"].as<std::string>(), args["method_params"].as<std::string>(),
                                   args["pose_type"].as<std::string>());
  }

  // Build the Communication Model
  experiments::CommunicationModelParams params =
      experiments::parseCommunicationModelParams(args["comm_params"].as<std::string>());
  CommunicationModel comm_model(params, dataset, args["random_seed"].as<size_t>());

  // Setup the save out locations
  // Ensure that the output location exists
  bfs::create_directory(args["output_dir"].as<std::string>());
  // Generate the output dir
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream time_ss;
  time_ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  std::string output_directory = args["output_dir"].as<std::string>() + "/" + dataset.name() + "_" +
                                 agents.begin()->second->name() + "_" + time_ss.str();
  bfs::create_directory(output_directory);
  bfs::create_directory(output_directory + "/iterations");

  // run
  ExperimentRunner runner(args["pose_type"].as<std::string>(), !args.count("no_align"));
  runner.runExperiment(dataset, agents, comm_model, output_directory, args["output_rate"].as<size_t>());

  std::cout << "Status: Done" << std::endl;
  return 0;
}
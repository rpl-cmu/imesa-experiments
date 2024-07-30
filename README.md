# iMESA Experiments
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

This repository houses the code + infrastructure + scripts to run the experiments found in our RSS 2024 paper "iMESA: Incremental Distributed Optimization for Collaborative Simultaneous Localization and Mapping" which can be found [arXiv](https://arxiv.org/pdf/2406.07371). If you use this package please cite our paper as:

```
@inproceedings{mcgann_imesa_2024, 
    title = {{iMESA}: Incremental Distributed Optimization for Collaborative Simultaneous Localization and Mapping},
    author = {D. McGann and M. Kaess},
    fullauthor = {Daniel McGann and Michael Kaess},
    booktitle = {Proc. Robotics: Science and Systems (RSS)},
    year = 2024,
    pages = {n/a}, % will be added soon
    address = {Delft, {NL}}
}
```

The actual implementation of iMESA is housed separately [here](https://github.com/rpl-cmu/imesa) to make it easier for users to incorporate into their own projects.

## Structure
This repository is broken down into two main modules:
* `experiments/` - Houses code to run experiments, and houses scripts to generate data, and analyze/plot results.
* `prior_work/` - Houses implementation of prior works and baselines for use as comparison in the experiments

Each of these modules has a README with additional information and documentation.

## Run Experiments
This project provides a main entry point for running iMESA and prior works on datasets via the `run-trial` program (find in `build/experiments/run-trial`). More details can be found in the [Experiments Module](./experiments/README.md).

## Prior works

This repository currently provides implementations of a Centralized baseline, and Independent Baseline, and DDF-SAM2. Descriptions of each can be found in the [Prior Work Module](./prior_work/README.md).

A keen reader will notice that Distributed Loopy Belief Propagation (DLGBP) (an important comparison in the paper above) is missing from this list. The original authors of DLGBP were kind enough to provide us with their internal implementation of the algorithm to use in our experiments (Shoutout to Riku Murai for all their help!). However, as it was their internal implementation, it is not our place to release it. Therefore we have not included DLGBP in the public release of this repository.

## Dependencies
  * GTSAM - [Github](https://github.com/borglab/gtsam) - Factor-graph library for SLAM.
  * nlohmann-json - [Github](https://github.com/nlohmann/json) - JSON library provides parsing/serializing of JSON.
  * JRL - [Github](https://github.com/DanMcGann/jrl) - SLAM dataset library for IO of datasets and results.
  * iMESA - [Github](https://github.com/rpl-cmu/imesa) - Implementation of the iMESA algorithm.

## Setup Instructions

The following instructions are designed to construct a local build of iMESA experiments with the proper versions of dependencies. We link the projects to each other using `*_DIR` and `*_INCLUDE_DIR` CMake variables to ensure we build everything against the correct version, and permit users to have other versions of GTSAM or JRL installed on their system.


1. Install System Dependencies
  * nlohmann-json - `sudo apt-get install nlohmann-json3-dev`
    * Note this is the only system-wide dependency. All other dependencies are handled locally to allow users to have other versions of GTSAM etc. installed on their machine.
2. Construct a workspace directory
  * `cd /path/to/prefered/location`
  * `mkdir WORKSPACE`
  * The following instructions will refer WORKSPACE as a path and users should insert the absolute path to their chosen workspace. 
2. Build GTSAM
  * `cd WORKSPACE`
  * `git clone -b 4.2.0-imesa https://github.com/DanMcGann/gtsam.git`
    * This version is GTSAM v4.2.0 with a few additional change that are detailed in the [iMESA repository](https://github.com/rpl-cmu/imesa).
  * `cd gtsam && mkdir build && cd build`
  * `cmake ..`
  * `make`
3. Build JRL
  * `cd WORKSPACE`
  * `git clone -b v1.0.0 https://github.com/DanMcGann/jrl.git`
  * `cd jrl && mkdir build && cd build`
  * `cmake .. -DGTSAM_DIR=WORKSPACE/gtsam/build -DGTSAM_INCLUDE_DIR=WORKSPACE/gtsam/gtsam`
  * `make`
4. Build the iMESA Experiments
  * `cd WORKSPACE`
  * `git clone https://github.com/rpl-cmu/imesa-experiments.git`
  * `cd imesa-experiments && mkdir build && cd build`
  * `cmake .. -DGTSAM_DIR=WORKSPACE/gtsam/build -DGTSAM_INCLUDE_DIR=WORKSPACE/gtsam/gtsam -Djrl_DIR=WORKSPACE/jrl/build -Djrl_INCLUDE_DIR=WORKSPACE/jrl/include`
  * `make`

Note: iMESA is included automatically via FetchContent.

## Issues
If you encounter issues while using this project please file a bug report on github!

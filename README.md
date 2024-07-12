# iMESA Experiments
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

This repository houses the code + infrastructure + scripts to run the experiments found in our RSS 2024 paper "iMESA: Incremental Distributed Optimization for Collaborative Simultaneous Localization and Mapping" which can be found [TODO arXiv LINK](). If you use this package please cite our paper as:

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

## Installation Instructions

The following installation instructions assume the user is working in Ubuntu 20.04. The project work under other development environments, but is untested.

1. Install Dependencies
  * GTSAM - See [GTSAM's Installation Documentation](https://github.com/borglab/gtsam/tree/develop?tab=readme-ov-file#quickstart)
  * nlohmann-json - `sudo apt-get install nlohmann-json3-dev`
  * JRL - See [JRL Installation Documentation](https://github.com/DanMcGann/jrl/blob/main/LIBRARY.md#install-instructions)
  * Note: the [iMESA implementation](https://github.com/rpl-cmu/imesa) is automatically handled using FetchContent
2. Clone this repository
  * SSH:`clone git@github.com:rpl-cmu/imesa.git`
  * HTTPS: `clone https://github.com/rpl-cmu/imesa.git`
3. Build this project
  * `mkdir build && cd build/`
  * `cmake ..`
  * `make`

## Issues
If you encounter issues while using this project please file a bug report on github!

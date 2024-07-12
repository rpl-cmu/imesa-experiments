# Experiments
This module contains the implementation of the main entry point to run trials using this package (see `run-trial.cpp`). It also contains default configuration for all supported methods, and scripts for generating data and analysis/plotting of results.

## Run Trial
The main entry point for running trials. Executable is found in `build/experiments/run-trial`. Usage can be found via `--help` (copied below).
```
run-trial: Main entry point to run incremental distributed SLAM methods on JRL datasets. Please provide required arguments: 
Allowed options:
  -h [ --help ]              Produce help message
  -d [ --input_dataset ] arg (Required) Path to JSON Robot Log dataset file.
  -m [ --method ] arg        (Required) The name of the method to run (see 
                             factory.h for list of method names).
  -p [ --method_params ] arg (Optional) Path to method parameters file.
  -c [ --comm_params ] arg   (Optional) Path to communication model parameters 
                             file.
  -o [ --output_dir ] arg    (Required) Directory to which the results will be 
                             saved.
  -r [ --output_rate ] arg   (Required) Rate that intermediate results will be 
                             saved in iterations.
  -s [ --random_seed ] arg   (Required) The random seed to use for this 
                             trial.
  -t [ --pose_type ] arg     (Required) The type of the poses in the dataset 
                             (for metric computation) [POSE2, POSE3, VEC2, 
                             VEC3].
  --no_align                 (Optional) Flag indicating we should not align 
                             trajectories in metric computation.
```

Run experiment will output results into a directory with the following structure
```
`<dataset_name>_<method_name>_<date+time>/
  - final_results.jrr.cbor        - The final results in JSON Robot Results format
  - final_metrics.jrm.cbor        - The metrics for the final results in JSON Robot Metrics format
  - robot_iteration_runtimes.json - The iteration runtimes for each robot as a JSON dictionary {"rid": [iter_time_0, ...]}
  - iterations/                   - Directory containing results+metrics for intermediate iterations saved out at the rate specified by -r
```

## Running Experiments 
Often we want to run more than one trial, (i.e. an experiment consisting of many trials of many methods). Functionality to do so (and use multi-processing) can be found in `scripts/run-experiment`. This python script delegates to `run-trial` to run many methods on many datasets. This script requires that an experiment is setup in a directory with the following structure:

```
experiment_dir/
  - datasets/
      - independent_var_0/
      - independent_var_1/
      - ...
  - results/
      - independent_var_0/
      - independent_var_1/
      - ...
```
Running the script will run each method on each dataset in `datasets/<iv>/*` and output the results into `results/<iv>/<method>/*` where `iv` is the name of the independent variable.

## Submodules
The following are submodules that are used by `run-trial`
* `communication_model` - Class that models sparse, range-limited, communication between agents.
* `config` - Provides support to parse method and communication parameter files. (See examples in `config/`)
* `factory` - Provides support to construct instances of different methods. See for list of method names.

## Scripts
The `scripts/` directory also contains many scripts for creating datasets and analyzing+plotting results. For usage of each script see run them with `--help` and see their in-line documentation. Below is a outlier of the purpose of each script to make things easier

* Scripts for creating datasets
  - `make-multi-robot-dataset-2d`
  - `make-multi-robot-dataset-3d`
  - `make-multi-robot-landmark-dataset-2d`
* Scripts for analyzing individual trials (i.e. the outputs from `run-trial`)
  - `compute-iate`
  - `plot-mean-residual`
  - `plot-timing`
  - `plot-timing-async`
* Scripts for analyzing results from experiments (i.e. outputs from `run-experiment`)
  - `summarize-experiment`
  - `summarize-iteration-times`
  - `summarize-timing`
* Scripts for plotting the trajectory results of trials
  - `animate-jrl-2d`
  - `plot-jrl`
  - `plot-many-results`
  - `plot-many-result-error`

## Configuration
The `config/` directory contains default method and communication configuration files. These files can be provided to `run-trial` to configure method and communication model behaviors. See the corresponding class implementations for details on the meaning of the parameters set by these files.


## Delay Experiment
A keen observer will note that the delay experiment presented in the paper is not reproducible via this module. Modeling delays requires changes to the source code of iMESA that break other functionality. As such we do not include the functionality to re-run the delay experiment in favor of supporting other more useful functionality and maintaining a clean implementation.

# Example Dataset Generation
Below are some example commands for generating synthetic datasets like those used for the experimental results section of our paper.

<details>
  <summary>Example Scale Experiment Dataset Generation</summary>
  
  ```
  /path/to/scripts/make-multi-robot-dataset-2d -o . -n scale_exp_5 -r 1 -nr 5 -np 500 --odom_probs 0.85 0.1 0.05 --odom_type gridworld --loop_closure_distance_threshold 2 --loop_closure_probability 0.5 --loop_closure_index_threshold 5 --comm_range 30 --comm_freq 5 --prior_noise_sigmas 0.01 0.01 0.1 --robot_zero_prior_noise_sigmas 0.01 0.01 0.1 --odom_noise_sigmas 0.05 0.05 1 --loop_noise_sigmas 0.1 0.1 1 --comm_loop_measurement_type pose --comm_loop_noise_sigmas 0.1 0.1 1 --xlims -45 45 --ylims -45 45
  ```
</details>

<details>
  <summary>Example Lifelong Experiment Dataset Generation</summary>
  
  ```
 /path/to/scripts/make-multi-robot-dataset-2d -o . -n lifelong_exp_5000 -r 1 -nr 5 -np 5000 --odom_probs 0.9 0.06 0.04 --odom_type gridworld --loop_closure_distance_threshold 2 --loop_closure_probability 0.3 --loop_closure_index_threshold 10 --comm_range 30 --comm_freq 5 --prior_noise_sigmas 0.01 0.01 0.1 --robot_zero_prior_noise_sigmas 0.01 0.01 0.1 --odom_noise_sigmas 0.05 0.05 1 --loop_noise_sigmas 0.1 0.1 1 --comm_loop_measurement_type pose --comm_loop_noise_sigmas 0.1 0.1 1 --xlims -80.0 80.0 --ylims -80.0 80.0
  ```
</details>

<details>
  <summary>Example 3D Dataset Generation</summary>
  
  ```
  /path/to/scripts/make-multi-robot-dataset-3d -o . -n model_exp_3D_pgo -r 1 -nr 5 -np 1000 --loop_closure_distance_threshold 2 --loop_closure_probability 0.4 --loop_closure_index_threshold 10 --comm_range 30 --comm_freq 5 --prior_noise_sigmas 0.001 0.001 0.001 0.01 0.01 0.01 --robot_zero_prior_noise_sigmas 0.001 0.001 0.001 0.01 0.01 0.01 --odom_noise_sigmas 0.02 0.02 0.02 0.05 0.05 0.05 --loop_noise_sigmas 0.02 0.02 0.02 0.1 0.1 0.1 --comm_loop_measurement_type pose --comm_loop_noise_sigmas 0.02 0.02 0.02 0.1 0.1 0.1 --xlims -30 30 --ylims -30 30 --zlims -10 10
  ```
</details>

<details>
  <summary>Example Bearing Range Dataset Generation</summary>
  
  ```
  /path/to/scripts/make-multi-robot-dataset-2d -o . -n model_exp_2D_bearing_range -r 1 -nr 5 -np 1000 --odom_probs 0.9 0.06 0.04 --odom_type gridworld --loop_closure_distance_threshold 2 --loop_closure_probability 0.4 --loop_closure_index_threshold 10 --comm_range 30 --comm_freq 5 --prior_noise_sigmas 0.01 0.01 0.1 --robot_zero_prior_noise_sigmas 0.01 0.01 0.1 --odom_noise_sigmas 0.05 0.05 1 --loop_noise_sigmas 0.1 0.1 1 --comm_loop_measurement_type bearing_range --comm_loop_noise_sigmas 1 0.1 --xlims -50 50 --ylims -50 50
  ```
</details>
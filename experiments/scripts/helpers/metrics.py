import os
from .parsing import read_results_metrics_all
import jrl
import json
import numpy as np


def aggregate_metrics(result_dir, accumulator):
    parser = jrl.Parser()
    if os.path.isfile(os.path.join(result_dir, "final_metrics.jrm.cbor")):
        # First Parse and aggregate the final results
        method_dataset_file_result_file = os.path.join(
            result_dir, "final_metrics.jrm.cbor"
        )
        ms = parser.parseMetricSummary(method_dataset_file_result_file, True)
        accumulator["ate_trans"].append(ms.total_ate[0])
        accumulator["ate_rot"].append(ms.total_ate[1])
        accumulator["mean_residual"].append(ms.mean_residual)

        # Next parse all intermediate results and compute incremental metrics
        iterations, metrics = read_results_metrics_all(result_dir)
        iter_sum = sum(iterations)
        iate_trans, iate_rot, iate_mr = 0, 0, 0
        for i, ms in zip(iterations, metrics):
            iate_trans += (i / iter_sum) * ms.total_ate[0]
            iate_rot += (i / iter_sum) * ms.total_ate[1]
            iate_mr += (i / iter_sum) * ms.mean_residual

        # Next parse the timing file and save the average cumulative runtime
        timing_file = os.path.join(result_dir, "robot_iteration_runtimes.json")
        cumulative_runtimes = []
        avg_cumulative_runtime = 0
        with open(timing_file) as f:
            robot_times = json.load(f)
            for _, times in robot_times.items():
                cumulative_runtimes.append(
                    np.sum(np.array(times) * 1e-9)  # convert to seconds
                )
            if "centralized" in result_dir:
                avg_cumulative_runtime = np.sum(np.array(cumulative_runtimes))
            else:
                avg_cumulative_runtime = np.average(np.array(cumulative_runtimes))

        accumulator["iate_trans"].append(iate_trans)
        accumulator["iate_rot"].append(iate_rot)
        accumulator["imean_residual"].append(iate_mr)
        accumulator["average_total_runtime"].append(avg_cumulative_runtime)
        accumulator["statuses"].append(True)
    else:
        accumulator["statuses"].append(False)

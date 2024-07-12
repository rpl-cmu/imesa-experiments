import glob
import os

import jrl
import numpy as np


def read_results_metrics_all(result_dir):
    parser = jrl.Parser()
    metric_summaries = []
    iterations = []

    # Aggregate all the metric summary files
    metrics_files = sorted(glob.glob(os.path.join(result_dir, "iterations", "*.jrm*")))
    # metrics_files.append(os.path.join(result_dir, "final_metrics.jrm.cbor"))

    for mf in metrics_files:
        if os.path.isfile(mf):
            file_name = os.path.basename(mf)
            iter_num = int(file_name.split("_")[0])
            metric_summaries.append(parser.parseMetricSummary(mf, True))
            iterations.append(iter_num)
        else:
            return None
    return iterations, metric_summaries



def read_results_all(result_dir):
    parser = jrl.Parser()
    results = []

    # Aggregate all the metric summary files
    result_files = sorted(glob.glob(os.path.join(result_dir, "iterations", "*.jrr*")))
    result_files.append(os.path.join(result_dir, "final_results.jrr.cbor"))

    for rf in result_files:
        if os.path.isfile(rf):
            results.append(parser.parseResults(rf, True))
        else:
            raise "Error read_results_all could not parse an iteration or final results file"
    return results
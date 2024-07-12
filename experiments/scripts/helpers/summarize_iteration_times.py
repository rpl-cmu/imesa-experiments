import glob
import json
import os
import pickle

import jrl
import matplotlib.pyplot as plt
import numpy as np
from helpers.method_style_sheet import METHOD_STYLE_SHEET
from matplotlib.ticker import Locator

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["pdf.fonttype"] = 42

OFFSET = {
    "centralized": 0.04,
    "ddfsam2": -0.02,
    "imesa": 0.003,
    "independent": -0.003,
    "dlgbp": 0,
    "dlgbp_windowed": 0,
}


class MinorSymLogLocator(Locator):
    """
    Dynamically find minor tick positions based on the positions of
    major ticks for a symlog scaling.
    """

    def __init__(self, linthresh):
        """
        Ticks will be placed between the major ticks.
        The placement is linear for x between -linthresh and linthresh,
        otherwise its logarithmically
        """
        self.linthresh = linthresh

    def __call__(self):
        "Return the locations of the ticks"
        majorlocs = self.axis.get_majorticklocs()
        majorlocs = np.array(majorlocs.tolist() + [majorlocs[-1] * 10.0])

        # iterate through minor locs
        minorlocs = []

        # handle the lowest part
        for i in range(1, len(majorlocs)):
            majorstep = majorlocs[i] - majorlocs[i - 1]
            if abs(majorlocs[i - 1] + majorstep / 2) < self.linthresh:
                ndivs = 10
            else:
                ndivs = 9
            minorstep = majorstep / ndivs
            locs = np.arange(majorlocs[i - 1], majorlocs[i], minorstep)[1:]
            minorlocs.extend(locs)

        return self.raise_if_exceeds(np.array(minorlocs))

    def tick_values(self, vmin, vmax):
        raise NotImplementedError(
            "Cannot get tick locations for a " "%s type." % type(self)
        )


def aggregate_timing(result_dir, accumulator):
    if os.path.isfile(os.path.join(result_dir, "final_metrics.jrm.cbor")):
        # Parse the timing file and save the average cumulative runtime
        timing_file = os.path.join(result_dir, "robot_iteration_runtimes.json")
        iteration_runtimes = []
        with open(timing_file) as f:
            robot_times = json.load(f)
            for _, times in robot_times.items():
                iteration_runtimes.append(np.array(times) * 1e-9)  # convert to seconds
            iteration_runtimes = np.stack(iteration_runtimes)

            if "centralized" in result_dir:
                avg_iteration_runtime = np.sum(iteration_runtimes, axis=0)
            else:
                avg_iteration_runtime = np.average(iteration_runtimes, axis=0)
        accumulator.append(avg_iteration_runtime)


def aggregate_results(
    dataset_dir, result_dir, independent_variables, all_methods, incremental=False
):
    parser = jrl.Parser()

    # Setup storage for the results for each method
    aggregated_results = {}
    for method in all_methods:
        aggregated_results[method] = {}
        for iv in independent_variables:
            aggregated_results[method][iv] = []

    for iv in independent_variables:
        # Lets get all the datasets
        all_dataset_files = sorted(glob.glob(os.path.join(dataset_dir, iv, "*.jrl")))
        for dataset_file in all_dataset_files:
            # Parse the dataset
            dataset = parser.parseDataset(dataset_file, False)

            for method in all_methods:
                method_result_dir = os.path.join(result_dir, iv, method)
                print(os.path.join(method_result_dir, "{}*/".format(dataset.name())))
                method_dataset_result_dir = glob.glob(
                    os.path.join(method_result_dir, "{}*/".format(dataset.name()))
                )[0]

                aggregate_timing(
                    method_dataset_result_dir, aggregated_results[method][iv]
                )
    return aggregated_results


def summarize_iteration_times(
    experiment_name,
    dataset_dir,
    result_dir,
    independent_variables,
    all_methods,
    xlabel,
    legend,
    output,
):
    pkl_file = os.path.join(result_dir, "iteration_timing_summary.pkl")
    if not os.path.exists(pkl_file):
        aggregated_results = aggregate_results(
            dataset_dir,
            result_dir,
            independent_variables,
            all_methods,
        )
        with open(pkl_file, "wb") as handle:
            pickle.dump(aggregated_results, handle)
    else:
        with open(pkl_file, "rb") as pickle_file:
            aggregated_results = pickle.load(pickle_file)

    # Lets setup the figure
    num_iv = len(independent_variables)
    for iv in independent_variables:
        fig, ax = plt.subplots(
            1,
            1,
            figsize=[4 if not legend else 5, 1.6 if experiment_name == "" else 2.5],
            dpi=200,
        )
        fig.suptitle(experiment_name, fontsize=12)

        for i, method in enumerate(all_methods):
            times = np.stack(aggregated_results[method][iv])
            trial_avg_times = np.average(times, axis=0)
            WINDOW = 100
            augmented_average_times = np.concatenate(
                (trial_avg_times, np.array([trial_avg_times[-1]] * (WINDOW - 1)))
            )
            smoothed_avg_times = np.convolve(
                augmented_average_times,
                np.ones(WINDOW) / float(WINDOW),
                mode="valid",
            )
            print(method, smoothed_avg_times[-5:-1])

            ax.plot(
                np.arange(0, int(iv)),
                smoothed_avg_times,
                color=METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                linewidth=1,
            )
            ax.plot(
                [int(iv) + 200],
                smoothed_avg_times[-1] + OFFSET[method] * 1.5,
                color=METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                marker=METHOD_STYLE_SHEET[method]["symbol"],
                markersize=3,
                linewidth=1,
                label=METHOD_STYLE_SHEET[method]["name"],
            )

        ax.set_ylabel("Runtime (s)")
        ax.set_xlabel("Iteration")

        if legend:
            ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))

        ax.set_yscale("symlog", linthresh=0.1)
        ax.set_ylim(bottom=0, top=ax.get_ylim()[1] * 1.2)
        ax.yaxis.set_minor_locator(MinorSymLogLocator(1e1))
        ax.grid(b=True, which="major", axis="both", alpha=0.5)
        ax.grid(b=True, which="both", axis="y", alpha=0.5)

        fig.tight_layout(pad=0.1)
        fig.subplots_adjust(wspace=0.2, hspace=0.05)
        if experiment_name != "":
            fig.subplots_adjust(top=0.9)

        if output:
            plt.savefig(output)

    plt.show()

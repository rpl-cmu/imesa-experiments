import glob
import os
import pickle

import gtsam
import jrl
import matplotlib.pyplot as plt
import matplotlib.ticker as tck
import numpy as np
from helpers.method_style_sheet import METHOD_STYLE_SHEET
from helpers.metrics import aggregate_metrics
from helpers.plot_summaries import boxplot, finish_boxplot_axes, plot_boxplot_symbol

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["pdf.fonttype"] = 42

import codecs


def aggregate_results(
    dataset_dir, result_dir, independent_variables, all_methods, incremental=False
):
    parser = jrl.Parser()

    # Setup storage for the results for each method
    aggregated_results = {}
    for iv in independent_variables:
        aggregated_results[iv] = {}
        for method in all_methods:
            aggregated_results[iv][method] = {
                "ate_trans": [],
                "ate_rot": [],
                "mean_residual": [],
                "iate_trans": [],
                "iate_rot": [],
                "imean_residual": [],
                "average_total_runtime": [],
                "statuses": [],
            }
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

                aggregate_metrics(
                    method_dataset_result_dir, aggregated_results[iv][method]
                )
    return aggregated_results


def summarize_experiment(
    experiment_name,
    dataset_dir,
    result_dir,
    independent_variables,
    independent_variable_labels,
    all_methods,
    xlabel,
    legend,
    output,
):
    pkl_file = os.path.join(result_dir, "metric_summary.pkl")
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
    fig, ax = plt.subplots(
        2,
        num_iv,
        figsize=[4 if not legend else 6, 3 if experiment_name == "" else 4],
        dpi=200,
    )
    # ax[0].set_yscale("log")  # , linthresh=10)
    fig.suptitle(experiment_name, fontsize=12)

    for j, iv in enumerate(independent_variables):
        for i, method in enumerate(all_methods):
        
            boxplot(
                aggregated_results[iv][method]["iate_trans"],
                ax[0, j],
                0,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                width=0.14,
            )
            boxplot(
                aggregated_results[iv][method]["iate_rot"],
                ax[1, j],
                0,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                width=0.14,
            )
            """
            boxplot(
                aggregated_results[iv][method]["average_total_runtime"],
                ax[2, j],
                0,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                width=0.15,
            )
            """

    for j in range(num_iv):
        finish_boxplot_axes(ax[0, j], [independent_variables[j]])
        finish_boxplot_axes(ax[1, j], [independent_variables[j]])
        #finish_boxplot_axes(ax[2, j], [independent_variables[j]])
        #ax[2, j].set_yscale("log")

    for j, iv in enumerate(independent_variables):
        for i, method in enumerate(all_methods):
            plot_boxplot_symbol(
                ax[0, j],
                0,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                symbol=METHOD_STYLE_SHEET[method]["symbol"],
                symbol_size=6,
                label=METHOD_STYLE_SHEET[method]["name"] if j == num_iv - 1 else None,
                linestyle="",  # METHOD_STYLE_SHEET[method]["linestyle"],
            )

    ax[0, 0].set_ylabel("iATE (Translation)")
    ax[1, 0].set_ylabel("iATE (Rotation)")
    ax[1, 0].set_xlabel(xlabel)
    #ax[2, 0].set_ylabel("Cumulative Runtime / Robot")

    # The following 3 lines were used for the measurement type experiment plot
    # ax.yaxis.set_major_locator(tck.LogLocator(base=100.0, numticks=5))
    # x.yaxis.set_minor_locator(tck.LogLocator(base=100.0, numticks=1000 ,subs=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)))
    # plt.setp(ax.get_yminorticklabels(), visible=False)

    if legend:
        ax[0, num_iv - 1].legend(loc="center left", bbox_to_anchor=(1, 0.5))
    for j in range(num_iv):
        ax[1, j].set_xticklabels([independent_variable_labels[j]], minor=True)

    fig.tight_layout(pad=0.1)
    fig.subplots_adjust(wspace=0.25, hspace=0.1)
    fig.align_labels()
    if experiment_name != "":
        fig.subplots_adjust(top=0.9)

    if output:
        plt.savefig(output)

    plt.show()

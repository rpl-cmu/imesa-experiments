import gtsam
import numpy as np
import matplotlib.pyplot as plt


def getPoint(k, vals, linear, is3d):
    if linear:
        if is3d:
            return vals.atPoint3(k)
        else:
            return vals.atPoint2(k)
    else:
        if is3d:
            return vals.atPose3(k).translation()
        else:
            return vals.atPose2(k).translation()


def plot_loops(ax, dataset, results, colors, plot_intra, plot_inter, is3d):
    for ridx, robot in enumerate(dataset.robots()):
        for entry in dataset.measurements(robot):
            for i in range(entry.measurements.nrFactors()):
                factor = entry.measurements.at(i)
                keys = factor.keys()
                if len(keys) > 1:
                    k1, k2 = keys
                    s1, s2 = gtsam.Symbol(k1), gtsam.Symbol(k2)
                    rid1, rid2 = chr(s1.chr()), chr(s2.chr())

                    # Results may only be partial
                    if results.robot_solutions[rid1].values.exists(
                        k1
                    ) and results.robot_solutions[rid2].values.exists(k2):
                        if s2.chr() == ord("#"):
                            if plot_intra or plot_inter:
                                pts = np.array(
                                    [
                                        getPoint(
                                            k1,
                                            results.robot_solutions[rid1].values,
                                            False,
                                            is3d,
                                        ),
                                        getPoint(
                                            k2,
                                            results.robot_solutions[rid2].values,
                                            True,
                                            is3d,
                                        ),
                                    ]
                                )
                                if is3d:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        pts.T[2],
                                        color="black",
                                        alpha=0.1,
                                        linewidth=0.6,
                                    )
                                else:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        color="black",
                                        alpha=0.1,
                                        linewidth=0.6,
                                    )
                        else:
                            pts = np.array(
                                [
                                    getPoint(
                                        k1,
                                        results.robot_solutions[rid1].values,
                                        False,
                                        is3d,
                                    ),
                                    getPoint(
                                        k2,
                                        results.robot_solutions[rid2].values,
                                        False,
                                        is3d,
                                    ),
                                ]
                            )

                            if (
                                chr(s1.chr()) == chr(s2.chr())
                                and (abs(s1.index() - s2.index()) != 1)
                                and plot_intra
                            ):
                                if is3d:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        pts.T[2],
                                        color=colors[ridx],
                                        alpha=0.5,
                                        marker=".",
                                    )
                                else:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        color=colors[ridx],
                                        alpha=0.5,
                                        marker=".",
                                    )

                            if chr(s1.chr()) != chr(s2.chr()) and plot_inter:
                                if is3d:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        pts.T[2],
                                        color="black",
                                        alpha=0.1,
                                        linewidth=0.6,
                                    )
                                else:
                                    ax.plot(
                                        pts.T[0],
                                        pts.T[1],
                                        color="black",
                                        alpha=0.1,
                                        linewidth=0.6,
                                    )


def plot_traj_2d(
    ax,
    dataset,
    results,
    colors,
    label="",
    include_gt=False,
    include_shared_vars=False,
    linear=False,
):
    for idx, robot in enumerate(dataset.robots()):
        if dataset.containsGroundTruth() and include_gt:
            gtx, gty = [], []
            gtvals = dataset.groundTruth(robot)
            for k in gtvals.keys():
                s = gtsam.Symbol(k)
                if chr(s.chr()) == robot:
                    if linear:
                        p = gtvals.atPoint2(k)
                        gtx.append(p[0])
                        gty.append(p[1])
                    else:
                        p = gtvals.atPose2(k)
                        gtx.append(p.x())
                        gty.append(p.y())
            ax.plot(gtx, gty, alpha=0.5, color=colors[idx], linewidth=0.8)

        sx, sy = [], []
        lx, ly = [], []
        ox, oy = [], []
        svals = results.robot_solutions[robot].values
        for k in svals.keys():
            s = gtsam.Symbol(k)
            if linear or s.chr() == ord("#"):
                p = svals.atPoint2(k)
            else:
                p = svals.atPose2(k)
                p = [p.x(), p.y()]
            if chr(s.chr()) == robot:
                sx.append(p[0])
                sy.append(p[1])
            elif s.chr() == ord("#"):
                lx.append(p[0])
                ly.append(p[1])
            else:
                ox.append(p[0])
                oy.append(p[1])

        ax.plot(sx, sy, alpha=1, color=colors[idx], label=label, linewidth=0.8)

        if include_shared_vars:
            ax.plot(ox, oy, "o", color=colors[idx])
            ax.plot(
                lx,
                ly,
                "o",
                color=colors[idx],
                markersize=3 + 2 * idx,
                fillstyle="none",
                zorder=99,
            )
    plot_loops(ax, dataset, results, colors, False, True, False)


def plot_traj_3d(
    ax,
    dataset,
    results,
    colors,
    label="",
    include_gt=False,
    include_shared_vars=False,
    linear=False,
):
    # fig, td_ax = plt.subplots(1, 1, figsize=[4, 4], dpi=200)
    for idx, robot in enumerate(dataset.robots()):
        if dataset.containsGroundTruth() and include_gt:
            gt = []
            gtvals = dataset.groundTruth(robot)
            for k in gtvals.keys():
                s = gtsam.Symbol(k)
                if chr(s.chr()) == robot:
                    if linear:
                        gt.append(gtvals.atPoint3(k))
                    else:
                        gt.append(gtvals.atPose3(k).translation())
            gt = np.stack(gt)
            ax.plot(gt.T[0], gt.T[1], gt.T[2], alpha=0.5, color=colors[idx])
            # td_ax.plot(gt.T[1], gt.T[0], alpha=0.5, color=colors[idx])

        prev = np.zeros(3)
        sol = [[]]
        oth = []
        svals = results.robot_solutions[robot].values
        for k in svals.keys():
            s = gtsam.Symbol(k)
            if linear:
                p = svals.atPoint3(k)
            else:
                p = svals.atPose3(k).translation()
                if np.linalg.norm(prev - p) > 7:
                    sol.append([])
                prev = p
            if chr(s.chr()) == robot:
                sol[-1].append(p)
            else:
                oth.append(p)
        for partial_sol in sol:
            if len(partial_sol) > 0:
                partial_sol = np.stack(partial_sol)
                if len(oth) > 0:
                    oth = np.stack(oth)
                ax.plot(
                    partial_sol.T[0],
                    partial_sol.T[1],
                    partial_sol.T[2],
                    alpha=1,
                    color=colors[idx],
                    label=label,
                )
                # td_ax.plot(
                #    partial_sol.T[1],
                #    partial_sol.T[0],
                #    alpha=1,
                #    color=colors[idx],
                #    label=label,
                # )

        if include_shared_vars and (len(oth) > 0):
            ax.plot(oth.T[0], oth.T[1], oth.T[2], "o", color=colors[idx])

    plot_loops(ax, dataset, results, colors, True, True, True)
    # ax.view_init(elev=90, azim=-90)
    # td_ax.set_aspect("equal")
    # td_ax.set_axis_off()
    # print(td_ax.get_xlim(), td_ax.get_ylim())
    # fig.tight_layout(pad=0.25)

"""Generate a square-run figure and per-side distance/closure diagnostics."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    summary = json.loads((directory / "summary.json").read_text())
    route = summary["square"]
    if not summary["completed"] or len(route["corners"]) != 4:
        raise ValueError("This report requires a completed square")
    keys = ["t", "capo_x", "capo_y", "capo_yaw", "gt_x", "gt_y", "gt_yaw"]
    with (directory / "samples.csv").open() as f:
        data = np.array([[float(r[k]) for k in keys] for r in csv.DictReader(f)])
    # The summary defines the evaluation segment, excluding warmup.
    data = data[-summary["samples"]:]
    truth = (data[:, 4:6]-data[0, 4:6]) @ rotation(data[0, 6])
    estimate = (data[:, 1:3]-data[0, 1:3]) @ rotation(data[0, 3])
    error = np.linalg.norm(estimate-truth, axis=1)
    travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
    indices = [0]+[int(np.argmin(abs(data[:, 0]-c["time_s"]))) for c in route["corners"]]
    sides = []
    for side, (a, b) in enumerate(zip(indices, indices[1:]), 1):
        actual = float(np.linalg.norm(truth[b]-truth[a]))
        estimated = float(np.linalg.norm(estimate[b]-estimate[a]))
        sides.append({"side": side, "actual_side_chord_m": actual,
                      "estimated_side_chord_m": estimated,
                      "distance_underestimate_percent": 100*(actual-estimated)/actual,
                      "corner_position_error_m": float(error[b])})
    (directory / "side_metrics.json").write_text(json.dumps(sides, indent=2)+"\n")
    np.testing.assert_allclose(error[-1], summary["endpoint_error_m"], atol=1e-10)

    import matplotlib
    matplotlib.use("Agg")
    import mpl_toolkits
    toolkit = Path(matplotlib.__file__).parent.parent / "mpl_toolkits"
    if toolkit.is_dir(): mpl_toolkits.__path__ = [str(toolkit)]
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    length = route["side_length_m"]
    ideal = length*np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
    ax = axes[0, 0]
    ax.plot(ideal[:, 0], ideal[:, 1], "k--", alpha=.45, label="Target square")
    ax.plot(truth[:, 0], truth[:, 1], label="MuJoCo ground truth", linewidth=1.5)
    ax.plot(estimate[:, 0], estimate[:, 1], label="CAPO odometry", linewidth=1.5)
    for i in range(1, 4):
        ax.scatter(*truth[indices[i]], color="C0", s=25)
        ax.scatter(*estimate[indices[i]], color="C1", s=25)
        ax.annotate(f"Corner {i}", truth[indices[i]], xytext=(5, 5), textcoords="offset points")
    ax.scatter(0, 0, c="green", marker="o", label="Start")
    ax.set(xlabel="x in start frame [m]", ylabel="y in start frame [m]", title=f"{length:g} m square: full trajectory")
    ax.set_aspect("equal", adjustable="box"); ax.legend(loc="center")
    ax = axes[0, 1]
    ax.plot(travel, error, color="C3")
    for i in range(1, 5):
        ax.scatter(travel[indices[i]], error[indices[i]], color="black", s=20)
        ax.annotate(str(i), (travel[indices[i]], error[indices[i]]), xytext=(3, 7), textcoords="offset points")
    ax.set(xlabel="Actual travelled distance [m]", ylabel="Position error [m]",
           title=f"RMSE {summary['position_rmse_m']:.3f} m; maximum {error.max():.3f} m")
    ax.grid(alpha=.2)
    ax = axes[1, 0]
    ax.scatter(0, 0, c="green", marker="o", s=70, label="Start")
    ax.scatter(*(100*truth[-1]), c="C0", s=70, label="Actual finish")
    ax.scatter(*(100*estimate[-1]), c="C1", marker="x", s=90, label="Odometry finish")
    ax.plot([100*truth[-1, 0], 100*estimate[-1, 0]],
            [100*truth[-1, 1], 100*estimate[-1, 1]], "k--", label=f"Difference {100*error[-1]:.2f} cm")
    ax.axhline(0, color="gray", linewidth=.5); ax.axvline(0, color="gray", linewidth=.5)
    ax.set(xlabel="x [cm]", ylabel="y [cm]", title="Finish near the start (zoom)")
    ax.set_aspect("equal", adjustable="box"); ax.legend()
    ax = axes[1, 1]
    x = np.arange(4)
    ax.bar(x-.18, [r["actual_side_chord_m"] for r in sides], .36, label="Actual corner-to-corner")
    ax.bar(x+.18, [r["estimated_side_chord_m"] for r in sides], .36, label="Estimated corner-to-corner")
    for i, row in enumerate(sides):
        ax.text(i, length*1.03, f"-{row['distance_underestimate_percent']:.2f}%", ha="center")
    ax.set(xticks=x, xticklabels=["Side 1", "Side 2", "Side 3", "Side 4"], ylabel="Distance [m]",
           ylim=(0, length*1.15), title="Distance underestimation persists on every side")
    ax.legend(loc="lower center")
    fig.suptitle("Marsdog CAPO odometry — square trajectory test", fontsize=17)
    fig.savefig(directory / "square_report.png", dpi=180)
    fig.savefig(directory / "square_report.pdf")
    plt.close(fig)

    lines = [f"# Marsdog {length:g} 米方形里程计测试", "", "测试日期：2026-11-09（Asia/Shanghai）。", "",
             "已连续走完四条边，全程没有重置里程计。路线控制使用 MuJoCo 位置和 IMU 航向到达方形路点；CAPO 只接收 IMU 与关节数据。起始位姿只对齐一次，没有尺度拟合或轨迹校正。", "",
             f"目标边长 **{length:.2f} m**，目标周长 **{4*length:.2f} m**。实际累计行程 **{summary['ground_truth_path_m']:.3f} m**（包含转弯位移和步态摆动）；评测时长 **{summary['duration_s']:.2f} 秒**，另有 4 秒预热。", "",
             "| 指标 | 结果 |", "|---|---:|",
             f"| 全程位置 RMSE | {summary['position_rmse_m']:.4f} m |",
             f"| 全程最大位置误差 | {error.max():.4f} m |",
             f"| 最终里程计与真实终点的误差 | {100*error[-1]:.2f} cm |",
             f"| 真实终点距离起点 | {100*summary['gt_endpoint_distance_from_start_m']:.2f} cm |",
             f"| 里程计终点距离其起点 | {100*summary['odom_endpoint_distance_from_start_m']:.2f} cm |",
             f"| 相对航向 RMSE | {summary['yaw_rmse_deg']:.4f}° |", "",
             "| 边 | 实际拐角间距 | 里程计拐角间距 | 距离低估 | 到该拐角的位置误差 |", "|---|---:|---:|---:|---:|"]
    for row in sides:
        lines.append(f"| {row['side']} | {row['actual_side_chord_m']:.3f} m | {row['estimated_side_chord_m']:.3f} m | {row['distance_underestimate_percent']:.2f}% | {row['corner_position_error_m']:.3f} m |")
    lines += ["", "**四条边方向相反，部分误差可能在闭环时抵消，应同时查看各边距离、全程误差与最终误差。** 这次结果来自理想传感器、平地仿真，尚不能代表真机。", "",
              "## 复现命令", "", "```bash", "cd /home/sunbird/CAPO-Go2-Odometry",
              f"scripts/run_marsdog_sim.sh --scenario square --side-length {length:g} --vx 0.25", "```", "",
              "需要显示窗口和按实际时间运行时，追加 `--viewer --realtime`。方形按路点完成后停止，`--duration` 不控制方形时长；可用 `--square-timeout` 指定最长仿真秒数。", "",
              f"结果目录：`{directory}`。", "",
              f"[轨迹与误差图]({directory}/square_report.png) · [PDF]({directory}/square_report.pdf) · [原始统计]({directory}/summary.json)", "",
              "`inputs.csv` 保存连续的 14 轴和 IMU 输入，`samples.csv` 保存 500 Hz 对齐的里程计、真值、接触估计及路线阶段，`side_metrics.json` 保存各边测量结果。", ""]
    (directory / "square_report.md").write_text("\n".join(lines))
    print(json.dumps({"report": str(directory / "square_report.md"), "sides": sides}, ensure_ascii=False, indent=2))


if __name__ == "__main__": main()

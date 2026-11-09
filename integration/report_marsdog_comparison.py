"""Compare square runs on identical sensor inputs and export a shareable report."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

from report_square_run import rotation


def digest(path):
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def read_run(directory):
    summary = json.loads((directory / "summary.json").read_text())
    if not summary["completed"] or summary["scenario"] != "square":
        raise ValueError("Both runs must have completed the square")
    keys = ["t", "capo_x", "capo_y", "capo_yaw", "gt_x", "gt_y", "gt_yaw"]
    with (directory / "samples.csv").open() as stream:
        data = np.array([[float(row[k]) for k in keys] for row in csv.DictReader(stream)])
    data = data[-summary["samples"]:]
    truth = (data[:, 4:6]-data[0, 4:6]) @ rotation(data[0, 6])
    estimate = (data[:, 1:3]-data[0, 1:3]) @ rotation(data[0, 3])
    error = np.linalg.norm(estimate-truth, axis=1)
    indices = [0]+[int(np.argmin(abs(data[:, 0]-c["time_s"]))) for c in summary["square"]["corners"]]
    sides = []
    for a, b in zip(indices, indices[1:]):
        real, estimated = np.linalg.norm(truth[b]-truth[a]), np.linalg.norm(estimate[b]-estimate[a])
        sides.append({"actual_m": float(real), "estimated_m": float(estimated),
                      "signed_distance_error_percent": float(100*(estimated-real)/real)})
    return summary, data, truth, estimate, error, sides


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("optimized", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    before, after = read_run(args.baseline), read_run(args.optimized)
    first_hash, second_hash = digest(args.baseline / "inputs.csv"), digest(args.optimized / "inputs.csv")
    if first_hash != second_hash:
        raise ValueError("Sensor inputs differ; this report requires a controlled comparison")
    np.testing.assert_array_equal(before[1][:, [0, 4, 5, 6]], after[1][:, [0, 4, 5, 6]])
    args.output.mkdir(parents=True, exist_ok=True)
    metrics = ("position_rmse_m", "max_position_error_m", "endpoint_error_m", "velocity_rmse_m_s", "yaw_rmse_deg", "height_rmse_m")
    result = {"date": datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d"),
              "baseline": str(args.baseline.resolve()), "optimized": str(args.optimized.resolve()),
              "identical_sensor_input_sha256": first_hash,
              "actual_path_m": after[0]["ground_truth_path_m"], "frames": after[0]["all_frames"],
              "metrics": {key: {"baseline": before[0][key], "optimized": after[0][key],
                                 "reduction_percent": 100*(1-after[0][key]/before[0][key])} for key in metrics},
              "sides": {"baseline": before[5], "optimized": after[5]}}
    (args.output / "comparison.json").write_text(json.dumps(result, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import mpl_toolkits
    toolkit = Path(matplotlib.__file__).parent.parent / "mpl_toolkits"
    if toolkit.is_dir(): mpl_toolkits.__path__ = [str(toolkit)]
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)
    truth = after[2]
    travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
    axes[0, 0].plot(*truth.T, "k--", label="Ground truth", linewidth=1.5)
    for name, run, color in (("Baseline", before, "C1"), ("Optimized", after, "C0")):
        axes[0, 0].plot(*run[3].T, color=color, label=name)
        axes[0, 1].plot(travel, run[4], color=color, label=name)
        axes[1, 1].scatter(*(100*run[3][-1]), color=color, label=name, s=65)
    axes[0, 0].set(title="10 m square: same measured URDF and policy", xlabel="x [m]", ylabel="y [m]")
    axes[0, 0].set_aspect("equal"); axes[0, 0].legend()
    axes[0, 1].set(title="Position error along the complete route", xlabel="Actual travel [m]", ylabel="Error [m]")
    axes[0, 1].legend(); axes[0, 1].grid(alpha=.2)
    x = np.arange(4)
    axes[1, 0].bar(x-.18, [s["signed_distance_error_percent"] for s in before[5]], .36, color="C1", label="Baseline")
    axes[1, 0].bar(x+.18, [s["signed_distance_error_percent"] for s in after[5]], .36, color="C0", label="Optimized")
    axes[1, 0].axhline(0, color="black", linewidth=.7)
    axes[1, 0].set(xticks=x, xticklabels=["Side 1", "Side 2", "Side 3", "Side 4"],
                   ylabel="Signed distance error [%]", title="Negative = too short; positive = too long")
    axes[1, 0].legend()
    axes[1, 1].scatter(0, 0, color="green", label="Start", s=65)
    axes[1, 1].scatter(*(100*truth[-1]), color="black", marker="x", label="Actual finish", s=80)
    axes[1, 1].set(title="Finish zoom: closure is a separate metric", xlabel="x [cm]", ylabel="y [cm]")
    axes[1, 1].set_aspect("equal"); axes[1, 1].legend()
    fig.savefig(args.output / "comparison.png", dpi=180)
    fig.savefig(args.output / "comparison.pdf")
    plt.close(fig)
    lines = ["# Marsdog 新 URDF 优化前后对比", "", f"日期：{result['date']}（Asia/Shanghai）。", "",
             "机械模型来自 `/home/sunbird/marsdog_v17_source/quadruped.urdf`；策略、PD 参数和运行器使用原 SDK。两次测试的全部 IMU/14 轴输入文件 SHA-256 完全相同，真实轨迹逐帧一致。", "",
             f"连续完成 10×10 米方形，实际累计行程 {result['actual_path_m']:.3f} 米，共 {result['frames']:,} 帧。只做起始位姿对齐，没有尺度拟合；位置真值只用于路线控制和评分，接触真值未输入估计器。", "",
             "| 指标 | 同模型基线 | 优化后 |", "|---|---:|---:|"]
    for label, key, unit in (("全程位置 RMSE", "position_rmse_m", "m"), ("最大位置误差", "max_position_error_m", "m"),
                             ("终点估计与真实终点差", "endpoint_error_m", "m"), ("水平速度 RMSE", "velocity_rmse_m_s", "m/s"),
                             ("相对航向 RMSE", "yaw_rmse_deg", "°"), ("高度 RMSE", "height_rmse_m", "m")):
        lines.append(f"| {label} | {before[0][key]:.5f} {unit} | {after[0][key]:.5f} {unit} |")
    lines += ["", "| 边 | 实际距离 | 基线估计 | 优化估计 | 优化距离误差 |", "|---|---:|---:|---:|---:|"]
    for i, (b, a) in enumerate(zip(before[5], after[5]), 1):
        lines.append(f"| {i} | {a['actual_m']:.3f} m | {b['estimated_m']:.3f} m | {a['estimated_m']:.3f} m | {a['signed_distance_error_percent']:+.3f}% |")
    closure = "减小" if after[0]["endpoint_error_m"] < before[0]["endpoint_error_m"] else "未减小"
    lines += ["", "优化包括：机身角速度补偿、球形脚掌滚动位置补偿、按载荷和速度一致性加权支撑脚观测，以及按实际时间戳预测（修正 5 ms 固定步长与 2 ms 输入不一致）。未乘经验距离比例。", "",
              f"本次全程位置 RMSE 降低 {result['metrics']['position_rmse_m']['reduction_percent']:.2f}%，闭环终点差{closure}。相反方向误差可在回程抵消，因此不能单看闭环。当前仍是平地、理想传感器仿真，残余足底滑动、力矩动力学补偿和真机标定仍待处理。", "",
              "![轨迹与误差对比](comparison.png)", "", "[PDF](comparison.pdf) · [数值](comparison.json)", "",
              "复现：", "", "```bash", "scripts/run_marsdog_sim.sh --scenario square --side-length 10 --estimator-profile baseline",
              "scripts/run_marsdog_sim.sh --scenario square --side-length 10 --estimator-profile optimized", "```", ""]
    (args.output / "report.md").write_text("\n".join(lines))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

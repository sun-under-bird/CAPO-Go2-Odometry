#!/usr/bin/env python3
"""RL 步态 CAPO 验证：从 /tmp/capo_eval.csv 计算运动段最终误差指标。

取 GT 行程停止增长前的最后时刻为"运动终点"（排除测试结束后趴地/摔倒段），
输出终点误差、相对误差（误差行程比）、RMSE 2D、yaw RMSE。
用法：python3 eval_final.py [csv路径] [静止阈值m/s]
"""
import math
import sys

csv_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/capo_eval.csv'
still_v = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02  # m/s，GT 速度低于此视为静止

rows = []
with open(csv_path) as f:
    f.readline()
    for line in f:
        p = line.strip().split(',')
        if len(p) >= 9:
            rows.append([float(x) for x in p[:9]])
if not rows:
    print('无数据'); sys.exit(1)

dt = 0.005
n = len(rows)
# 找最后一个"运动时刻"：GT 速度连续 > 阈值的末端
# 从尾部往前扫，找到最后一个 1s 窗口平均速度 > 阈值 的窗口右端
win = 200  # 1s
moving_end = None
for i in range(n - 1, win, -1):
    d = math.hypot(rows[i][5] - rows[i - win][5], rows[i][6] - rows[i - win][6])
    if d / (win * dt) > still_v:
        moving_end = i
        break
if moving_end is None:
    moving_end = n - 1
    print('警告：未检测到明显运动段')

# 从运动段尾部再回退 0.5s，避开减速瞬间的姿态扰动
end = max(moving_end - 100, 0)

# 运动起点：首次速度 > 阈值（忽略链路启动静止段）
start = 0
for i in range(win, n):
    d = math.hypot(rows[i][5] - rows[i - win][5], rows[i][6] - rows[i - win][6])
    if d / (win * dt) > still_v:
        start = i - win
        break

sx, sy, syaw = rows[start][5], rows[start][6], rows[start][8]
ex, ey, ez, eyaw = rows[end][5], rows[end][6], rows[end][7], rows[end][8]
cx, cy, cyaw = rows[end][1], rows[end][2], rows[end][4]

# GT 行程（运动段内累计）
dist_gt = 0.0
dist_capo = 0.0
for i in range(start + 1, end + 1):
    dist_gt += math.hypot(rows[i][5] - rows[i-1][5], rows[i][6] - rows[i-1][6])
    dist_capo += math.hypot(rows[i][1] - rows[i-1][1], rows[i][2] - rows[i-1][2])

# 终点误差
dx = ex - cx
dy = ey - cy
dyaw = eyaw - cyaw
while dyaw > math.pi: dyaw -= 2 * math.pi
while dyaw < -math.pi: dyaw += 2 * math.pi
final_err = math.hypot(dx, dy)

# RMSE（运动段）
s2 = z2 = yaw2 = cnt = 0
for i in range(start, end + 1):
    exx = rows[i][5] - rows[i][1]
    eyy = rows[i][6] - rows[i][2]
    eyy_yaw = rows[i][8] - rows[i][4]
    while eyy_yaw > math.pi: eyy_yaw -= 2 * math.pi
    while eyy_yaw < -math.pi: eyy_yaw += 2 * math.pi
    s2 += exx*exx + eyy*eyy
    yaw2 += eyy_yaw*eyy_yaw
    cnt += 1

print(f'运动段: 样本[{start}:{end}] ({(end-start)*dt:.1f}s)')
print(f'GT 行程 {dist_gt:.2f} m | CAPO 行程 {dist_capo:.2f} m | 行程比 {dist_capo/max(dist_gt,1e-6):.3f}')
print(f'终点: GT=({ex:+.2f},{ey:+.2f}) CAPO=({cx:+.2f},{cy:+.2f})')
print(f'终点误差: x={dx:+.3f} y={dy:+.3f} |模|={final_err:.3f} m | 相对误差 {final_err/max(dist_gt,1e-6)*100:.2f}%')
print(f'终点 yaw 误差: {dyaw:+.4f} rad ({math.degrees(dyaw):+.2f}°)')
print(f'RMSE: 2D={math.sqrt(s2/cnt):.3f} m | yaw={math.sqrt(yaw2/cnt):.4f} rad')

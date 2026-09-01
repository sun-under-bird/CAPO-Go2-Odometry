#!/usr/bin/env python3
"""从 NP3O model_10000.pt 提取可部署的 actor（MlpBarlowTwinsActor），导出 onnx。
输入: obs (1,45) 本体感受（无 lin_vel），obs_hist (1,10,45) 历史栈
输出: 12 维 action（× action_scale 0.25 + 默认关节角 = 目标角）
"""
import sys
import numpy as np
import torch

sys.path.insert(0, '/home/yahboom/np3o')
from modules.actor_critic import ActorCriticBarlowTwins

# ---- 按 go2_constraint_him.py config 重建策略网络 ----
policy = ActorCriticBarlowTwins(
    num_prop=48,          # n_proprio = 45+3（前 3 维 lin_vel 会被 act_teacher 丢弃）
    num_scan=187,
    num_critic_obs=762,   # 48+187+47+480
    num_priv_latent=47,
    num_hist=10,
    num_actions=12,
    scan_encoder_dims=None,       # config: None -> scan_encoder=Identity
    actor_hidden_dims=[512, 256, 128],
    critic_hidden_dims=[512, 256, 128],
    activation='elu',
    priv_encoder_dims=[],         # config: []
    num_costs=3,
    teacher_act=True,
    imi_flag=True,
)

ck = torch.load('/home/yahboom/np3o/model_10000.pt', map_location='cpu')
missing, unexpected = policy.load_state_dict(ck['model_state_dict'], strict=True), None
print('iter:', ck['iter'], '| load_state_dict: OK')
policy.eval()

backbone = policy.actor_teacher_backbone  # MlpBarlowTwinsActor，输入不含任何特权信息

# ---- sanity check：权重确实训练过（对比默认初始化的统计特征） ----
w = backbone.actor[0].weight
print(f'actor 首层权重: mean={w.mean():.2e} std={w.std():.3f} '
      f'（std 远大于 0.03 初始量级 → 已训练）')
print(f'obs_normalizer._mean 范数: {backbone.obs_normalizer._mean.norm():.2f} '
      f'（≈0 表示未训练，>1 说明统计过观测分布）')

# ---- 推理冒烟测试：站姿零指令观测 ----
obs45 = torch.zeros(1, 45)
hist = torch.zeros(1, 10, 45)
with torch.no_grad():
    out = backbone(obs45, hist)
print(f'\n冒烟测试: 输入零观测/零指令 → action shape={tuple(out.shape)} '
      f'range=[{out.min():.3f}, {out.max():.3f}]')

# ---- 导出 onnx ----
class DeployWrapper(torch.nn.Module):
    """固定 eval 流程，避免导出控制流问题"""
    def __init__(self, m):
        super().__init__()
        self.m = m
    def forward(self, obs, hist):
        return self.m(obs, hist)

wrapper = DeployWrapper(backbone).eval()
dummy_obs = torch.zeros(1, 45)
dummy_hist = torch.zeros(1, 10, 45)
torch.onnx.export(
    wrapper, (dummy_obs, dummy_hist),
    '/home/yahboom/np3o/go2_policy.onnx',
    input_names=['obs', 'hist'],
    output_names=['action'],
    opset_version=17,
    dynamo=False,
)
print('\nonnx 已导出: /home/yahboom/np3o/go2_policy.onnx')

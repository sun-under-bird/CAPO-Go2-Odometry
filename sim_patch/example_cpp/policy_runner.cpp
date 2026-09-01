// policy_runner.cpp
// RL 步态策略推理节点（RL_GAIT_PLAN.md Phase B2）。
//
// 用途：替换 gait_go2（手工 crawl 步态）作为 CAPO 仿真测试的激励源。
// 策略来自社区仓库 LocomotionWithNP3O（Go2, IsaacGym 训练, N-P3O/HIM 架构），
// 由 /tmp/export_onnx.py 从 model_10000.pt 提取 actor（MlpBarlowTwinsActor，
// 自监督历史编码，无特权观测/无地形扫描输入）导出为 onnx。
//
// 模型接口（导出时已验证与 PyTorch 数值一致，误差 < 1e-5）：
//   输入  obs  (1,45)    本体感受（不含 lin_vel——act_teacher 显式丢弃前 3 维）
//   输入  hist (1,10,45) 前 10 帧观测栈（模型内部自行拼接当前帧并取后 5 帧）
//   输出  action (1,12)  关节位置增量（SDK 电机序 FR,FL,RR,RL）
//
// 观测 45 维布局（legged_robot.py compute_observations + config，SDK 顺序）：
//   [0:3]   base_ang_vel * 0.25          （IMU 陀螺仪，机体系）
//   [3:6]   projected_gravity            （机体系重力方向，无缩放）
//   [6:9]   cmd * [2.0, 2.0, 0.25]       （vx, vy, yaw_rate）
//   [9:21]  (dof_pos - default) * 1.0
//   [21:33] dof_vel * 0.05
//   [33:45] last_action（未经滤波的原始策略输出）
//
// 动作链（_compute_torques + config，SDK 顺序逐元素等价实现）：
//   filtered = 0.8*action + 0.2*last_filtered   （use_filter=True）
//   target = default + filtered * 0.25          （hip 关节再 ×0.5）
//   LowCmd: kp=40, kd=1.0（config stiffness/damping）
//
// 用法：./policy_runner <duration_s> <vx> <vy> <wz> [onnx_path]
//   指令域（训练采样范围）：vx,vy ∈ [-0.5, 0.5] m/s，wz ∈ [-1, 1] rad/s

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>
#include <mutex>
#include <algorithm>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <onnxruntime_cxx_api.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

// ==== 训练配置常量（configs/go2_constraint_him.py，勿随意改动）====
constexpr int    kObsDim = 45;         // n_proprio - 3（丢弃 lin_vel）
constexpr int    kHistLen = 10;        // history_len
constexpr double kPolicyDt = 0.02;     // 50 Hz（sim dt 0.005 × decimation 4）
constexpr double kActionScale = 0.25;  // action_scale
constexpr double kHipScaleReduction = 0.5;   // hip_scale_reduction
constexpr double kFilterAlpha = 0.8;   // _low_pass_action_filter: 0.8*new+0.2*last
constexpr double kP_Gain = 40.0;       // stiffness
constexpr double kD_Gain = 1.0;        // damping
constexpr double kScaleAngVel = 0.25;  // obs_scales.ang_vel
constexpr double kScaleDofVel = 0.05;  // obs_scales.dof_vel
constexpr double kScaleCmdLin = 2.0;   // commands_scale[0:2] = lin_vel scale
constexpr double kScaleCmdAng = 0.25;  // commands_scale[2] = ang_vel scale

// 默认关节角（SDK 电机序 FR, FL, RR, RL；hip, thigh, calf 交错）。
// 训练 config default_joint_angles：FL/RL_hip 0.1, FR/RR_hip -0.1,
// FL/FR_thigh 0.8, RL/RR_thigh 1.0, calf 全 -1.5。
constexpr double kDefaultJointPos[12] = {
    /*FR*/ -0.1, 0.8, -1.5,
    /*FL*/  0.1, 0.8, -1.5,
    /*RR*/ -0.1, 1.0, -1.5,
    /*RL*/  0.1, 1.0, -1.5,
};

// 趴姿（unitree_mujoco Go2 初始关节角，与 gait_go2 的起立起点一致）
constexpr double kStandDown[12] = {
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};

// 起立过渡：趴姿 → RL 默认站姿（3 s tanh 平滑，防止目标角跳变冲击）
constexpr double kRiseTime = 3.0;
// 指令淡入：起立完成后速度指令从 0 线性升到目标值。
// 3 s（原 1 s 会激励步态突变，实测全速瞬间数秒后失稳弹飞；缓慢过渡可让
// 策略在步态频率渐变中保持稳定）
constexpr double kCmdRampTime = 3.0;

// ==== 键盘遥控（--teleop）：终端 raw 模式全局状态 ====
static struct termios g_term_old;
static bool g_term_raw = false;

static void SetupTerminalRaw()
{
    if (g_term_raw) return;
    tcgetattr(STDIN_FILENO, &g_term_old);
    termios newt = g_term_old;
    newt.c_lflag &= ~(ICANON | ECHO);   // 关行缓冲与回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);  // 非阻塞读
    g_term_raw = true;
}

static void RestoreTerminal()
{
    if (!g_term_raw) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &g_term_old);
    g_term_raw = false;
}

class PolicyRunner
{
public:
    // 指令段（脚本模式）：duration 内执行 (vx, vy) 速度指令。
    // yaw_mode: 0 = wz 增量转向（heading_target 按 wz 递增），
    //           1 = 绝对航向（heading_target = yaw_target，段内持续闭环纠偏）
    struct CmdSeg { double dur, vx, vy, wz, yaw_target; int yaw_mode; };

    PolicyRunner(double duration, double vx, double vy, double wz, std::string onnx_path)
        : duration_(duration), vx_(vx), vy_(vy), wz_(wz),
          onnx_path_(std::move(onnx_path)) {}

    // 脚本模式：segments 依次执行（Test 4 矩形等闭环轨迹用）
    PolicyRunner(std::vector<CmdSeg> segs, std::string onnx_path)
        : duration_(0), vx_(0), vy_(0), wz_(0), segs_(std::move(segs)),
          onnx_path_(std::move(onnx_path))
    {
        for (auto &s : segs_) duration_ += s.dur;
    }

    // 键盘遥控模式：指令由终端按键实时给出（--teleop）
    PolicyRunner(std::string onnx_path, bool teleop)
        : duration_(600.0), vx_(0), vy_(0), wz_(0), teleop_(teleop),
          onnx_path_(std::move(onnx_path)) {}

    void Init()
    {
        // onnxruntime 会话（CPU；MLP ~50 万参数，单次推理 << 1 个 20ms 周期）
        env_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "policy_runner"));
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);  // 50Hz 轻负载，单线程足够且不抢 sim 的核
        session_.reset(new Ort::Session(*env_, onnx_path_.c_str(), opts));

        InitLowCmd();
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher->InitChannel();
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber->InitChannel(
            std::bind(&PolicyRunner::LowStateMessageHandler, this, std::placeholders::_1), 1);

        // 预热推理：onnxruntime 首次 Run 含图优化/内存池初始化，耗时可能超过
        // 20ms 控制周期。在写线程启动前先跑一次，避免第一拍超时导致线程重入。
        {
            float warmup[12];
            InferAction(warmup);
        }

        if (teleop_) {
            // 键盘遥控：终端 raw 模式 + 键位说明
            SetupTerminalRaw();
            std::cout << "\n=== 键盘遥控（RL 步态）===" << std::endl
                      << "  W/S: 前进/后退  ±0.05 m/s（|v|<0.2 为训练死区，策略会站立）" << std::endl
                      << "  Q/E: 左移/右移  ±0.05 m/s" << std::endl
                      << "  A/D: 左转/右转  ±0.10 rad/s" << std::endl
                      << "  空格: 急停站立    X: 退出" << std::endl
                      << "  起立 3 s 后策略接管，从静止到迈步有 5~10 s 启动期" << std::endl;
        }

        // 50 Hz 控制频率 = 训练策略频率（20 ms 周期）
        lowCmdWriteThreadPtr = CreateRecurrentThreadEx(
            "policy_write", UT_CPU_ID_NONE, 20000, &PolicyRunner::LowCmdWrite, this);
        std::cout << "策略推理节点启动: dur=" << duration_ << "s cmd=("
                  << vx_ << "," << vy_ << "," << wz_ << ")" << std::endl;
    }

private:
    double duration_;
    double vx_, vy_, wz_;
    bool   teleop_ = false;           // 键盘遥控模式
    double tvx_ = 0, tvy_ = 0, twz_ = 0;  // 遥控当前指令
    bool   exit_req_ = false;         // 遥控 X 键退出
    std::vector<CmdSeg> segs_;  // 脚本模式指令序列（空 = 单指令模式）
    int    cur_seg_ = -1;       // 当前段索引（段切换时重置 heading 与淡入）
    double seg_t0_ = 0.0;       // 当前段起始时刻（策略接管后计时）
    std::string onnx_path_;
    double running_time_ = 0.0;
    bool   policy_started_ = false;    // 起立完成、策略接管
    bool   hist_initialized_ = false;  // 首帧观测填满历史栈（对齐训练 reset 语义）
    bool   heading_init_ = false;      // heading 目标航向在策略接管时初始化为当前 yaw
    double heading_target_ = 0.0;      // heading 模式目标航向（rad）

    std::array<float, kObsDim> obs_{};
    std::array<float, kObsDim * kHistLen> hist_{};  // 10×45 行主序
    std::array<float, 12> last_raw_action_{};       // obs 的 last_action 分量（未滤波）
    std::array<float, 12> last_filtered_action_{};  // 滤波器状态

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::MemoryInfo> mem_;
    std::mutex write_mutex_;  // LowCmdWrite 防重入（onnx 推理可能超周期）
    int step_count_ = 0;

    void InitLowCmd()
    {
        low_cmd.head()[0] = 0xFE;
        low_cmd.head()[1] = 0xEF;
        low_cmd.level_flag() = 0xFF;
        low_cmd.gpio() = 0;
        for (int i = 0; i < 20; i++) {
            low_cmd.motor_cmd()[i].mode() = 0x01;
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kd() = 0;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    void LowStateMessageHandler(const void *message)
    {
        low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    }

    // 机体系投影重力：v_body = q^{-1} ⊗ [0,0,-1] ⊗ q
    //   = (2w²-1)·g + 2(u·g)·u - 2w·(u×g)，g=[0,0,-1]，q=(w,x,y,z)
    // 已用 200 组随机四元数与四元数乘法数值验证一致。
    static void ProjectedGravity(const std::array<float, 4> &q, double out[3])
    {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        const double gx = 0.0, gy = 0.0, gz = -1.0;
        const double ux = x, uy = y, uz = z;
        const double u_dot_g = ux * gx + uy * gy + uz * gz;
        // u × g
        const double cx = uy * gz - uz * gy;
        const double cy = uz * gx - ux * gz;
        const double cz = ux * gy - uy * gx;
        const double s = 2.0 * w * w - 1.0;
        out[0] = s * gx + 2.0 * u_dot_g * ux - 2.0 * w * cx;
        out[1] = s * gy + 2.0 * u_dot_g * uy - 2.0 * w * cy;
        out[2] = s * gz + 2.0 * u_dot_g * uz - 2.0 * w * cz;
    }

    // 从四元数提取偏航角（ZYX 欧拉），用于 heading 指令闭环。
    // 训练侧（legged_robot.py _post_physics_step_callback）heading 定义为
    // atan2(forward_y, forward_x)，即机体 +x 轴在世界平面上的方位角。
    static double YawFromQuat(const std::array<float, 4> &q)
    {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        // 机体系 +x 轴在世界系 xy 平面上的方向：R[0:2, 0]
        const double fx = 1.0 - 2.0 * (y * y + z * z);
        const double fy = 2.0 * (x * y + w * z);
        return std::atan2(fy, fx);
    }

    // 角度环绕到 [-π, π]（训练侧 wrap_to_pi 的等价实现）
    static double WrapToPi(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // 键盘遥控：非阻塞轮询按键并更新指令。
    // W/S: vx ±0.05  Q/E: vy ±0.05  A/D: wz ±0.1  空格: 急停  X: 退出
    void PollKeyboard()
    {
        char ch;
        while (read(STDIN_FILENO, &ch, 1) == 1) {
            switch (ch) {
                case 'w': case 'W': tvx_ = std::min(tvx_ + 0.05, 0.5); break;
                case 's': case 'S': tvx_ = std::max(tvx_ - 0.05, -0.5); break;
                case 'q': case 'Q': tvy_ = std::min(tvy_ + 0.05, 0.5); break;
                case 'e': case 'E': tvy_ = std::max(tvy_ - 0.05, -0.5); break;
                case 'a': case 'A': twz_ = std::min(twz_ + 0.10, 1.0); break;
                case 'd': case 'D': twz_ = std::max(twz_ - 0.10, -1.0); break;
                case ' ': tvx_ = tvy_ = twz_ = 0.0; break;
                case 'x': case 'X': exit_req_ = true; break;
                default: break;  // 忽略 ESC 方向键序列等无关字节
            }
        }
    }

    // 组装 45 维观测（SDK 顺序）。必须在收到 lowstate 后调用。
    // 注意：dof_pos/dof_vel/last_action 在训练观测里均已 reindex 为 SDK 顺序
    //（legged_robot.py reindex [3,4,5,0,1,2,9,10,11,6,7,8]，env(FL,FR,RL,RR)→SDK(FR,FL,RR,RL)），
    // 而 unitree lowstate 的 motor 序本身就是 SDK 顺序，直接取用即可。
    void BuildObs(double cmd_scale_vx, double cmd_scale_vy, double cmd_scale_wz)
    {
        const auto &imu = low_state.imu_state();
        const auto &gyro = imu.gyroscope();          // 机体系角速度 rad/s
        const auto &quat = imu.quaternion();          // (w,x,y,z)

        // [0:3] ang_vel * 0.25
        obs_[0] = (float)(gyro[0] * kScaleAngVel);
        obs_[1] = (float)(gyro[1] * kScaleAngVel);
        obs_[2] = (float)(gyro[2] * kScaleAngVel);

        // [3:6] projected gravity
        double g[3];
        ProjectedGravity(quat, g);
        obs_[3] = (float)g[0];
        obs_[4] = (float)g[1];
        obs_[5] = (float)g[2];

        // [6:9] cmd * [2, 2, 0.25]
        obs_[6] = (float)(cmd_scale_vx * kScaleCmdLin);
        obs_[7] = (float)(cmd_scale_vy * kScaleCmdLin);
        obs_[8] = (float)(cmd_scale_wz * kScaleCmdAng);

        // [9:33] dof_pos - default（×1.0）、dof_vel × 0.05
        for (int i = 0; i < 12; i++) {
            obs_[9 + i] = (float)(low_state.motor_state()[i].q() - kDefaultJointPos[i]);
            obs_[21 + i] = (float)(low_state.motor_state()[i].dq() * kScaleDofVel);
        }

        // [33:45] last raw action
        for (int i = 0; i < 12; i++)
            obs_[33 + i] = last_raw_action_[i];
    }

    // 历史栈更新：hist ← [hist[1:], obs]（legged_robot.py post_physics_step）
    void PushHist()
    {
        if (!hist_initialized_) {
            // 训练 reset 语义：episode 开始用首帧观测填满全部历史
            for (int k = 0; k < kHistLen; k++)
                std::memcpy(&hist_[k * kObsDim], obs_.data(), sizeof(obs_));
            hist_initialized_ = true;
            return;
        }
        std::memmove(hist_.data(), hist_.data() + kObsDim,
                     sizeof(float) * kObsDim * (kHistLen - 1));
        std::memcpy(&hist_[(kHistLen - 1) * kObsDim], obs_.data(), sizeof(obs_));
    }

    // onnx 推理：action = f(obs, hist)
    void InferAction(float action[12])
    {
        if (!mem_)
            mem_.reset(new Ort::MemoryInfo(Ort::MemoryInfo::CreateCpu(
                OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeDefault)));

        std::array<int64_t, 2> obs_shape{1, kObsDim};
        std::array<int64_t, 3> hist_shape{1, kHistLen, kObsDim};
        std::array<Ort::Value, 2> inputs{
            Ort::Value::CreateTensor<float>(*mem_, obs_.data(), kObsDim,
                                            obs_shape.data(), obs_shape.size()),
            Ort::Value::CreateTensor<float>(*mem_, hist_.data(),
                                            kObsDim * kHistLen,
                                            hist_shape.data(), hist_shape.size())};
        const char *in_names[] = {"obs", "hist"};
        const char *out_names[] = {"action"};
        auto outputs = session_->Run(Ort::RunOptions{nullptr}, in_names,
                                     inputs.data(), inputs.size(),
                                     out_names, 1);
        const float *out = outputs[0].GetTensorData<float>();
        for (int i = 0; i < 12; i++)
            action[i] = out[i];
    }

    void LowCmdWrite()
    {
        // 防重入：本函数含 onnx 推理，若某拍耗时超过 20ms 周期，调度器重入会
        // 导致同一 DDS writer 并发写与成员竞态（实测触发过 CycloneDDS 内部断言
        // 与堆损坏崩溃）。上一拍未完成时直接丢弃本拍。
        if (!write_mutex_.try_lock()) return;
        std::lock_guard<std::mutex> lock(write_mutex_, std::adopt_lock);

        running_time_ += kPolicyDt;

        if (!policy_started_) {
            // ---- 起立过渡：趴姿 → RL 默认站姿（策略接管前的姿态准备）----
            double phase = std::tanh(running_time_ / 1.2);
            for (int i = 0; i < 12; i++) {
                low_cmd.motor_cmd()[i].q() =
                    phase * kDefaultJointPos[i] + (1.0 - phase) * kStandDown[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = phase * kP_Gain + (1.0 - phase) * 20.0;
                low_cmd.motor_cmd()[i].kd() = kD_Gain;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
            if (running_time_ >= kRiseTime) {
                policy_started_ = true;
                std::cout << "起立完成，策略接管" << std::endl;
            }
            return PublishLowCmd();
        }

        // ---- 策略阶段 ----
        const double t = running_time_ - kRiseTime;
        if (t > duration_) {
            // 测试结束：冻结当前策略指令，由外部结束进程
            return PublishLowCmd();
        }

        // ---- 当前指令：teleop / 脚本 / 单指令 ----
        double cmd_vx, cmd_vy, cmd_wz, t_in;
        if (teleop_) {
            // 键盘遥控：轮询按键，X 退出
            PollKeyboard();
            if (exit_req_) {
                std::cout << "\n收到退出键，结束" << std::endl;
                RestoreTerminal();
                std::_Exit(0);
            }
            // 起立接管后 1.5 s 内指令渐升（防姿态突变），之后直接跟随键盘
            const double spin_up = std::min(t / 1.5, 1.0);
            cmd_vx = tvx_ * spin_up;
            cmd_vy = tvy_ * spin_up;
            cmd_wz = twz_;
            t_in = 1e9;  // 跳过段淡入（spin_up 已处理）
        } else if (!segs_.empty()) {
            // 脚本模式：按策略接管后的累计时间选段，末段保持
            double acc = 0.0;
            int idx = (int)segs_.size() - 1;
            for (size_t k = 0; k < segs_.size(); k++) {
                acc += segs_[k].dur;
                if (t < acc) { idx = (int)k; break; }
            }
            if (idx != cur_seg_) {
                cur_seg_ = idx;
                seg_t0_ = t;
                // 绝对航向段自带 heading_target，勿重置；wz 增量段从当前 yaw 起步
                if (segs_[idx].yaw_mode != 1) heading_init_ = false;
            }
            const CmdSeg &s = segs_[cur_seg_];
            cmd_vx = s.vx; cmd_vy = s.vy; cmd_wz = s.wz;
            t_in = t - seg_t0_;
        } else {
            cmd_vx = vx_; cmd_vy = vy_; cmd_wz = wz_;
            t_in = t;
        }

        // 线速度指令淡入（起立/段切换后大指令会激励姿态突变）
        const double ramp = std::min(t_in / kCmdRampTime, 1.0);
        const double cvx = cmd_vx * ramp, cvy = cmd_vy * ramp;

        // ---- yaw 指令：复刻训练侧 heading 模式的指令生成 ----
        // 训练（legged_robot.py _post_physics_step_callback）从不直接下发恒定
        // yaw 速率，而是每步重算 wz_cmd = clip(0.5·wrap(heading_target - yaw), ±1)，
        // 指令随转向完成自然衰减。恒定 wz 不在训练分布内（实测策略转 ~0.56 rad
        // 后即停止）。部署侧维护 heading_target，再由航向误差闭环得到 wz_cmd，
        // 与训练分布完全一致。
        // 脚本模式两种段：wz 增量段（heading_target 按 wz 递增）与
        // 绝对航向段（heading_target = 指定值，段内持续闭环纠偏——用于矩形
        // 闭环轨迹的精确 90° 转角）。
        if (!heading_init_) {
            heading_target_ = YawFromQuat(low_state.imu_state().quaternion());
            heading_init_ = true;
        }
        bool abs_yaw = false;
        if (!segs_.empty() && cur_seg_ >= 0 && segs_[cur_seg_].yaw_mode == 1) {
            heading_target_ = segs_[cur_seg_].yaw_target;
            abs_yaw = true;
        }
        if (!abs_yaw)
            heading_target_ += cmd_wz * kPolicyDt;  // wz 增量段：目标航向递增
        const double yaw_now = YawFromQuat(low_state.imu_state().quaternion());
        const double cwz = std::max(-1.0, std::min(1.0, 0.5 * WrapToPi(heading_target_ - yaw_now)));

        BuildObs(cvx, cvy, cwz);

        float action[12];
        InferAction(action);

        // 滤波 + 缩放 + 默认角 → 目标角（SDK 顺序逐元素，与训练 env 顺序实现等价：
        // 滤波/缩放/加 default 均为逐关节线性运算，重排不改变结果）
        for (int i = 0; i < 12; i++) {
            const double filtered =
                kFilterAlpha * action[i] + (1.0 - kFilterAlpha) * last_filtered_action_[i];
            last_filtered_action_[i] = filtered;

            double scaled = filtered * kActionScale;
            if (i % 3 == 0) scaled *= kHipScaleReduction;  // hip 关节幅值减半

            low_cmd.motor_cmd()[i].q() = kDefaultJointPos[i] + scaled;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = kP_Gain;
            low_cmd.motor_cmd()[i].kd() = kD_Gain;
            low_cmd.motor_cmd()[i].tau() = 0;
        }

        // 更新观测历史与 last_action（注意：obs 的 last_action 用未滤波原始输出，
        // 训练时 action_history_buf 在 reindex/滤波之前入栈）
        PushHist();
        for (int i = 0; i < 12; i++)
            last_raw_action_[i] = action[i];

        if (teleop_) {
            // 遥控模式：单行刷新显示当前指令（0.5 s 一次），不打 dbg
            if (++step_count_ % 25 == 0) {
                const bool dead = (std::hypot(tvx_, tvy_) > 0.0 &&
                                   std::hypot(tvx_, tvy_) < 0.2);
                std::printf("\r[vx=%+.2f vy=%+.2f wz=%+.2f]%s W/S前后 Q/E侧移 "
                            "A/D转向 空格:停 X:退   ",
                            tvx_, tvy_, twz_, dead ? " <0.2死区:站立" : "");
                std::fflush(stdout);
            }
        } else if (++step_count_ % 250 == 0) {  // 每 5 s 打印一次心跳
            std::cout << "t=" << t << "s 步数=" << step_count_ << " ok" << std::endl;
            // 调试：观测前 9 维（ang_vel/gravity/cmd）+ 动作统计，用于 sim2sim 排障
            std::cout << "  dbg obs[0:9]=";
            for (int i = 0; i < 9; i++) std::cout << obs_[i] << " ";
            float dq_min = obs_[21], dq_max = obs_[21];
            for (int i = 22; i < 33; i++) {
                dq_min = std::min(dq_min, obs_[i]);
                dq_max = std::max(dq_max, obs_[i]);
            }
            // obs[9:21] = dof_pos - default（站立时应≈0；SDK 序 FR,FL,RR,RL）
            std::cout << "\n       dofpos_dev=";
            for (int i = 9; i < 21; i++) std::cout << obs_[i] << " ";
            std::cout << "| act∈[" << *std::min_element(action, action + 12) << ","
                      << *std::max_element(action, action + 12) << "]"
                      << " dq_obs∈[" << dq_min << "," << dq_max << "]" << std::endl;
        }

        PublishLowCmd();
    }

    // 计算 CRC 并发布 lowcmd
    void PublishLowCmd()
    {
        low_cmd.crc() = Crc32Core((uint32_t *)&low_cmd,
                                  (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        lowcmd_publisher->Write(low_cmd);
    }

    static uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
    {
        unsigned int xbit = 0;
        unsigned int data = 0;
        unsigned int CRC32 = 0xFFFFFFFF;
        const unsigned int dwPolynomial = 0x04c11db7;
        for (unsigned int i = 0; i < len; i++) {
            xbit = 1 << 31;
            data = ptr[i];
            for (unsigned int bits = 0; bits < 32; bits++) {
                if (CRC32 & 0x80000000) { CRC32 <<= 1; CRC32 ^= dwPolynomial; }
                else { CRC32 <<= 1; }
                if (data & xbit) CRC32 ^= dwPolynomial;
                xbit >>= 1;
            }
        }
        return CRC32;
    }
};

// 统一退出：起立 3 s + 指令总时长 + 余量 5 s 后结束进程。
// 与 gait_go2 相同：不能走正常析构（写线程仍在访问成员），用 _Exit 跳过。
static int RunAndExit(PolicyRunner *runner, double total_cmd_duration)
{
    Sleep((int)((total_cmd_duration + kRiseTime + 5.0) * 1000000));
    std::cout << "策略推理节点结束" << std::endl;
    std::_Exit(0);
    return 0;
}

int main(int argc, char **argv)
{
    // 用法 1（单指令）: ./policy_runner <duration_s> <vx> <vy> <wz> [onnx_path]
    // 用法 2（脚本模式）: ./policy_runner --script <seq.txt> [onnx_path]
    //   seq.txt 每行: <duration_s> <vx> <vy> <wz>，依次执行（Test 4 矩形等）
    // 用法 3（键盘遥控）: ./policy_runner --teleop [onnx_path]
    if (argc >= 2 && std::string(argv[1]) == "--teleop") {
        const std::string onnx = (argc > 2) ? argv[2] : "/home/yahboom/np3o/go2_policy.onnx";
        // DDS：domain 1 + lo 网卡，与 unitree_mujoco simulate/config.yaml 一致
        ChannelFactory::Instance()->Init(1, "lo");
        PolicyRunner runner(onnx, true);
        runner.Init();
        // 上限 10 分钟（X 键或超时后 _Exit 前恢复终端）
        Sleep((int)(600.0 * 1000000));
        RestoreTerminal();
        std::cout << "\n策略推理节点结束（超时）" << std::endl;
        std::_Exit(0);
    }
    if (argc >= 3 && std::string(argv[1]) == "--script") {
        std::vector<PolicyRunner::CmdSeg> segs;
        std::ifstream fin(argv[2]);
        if (!fin) {
            std::cout << "无法打开脚本文件: " << argv[2] << std::endl;
            return 1;
        }
        std::string line;
        int ln = 0;
        while (std::getline(fin, line)) {
            ln++;
            // 跳过空行与 # 注释
            size_t h = line.find('#');
            if (h != std::string::npos) line = line.substr(0, h);
            std::istringstream iss(line);
            PolicyRunner::CmdSeg s{};
            s.yaw_mode = 0; s.yaw_target = 0;
            double yaw_deg = 0.0;
            if (iss >> s.dur >> s.vx >> s.vy >> s.wz) {
                // 可选第 5 列：绝对航向（度）。出现则该段用绝对航向闭环
                if (iss >> yaw_deg) {
                    s.yaw_mode = 1;
                    s.yaw_target = yaw_deg * M_PI / 180.0;
                }
                segs.push_back(s);
            } else if (!line.find_first_not_of(" \t\r\n") == std::string::npos) {
                std::cout << "警告: 脚本第 " << ln << " 行解析失败，已跳过" << std::endl;
            }
        }
        if (segs.empty()) {
            std::cout << "脚本无有效指令段: " << argv[2] << std::endl;
            return 1;
        }
        const std::string onnx = (argc > 3) ? argv[3] : "/home/yahboom/np3o/go2_policy.onnx";
        double total = 0.0;
        for (auto &s : segs) total += s.dur;
        // DDS：domain 1 + lo 网卡，与 unitree_mujoco simulate/config.yaml 一致
        ChannelFactory::Instance()->Init(1, "lo");
        PolicyRunner runner(segs, onnx);
        runner.Init();
        return RunAndExit(&runner, total);
    }

    if (argc < 5) {
        std::cout << "用法: " << argv[0]
                  << " <duration_s> <vx> <vy> <wz> [onnx_path]" << std::endl
                  << "      " << argv[0] << " --script <seq.txt> [onnx_path]" << std::endl
                  << "  指令域: vx,vy ∈ [-0.5,0.5] m/s（|v|<0.2 为训练死区，勿用）," << std::endl
                  << "          wz ∈ [-1,1] rad/s" << std::endl
                  << "  示例: 站立 ./policy_runner 30 0 0 0" << std::endl
                  << "        直行 ./policy_runner 60 0.5 0 0" << std::endl;
        return 1;
    }
    const double duration = std::atof(argv[1]);
    const double vx = std::atof(argv[2]);
    const double vy = std::atof(argv[3]);
    const double wz = std::atof(argv[4]);
    const std::string onnx = (argc > 5) ? argv[5] : "/home/yahboom/np3o/go2_policy.onnx";

    // DDS：domain 1 + lo 网卡，与 unitree_mujoco simulate/config.yaml 一致
    ChannelFactory::Instance()->Init(1, "lo");

    PolicyRunner runner(duration, vx, vy, wz, onnx);
    runner.Init();
    return RunAndExit(&runner, duration);
}

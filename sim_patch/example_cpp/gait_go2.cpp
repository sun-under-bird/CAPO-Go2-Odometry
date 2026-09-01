// gait_go2.cpp
// CAPO 仿真测试用简易步态控制器（阶段十八新增）。
//
// 基于 unitree_mujoco 官方示例 stand_go2.cpp 的 PD 位置控制框架扩展：
//   - stand 模式：保持站立姿态（Test 1 静止漂移测试）
//   - walk 模式：对角小跑（trot）步态直线前进（Test 2 直线 / Test 5 坡道）
//   - turn 模式：左右腿差速摆幅实现原地慢转（Test 3 旋转）
//
// 步态原理（纯关节空间轨迹，不含平衡控制器）：
//   每条腿一个周期 T：摆动相（40%，抬腿向前收）+ 支撑相（60%，伸腿向后蹬）。
//   对角腿（FR+RL / FL+RR）相位相差半个周期。非对称的摆动/支撑时间比
//   产生净前进位移。turn 模式下左右腿摆幅偏置相反，产生偏航力矩。
//
// 用法：./gait_go2 <mode> <duration_s> [speed]
//   mode: stand | walk | turn，speed 默认 1.0（摆幅缩放系数）

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <array>

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

// 站立基础关节角（hip, thigh, calf 顺序按 Unitree 电机序：FR, FL, RR, RL）
// 取自官方 stand_go2 示例的 stand_up_joint_pos，保证标准站姿身高（约 0.3 m）
constexpr double kStandHip = 0.00571868;
constexpr double kStandThigh = 0.608813;
constexpr double kStandCalf = -1.21763;

// ==== 步态形式：crawl 静步态（三足支撑）====
// 最初实现为对角小跑 trot（对角双腿同时摆动，任意时刻仅 2 腿支撑），但纯关节位置
// 控制没有俯仰/横滚平衡回路，实测无论怎么调参（周期 0.5~0.65s、摆幅 0.25~0.4rad、
// kp 50~80）都会在数秒内摔倒或严重打滑。改为 crawl 静步态：四腿按 FR→RL→FL→RR
// 依次摆动，摆动占比 0.25，任意时刻至多 1 腿离地，重心始终处于支撑三角形内，
// 静态稳定不依赖平衡控制。速度慢但可靠。
constexpr double kGaitPeriod = 1.2;    // 一个步态周期（crawl 放慢保证静稳定）
constexpr double kSwingRatio = 0.25;   // 摆动相占比（=0.25 时任意时刻至多 1 腿离地）
constexpr double kThighSwing = 0.25;   // 摆动相 thigh 前摆基础幅度（rad），按 speed 放大
constexpr double kCalfLift = 0.35;     // 摆动相 calf 收缩抬脚幅度（rad），对 speed 限幅（过大易摔）

// 前进方向系数：实测原摆动/支撑时序下机体实际朝 -x（机头反方向）移动，
// 即支撑相"后蹬"实际把机体推向机头后方。取 -1 反转 thigh 摆动相位，
// 使机体朝 +x（机头方向）前进（Test 5 坡道在 +x 方向）。
constexpr double kWalkDir = -1.0;

class GaitController
{
public:
    explicit GaitController(std::string mode, double duration, double speed, double heading)
    : mode_(std::move(mode)), duration_(duration), speed_(speed), heading_(heading) {}

    void Init()
    {
        InitLowCmd();
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher->InitChannel();
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber->InitChannel(
            std::bind(&GaitController::LowStateMessageHandler, this, std::placeholders::_1), 1);
        // 500 Hz 控制频率（与真机 lowcmd 一致）
        lowCmdWriteThreadPtr = CreateRecurrentThreadEx(
            "gait_write", UT_CPU_ID_NONE, 2000, &GaitController::LowCmdWrite, this);
    }

private:
    std::string mode_;
    double duration_;
    double speed_;
    double running_time_ = 0.0;
    double dt_ = 0.002;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;

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

    // 计算第 leg 条腿（0=FR 1=FL 2=RR 3=RL，Unitree 电机序）在时刻 t 的关节目标角。
    // crawl 静步态：四腿按 FR→RL→FL→RR 依次摆动。
    // ramp ∈ [0,1] 为步态幅值淡入系数：步态刚启动时若目标角直接从站立位跳到
    // 摆动相起点（-A），kp=80 的 PD 会瞬时猛拉该腿导致机体失衡摔倒（实测复现），
    // 故前 2 秒内摆幅从 0 线性升到满幅，保证目标轨迹连续。
    static void LegTarget(int leg, double t, double speed_scale, double ramp,
                          double &hip, double &thigh, double &calf)
    {
        // crawl 相位：FR=0, RL=0.25, FL=0.5, RR=0.75（绕体顺序依次摆动）
        static const double kCrawlPhase[4] = {0.0, 0.5, 0.75, 0.25}; // FR, FL, RR, RL
        double phase_offset = kCrawlPhase[leg];
        double phase = std::fmod(t / kGaitPeriod + phase_offset, 1.0);

        // 摆动相：thigh 从后（-A）平滑摆到前（+A），calf 中段收缩抬脚
        // 支撑相：thigh 从前（+A）匀速回到后（-A），足端蹬地推进机身
        // thigh 摆幅按 speed 放大（决定步长/推进力）；calf 抬脚高度限幅在 1.0 倍以内，
        // 抬脚过高会显著增大落脚冲击（实测 speed=1.4 时直接摔倒在第 10 秒左右）。
        double thigh_amp = kWalkDir * kThighSwing * speed_scale * ramp;
        double calf_amp = kCalfLift * std::min(speed_scale, 1.0) * ramp;
        if (phase < kSwingRatio) {
            double s = phase / kSwingRatio;              // 0..1
            double smooth = 0.5 * (1.0 - std::cos(M_PI * s));  // 余弦平滑
            thigh = kStandThigh - thigh_amp + 2.0 * thigh_amp * smooth;
            calf = kStandCalf - calf_amp * std::sin(M_PI * s); // 中段抬脚最高
        } else {
            double s = (phase - kSwingRatio) / (1.0 - kSwingRatio);  // 0..1
            thigh = kStandThigh + thigh_amp - 2.0 * thigh_amp * s;   // 匀速后蹬
            calf = kStandCalf;
        }
        hip = kStandHip;
    }

    // 步态前的起立过渡时间：walk/turn 先从趴姿平滑站起（3 秒），避免目标角突变产生冲击
    static constexpr double kRiseTime = 3.0;

    // 航向闭环增益（阶段十八 Test 2 新增）：纯开环关节步态的航向会随机漂移（实测绕圈），
    // 用 IMU yaw 相对初始航向的误差做 PD 修正，经左右腿 thigh 差速偏置产生偏航力矩。
    static constexpr double kYawKp = 1.2;
    static constexpr double kYawKd = 0.08;

    bool yaw_initialized_ = false;   // 起立完成后采样初始航向
    double yaw_ref_ = 0.0;           // 目标航向（起立完成时刻的 yaw）
    double heading_ = 0.0;           // 命令行指定的目标航向（相对初始朝向，rad）

    void LowCmdWrite()
    {
        running_time_ += dt_;
        if (running_time_ > duration_ + kRiseTime) {
            // 测试结束：站回站立姿态后由外部结束进程
            for (int i = 0; i < 12; i++) {
                low_cmd.motor_cmd()[i].q() = (i % 3 == 0) ? kStandHip
                                             : (i % 3 == 1) ? kStandThigh : kStandCalf;
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = 50;
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
        } else if (mode_ == "walk" || mode_ == "turn") {
            if (running_time_ < kRiseTime) {
                // 起立过渡：与 stand 模式相同的 tanh 平滑过程
                double phase = std::tanh(running_time_ / 1.2);
                static const double kStandDown[12] = {
                    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
                for (int i = 0; i < 12; i++) {
                    const double stand_up = (i % 3 == 0) ? kStandHip
                                             : (i % 3 == 1) ? kStandThigh : kStandCalf;
                    low_cmd.motor_cmd()[i].q() =
                        phase * stand_up + (1.0 - phase) * kStandDown[i];
                    low_cmd.motor_cmd()[i].dq() = 0;
                    low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1.0 - phase) * 20.0;
                    low_cmd.motor_cmd()[i].kd() = 3.5;
                    low_cmd.motor_cmd()[i].tau() = 0;
                }
                return PublishLowCmd();
            }
            const double gait_t = running_time_ - kRiseTime; // 步态相位时间
            // 航向闭环：误差 = 当前 yaw - 初始 yaw（正 = 左偏需右转）。
            // 左腿 thigh 前摆更多会使左侧步长更长 → 机体右转（yaw 减小），故左 + 右 -。
            const double yaw = YawFromQuat(low_state.imu_state().quaternion());
            if (!yaw_initialized_) {
                yaw_ref_ = yaw + heading_; // 目标航向 = 初始航向 + 指定偏航
                yaw_initialized_ = true;
                std::cout << "航向闭环基准 yaw0=" << yaw_ref_
                          << "（含指定偏航 " << heading_ << " rad）" << std::endl;
            }
            double yaw_err = yaw - yaw_ref_;
            if (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            if (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
            const double turn_bias =
                kYawKp * yaw_err + kYawKd * low_state.imu_state().gyroscope()[2];
            for (int leg = 0; leg < 4; ++leg) {
                double hip, thigh, calf;
                // 幅值淡入：前 2 秒摆幅从 0 升到满幅（见 LegTarget 注释）
                const double ramp = std::min(gait_t / 2.0, 1.0);
                LegTarget(leg, gait_t, speed_, ramp, hip, thigh, calf);
                const bool is_left = (leg == 1 || leg == 3);
                if (mode_ == "turn") {
                    // 差速转弯：固定偏置产生持续偏航。偏置同样需要 ramp 淡入
                    // （无淡入时起立结束瞬间目标角跳变，实测立即失衡摔倒），
                    // 且幅度限制在 0.06 rad（0.12 时侧向扰动过大也会摔倒）。
                    thigh += (is_left ? 1.0 : -1.0) * 0.06 * speed_ * ramp;
                } else {
                    // walk：航向闭环修正
                    thigh += (is_left ? 1.0 : -1.0) * turn_bias;
                }
                const int base = leg * 3;
                low_cmd.motor_cmd()[base + 0].q() = hip;
                low_cmd.motor_cmd()[base + 1].q() = thigh;
                low_cmd.motor_cmd()[base + 2].q() = calf;
                low_cmd.motor_cmd()[base + 0].dq() = 0;
                low_cmd.motor_cmd()[base + 1].dq() = 0;
                low_cmd.motor_cmd()[base + 2].dq() = 0;
                low_cmd.motor_cmd()[base + 0].kp() = 50;
                low_cmd.motor_cmd()[base + 1].kp() = 50;
                low_cmd.motor_cmd()[base + 2].kp() = 50;
                low_cmd.motor_cmd()[base + 0].kd() = 3.5;
                low_cmd.motor_cmd()[base + 1].kd() = 3.5;
                low_cmd.motor_cmd()[base + 2].kd() = 3.5;
                low_cmd.motor_cmd()[base + 0].tau() = 0;
                low_cmd.motor_cmd()[base + 1].tau() = 0;
                low_cmd.motor_cmd()[base + 2].tau() = 0;
            }
        } else {
            // stand 模式：先从趴姿平滑站起（3 秒），再保持
            double phase = std::tanh(running_time_ / 1.2);
            static const double kStandDown[12] = {
                0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
                0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
            static const double kStandUp[12] = {
                kStandHip, kStandThigh, kStandCalf, kStandHip, kStandThigh, kStandCalf,
                kStandHip, kStandThigh, kStandCalf, kStandHip, kStandThigh, kStandCalf};
            for (int i = 0; i < 12; i++) {
                low_cmd.motor_cmd()[i].q() =
                    phase * kStandUp[i] + (1.0 - phase) * kStandDown[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1.0 - phase) * 20.0;
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
        }

        PublishLowCmd();
    }

    // 从四元数（w,x,y,z）提取 yaw（避免 rpy 欧拉角在俯仰较大时的万向锁问题）
    static double YawFromQuat(const std::array<float, 4> &q)
    {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    // 计算 CRC 并发布 lowcmd（各分支统一出口）
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

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <stand|walk|turn> <duration_s> [speed] [heading_rad]" << std::endl;
        return 1;
    }
    const std::string mode = argv[1];
    const double duration = std::atof(argv[2]);
    const double speed = (argc > 3) ? std::atof(argv[3]) : 1.0;
    // 第 4 参数：目标偏航角（rad，正 = 左转），用于 walk 模式走向指定方位
    const double heading = (argc > 4) ? std::atof(argv[4]) : 0.0;

    // 初始化 DDS：domain 1 + lo 网卡，与 unitree_mujoco simulate/config.yaml 一致
    ChannelFactory::Instance()->Init(1, "lo");

    std::cout << "步态控制器启动: mode=" << mode << " duration=" << duration
              << "s speed=" << speed << std::endl;

    GaitController controller(mode, duration, speed, heading);
    controller.Init();

    // 运行 duration 后自动退出。walk/turn 模式前有 3 秒起立过渡，多留 5 秒让机器人稳住。
    // 注意：不能走正常析构（写线程仍在访问成员，析构会引发堆损坏），
    // 与官方 stand_go2 的 while(1) 处理方式等价，改用 _Exit 跳过析构。
    Sleep((int)((duration + 5.0) * 1000000));
    std::cout << "步态控制器结束" << std::endl;
    std::_Exit(0);
    return 0;
}

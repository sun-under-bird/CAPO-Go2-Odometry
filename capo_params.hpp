// capo_params.hpp
// CAPO 参数文件路径解析（仿真适配新增，真机行为不变）。
//
// 背景：原实机代码在 main() 中硬编码加载 /root/CAPO-LeggedRobotOdometry/config.yaml，
// 该路径仅存在于 Go2 真机板载系统上；本机 MuJoCo 仿真运行时会因文件不存在而启动失败。
//
// 解析优先级：
//   1. 环境变量 CAPO_PARAMS_FILE 显式指定的路径
//   2. 真机路径 /root/CAPO-LeggedRobotOdometry/config.yaml（文件存在时沿用，真机零改动）
//   3. fusion_estimator 包安装目录下的 config/config.yaml（本机仿真使用，见 CMakeLists install）
// 都不可用时返回空串，节点以默认参数启动（由 launch 显式传参的场合不受影响）。
#pragma once

#include <unistd.h>

#include <cstdlib>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

inline std::string resolve_params_file()
{
    // 1. 环境变量覆盖，便于测试不同参数组合
    if (const char *env_path = std::getenv("CAPO_PARAMS_FILE")) {
        if (access(env_path, R_OK) == 0) {
            return std::string(env_path);
        }
    }

    // 2. 真机 Go2 板载路径，保持既有实机部署完全不变
    static const char *kRobotParamsFile = "/root/CAPO-LeggedRobotOdometry/config.yaml";
    if (access(kRobotParamsFile, R_OK) == 0) {
        return std::string(kRobotParamsFile);
    }

    // 3. 本机仿真：使用随 fusion_estimator 包安装的 config.yaml
    try {
        const std::string installed =
            ament_index_cpp::get_package_share_directory("fusion_estimator") + "/config/config.yaml";
        if (access(installed.c_str(), R_OK) == 0) {
            return installed;
        }
    } catch (const std::exception &) {
        // ament_index 未找到包（例如直接以可执行文件运行），回退为空
    }

    return std::string();
}

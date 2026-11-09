#include "FusionEstimator/PointLegKinematics.h"
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    try {
        const bool motion = argc == 2 && std::string(argv[1]) == "--motion";
        std::array<DataFusion::PointLegKinematics, 4> legs;
        for (auto& leg : legs) {
            int count;
            double radius;
            if (!(std::cin >> count >> radius) || count < 1 || count > 12) return 2;
            std::vector<double> flat(count*9);
            for (double& v : flat) if (!(std::cin >> v)) return 2;
            leg.configure(flat, radius);
        }
        int count;
        if (!(std::cin >> count)) return 2;
        std::cout << std::setprecision(17);
        while (count-- > 0) {
            double message[48], quaternion[4], omega[3] = {};
            for (double& x : message) if (!(std::cin >> x)) return 2;
            for (double& x : quaternion) if (!(std::cin >> x)) return 2;
            if (motion) for (double& x : omega) if (!(std::cin >> x)) return 2;
            for (const auto& leg : legs) {
                auto result = leg.compute(message, quaternion, motion ? omega : nullptr);
                for (const auto& vector : {result.positions[3], result.velocity, result.force})
                    for (double x : vector) std::cout << x << ' ';
                if (motion) for (double x : result.rolling_velocity) std::cout << x << ' ';
            }
            std::cout << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}

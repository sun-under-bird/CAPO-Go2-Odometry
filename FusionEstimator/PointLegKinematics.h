#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace DataFusion {

// A serial point-foot chain. Shared joints contribute to velocity, but only
// the three leg-specific torque measurements enter the foot-force solve.
class PointLegKinematics {
public:
    using Vec = std::array<double, 3>;
    using Mat = std::array<Vec, 3>;
    struct Result {
        std::array<Vec, 4> positions{}; // hip, thigh, calf, contact (legacy ABI)
        Vec velocity{}, force{};
    };

    // Per node: motor slot, axis (-1 fixed, 0 X, 1 Y, 2 Z), use torque,
    // translation xyz [m], fixed rotation roll/pitch/yaw [rad].
    void configure(const std::vector<double>& flat, double radius) {
        if (flat.empty() || flat.size() % 9 || flat.size() / 9 > 12 ||
            !std::isfinite(radius) || radius < 0 || radius > 0.1)
            throw std::invalid_argument("Invalid point-leg geometry or radius");
        std::vector<Node> parsed;
        std::array<bool, 16> slots{};
        int forces = 0;
        for (size_t i = 0; i < flat.size(); i += 9) {
            for (size_t k = 0; k < 9; ++k)
                if (!std::isfinite(flat[i+k])) throw std::invalid_argument("Non-finite geometry");
            for (size_t k = 0; k < 3; ++k)
                if (flat[i+k] != std::trunc(flat[i+k]))
                    throw std::invalid_argument("Geometry indices must be integers");
            Node n;
            n.slot = static_cast<int>(flat[i]);
            n.axis = static_cast<int>(flat[i+1]);
            n.force = flat[i+2] == 1;
            if (n.slot < -1 || n.slot >= 16 || n.axis < -1 || n.axis > 2 ||
                (n.slot == -1) != (n.axis == -1) ||
                (flat[i+2] != 0 && flat[i+2] != 1) || (n.force && n.slot < 0))
                throw std::invalid_argument("Invalid point-leg motor/axis/torque selection");
            if (n.slot >= 0) {
                if (slots[n.slot]) throw std::invalid_argument("Duplicate motor in chain");
                slots[n.slot] = true;
            }
            n.translation = {flat[i+3], flat[i+4], flat[i+5]};
            n.rotation = rpy(flat[i+6], flat[i+7], flat[i+8]);
            forces += n.force;
            parsed.push_back(n);
        }
        if (forces != 3 || parsed.back().slot != -1)
            throw std::invalid_argument("Point leg needs three independent torques and a fixed endpoint");
        nodes_ = std::move(parsed);
        radius_ = radius;
    }

    Result compute(const double* message, const double* quaternion) const {
        Result out;
        Mat rotation = quat(quaternion);
        Vec position{};
        struct Joint { Vec origin, axis; int slot; bool force; };
        std::vector<Joint> joints;
        int leg_joint = 0;
        for (const auto& n : nodes_) {
            position = add(position, multiply(rotation, n.translation));
            rotation = multiply(rotation, n.rotation);
            if (n.slot >= 0) {
                Vec axis{rotation[0][n.axis], rotation[1][n.axis], rotation[2][n.axis]};
                joints.push_back({position, axis, n.slot, n.force});
                if (n.force) out.positions[leg_joint++] = position;
                Vec angle{};
                angle[n.axis] = message[n.slot];
                rotation = multiply(rotation, rpy(angle[0], angle[1], angle[2]));
            }
        }
        // Flat-ground spherical-foot contact, expressed relative to the base
        // in world axes. Radius is model geometry, never a contact measurement.
        position[2] -= radius_;
        out.positions[3] = position;
        Mat normal{};
        Vec rhs{};
        for (const auto& j : joints) {
            const Vec column = cross(j.axis, subtract(position, j.origin));
            for (int a = 0; a < 3; ++a) {
                out.velocity[a] += column[a] * message[16+j.slot];
                if (!j.force) continue;
                rhs[a] += column[a] * message[32+j.slot];
                for (int b = 0; b < 3; ++b) normal[a][b] += column[a]*column[b];
            }
        }
        // Damped least squares remains finite near a straight-leg singularity.
        for (int a = 0; a < 3; ++a) normal[a][a] += 1e-8;
        out.force = solve(normal, rhs);
        return out;
    }

private:
    struct Node { int slot, axis; bool force; Vec translation; Mat rotation; };
    std::vector<Node> nodes_;
    double radius_ = 0;
    static Vec add(Vec a, const Vec& b) { for (int i=0;i<3;++i) a[i]+=b[i]; return a; }
    static Vec subtract(Vec a, const Vec& b) { for (int i=0;i<3;++i) a[i]-=b[i]; return a; }
    static Vec cross(const Vec& a, const Vec& b) {
        return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    static Vec multiply(const Mat& a, const Vec& b) {
        Vec c{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) c[i]+=a[i][j]*b[j]; return c;
    }
    static Mat multiply(const Mat& a, const Mat& b) {
        Mat c{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j)
            for(int k=0;k<3;++k) c[i][j]+=a[i][k]*b[k][j]; return c;
    }
    static Mat rpy(double r, double p, double y) {
        const double cr=std::cos(r), sr=std::sin(r), cp=std::cos(p), sp=std::sin(p), cy=std::cos(y), sy=std::sin(y);
        return {{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
                 {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}}};
    }
    static Mat quat(const double* q) {
        const double w=q[0],x=q[1],y=q[2],z=q[3];
        return {{{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)},
                 {2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)},
                 {2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}}};
    }
    static Vec solve(Mat a, Vec b) {
        for(int k=0;k<3;++k) {
            int pivot=k;
            for(int i=k+1;i<3;++i) if(std::abs(a[i][k])>std::abs(a[pivot][k])) pivot=i;
            if(std::abs(a[pivot][k])<1e-14) return {};
            std::swap(a[k],a[pivot]); std::swap(b[k],b[pivot]);
            const double d=a[k][k];
            for(int j=k;j<3;++j) a[k][j]/=d;
            b[k]/=d;
            for(int i=0;i<3;++i) if(i!=k) {
                const double f=a[i][k];
                for(int j=k;j<3;++j) a[i][j]-=f*a[k][j];
                b[i]-=f*b[k];
            }
        }
        return b;
    }
};
} // namespace DataFusion

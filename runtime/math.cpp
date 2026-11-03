#include "runtime/math.hpp"

#include <cmath>

namespace T1
{

    double wrap_angle(double angle)
    {
        const double a = std::atan2(std::sin(angle), std::cos(angle));
        if (a <= -kPi)
        {
            return kPi;
        }
        return a;
    }

    Mat3 skew(const Vec3& v)
    {
        Mat3 m;
        m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return m;
    }

    Vec3 so3_log(const Mat3& rotation)
    {
        const Eigen::AngleAxisd aa(rotation);
        if (aa.angle() < 1e-12)
        {
            return Vec3::Zero();    // 轴方向无定义
        }
        return aa.angle() * aa.axis();
    }

    double yaw_of(const Mat3& rotation)
    {
        return std::atan2(rotation(1, 0), rotation(0, 0));
    }

    VecX saturate_norm(const VecX& v, double max_norm)
    {
        const double n = v.norm();
        if (n <= max_norm)
        {
            return v;
        }
        return v * (max_norm / n);
    }

}  // namespace T1

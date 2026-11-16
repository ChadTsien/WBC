#include "control/reference.hpp"

#include "control/types.hpp"
#include "runtime/math.hpp"

#include <algorithm>

namespace T1::wbc
{

    namespace
    {
        double lerp(double a, double b, double s)
        {
            return a + s * (b - a);
        }

        Reference lerp_ref(const Reference& a, const Reference& b, double s)
        {
            Reference r = a;   // 离散字段保持左端点
            
            // 底盘位置
            r.chassis_pose.x = lerp(a.chassis_pose.x, b.chassis_pose.x, s);
            r.chassis_pose.y = lerp(a.chassis_pose.y, b.chassis_pose.y, s);
            r.chassis_pose.yaw =
                wrap_angle(a.chassis_pose.yaw
                           + s * wrap_angle(b.chassis_pose.yaw - a.chassis_pose.yaw));

            // 底盘速度
            r.chassis_twist.v = lerp(a.chassis_twist.v, b.chassis_twist.v, s);
            r.chassis_twist.omega = lerp(a.chassis_twist.omega, b.chassis_twist.omega, s);
            
            // 工具坐标
            r.ee_pose.translation() = 
                a.ee_pose.translation() + s * (b.ee_pose.translation() - a.ee_pose.translation());
            
            // 工具姿态
            const Quat qa(a.ee_pose.linear());
            const Quat qb(b.ee_pose.linear());
            r.ee_pose.linear() = qa.slerp(s, qb).toRotationMatrix();

            // 工具速度
            r.ee_twist.linear =
                a.ee_twist.linear + s * (b.ee_twist.linear - a.ee_twist.linear);
            r.ee_twist.angular =
                a.ee_twist.angular + s * (b.ee_twist.angular - a.ee_twist.angular);
            
            r.surface_normal =
                (a.surface_normal + s * (b.surface_normal - a.surface_normal)).normalized();
            
            r.desired_normal_force =
                lerp(a.desired_normal_force, b.desired_normal_force, s);

            return r;
        }
    }

    Reference sample_reference(const Trajectory& traj, double t)
    {
        // later：第一个时刻严格大于 t 的点，即当前区间右端
        const auto later = std::upper_bound(
            traj.begin(), traj.end(), t,
            [](double time, const TimedReference& p) { return time < p.t; });

        if (later == traj.begin())
        {
            return traj.front().ref;  // t 早于首点：还没开始
        }
        if (later == traj.end())
        {
            Reference r = traj.back().ref;
            if (t > traj.back().t)
            {
                r.chassis_twist = {};  // 超出末点：停在最后位姿
                r.ee_twist = {};
            }
            return r;
        }

        const TimedReference& a = *(later - 1);
        const TimedReference& b = *later;
        const double dt = b.t - a.t;
        const double s = (dt > 0.0) ? (t - a.t) / dt : 0.0;  // [0, 1)，落在 [a, b) 的比例
        return lerp_ref(a.ref, b.ref, s);
    }

}  // namespace T1::wbc

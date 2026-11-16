#include "sim/world.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include "model/chassis_model.hpp"
#include "types/types.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <mujoco/mujoco.h>
#include "runtime/math.hpp"


namespace T1::sim
{
    struct MujocoWorld::Impl
    {
        WorldConfig config;
        MitCommand arm_command;
        WheelSpeeds wheel_command;
        std::mutex mutex;                  // step / viewer / 读状态共用

        mjModel* model = nullptr;          // 结构：关节、执行器、质量
        mjData* data = nullptr;            // 状态：qpos / qvel / ctrl，每拍覆盖

        int arm_jnt[kNumArmJoints]{};      // 关节 id
        int arm_qadr[kNumArmJoints]{};     // 写入 qpos[] 的下标
        int arm_vadr[kNumArmJoints]{};     // 写入 qvel[] 的下标
        int arm_act[kNumArmJoints]{};      // motor id，写入 ctrl[]
        int wheel_jnt[2]{};                // qvel[] 下标
        int wheel_qadr[2]{};               // velocity 执行器，写入 ctrl[]
        int wheel_vadr[2]{};               // velocity 执行器
        int wheel_act[2]{};                // freejoint base_free，7 qpos / 6 qvel
        int base_jnt = -1;
        int base_qadr = -1;
        int base_vadr = -1;
        int chassis_body = -1;
        int brush_site = -1;
        int imu_quat_adr = -1;              // sensordata：framequat wxyz
        int imu_gyro_adr = -1;              // 角速度，IMU 系
        int imu_accel_adr = -1;             // 加速度（含重力），IMU 系

        ~Impl()
        {
            mj_deleteData(data);
            mj_deleteModel(model);
        }
    };

    namespace
    {
        int require_id(const mjModel* m, mjtObj type, const char* name)
        {
            // 名字必须与 scene.xml / NAME_MAPPING 一致
            const int id = mj_name2id(m, type, name);
            if (id < 0)
            {
                throw std::runtime_error(std::string("MujocoWorld: missing ") + name);
            }
            return id;
        }
    }

    MujocoWorld::MujocoWorld(const WorldConfig& config) : impl_(std::make_unique<Impl>())
    {
        impl_->config = config;
        if (config.mjcf_path.empty())
        {
            throw std::runtime_error("MujocoWorld: empty mjcf_path");
        }

        char err[1024] = {0};
        impl_->model = mj_loadXML(config.mjcf_path.c_str(), nullptr, err, sizeof(err));
        if (impl_->model == nullptr)
        {
            throw std::runtime_error(std::string("MujocoWorld: ") + err);
        }
        impl_->data = mj_makeData(impl_->model);

        const mjModel* m = impl_->model;
        const char* arm_names[kNumArmJoints] = {
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5", "arm_joint6"};
        for (int i = 0; i < kNumArmJoints; i++)
        {
            impl_->arm_jnt[i] = require_id(m, mjOBJ_JOINT, arm_names[i]);
            impl_->arm_act[i] = require_id(m, mjOBJ_ACTUATOR, arm_names[i]);
            impl_->arm_qadr[i] = m->jnt_qposadr[impl_->arm_jnt[i]];
            impl_->arm_vadr[i] = m->jnt_dofadr[impl_->arm_jnt[i]];
        }

        const char* wheel_names[2] = {"wheel_left_joint", "wheel_right_joint"};
        for (int i = 0; i < 2; ++i)
        {
            impl_->wheel_jnt[i] = require_id(m, mjOBJ_JOINT, wheel_names[i]);
            impl_->wheel_act[i] = require_id(m, mjOBJ_ACTUATOR, wheel_names[i]);
            impl_->wheel_qadr[i] = m->jnt_qposadr[impl_->wheel_jnt[i]];
            impl_->wheel_vadr[i] = m->jnt_dofadr[impl_->wheel_jnt[i]];
        }

        impl_->base_jnt = require_id(m, mjOBJ_JOINT, "base_free");
        impl_->base_qadr = m->jnt_qposadr[impl_->base_jnt];
        impl_->base_vadr = m->jnt_dofadr[impl_->base_jnt];
        impl_->chassis_body = require_id(m, mjOBJ_BODY, "chassis_link");
        impl_->brush_site = require_id(m, mjOBJ_SITE, "brush_tip");

        impl_->imu_quat_adr = m->sensor_adr[require_id(m, mjOBJ_SENSOR, "imu_orientation")];
        impl_->imu_gyro_adr = m->sensor_adr[require_id(m, mjOBJ_SENSOR, "imu_gyro")];
        impl_->imu_accel_adr = m->sensor_adr[require_id(m, mjOBJ_SENSOR, "imu_accel")];
    }

    MujocoWorld::~MujocoWorld() = default;

    void MujocoWorld::reset()
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        mj_resetData(impl_->model, impl_->data);
        mj_forward(impl_->model, impl_->data);       // 填 xpos / site，否则 ground_truth 是空的
    }

    void MujocoWorld::step()
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        mjData* d = impl_->data;
        const MitCommand& cmd = impl_->arm_command;

        for (int i = 0; i < kNumArmJoints; ++i)
        {
            const double q = d->qpos[impl_->arm_qadr[i]];
            const double dq = d->qvel[impl_->arm_vadr[i]];

            // MIT: τ = kp(q_des − q) + kd(dq_des − dq) + τ_ff
            double tau = cmd.kp[i] * (cmd.q[i] - q) + cmd.kd[i] * (cmd.dq[i] - dq) + cmd.tau_ff[i];
            
            // 限幅
            const double lim = impl_->config.effort_limits[i];
            tau = std::clamp(tau, -lim, lim);

            d->ctrl[impl_->arm_act[i]] = tau;
        }

        d->ctrl[impl_->wheel_act[0]] = impl_->wheel_command.left;
        d->ctrl[impl_->wheel_act[1]] = impl_->wheel_command.right;
        mj_step(impl_->model, d);
    }

    double MujocoWorld::time() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->data->time;
    }

    void MujocoWorld::set_arm_command(const MitCommand& command)
    {
        impl_->arm_command = command;
    }

    void MujocoWorld::set_wheel_command(const WheelSpeeds& speeds)
    {
        impl_->wheel_command = speeds;
    }

    ArmState MujocoWorld::arm_state() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const mjData* d = impl_->data;
        ArmState s;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            s.q[i] = d->qpos[impl_->arm_qadr[i]];
            s.dq[i] = d->qvel[impl_->arm_vadr[i]];
            s.tau[i] = d->actuator_force[impl_->arm_act[i]];
        }
        return s;
    }

    WheelSpeeds MujocoWorld::wheel_speeds() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        WheelSpeeds w;
        w.left = impl_->data->qvel[impl_->wheel_vadr[0]];
        w.right = impl_->data->qvel[impl_->wheel_vadr[1]];
        return w;
    }

    WheelSpeeds MujocoWorld::wheel_positions() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        WheelSpeeds w;
        w.left = impl_->data->qpos[impl_->wheel_qadr[0]];
        w.right = impl_->data->qpos[impl_->wheel_qadr[1]];
        return w;
    }

    Wrench MujocoWorld::ft_raw() const
    {
        // TODO(M3a)
        return Wrench{};
    }

    Quat MujocoWorld::imu_orientation() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const double* s = impl_->data->sensordata + impl_->imu_quat_adr;
        return Quat(s[0], s[1], s[2], s[3]);   // MuJoCo  wxyz
    }

    Vec3 MujocoWorld::imu_angular_velocity() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const double* s = impl_->data->sensordata + impl_->imu_gyro_adr;
        return Vec3(s[0], s[1], s[2]);
    }

    Vec3 MujocoWorld::imu_linear_acceleration() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const double* s = impl_->data->sensordata + impl_->imu_accel_adr;
        return Vec3(s[0], s[1], s[2]);
    }

    GroundTruth MujocoWorld::ground_truth() const
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const mjData* d = impl_->data;
        const double* pq = d->qpos + impl_->base_qadr;    // x y z qw qx qy qz
        const double* pv = d->qvel + impl_->base_vadr;    // 线速度在世界，角速度在 base_link

        GroundTruth gt;
        gt.chassis_pose.x = pq[0];
        gt.chassis_pose.y = pq[1];
        const Eigen::Quaterniond quat(pq[3], pq[4], pq[5], pq[6]);   // wxyz
        const Mat3 R = quat.toRotationMatrix();
        gt.chassis_pose.yaw = yaw_of(R);

        const Vec3 v_world(pv[0], pv[1], pv[2]);
        gt.chassis_twist.v = (R.transpose() * v_world).x();  // 沿 base_link +X
        gt.chassis_twist.omega = pv[5];                      // 绕 base_link +Z

        const double* p = d->site_xpos + 3 * impl_->brush_site;
        const double* M = d->site_xmat + 9 * impl_->brush_site;
        gt.tool_pose_world.translation() = Vec3(p[0], p[1], p[2]);
        Mat3 Rt;
        Rt << M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8];
        gt.tool_pose_world.linear() = Rt;
        return gt;
    }

    std::mutex& MujocoWorld::mutex() const
    {
        return impl_->mutex;
    }
    void* MujocoWorld::raw_model() const
    {
        return impl_->model;
    }
    void* MujocoWorld::raw_data() const
    {
        return impl_->data;
    }

}  // namespace T1::sim

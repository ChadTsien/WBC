#include "control/config.hpp"

#include "runtime/yaml_params.hpp"

#include <stdexcept>
#include <string>

namespace T1::wbc
{
namespace
{
    YAML::Node child(const YAML::Node& n, const char* key)
    {
        if (!n[key])
        {
            throw std::runtime_error("missing yaml key: " + std::string(key));
        }
        return n[key];
    }

    GainSet load_gains(const YAML::Node& n)
    {
        GainSet g;
        g.kp = require_vector(n, "kp", kNumArmJoints);
        g.kd = require_vector(n, "kd", kNumArmJoints);
        return g;
    }

    EeTaskConfig load_ee(const YAML::Node& n)
    {
        EeTaskConfig c;
        c.kp = require_vector(n, "kp", kTaskDim);
        c.kd = require_vector(n, "kd", kTaskDim);
        c.weight = require_vector(n, "weight", kTaskDim);
        c.max_error_rate = require<double>(n, "max_error_rate");
        return c;
    }

    PostureTaskConfig load_posture(const YAML::Node& n)
    {
        PostureTaskConfig c;
        c.q_nominal = require_vector(n, "q_nominal", kNumArmJoints);
        c.weight = require_vector(n, "weight", kNumArmJoints);
        c.kp = require<double>(n, "kp");
        return c;
    }

    ChassisTaskConfig load_chassis_task(const YAML::Node& n)
    {
        ChassisTaskConfig c;
        c.regularization = require<double>(n, "regularization");
        c.track_weight = require<double>(n, "track_weight");
        c.kp_position = require<double>(n, "kp_position");
        c.kp_yaw = require<double>(n, "kp_yaw");
        return c;
    }

    LimitsConfig load_limits(const YAML::Node& n)
    {
        LimitsConfig c;
        c.q_min = require_vector(n, "q_min", kNumArmJoints);
        c.q_max = require_vector(n, "q_max", kNumArmJoints);
        c.dq_max = require_vector(n, "dq_max", kNumArmJoints);
        c.ddq_max = require_vector(n, "ddq_max", kNumArmJoints);
        c.v_max = require<double>(n, "v_max");
        c.omega_max = require<double>(n, "omega_max");
        c.dv_max = require<double>(n, "dv_max");
        c.domega_max = require<double>(n, "domega_max");
        c.position_margin = require<double>(n, "position_margin");
        c.position_gain = require<double>(n, "position_gain");
        return c;
    }

    QpSolverConfig load_qp(const YAML::Node& n)
    {
        QpSolverConfig c;
        c.max_iterations = require<int>(n, "max_iterations");
        c.eps_abs = require<double>(n, "eps_abs");
        c.warm_start = require<bool>(n, "warm_start");
        return c;
    }

    CommandConfig load_command(const YAML::Node& n)
    {
        CommandConfig c;
        c.free_motion = load_gains(child(n, "free_motion"));
        c.contact = load_gains(child(n, "contact"));
        c.max_tracking_error = require<double>(n, "max_tracking_error");
        return c;
    }

    ContactConfig load_contact(const YAML::Node& n)
    {
        ContactConfig c;
        c.approach_speed = require<double>(n, "approach_speed");
        c.force_ramp_rate = require<double>(n, "force_ramp_rate");
        c.retract_speed = require<double>(n, "retract_speed");
        c.released_force = require<double>(n, "released_force");
        c.max_force = require<double>(n, "max_force");
        return c;
    }

    AdmittanceConfig load_admittance(const YAML::Node& n)
    {
        AdmittanceConfig c;
        c.mass = require<double>(n, "mass");
        c.damping = require<double>(n, "damping");
        c.stiffness = require<double>(n, "stiffness");
        c.max_offset = require<double>(n, "max_offset");
        c.max_velocity = require<double>(n, "max_velocity");
        c.tilt_damping = require<double>(n, "tilt_damping");
        c.max_tilt = require<double>(n, "max_tilt");
        return c;
    }

    ChassisModelConfig load_chassis(const YAML::Node& n)
    {
        ChassisModelConfig c;
        c.wheel_radius = require<double>(n, "wheel_radius");
        c.wheel_separation = require<double>(n, "wheel_separation");
        return c;
    }

    est::EkfConfig load_ekf(const YAML::Node& n)
    {
        est::EkfConfig c;
        c.q_v = require<double>(n, "q_v");
        c.q_omega = require<double>(n, "q_omega");
        c.r_v = require<double>(n, "r_v");
        c.r_omega_odom = require<double>(n, "r_omega_odom");
        c.r_omega_gyro = require<double>(n, "r_omega_gyro");
        c.gate_chi2 = require<double>(n, "gate_chi2");
        return c;
    }

    ft::FtCompensatorConfig load_ft(const YAML::Node& n)
    {
        ft::FtCompensatorConfig c;
        c.cutoff_hz = require<double>(n, "cutoff_hz");
        c.inertia_enabled = require<bool>(n, "inertia_enabled");
        c.tare_samples = require<int>(n, "tare_samples");
        return c;
    }

    ft::ContactDetectorConfig load_contact_detector(const YAML::Node& n)
    {
        ft::ContactDetectorConfig c;
        c.contact_on_force = require<double>(n, "contact_on_force");
        c.contact_off_force = require<double>(n, "contact_off_force");
        c.detach_tolerance = require<double>(n, "detach_tolerance");
        c.detach_hold_time = require<double>(n, "detach_hold_time");
        return c;
    }
}  // namespace

    WbcConfig load_wbc_config(const std::string& yaml_path)
    {
        const YAML::Node root = load_yaml_file(yaml_path);
        WbcConfig c;
        c.control_rate_hz = require<double>(root, "control_rate_hz");
        c.max_inequality_rows = require<int>(root, "max_inequality_rows");
        c.ee = load_ee(child(root, "ee_task"));
        c.posture = load_posture(child(root, "posture_task"));
        c.chassis_task = load_chassis_task(child(root, "chassis_task"));
        c.limits = load_limits(child(root, "limits"));
        c.qp = load_qp(child(root, "qp"));
        c.command = load_command(child(root, "command"));
        c.command.q_min = c.limits.q_min;
        c.command.q_max = c.limits.q_max;
        c.contact = load_contact(child(root, "contact"));
        c.admittance = load_admittance(child(root, "admittance"));
        c.chassis = load_chassis(child(root, "chassis"));
        c.ekf = load_ekf(child(root, "ekf"));
        const VecX rpy = require_vector(root, "base_R_imu_rpy", 3);
        c.base_R_imu = (Eigen::AngleAxisd(rpy[2], Vec3::UnitZ()) *
                        Eigen::AngleAxisd(rpy[1], Vec3::UnitY()) *
                        Eigen::AngleAxisd(rpy[0], Vec3::UnitX()))
                           .toRotationMatrix();
        c.ft = load_ft(child(root, "ft"));
        c.contact_detector = load_contact_detector(child(root, "contact_detector"));
        c.brushes_yaml = require<std::string>(root, "brushes_yaml");
        c.default_brush = require<std::string>(root, "default_brush");
        return c;
    }

}  // namespace T1::wbc

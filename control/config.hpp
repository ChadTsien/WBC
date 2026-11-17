#pragma once

#include <string>

#include "control/admittance.hpp"
#include "control/command.hpp"
#include "control/contact_phase.hpp"
#include "control/limits.hpp"
#include "control/qp_solver.hpp"
#include "control/task_chassis.hpp"
#include "control/task_ee.hpp"
#include "control/task_posture.hpp"
#include "estimation/ekf.hpp"
#include "model/chassis_model.hpp"
#include "signal/compensate.hpp"
#include "signal/contact.hpp"

namespace T1::wbc
{

    /**
     * 闭环控制器全部参数，对应 config/wbc.yaml
     *
     * 成员
     * - control_rate_hz：控制频率，Hz
     * - max_inequality_rows：QP 不等式最大行数（M5 碰撞用）
     * - ee / posture / chassis_task / limits / qp / command：M1 任务与限位
     * - contact / admittance：M3b，现在也读进来，update 里先不用
     * - chassis：差速几何
     * - ekf / base_R_imu / ft / contact_detector：估计与力，M2/M3 用
     * - brushes_yaml / default_brush：工具参数文件
     */
    struct WbcConfig
    {
        double control_rate_hz = 500.0;
        int max_inequality_rows = 64;

        EeTaskConfig ee;
        PostureTaskConfig posture;
        ChassisTaskConfig chassis_task;
        LimitsConfig limits;
        QpSolverConfig qp;
        CommandConfig command;
        ContactConfig contact;
        AdmittanceConfig admittance;
        ChassisModelConfig chassis;

        est::EkfConfig ekf;
        Mat3 base_R_imu = Mat3::Identity();
        ft::FtCompensatorConfig ft;
        ft::ContactDetectorConfig contact_detector;
        std::string brushes_yaml;
        std::string default_brush = "default";
    };

    
    /**
     * 从 yaml 加载 WbcConfig
     *
     * 说明
     * - 缺键、长度不对、文件打不开都抛 std::runtime_error，what() 含键名或路径
     * - 每个小节一个静态函数，单函数不超过 50 行
     */
    WbcConfig load_wbc_config(const std::string& yaml_path);

}  // namespace T1::wbc

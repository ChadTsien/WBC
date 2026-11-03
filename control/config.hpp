#pragma once

#include <string>

#include "control/admittance.hpp"
#include "control/command.hpp"
#include "control/contact_phase.hpp"
#include "control/limits.hpp"
#include "control/qp_solver.hpp"
#include "control/task_base.hpp"
#include "control/task_ee.hpp"
#include "control/task_posture.hpp"
#include "estimation/ekf.hpp"
#include "kine/base.hpp"
#include "signal/compensate.hpp"
#include "signal/contact.hpp"

namespace T1::wbc
{

    // Everything the in-loop controller needs, loaded from one file (config/wbc.yaml).
    struct WbcConfig
    {
        double control_rate_hz = 500.0;
        int max_inequality_rows = 64;

        EeTaskConfig ee;
        PostureTaskConfig posture;
        BaseTaskConfig base;
        LimitsConfig limits;
        QpSolverConfig qp;
        CommandConfig command;
        ContactConfig contact;
        AdmittanceConfig admittance;
        DiffDriveParams diff_drive;

        est::EkfConfig ekf;
        Mat3 base_R_imu = Mat3::Identity();
        ft::FtCompensatorConfig ft;
        ft::ContactDetectorConfig contact_detector;
        std::string brushes_yaml;
        std::string default_brush = "default";
    };

    // Throws std::runtime_error naming the missing / malformed key.
    WbcConfig load_wbc_config(const std::string& yaml_path);

}  // namespace T1::wbc

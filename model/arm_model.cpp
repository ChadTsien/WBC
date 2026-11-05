#include "model/arm_model.hpp"

#include <algorithm>
#include <memory>
#include <pinocchio/multibody/fwd.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "types/types.hpp"

namespace T1
{

    struct ArmModel::Impl
    {
        pinocchio::Model model;
        std::unique_ptr<pinocchio::Data> data;
        int tool_frame = -1;
        int ft_frame = -1;
        int mount_frame = -1;
    };

    ArmModel::ArmModel(const ArmModelConfig& config) : impl_(std::make_unique<Impl>())
    {
        if (config.urdf_xml.empty())
        {
            throw std::runtime_error("ArmModel: empty URDF");
        }

        pinocchio::Model full;
        pinocchio::urdf::buildModelFromXML(config.urdf_xml, full);

        // universe(0) 不能锁. 锁轮子、相机偏航等所有非臂关节
        std::vector<pinocchio::JointIndex> locked;
        for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(full.njoints); ++i)
        {
            // 寻找是否为臂关节
            const bool is_arm = std::find(config.arm_joints.begin(), config.arm_joints.end(),
                                        full.names[i]) != config.arm_joints.end();
            
            if (!is_arm)
            {
                locked.push_back(i);
            }
        }

        // 锁非臂关节，只留 6 轴
        pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full), impl_->model);
        if (impl_->model.nq != kNumArmJoints)
        {
            throw std::runtime_error("ArmModel: nq=" + std::to_string(impl_->model.nq) +
                                 " (forgot to lock chassis_rgbd_joint?)");
        }

        // 检验裁减完后后的关节是否正确，且顺序是否被打乱
        int prev_id = 0;
        for (const auto& name : config.arm_joints)
        {
            if (!impl_->model.existJointName(name))
            {
                throw std::runtime_error("ArmModel: missing joint " + name);
            }
            const int id = static_cast<int>(impl_->model.getJointId(name));
            if (id <= prev_id)
            {
                throw std::runtime_error("ArmModel: joint order " + name);
            }
            prev_id = id;
        }

        auto require_frame = [&](const std::string& name) {
            if (!impl_->model.existFrame(name))
            {
                throw std::runtime_error("ArmModel: missing frame " + name);
            }
            return static_cast<int>(impl_->model.getFrameId(name));
        };
        impl_->tool_frame = require_frame(config.tool_frame);
        impl_->ft_frame = require_frame(config.ft_frame);
        impl_->mount_frame = require_frame(config.mount_frame);

        impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
    }

    ArmModel::~ArmModel() = default;

    const pinocchio::Model& ArmModel::model() const
    {
        return impl_->model;
    }

    pinocchio::Data& ArmModel::data()
    {
        return *impl_->data;
    }

    int ArmModel::tool_frame_id() const
    {
        return impl_->tool_frame;
    }

    int ArmModel::ft_frame_id() const
    {
        return impl_->ft_frame;
    }

    int ArmModel::mount_frame_id() const
    {
        return impl_->mount_frame;
    }

    int ArmModel::frame_id(const std::string& name) const
    {
        if (!impl_->model.existFrame(name))
        {
            throw std::runtime_error("ArmModel: missing frame " + name);
        }
        return static_cast<int>(impl_->model.getFrameId(name));
    }

    ArmVec ArmModel::lower_limits() const
    {
        ArmVec out;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            const auto jid = static_cast<pinocchio::JointIndex>(i) + 1;
            out[i] = impl_->model.lowerPositionLimit[impl_->model.idx_qs[jid]];
        }
        return out;
    }

    ArmVec ArmModel::upper_limits() const
    {
        ArmVec out;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            const auto jid = static_cast<pinocchio::JointIndex>(i) + 1;
            out[i] = impl_->model.upperPositionLimit[impl_->model.idx_qs[jid]];
        }
        return out;
    }

    ArmVec ArmModel::velocity_limits() const
    {
        ArmVec out;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            const auto jid = static_cast<pinocchio::JointIndex>(i) + 1;
            out[i] = impl_->model.velocityLimit[impl_->model.idx_vs[jid]];
        }
        return out;
    }

}  // namespace T1

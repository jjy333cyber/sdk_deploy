/**
 * @file m20_policy_runner.hpp
 * @brief m20_policy_runner
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-11-07
 *
 * @copyright Copyright (c) 2025 DeepRobotics
 *
 */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class M20PolicyRunner : public PolicyRunnerBase {
private:
    VecXf kp_, kd_;
    VecXf dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f max_cmd_vel_, gravity_direction = Vec3f(0., 0., -1.);
    VecXf dof_pos_default_;
    timespec system_time;

    const int motor_num = 16;
    const int single_obs_dim = 54;
    const int policy_history_length = 5;
    const int observation_dim = 3 + single_obs_dim * policy_history_length;
    const int action_dim = 16;
    float agent_timestep = 0.02;
    float current_time;
    bool is_fallen = true;

    VecXf joint_pos_rl = VecXf(action_dim);  // in policy order
    VecXf joint_vel_rl = VecXf(action_dim);  // in policy order

    const std::string policy_path_;

    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;
    float clip_actions_ = 100.0;
    float clip_observations_ = 100.0;
    VecXf imu_w_eigen, base_acc_eigen, motor_p_eigen, motor_v_eigen,
          current_action_eigen, last_action_eigen, current_observation_, projected_gravity,
          tmp_action_eigen;
    VecXf single_observation_;
    std::deque<VecXf> obs_history_;
    bool first_frame_ = true;
    bool history_newest_first_ = false;
    bool obs_debug_enabled_ = false;
    int obs_debug_prints_remaining_ = 10;

    RobotAction robot_action;
    std::vector<std::string> robot_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};

    std::vector<std::string> policy_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
        "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
    };

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5};

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};

    Ort::Env env_;
    std::vector<int> robot2policy_idx, policy2robot_idx;

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};
    VecXf command;
    Ort::MemoryInfo memory_info{nullptr};
    const std::array<int64_t, 2> input_observationShape = {1, observation_dim};

    float time_step = 0.;
    int stop_count = 1000;

public:
    M20PolicyRunner(const std::string& policy_name, const std::string& policy_path)
        : PolicyRunnerBase(policy_name),
          policy_path_(policy_path),
          env_(ORT_LOGGING_LEVEL_WARNING, "M20PolicyRunner"),
          session_options_{},
          session_{nullptr},
          memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
        if (const char* history_env = std::getenv("M20_HISTORY_NEWEST_FIRST")) {
            history_newest_first_ = std::string(history_env) != "0";
        }
        if (const char* debug_env = std::getenv("M20_DEBUG_OBS")) {
            obs_debug_enabled_ = std::string(debug_env) != "0";
        }

        dof_default_eigen_policy.setZero(action_dim);
        dof_default_eigen_robot.setZero(action_dim);
        dof_default_eigen_policy << 0.0, -0.6, 1.0,
                                    0.0, -0.6, 1.0,
                                    0.0, 0.6, -1.0,
                                    0.0, 0.6, -1.0,
                                    0.0, 0.0, 0.0, 0.0;
        dof_default_eigen_robot << 0.0, -0.6, 1.0, 0.0,
                                   0.0, -0.6, 1.0, 0.0,
                                   0.0, 0.6, -1.0, 0.0,
                                   0.0, 0.6, -1.0, 0.0;
        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(4);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "Model file not found: " << policy_path_ << std::endl;
            throw std::runtime_error("Model file missing");
        }

        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        kp_ = Vec4f(80, 80, 80, 0.).replicate(4, 1);
        kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);

        robot2policy_idx = generate_permutation(robot_order, policy_order);
        policy2robot_idx = generate_permutation(policy_order, robot_order);

        robot_action.kp = kp_;
        robot_action.kd = kd_;
        robot_action.tau_ff = VecXf::Zero(motor_num);
        robot_action.goal_joint_pos = VecXf::Zero(motor_num);
        robot_action.goal_joint_vel = VecXf::Zero(motor_num);

        current_observation_.setZero(observation_dim);
        single_observation_.setZero(single_obs_dim);
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        current_action_eigen.setZero(action_dim);

        memory_info = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    }

    ~M20PolicyRunner() override = default;

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to,
        int default_index = 0) {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < from.size(); ++i) {
            idx_map[from[i]] = i;
        }

        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) {
                perm.push_back(it->second);
            } else {
                perm.push_back(default_index);
            }
        }

        return perm;
    }

    void DisplayPolicyInfo() {}

    void MaybeDebugObservation(const Vec3f& command, const Vec3f& projected_gravity) {
        if (!obs_debug_enabled_ || obs_debug_prints_remaining_ <= 0 || obs_history_.empty()) {
            return;
        }

        std::cout << "[M20 obs] order=" << (history_newest_first_ ? "newest->oldest" : "oldest->newest")
                  << " cmd=" << command.transpose()
                  << " projected_gravity=" << projected_gravity.transpose()
                  << " oldest_gravity=" << obs_history_.front().segment(3, 3).transpose()
                  << " latest_gravity=" << obs_history_.back().segment(3, 3).transpose()
                  << std::endl;
        --obs_debug_prints_remaining_;
    }

    void OnEnter() {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);
        obs_history_.clear();
        first_frame_ = true;
    }

    VecXf Onnx_infer(VecXf current_observation) {
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            current_observation.data(),
            current_observation.size(),
            input_observationShape.data(),
            input_observationShape.size());

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(input_tensor));
        auto outputs = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_,
            inputs.data(),
            1,
            output_names_,
            1);

        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, action_dim);
        return VecXf(action_map);
    }

    RobotAction getRobotAction(const RobotBasicState& ro, const UserCommand& uc) {
        Vec3f base_omgea = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        // joint_pos in policy order; wheel position channels stay zero.
        for (int i = 0; i < action_dim; ++i) {
            joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
        }
        joint_pos_rl.segment(12, 4).setZero();
        joint_pos_rl -= dof_default_eigen_policy;

        // joint_vel in the same policy order as training.
        for (int i = 0; i < action_dim; ++i) {
            joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
        }

        single_observation_ << base_omgea,
                               projected_gravity,
                               last_action_eigen,
                               joint_pos_rl,
                               joint_vel_rl;

        if (first_frame_) {
            obs_history_.clear();
            for (int i = 0; i < policy_history_length; ++i) {
                obs_history_.push_back(single_observation_);
            }
            first_frame_ = false;
        } else {
            obs_history_.pop_front();
            obs_history_.push_back(single_observation_);
        }

        current_observation_.head(3) = command;
        for (int i = 0; i < policy_history_length; ++i) {
            const int src_idx = history_newest_first_ ? (policy_history_length - 1 - i) : i;
            current_observation_.segment(3 + i * single_obs_dim, single_obs_dim) = obs_history_[src_idx];
        }

        MaybeDebugObservation(command, projected_gravity);

        current_observation_ =
            current_observation_.cwiseMax(-clip_observations_).cwiseMin(clip_observations_);

        current_action_eigen = Onnx_infer(current_observation_);
        last_action_eigen = current_action_eigen.cwiseMax(-clip_actions_).cwiseMin(clip_actions_);

        for (int i = 0; i < action_dim; ++i) {
            tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);
            tmp_action_eigen(i) *= action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;

        for (int i = 0; i < 4; ++i) {
            robot_action.goal_joint_pos.segment(i * 4, 3) = tmp_action_eigen.segment(i * 4, 3);
            robot_action.goal_joint_vel(i * 4 + 3) = tmp_action_eigen(i * 4 + 3);
        }

        ++run_cnt_;
        ++time_step;
        return robot_action;
    }

    void setDefaultJointPos(const VecXf& pos) {
        dof_pos_default_.setZero(motor_num);
        for (int i = 0; i < motor_num; ++i) {
            dof_pos_default_(i) = pos(i);
        }
    }

    double getCurrentTime() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    }
};

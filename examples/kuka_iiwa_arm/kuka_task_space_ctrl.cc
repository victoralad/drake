/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/math/rigid_transform.h"

DEFINE_double(x, 0., "x coordinate to move to");
DEFINE_double(y, 0., "y coordinate to move to");
DEFINE_double(z, 0., "z coordinate to move to");
DEFINE_double(roll, 0., "target roll about world x axis for end effector");
DEFINE_double(pitch, 0., "target pitch about world y axis for end effector");
DEFINE_double(yaw, 0., "target yaw about world z axis for end effector");

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(multibody::MultibodyPlant<double>& plant, const multibody::ModelInstanceIndex iiwa_instance)
      : plant_(&plant), iiwa_instance_(iiwa_instance) {
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
    // Ensure that a status message is received before initializing robot parameters.
    while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }
    InitDynamicParam();
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = kNumJoints;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      
      UpdateDynamicParam();
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      cur_time_us = iiwa_status_.utime;

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }

        GetDesiredEEPose(start_time_us, cur_time_us);

        // Compute pose and velocity errors.
        Eigen::VectorXd error_ee_pose = desired_ee_pose_ - ee_pose_;
        Eigen::VectorXd error_velocity = desired_ee_velocity_ - ee_velocity_;

        double radius = 0.01;
        double distSq = 0;
        for (int i = 0; i < 3; i++) {
          distSq += error_ee_pose[i] * error_ee_pose[i];
        }

        std::cout << "bbbbbbbbbbbbb" << std::endl;


        do {
          std::cout << "--------------------------" << std::endl;
          UpdateDynamicParam();
          // Compute pose and velocity errors.
          error_ee_pose = desired_ee_pose_ - ee_pose_;
          error_velocity = desired_ee_velocity_ - ee_velocity_;
          distSq = 0;
          for (int i = 0; i < 3; i++) {
            distSq += error_ee_pose[i] * error_ee_pose[i];
          }
          // Compute control torques.
          Eigen::VectorXd cartesian_force = Eigen::VectorXd::Zero(6);
          cartesian_force = Kp_ * error_ee_pose + Kv_ * error_velocity;
          
          std::cout << " aaaa " << sqrt(distSq) << "  " << radius << std::endl;
          // std::cout << "desired   actual " << std::endl;
          // for (int i = 0; i < 6; ++i) {
          //   std::cout << desired_ee_pose_[i] << "      " << ee_pose_[i] << std::endl;
          // }
          std::cout << cartesian_force << std::endl;

          Eigen::VectorXd joint_torque_cmd = Eigen::VectorXd::Zero(7);
          joint_torque_cmd = Jq_V_WE_.transpose() * cartesian_force + coriolis_;
        
          iiwa_command.utime = iiwa_status_.utime;

          for (int joint = 0; joint < kNumJoints; joint++) {
            iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
            iiwa_command.joint_torque[joint] = joint_torque_cmd[joint];
            // iiwa_command.joint_torque[joint] = 0.0;
            // iiwa_command.joint_position[joint] = temp_q_[joint];
          }
          lcm_.publish(kLcmCommandChannel, &iiwa_command);

        } while (sqrt(distSq) > radius);

        /*
        // While the hyperplane distance between the current pose of the end effector and the current desired pose
        // is greater than radius.
        while (sqrt(distSq) > radius) {
          UpdateDynamicParam();
          std::cout << " aaaa " << sqrt(distSq) << "  " << radius << std::endl;
          
          // Compute pose and velocity errors.
          error_ee_pose = desired_ee_pose_ - ee_pose_;
          error_velocity = desired_ee_velocity_ - ee_velocity_;
          distSq = 0;
          for (int i = 0; i < 3; i++) {
            distSq += error_ee_pose[i] * error_ee_pose[i];
          }
          // Compute control torques.
          Eigen::VectorXd cartesian_force = Eigen::VectorXd::Zero(6);
          cartesian_force = Kp_ * error_ee_pose + Kv_ * error_velocity;
          
          // std::cout << "desired   actual " << std::endl;
          // for (int i = 0; i < 6; ++i) {
          //   std::cout << desired_ee_pose_[i] << "      " << ee_pose_[i] << std::endl;
          // }

          std::cout << cartesian_force << std::endl;

          Eigen::VectorXd joint_torque_cmd = Eigen::VectorXd::Zero(7);
          joint_torque_cmd = Jq_V_WE_.transpose() * cartesian_force + coriolis_;
        
          iiwa_command.utime = iiwa_status_.utime;

          for (int joint = 0; joint < kNumJoints; joint++) {
            iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
            iiwa_command.joint_torque[joint] = joint_torque_cmd[joint];
            // iiwa_command.joint_torque[joint] = 0.0;
          }
          std::cout << "--------------------------" << std::endl;

        
        
          lcm_.publish(kLcmCommandChannel, &iiwa_command);
        }
        */
      }
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t* plan) {
    std::cout << "New plan received." << std::endl;
    if (iiwa_status_.utime == -1) {
      std::cout << "Discarding plan, no status message received yet"
                << std::endl;
      return;
    } else if (plan->num_states < 2) {
      std::cout << "Discarding plan, Not enough knot points." << std::endl;
      return;
    }

    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints, 1));
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (!plant_->HasJointNamed(state.joint_name[j])) {
          continue;
        }
        const multibody::Joint<double>& joint =
            plant_->GetJointByName(state.joint_name[j]);
        DRAKE_DEMAND(joint.num_positions() == 1);
        const int idx = joint.position_start();
        DRAKE_DEMAND(idx < kNumJoints);

        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(iiwa_status_.utime != -1);
          knots[0](idx, 0) = iiwa_status_.joint_position_commanded[j];

        } else {
          knots[i](idx, 0) = state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      std::cout << knots[i] << std::endl;
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
    plan_.reset(new PiecewisePolynomial<double>(
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            input_time, knots, knot_dot, knot_dot)));
    ++plan_number_;
  }

  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t*) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  void InitDynamicParam() {
    context_ = plant_->CreateDefaultContext();
    ee_link_ = "iiwa_link_7";
    frame_E_ = &plant_->GetBodyByName(ee_link_).body_frame();

    iiwa_q_ = Eigen::VectorXd::Zero(iiwa_status_.num_joints); // Joint positions.
    iiwa_qdot_ = Eigen::VectorXd::Zero(iiwa_status_.num_joints); // Joint velocities.
    temp_q_ = Eigen::VectorXd::Zero(iiwa_status_.num_joints);

    int nv = plant_->num_velocities();
    M_ = Eigen::MatrixXd::Zero(nv, nv);
    coriolis_ = Eigen::VectorXd::Zero(nv);

    int task_dim = 6; // Dimenstion of task space.
    desired_ee_pose_ = Eigen::VectorXd::Zero(task_dim);
    desired_ee_pose_[2] = 1.0; // This is set so that if all else fails, robot tries to move the EE to a feasible pose.
    desired_ee_velocity_ = Eigen::VectorXd::Zero(task_dim);
    ee_pose_ = Eigen::VectorXd::Zero(task_dim);
    ee_velocity_ = Eigen::VectorXd::Zero(task_dim);
    Jq_V_WE_ = Eigen::MatrixXd::Zero(task_dim, nv);
    Kp_ = Eigen::MatrixXd::Identity(task_dim, task_dim);
    Kv_ = Eigen::MatrixXd::Identity(task_dim, task_dim);
    for (int i = 0; i < 3; i++) {
      Kp_(i, i) = 15.0;
      Kp_(i+3, i+3) = 1.5;
      Kv_(i, i) = 2.0;
      Kv_(i+3, i+3) = 0.5;
    }

  }

  void GetDesiredEEPose(int64_t start_time_us, int64_t cur_time_us) {
    const double cur_traj_time_s =
        static_cast<double>(cur_time_us - start_time_us) / 1e6;
    const auto desired_next_position = plan_->value(cur_traj_time_s);
    const auto desired_next_velocity = plan_->derivative().value(cur_traj_time_s);

    const multibody::MultibodyPlant<double>* plant = plant_;
    std::unique_ptr<systems::Context<double>> plan_context = plant->CreateDefaultContext();
    Eigen::VectorXd iiwa_q = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd iiwa_qdot = Eigen::VectorXd::Zero(7);
    for (int joint = 0; joint < kNumJoints; joint++) {
      iiwa_q[joint] = desired_next_position(joint);
      iiwa_qdot[joint] = desired_next_velocity(joint);
      temp_q_[joint] = desired_next_position(joint);
    }
    plant->SetPositions(plan_context.get(), iiwa_instance_, iiwa_q);
    plant->SetVelocities(plan_context.get(), iiwa_instance_, iiwa_q);

    // Get intermediate end effector goal pose.
    math::RigidTransform<double> ee_link_pose_obj = plant->EvalBodyPoseInWorld(*plan_context, plant->GetBodyByName(ee_link_));
    desired_ee_pose_.head(3) = ee_link_pose_obj.translation();
    const math::RollPitchYaw<double> rpy(ee_link_pose_obj.rotation());
    desired_ee_pose_.tail(3) = rpy.vector();

    // Get intermediate end effector velocity.
    multibody::SpatialVelocity<double> ee_link_velocity_obj = plant_->EvalBodySpatialVelocityInWorld(*context_, plant_->GetBodyByName(ee_link_));
    desired_ee_velocity_.head(3) = ee_link_velocity_obj.translational();
    desired_ee_velocity_.tail(3) = ee_link_velocity_obj.rotational();
  }

  void UpdateDynamicParam() {
    
    for (int i = 0; i < iiwa_status_.num_joints; i++) {
      iiwa_q_[i] = iiwa_status_.joint_position_measured[i];
      iiwa_qdot_[i] = iiwa_status_.joint_velocity_estimated[i];
    }
    
    // Update context.
    plant_->SetPositions(context_.get(), iiwa_instance_, iiwa_q_);
    plant_->SetVelocities(context_.get(), iiwa_instance_, iiwa_qdot_);
    

    // Calculate mass matrix.
    plant_->CalcMassMatrix(*context_, &M_);

    // Get end effector pose.
    ee_link_pose_obj_ = plant_->EvalBodyPoseInWorld(*context_, plant_->GetBodyByName(ee_link_));
    ee_pose_.head(3) = ee_link_pose_obj_.translation();
    const math::RollPitchYaw<double> rpy(ee_link_pose_obj_.rotation());
    ee_pose_.tail(3) = rpy.vector();

    // Get end effector velocity.
    ee_link_velocity_obj_ = plant_->EvalBodySpatialVelocityInWorld(*context_, plant_->GetBodyByName(ee_link_));
    ee_velocity_.head(3) = ee_link_velocity_obj_.translational();
    ee_velocity_.tail(3) = ee_link_velocity_obj_.rotational();
    
    // Calculate Jacobian.
    Eigen::MatrixXd Jq_V_WE(6, plant_->num_positions());
    plant_->CalcJacobianSpatialVelocity(*context_, multibody::JacobianWrtVariable::kQDot,
                                        *frame_E_, Eigen::Vector3d::Zero() /* p_EQi */, plant_->world_frame(),
                                        plant_->world_frame(), &Jq_V_WE);
    // Place translation before rotation in jacobian matrix.
    Jq_V_WE_.topRows<3>() = Jq_V_WE.bottomRows<3>();
    Jq_V_WE_.bottomRows<3>() = Jq_V_WE.topRows<3>();

    // Calculate Coriolis forces.
    plant_->CalcBiasTerm(*context_, &coriolis_);
    
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>* plant_;
  lcmt_iiwa_status iiwa_status_;
  std::unique_ptr<systems::Context<double>> context_;
  Eigen::MatrixXd M_; // Mass matrix.
  math::RigidTransform<double> ee_link_pose_obj_;
  multibody::SpatialVelocity<double> ee_link_velocity_obj_;
  std::string ee_link_;
  Eigen::VectorXd iiwa_q_; // Joint positions.
  Eigen::VectorXd iiwa_qdot_; // Joint velocities.
  const multibody::Frame<double>* frame_E_; // End effector frame.
  Eigen::MatrixXd Jq_V_WE_; // Jacobian matrix.
  Eigen::VectorXd coriolis_; // Coriolis vector.
  Eigen::VectorXd desired_ee_pose_;
  Eigen::VectorXd desired_ee_velocity_;
  Eigen::VectorXd ee_pose_;
  Eigen::VectorXd ee_velocity_;
  Eigen::MatrixXd Kp_; // Stiffness gain matrix.
  Eigen::MatrixXd Kv_; // Damping gain matrix.
  const multibody::ModelInstanceIndex iiwa_instance_; // Arm instance (does not include the gripper).
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  Eigen::VectorXd temp_q_; // Placeholder for joint positions to implement position control.

};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  auto iiwa_instance = multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7/"
                          "iiwa7_with_box_collision.sdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("iiwa_link_0").body_frame());
  
  // ------------------------- Add gripper --------------------------------------
  std::string sdf_path_shunk = FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf"
          "/schunk_wsg_50_with_tip.sdf");
  const multibody::Frame<double>& iiwa_link7 = plant.GetBodyByName("iiwa_link_7").body_frame();
  const math::RigidTransform<double> X_7G(math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.075));
  const multibody::ModelInstanceIndex new_model =
      multibody::Parser(&plant).AddModelFromFile(sdf_path_shunk, "gripper");
  const auto& child_frame = plant.GetFrameByName("body", new_model);
  plant.WeldFrames(iiwa_link7, child_frame, X_7G);
  // ----------------------------------------------------------------------------

  plant.Finalize();

  RobotPlanRunner runner(plant, iiwa_instance);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::do_main();
}

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
    // Ensure that a status message is received before initializing robot parameters.
    while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }
    InitDynamicParam();
    std::cout << "aaaaaaaaaaaaaaa" << std::endl;
  }

  void Run() {
    int64_t cur_time_us = -1;
    // int64_t start_time_us = -1;

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

      iiwa_command.utime = iiwa_status_.utime;

      // Compute control torques.
      Eigen::VectorXd cartesian_force = Eigen::VectorXd::Zero(7);
      cartesian_force = Kp * (desired_ee_pose_ - ee_pose_) + Kv * (desired_ee_velocity_ - ee_velocity_);
      
      std::cout << "desired   actual " << std::endl;
      for (int i = 0; i < 6; ++i) {
        std::cout << desired_ee_pose_[i] << "      " << ee_pose_[i] << std::endl;
      }

      Eigen::VectorXd joint_torque_cmd = Eigen::VectorXd::Zero(7);
      joint_torque_cmd = Jq_V_WE_.transpose() * cartesian_force + coriolis_;
     

      for (int joint = 0; joint < kNumJoints; joint++) {
        iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
        iiwa_command.joint_torque[joint] = joint_torque_cmd[joint];
        // iiwa_command.joint_torque[joint] = 0.0;
      }
      std::cout << "--------------------------" << std::endl;
      
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void InitDynamicParam() {
    context_ = plant_->CreateDefaultContext();
    ee_link_ = "iiwa_link_7";
    frame_E_ = &plant_->GetBodyByName(ee_link_).body_frame();

    iiwa_q_ = Eigen::VectorXd::Zero(iiwa_status_.num_joints); // Joint positions.
    iiwa_qdot_ = Eigen::VectorXd::Zero(iiwa_status_.num_joints); // Joint velocities.

    int nv = plant_->num_velocities();
    M_ = Eigen::MatrixXd::Zero(nv, nv);
    coriolis_ = Eigen::VectorXd::Zero(nv);

    int task_dim = 6; // Dimenstion of task space.
    desired_ee_pose_ = Eigen::VectorXd::Zero(task_dim);
    desired_ee_velocity_ = Eigen::VectorXd::Zero(task_dim);
    ee_pose_ = Eigen::VectorXd::Zero(task_dim);
    ee_velocity_ = Eigen::VectorXd::Zero(task_dim);
    Kp = Eigen::MatrixXd::Zero(task_dim, task_dim);
    Kv = Eigen::MatrixXd::Zero(task_dim, task_dim);
    Kp << 5.0, 5.0, 5.0, 2.5, 2.5, 2.5;
    Kv << 0.2, 0.2, 0.2, 0.05, 0.05, 0.05;
    Jq_V_WE_ = Eigen::MatrixXd::Zero(task_dim, nv);

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

    // Get desired end effector pose (and velocities).
    desired_ee_pose_ << FLAGS_x, FLAGS_y, FLAGS_z, FLAGS_roll, FLAGS_pitch, FLAGS_yaw;
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
  Eigen::MatrixXd Kp; // Stiffness gain matrix.
  Eigen::MatrixXd Kv; // Damping gain matrix.
  const multibody::ModelInstanceIndex iiwa_instance_; // Arm instance (does not include the gripper).

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

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
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    
    context_ = plant_.CreateDefaultContext();
    ee_link_ = "iiwa_link_7";
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
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      cur_time_us = iiwa_status_.utime;

      iiwa_command.utime = iiwa_status_.utime;

      for (int joint = 0; joint < kNumJoints; joint++) {
        iiwa_command.joint_position[joint] = iiwa_status_.joint_position_measured[joint];
        iiwa_command.joint_torque[joint] = 0.0;
      }
      std::cout << "--------------------------" << std::endl;
      // std::cout << FLAGS_x << " " << FLAGS_y << " " << FLAGS_yaw << std::endl;

      SetDynamicParam();
      // std::cout << M_ << std::endl;
      

      lcm_.publish(kLcmCommandChannel, &iiwa_command);
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void SetDynamicParam() {
    Eigen::VectorXd iiwa_q(iiwa_status_.num_joints); // Joint positions.
    Eigen::VectorXd iiwa_qdot(iiwa_status_.num_joints); // Joint velocities.
    iiwa_q_ = iiwa_q;
    iiwa_qdot_ = iiwa_qdot;
    for (int i = 0; i < iiwa_status_.num_joints; i++) {
      iiwa_q_[i] = iiwa_status_.joint_position_measured[i];
      iiwa_qdot_[i] = iiwa_status_.joint_velocity_estimated[i];
    }

    // Update context.
    plant_.SetPositions(context_.get(), iiwa_q_);
    plant_.SetVelocities(context_.get(), iiwa_qdot_);
    int nv = plant_.num_velocities();

    // Calculate mass matrix.
    Eigen::MatrixXd M(nv, nv);
    M_ = M;
    plant_.CalcMassMatrix(*context_, &M_);

    // Get end effector pose and velocity.
    ee_link_pose_ = plant_.EvalBodyPoseInWorld(*context_, plant_.GetBodyByName(ee_link_));
    ee_link_velocity_ = plant_.EvalBodySpatialVelocityInWorld(*context_, plant_.GetBodyByName(ee_link_));
    // std::cout << ee_link_pose_.translation() << std::endl;
    // const math::RollPitchYaw<double> rpy(ee_link_pose_.rotation());
    // std::cout << rpy.vector() << std::endl;
    
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  lcmt_iiwa_status iiwa_status_;
  std::unique_ptr<systems::Context<double>> context_;
  Eigen::MatrixXd M_; // Mass matrix.
  math::RigidTransform<double> ee_link_pose_;
  multibody::SpatialVelocity<double> ee_link_velocity_;
  std::string ee_link_;
  Eigen::VectorXd iiwa_q_; // Joint positions.
  Eigen::VectorXd iiwa_qdot_; // Joint velocities.

};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  RobotPlanRunner runner(plant);
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

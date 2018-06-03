#include "torque_calculator.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <assert.h>

namespace {
  VectorXd ComputeTorque(const RigidBodyTree<double>& tree, const VectorXd& q,
                         const VectorXd& v, const VectorXd& vd_d) {
    // Compute the expected torque.
    KinematicsCache<double> cache = tree.doKinematics(q, v);
    eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                    drake::TwistVector<double>>
        f_ext;

    return tree.massMatrix(cache) * vd_d + tree.dynamicsBiasTerm(cache, f_ext);
  }

  std::string exec(const char* cmd) {
      std::array<char, 128> buffer;
      std::string result;
      std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
      if (!pipe) throw std::runtime_error("popen() failed!");
      while (!feof(pipe.get())) {
          if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
              result += buffer.data();
      }
      return result;
  }
}

void TorqueCalculator::Init(std::unique_ptr<RigidBodyTree<double>> tree,
          bool pure_gravity_compensation) {
  tree_ = std::move(tree);
  inverse_dynamics_ = make_unique<InverseDynamics<double>>(
      *tree_, pure_gravity_compensation /* pure gravity compensation mode */);
  context_ = inverse_dynamics_->CreateDefaultContext();
  output_ = inverse_dynamics_->AllocateOutput(*context_);

  // Checks that the number of input ports in the Gravity Compensator system
  // and the Context are consistent.
  if (pure_gravity_compensation) {
    assert(inverse_dynamics_->get_num_input_ports() == 1);
    assert(context_->get_num_input_ports() == 1);
  } else {
    assert(inverse_dynamics_->get_num_input_ports() == 2);
    assert(context_->get_num_input_ports() == 2);
  }

  // Checks that no state variables are allocated in the context.
  assert(context_->get_continuous_state().size() == 0);
  
  // Checks that the number of output ports in the Gravity Compensator system
  // and the SystemOutput are consistent.
  assert(output_->get_num_ports() == 1);
  assert(inverse_dynamics_->get_num_output_ports() == 1);
}

void TorqueCalculator::CheckGravityTorque(const Eigen::VectorXd& position) {
  CheckTorque(position, VectorXd::Zero(tree_->get_num_velocities()),
              VectorXd::Zero(tree_->get_num_velocities()));
}

void TorqueCalculator::CheckTorque(const Eigen::VectorXd& position,
                 const Eigen::VectorXd& velocity,
                 const Eigen::VectorXd& acceleration_desired) {
  // desired acceleration.
  VectorXd vd_d = VectorXd::Zero(tree_->get_num_velocities());
  if (!inverse_dynamics_->is_pure_gravity_compenstation()) {
    vd_d = acceleration_desired;
  }

  auto state_input = make_unique<BasicVector<double>>(
      tree_->get_num_positions() + tree_->get_num_velocities());
  state_input->get_mutable_value() << position, velocity;
  context_->FixInputPort(
      inverse_dynamics_->get_input_port_estimated_state().get_index(),
      std::move(state_input));

  if (!inverse_dynamics_->is_pure_gravity_compenstation()) {
    auto vd_input =
        make_unique<BasicVector<double>>(tree_->get_num_velocities());
    vd_input->get_mutable_value() << vd_d;
    context_->FixInputPort(
        inverse_dynamics_->get_input_port_desired_acceleration().get_index(),
        std::move(vd_input));
  }

  // Hook input of the expected size.
  inverse_dynamics_->CalcOutput(*context_, output_.get());

  // Compute the expected torque.
  VectorXd expected_torque = ComputeTorque(*tree_, position, velocity, vd_d);

  // Checks the expected and computed gravity torque.
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  // EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
  //                             1e-10, MatrixCompareType::absolute));
  std::cout << "expected_torque: " << expected_torque << "\n";
  std::cout << "output_vector->get_value(): " << output_vector->get_value() << "\n";
}

void TorqueCalculator::LoadModel(double payloadWeight) {
  assert(payloadWeight >= 0);

  auto tree = std::make_unique<RigidBodyTree<double>>();

  if (payloadWeight > 0) {
    // Generate the new model file, based on payload weight
    std::string command = "cd /home/willfeng/.gazebo/models/rod_payload/ && python fill_template.py " + std::to_string(payloadWeight);
    exec(command.c_str());
    drake::parsers::sdf::AddModelInstancesFromSdfFile(
      "/home/willfeng/.gazebo/models/rod_payload/model.sdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  } else {
    drake::parsers::sdf::AddModelInstancesFromSdfFile(
      "/home/willfeng/.gazebo/models/rod_only/model.sdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  }
  Init(std::move(tree), true /* pure gravity compensation */);
}

// Tests that the expected value of the gravity compensating torque and the
// value computed by the InverseDynamics in pure gravity compensation mode
// for a given joint configuration of the KUKA IIWA Arm are identical.
void TorqueCalculator::RunGravityCompensationTest() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::FindResourceOrThrow("drake/manipulation/models/"
          "iiwa_description/urdf/iiwa14_primitive_collision.urdf"),
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree), true /* pure gravity compensation */);

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  CheckGravityTorque(robot_position);
}

void TorqueCalculator::BenchmarkCalcGravityCompensatingTorque() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::FindResourceOrThrow("drake/manipulation/models/"
          "iiwa_description/urdf/iiwa14_primitive_collision.urdf"),
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree), true /* pure gravity compensation */);

  // Init RNG
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-0.5, 0.5);

  auto velocity = VectorXd::Zero(tree_->get_num_velocities());
  // desired acceleration. 
  VectorXd vd_d = VectorXd::Zero(tree_->get_num_velocities());
  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);

  for (int i = 0; i < 10000; i++) {
    robot_position << dis(gen), dis(gen), dis(gen), dis(gen), dis(gen), dis(gen), dis(gen);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    auto state_input = make_unique<BasicVector<double>>(
        tree_->get_num_positions() + tree_->get_num_velocities());
    state_input->get_mutable_value() << robot_position, velocity;
    context_->FixInputPort(
        inverse_dynamics_->get_input_port_estimated_state().get_index(),
        std::move(state_input));

    // Hook input of the expected size.
    inverse_dynamics_->CalcOutput(*context_, output_.get());

    // Compute the expected torque.
    // VectorXd expected_torque = ComputeTorque(*tree_, robot_position, velocity, vd_d);

    // The above is equivalent to:

    // KinematicsCache<double> cache = (*tree_).doKinematics(robot_position, velocity);
    // eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

    // VectorXd expected_torque = (*tree_).massMatrix(cache) * vd_d + (*tree_).dynamicsBiasTerm(cache, f_ext);

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    // EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
    //                             1e-10, MatrixCompareType::absolute));
    // std::cout << "expected_torque: " << expected_torque << "\n";
    // std::cout << "output_vector->get_value(): " << output_vector->get_value() << "\n";
    
    std::chrono::duration<double> diff = high_resolution_clock::now() - t1;
    std::cout << "Duration: " << diff.count() * 1000 << " ms\n";
  }
}

double TorqueCalculator::CalcGravityCompensatingTorque(double angleFromPerpendicularToGround) {    
  auto velocity = VectorXd::Zero(tree_->get_num_velocities());
  // desired acceleration. 
  VectorXd vd_d = VectorXd::Zero(tree_->get_num_velocities());
  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(1);

  robot_position << angleFromPerpendicularToGround;

  auto state_input = make_unique<BasicVector<double>>(
      tree_->get_num_positions() + tree_->get_num_velocities());

  state_input->get_mutable_value() << robot_position, velocity;
  context_->FixInputPort(
      inverse_dynamics_->get_input_port_estimated_state().get_index(),
      std::move(state_input));

  inverse_dynamics_->CalcOutput(*context_, output_.get());

  // return output_->get_vector_data(0)->get_value(); // This returns the whole vector (values for all joints). Use it when needed.
  return output_->get_vector_data(0)->GetAtIndex(0);
}

// int main(int argc, char** argv) {
//   drake::systems::controllers::TorqueCalculator calculator;
//   calculator.LoadModel(0.0);
//   // // Init RNG
//   // std::random_device rd;  //Will be used to obtain a seed for the random number engine
//   // std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
//   // std::uniform_real_distribution<> dis(-0.5, 0.5);

//   // dis(gen) -> get random number

//   // high_resolution_clock::time_point t1 = high_resolution_clock::now();

//   calculator.CalcGravityCompensatingTorque(1.57);

//   // std::chrono::duration<double> diff = high_resolution_clock::now() - t1;
//   // std::cout << "Duration: " << diff.count() * 1000 << " ms\n";
// }

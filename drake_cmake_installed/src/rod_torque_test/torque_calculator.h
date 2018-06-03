#ifndef _torque_calculator_h
#define _torque_calculator_h

#include "drake/systems/controllers/inverse_dynamics.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <iostream>
#include <assert.h>
#include <random>
#include <chrono>
#include <math.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;
using namespace std::chrono;
using namespace drake;
using namespace drake::systems;
using namespace drake::systems::controllers;

class TorqueCalculator {
 public:
  void LoadModel(double payloadWeight);
  void RunGravityCompensationTest();
  void BenchmarkCalcGravityCompensatingTorque();
  double CalcGravityCompensatingTorque(double angleFromPerpendicularToGround);

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<InverseDynamics<double>> inverse_dynamics_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
 
 private:
  void Init(std::unique_ptr<RigidBodyTree<double>> tree,
          bool pure_gravity_compensation);
  void CheckGravityTorque(const Eigen::VectorXd& position);
  void CheckTorque(const Eigen::VectorXd& position,
                 const Eigen::VectorXd& velocity,
                 const Eigen::VectorXd& acceleration_desired);
};

#endif
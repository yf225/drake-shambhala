#include "torque_calculator.h"

int main(int argc, char** argv) {
  TorqueCalculator calculator;
  
  // // Init RNG
  // std::random_device rd;  //Will be used to obtain a seed for the random number engine
  // std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  // std::uniform_real_distribution<> dis(-0.5, 0.5);

  // dis(gen) -> get random number

  // high_resolution_clock::time_point t1 = high_resolution_clock::now();

  // double angleFromPerpendicularToGround = 1.57;

  // double rod_weight_kg = 1.105;
  // double rod_length_m = 1;
  // double g = 9.8;

  // double expected_torque_from_rod = (rod_weight_kg * g) * (rod_length_m / 2) * sin(angleFromPerpendicularToGround);

  // std::cout << "robot_position: " << angleFromPerpendicularToGround << ", " << "expected gravity torque (kg * cm): " << expected_torque_from_rod * 100 / g << "\n";

  // double payloadWeight = 0;
  // calculator.LoadModel(payloadWeight);
  // double calculated_torque = calculator.CalcGravityCompensatingTorque(angleFromPerpendicularToGround);

  // std::cout << "robot_position: " << angleFromPerpendicularToGround << ", " << "calculated torque (kg * cm): " << calculated_torque * 100 / g << "\n"; // NOTE: this is in SI unit (N * m)

  // std::chrono::duration<double> diff = high_resolution_clock::now() - t1;
  // std::cout << "Duration: " << diff.count() * 1000 << " ms\n";


  double angleFromPerpendicularToGround = 1.57;

  double rod_weight_kg = 1.105;
  double rod_length_m = 1;
  double payload_weight_kg = 1.03;
  double g = 9.8;

  double expected_torque_from_rod = (rod_weight_kg * g) * (rod_length_m / 2) * sin(angleFromPerpendicularToGround);
  double expected_torque_from_payload = (payload_weight_kg * g) * rod_length_m * sin(angleFromPerpendicularToGround);

  std::cout << "robot_position: " << angleFromPerpendicularToGround << ", " << "expected gravity torque (kg * cm): " << (expected_torque_from_rod + expected_torque_from_payload) * 100 / g << "\n";

  calculator.LoadModel(payload_weight_kg);
  double calculated_torque = calculator.CalcGravityCompensatingTorque(angleFromPerpendicularToGround);

  std::cout << "robot_position: " << angleFromPerpendicularToGround << ", " << "calculated torque (kg * cm): " << calculated_torque * 100 / g << "\n"; // NOTE: this is in SI unit (N * m)
}

// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// MLP engine class for motor output torque and current prediction.
// The motor is based on Apollo LRV.
//
// =============================================================================

#include <utility>

#include "MLPEngine.h"
#include "chrono/core/ChMathematics.h"

using namespace chrono;
using namespace chrono::vehicle;

const double rpm2rads = CH_C_PI / 30;

MLPEngine::MLPEngine(const std::string& name, const std::string& model_file, const std::string& model_file_current)
    : ChEngine(name),
      m_motor_speed(0),
      m_motor_torque(0),
      MLP_model(model_file),
      MLP_model_current(model_file_current) {}

// Set the max engine speed to the shaft is 100.0 rpm
double MLPEngine::GetMaxEngineSpeed() {
    return 100.0 * rpm2rads;
}

void MLPEngine::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChEngine::Initialize(chassis);
}

void MLPEngine::Synchronize(double time, const DriverInputs& driver_inputs, double motorshaft_speed) {
    // Clamp shaft speed to specified maximum, the unit of motor_speed is rad/s
    m_motor_speed = ChClamp(motorshaft_speed, 0.0, GetMaxEngineSpeed());

    // Motor torque is linearly interpolated by throttle value
    double throttle = driver_inputs.m_throttle;
    double m_motor_speed_to_engine = m_motor_speed / rpm2rads * 80.0;
    std::vector<double> input_to_MLP = {throttle, m_motor_speed_to_engine};

    // the mean and std of scaler of the MLP model
    std::vector<double> mean = {6.62211982e-01, 2.65447027e03};
    std::vector<double> std = {2.49578492e-01, 2.02346083e03};

    // scale the input
    for (size_t i = 0; i < input_to_MLP.size(); ++i) {
		input_to_MLP[i] = (input_to_MLP[i] - mean[i]) / std[i];
	}

    // predict the torque
	std::vector<double> output_from_MLP = MLP_model.predict(input_to_MLP);
    m_motor_torque = output_from_MLP[0] * 80.0 * 4.0;  // 80 times difference, and 4 motors
    m_motor_torque = ChClamp(m_motor_torque, 0.0, 100000.0);
}

double MLPEngine::GetEngineCurrent(const DriverInputs& driver_inputs, std::vector<double> motor_speeds) {
    bool is_any_negative = false;
    for (size_t i = 0; i < motor_speeds.size(); ++i) {
        if (motor_speeds[i] < 0.0001f) {
            is_any_negative = true;
            break;
        }
    }

    if (is_any_negative) {
        return 0.0;
    }

    double throttle = driver_inputs.m_throttle;
    for (size_t i = 0; i < motor_speeds.size(); ++i) {
        motor_speeds[i] = motor_speeds[i] * 80.0f;
    }
    // the mean and std of scaler of the MLP model
    std::vector<double> mean = {6.94128440e-01, 2.89877418e+03};
    std::vector<double> std = {2.37267360e-01, 1.95223354e+03};

    std::vector<double> motor_currents;

    // predict the current for each motor
    for (size_t i = 0; i < motor_speeds.size(); ++i) {
        std::vector<double> input_to_MLP = {throttle, motor_speeds[i]};

        // scale the input
        for (size_t j = 0; j < input_to_MLP.size(); ++j) {
            input_to_MLP[j] = (input_to_MLP[j] - mean[j]) / std[j];
        }

        std::vector<double> output_from_MLP = MLP_model_current.predict(input_to_MLP);
        motor_currents.push_back(output_from_MLP[0]);
    }

    double total_current = 0;
    for (size_t i = 0; i < motor_currents.size(); ++i) {
        total_current += motor_currents[i];
    }

    if (total_current < 0.0) {
		total_current = 0.0;
	}
    return total_current;
}

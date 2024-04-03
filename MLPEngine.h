// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// MLP engine class for motor output torque and current prediction.
// The motor is based on Apollo LRV.
//
// =============================================================================


#ifndef MLPENGINE_H
#define MLPENGINE_H

#include <utility>

#include "chrono_vehicle/ChEngine.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "MLP.h"

using namespace chrono::vehicle;

class MLPEngine : public ChEngine {
  public:
    MLPEngine(const std::string& name, const std::string& model_file, const std::string& model_file_current);
    virtual ~MLPEngine() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "MLPEngine"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motor_speed; }

    /// Return the output engine torque.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetOutputMotorshaftTorque() const override { return m_motor_torque; }

    /// Returen the total current of the engine
    double GetEngineCurrent(const DriverInputs& driver_inputs, std::vector<double> motor_speeds);

  protected:
    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed();
    

    /// Set the MLP engine model
    // virtual std::string SetMLPModel(const std::string& model_file) {}

  private:
    /// Initialize the engine system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this engine system at the current time.
    /// The engine is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_speed             ///< input transmission speed
                             ) override;

    /// Advance the state of this engine system by the specified time step.
    /// This function does nothing for this simplified engine model.
    virtual void Advance(double step) override {}

    MLP MLP_model;  ///< MLP engine model
    MLP MLP_model_current;  ///< MLP engine model for current
    double m_motor_speed;   ///< current engine speed
    double m_motor_torque;  ///< current engine torque
};

#endif
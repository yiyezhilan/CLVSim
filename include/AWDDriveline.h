// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// Class to model a driveline with four independent drive.
//
// =============================================================================

#ifndef AWDDRIVELINE_H
#define AWDDRIVELINE_H


#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
#include "chrono/core/ChMathematics.h"
#include <utility>
#include "MLP.h"

using namespace chrono;
using namespace chrono::vehicle;

class AWDDriveline : public ChDrivelineWV {
  public:
    AWDDriveline(const std::string& name, const std::string& model_file);

    virtual ~AWDDriveline() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AWDDriveline"; }

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const final override { return 2; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain system.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double driveshaft_torque            ///< input transmission torque
                             ) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

    double EvaluateTorque(double wheel_speed, const DriverInputs& driver_inputs);

    /// Return the output driveline speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to the transmission subsystem.
    virtual double GetOutputDriveshaftSpeed() const override { return m_driveshaft_speed; }

    virtual void Disconnect() override;

  private:
    MLP m_MLP_model;               ///< MLP engine model
    bool m_connected;
    double m_driveshaft_speed;               ///< output to transmisson
    std::shared_ptr<ChShaft> m_front_left;   ///< associated front left wheel axle
    std::shared_ptr<ChShaft> m_front_right;  ///< associated front right wheel axle
    std::shared_ptr<ChShaft> m_rear_left;    ///< associated rear left wheel axle
    std::shared_ptr<ChShaft> m_rear_right;   ///< associated rear right wheel axle
};

#endif  // !AWDDRIVELINE_H

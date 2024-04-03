// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// The MyWheeledVehicle class is a custom wheeled vehicle model that used
// AWDDriveline, the independent-driven wheels.
//
// =============================================================================

#ifndef MYWHEELEDVEHICLE_H
#define MYWHEELEDVEHICLE_H


#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/ChEngine.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono;
using namespace chrono::vehicle;

/// Wheeled vehicle model constructed from a JSON specification file.
class MyWheeledVehicle : public ChWheeledVehicle {
  public:
    /// Create a wheeled vehicle from the provided JSON specification file.
    /// The vehicle is added to a newly created Chrono system which uses the specified contact formulation. If
    /// indicated, an associated powertrain and tires are created (if specified in the JSON file).
    MyWheeledVehicle(const std::string& filename,
                     std::shared_ptr<ChDrivelineWV> driveline,
                     ChContactMethod contact_method = ChContactMethod::NSC,
                     bool create_powertrain = true,
                     bool create_tires = true);

    /// Create a wheeled vehicle from the provided JSON specification file.
    /// The vehicle is added to the given Chrono system. If indicated, an associated powertrain and tires are created
    /// (if specified in the JSON file).
    MyWheeledVehicle(ChSystem* system,
                   const std::string& filename,
                   std::shared_ptr<ChDrivelineWV> driveline,
                   bool create_powertrain = true,
                   bool create_tires = true);

    ~MyWheeledVehicle() {}

    virtual int GetNumberAxles() const override { return m_num_axles; }

    virtual double GetWheelbase() const override { return m_wheelbase; }
    virtual double GetMinTurningRadius() const override { return m_turn_radius; }
    virtual double GetMaxSteeringAngle() const override { return m_steer_angle; }

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(const std::string& filename,
                std::shared_ptr<ChDrivelineWV> driveline,
                bool create_powertrain,
                bool create_tires);

  private:
    int m_num_rear_chassis;                   // number of rear chassis subsystems for this vehicle
    std::vector<int> m_rearch_chassis_index;  // indexes of connected chassis (-1: main chassis, >=0: rear chassis)

    double m_wheelbase;  // vehicle wheel base

    int m_num_axles;                           // number of axles for this vehicle
    std::vector<ChVector<>> m_susp_locations;  // locations of the suspensions relative to chassis
    std::vector<ChVector<>> m_arb_locations;   // locations of the antirollbar subsystems relative to chassis
    std::vector<int> m_susp_steering_index;    // indexes of associated steering (-1: none, non-steered suspension)
    std::vector<int> m_susp_chassis_index;     // indexes of associated chassis (-1: main chassis, >=0: rear chassis)
    std::vector<int> m_susp_subchassis_index;  // indexes of associated subchassis (-1: none, connect to chassis only)

    int m_num_strs;                               // number of steering subsystems
    std::vector<ChVector<>> m_str_locations;      // locations of the steering subsystems relative to chassis
    std::vector<ChQuaternion<>> m_str_rotations;  // orientations of the steering subsystems relative to chassis
    std::vector<int> m_str_chassis_index;         // indexes of associated chassis (-1: main chassis, >=0: rear chassis)

    int m_num_subch;                            // number of subchassis subsystems
    std::vector<ChVector<>> m_subch_locations;  // locations of the subchassis subsystems relative to chassis
    std::vector<int> m_subch_chassis_index;     // indexes of associated chassis (-1: main chassis, >=0: rear chassis)

    std::vector<int> m_driven_axles;  // indexes of the driven axles

    std::vector<double> m_wheel_separations;  // wheel separations for each axle

    double m_turn_radius;  // minimum turning radius
    double m_steer_angle;  // maximum steering angle
};

#endif  // !MYWHEELEDVEHICLE_h

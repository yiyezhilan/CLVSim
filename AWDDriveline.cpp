// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// Class to model a driveline with four independent drive.
//
// =============================================================================

#include <cmath>

#include "AWDDriveline.h"

const double rpm2rads = CH_C_PI / 30;

// -----------------------------------------------------------------------------
// Construct a default 4WD simple driveline.
// -----------------------------------------------------------------------------
AWDDriveline::AWDDriveline(const std::string& name, const std::string& model_file)
    : ChDrivelineWV(name), m_connected(true), m_driveshaft_speed(0), m_MLP_model(model_file) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void AWDDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                              const ChAxleList& axles,
                              const std::vector<int>& driven_axles) {
    assert(driven_axles.size() == 2);

    // Create the driveshaft
    ChDriveline::Initialize(chassis);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_front_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_front_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);

    m_rear_left = axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT);
    m_rear_right = axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
// This function calculate the torque of a wheel based on its speed through MLP model
// -----------------------------------------------------------------------------
double AWDDriveline::EvaluateTorque(double wheel_speed, const DriverInputs& driver_inputs) {
    double throttle = driver_inputs.m_throttle;
    double m_wheel_speed_to_engine = wheel_speed / rpm2rads * 80.0;
    std::vector<double> input_to_MLP = {throttle, m_wheel_speed_to_engine};

    // the mean and std of scaler of the MLP model
    std::vector<double> mean = {6.62211982e-01, 2.65447027e03};
    std::vector<double> std = {2.49578492e-01, 2.02346083e03};

    // scale the input
    for (size_t i = 0; i < input_to_MLP.size(); ++i) {
        input_to_MLP[i] = (input_to_MLP[i] - mean[i]) / std[i];
    } 

    // predict the torque
    std::vector<double> output_from_MLP = m_MLP_model.predict(input_to_MLP);
    double torque = output_from_MLP[0] * 80.0;
    torque = ChClamp(torque, 0.0, 1000.0);
    return torque;  // 80 times difference, no 4 motor torque
}

void differentialSplit(double torque,
                       double max_bias,
                       double speed_left,
                       double speed_right,
                       double& torque_left,
                       double& torque_right) {
    double diff = std::abs(speed_left - speed_right);

    // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
    double bias = 1;
    if (diff > 0.5)
        bias = max_bias;
    else if (diff > 0.25)
        bias = 4 * (max_bias - 1) * diff + (2 - max_bias);

    // Split torque to the slow and fast wheels.
    double alpha = bias / (1 + bias);
    double slow = alpha * torque;
    double fast = torque - slow;

    if (std::abs(speed_left) < std::abs(speed_right)) {
        torque_left = slow;
        torque_right = fast;
    } else {
        torque_left = fast;
        torque_right = slow;
    }
}

// -----------------------------------------------------------------------------
// This is for AWD driveline, it drives four wheels independently
// -----------------------------------------------------------------------------
void AWDDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
    if (!m_connected)
        return;

    // Set driveshaft speed (output to transmission)
    double speed_front = 0.5 * (m_front_left->GetPos_dt() + m_front_right->GetPos_dt());
    double speed_rear = 0.5 * (m_rear_left->GetPos_dt() + m_rear_right->GetPos_dt());
    double alpha = 0.5;
    m_driveshaft_speed = alpha * speed_front + (1 - alpha) * speed_rear;

    double speed_front_left= -m_front_left->GetPos_dt();
    double speed_front_right = -m_front_right->GetPos_dt();
    double speed_rear_left = -m_rear_left->GetPos_dt();
    double speed_rear_right = -m_rear_right->GetPos_dt();

    if (driveshaft_torque != 0.0) {
        m_front_left->SetAppliedTorque(-EvaluateTorque(speed_front_left, driver_inputs));
        m_front_right->SetAppliedTorque(-EvaluateTorque(speed_front_right, driver_inputs));
        m_rear_left->SetAppliedTorque(-EvaluateTorque(speed_rear_left, driver_inputs));
        m_rear_right->SetAppliedTorque(-EvaluateTorque(speed_rear_right, driver_inputs));
    } else {
        m_front_left->SetAppliedTorque(0.0);
        m_front_right->SetAppliedTorque(0.0);
        m_rear_left->SetAppliedTorque(0.0);
        m_rear_right->SetAppliedTorque(0.0);
    }
}


// -----------------------------------------------------------------------------
// This is for simple driveline, it separates the torque to front and rear, but it restricted by motors
// -----------------------------------------------------------------------------
//void AWDDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
//    if (!m_connected)
//        return;
//
//    // Set driveshaft speed (output to transmission)
//    double speed_front = 0.5 * (m_front_left->GetPos_dt() + m_front_right->GetPos_dt());
//    double speed_rear = 0.5 * (m_rear_left->GetPos_dt() + m_rear_right->GetPos_dt());
//    double alpha = 0.5;
//    m_driveshaft_speed = alpha * speed_front + (1 - alpha) * speed_rear;
//
//    double speed_front_left = -m_front_left->GetPos_dt();
//    double speed_front_right = -m_front_right->GetPos_dt();
//    double speed_rear_left = -m_rear_left->GetPos_dt();
//    double speed_rear_right = -m_rear_right->GetPos_dt();
//
//    if (driveshaft_torque != 0.0) {
//        // Split the input torque front/back.
//        double torque_front = driveshaft_torque * alpha;
//        double torque_rear = driveshaft_torque - torque_front;
//
//        // Split the axle torques for the corresponding left/right wheels and apply
//        // them to the suspension wheel shafts.
//        double torque_left;
//        double torque_right;
//
//        DriverInputs max_throttle_inputs = driver_inputs;
//        max_throttle_inputs.m_throttle = 1.0;
//
//        differentialSplit(torque_front, 2.0, m_front_left->GetPos_dt(), m_front_right->GetPos_dt(), torque_left,
//                          torque_right);
//        auto max_torque_front_left = EvaluateTorque(speed_front_left, max_throttle_inputs);
//        auto max_torque_front_right = EvaluateTorque(speed_front_right, max_throttle_inputs);
//
//        if (std::abs(torque_left) > std::abs(max_torque_front_left)) {
//            m_front_left->SetAppliedTorque(-max_torque_front_left);
//        } else {
//            m_front_left->SetAppliedTorque(-torque_left);
//        }
//
//        if (std::abs(torque_right) > std::abs(max_torque_front_right)) {
//            m_front_right->SetAppliedTorque(-max_torque_front_right);
//        } else {
//            m_front_right->SetAppliedTorque(-torque_right);
//        }
//
//
//        differentialSplit(torque_rear, 2.0, m_rear_left->GetPos_dt(), m_rear_right->GetPos_dt(), torque_left,
//                          torque_right);
//        auto max_torque_rear_left = EvaluateTorque(speed_rear_left, max_throttle_inputs);
//        auto max_torque_rear_right = EvaluateTorque(speed_rear_right, max_throttle_inputs);
//
//        if (std::abs(torque_left) > std::abs(max_torque_rear_left)) {
//            m_rear_left->SetAppliedTorque(-max_torque_rear_left);
//        } else {
//            m_rear_left->SetAppliedTorque(-torque_left);
//        }
//        if (std::abs(torque_right) > std::abs(max_torque_rear_right)) {
//            m_rear_right->SetAppliedTorque(-max_torque_rear_right);
//        } else {
//            m_rear_right->SetAppliedTorque(-torque_right);
//        }
//
//
//    } else {
//        m_front_left->SetAppliedTorque(0.0);
//        m_front_right->SetAppliedTorque(0.0);
//        m_rear_left->SetAppliedTorque(0.0);
//        m_rear_right->SetAppliedTorque(0.0);
//    }
//}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double AWDDriveline::GetSpindleTorque(int axle, VehicleSide side) const {
    if (!m_connected)
        return 0;

    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front_left->GetAppliedTorque();
            case RIGHT:
                return -m_front_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_rear_left->GetAppliedTorque();
            case RIGHT:
                return -m_rear_right->GetAppliedTorque();
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void AWDDriveline::Disconnect() {
    m_connected = false;
}

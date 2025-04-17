// =============================================================================
// Author: Qingning Lan
// =============================================================================
//
// LRV with MLP engine, SPH terrain, FEM tire, and AWD driveline.
// All the subsystems are modeled and benchmarked based on Apollo 15 
// Lunar Roving Vehicle (LRV) and the Apollo 15 mission.
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <iostream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChNodeFEAbase.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"


#include "MLPEngine.h"
#include "AWDDriveline.h"
#include "MyWheeledVehicle.h"
#include "MonitorComponent.h"
#include "ANCFToroidalTireSPH.h"

#include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

// Output directories
const std::string out_dir = "CLVSim";
const std::string blender_dir = out_dir + "/BLENDER";
const std::string SPH_dir = out_dir + "/SPHTerrain";
const std::string particles_dir = SPH_dir + "/particles";
const std::string body_dir = out_dir + "/body";
const std::string img_dir = out_dir + "/IMG";
const std::string mesh_dir = out_dir + "/MESH";
const std::string MESH_CONNECTIVITY = out_dir + "/Flex_MESH.vtk";

// Post-processing output
bool blender_output = false;   // not working
bool create_fender = true;     // create fender or not
bool debug_mode = false;        // debug mode
bool moon_mode = true;          // moon mode
bool flexible_wheel = false;    // use flexible wheel or not
bool print_particles = true;   // print particles or not, it's nessary to render animation in Blender
bool flat_terrain = false;      // flat terrain or not
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// parameters for SPH terrain
double target_speed = 2.77777;           // 10 km/h
double inispacing = 0.02; 
double tend = 32;
double step_size = 4.0e-4;
ChVector<> active_box = ChVector<>(5.0, 2.0, 5.0);
ChVector<> gravity(0, 0, -9.81);
// ===================================================================================================================

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string MLPEngineJSON() const = 0;
    virtual std::string MLPEngineCurrentJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual ChContactMethod ContactMethod() const = 0;
};

class LRV_Model : public Vehicle_Model {
  public:
    std::string LRV = vehicle::GetDataFile("LRV");
    virtual std::string ModelName() const override { return "LRV"; }
    virtual std::string VehicleJSON() const override {
        if (!moon_mode) {
            return LRV + "/vehicle/LRV_Vehicle_4WD_reduced.json";
        } else {
            return LRV + "/vehicle/LRV_Vehicle_4WD_reduced_RSDA_analysis_render_full.json";
        }
    }
    virtual std::string TireJSON() const override { return LRV + "/tire/LRV_RigidMeshTire_render.json"; }
    virtual std::string EngineJSON() const override {
        if (!moon_mode) {
            return LRV + "/powertrain/HMMWV_EngineSimple.json";
        } else {
            return LRV + "/powertrain/LRV_EngineSimpleMap.json";
        }
    }

    virtual std::string MLPEngineJSON() const override { return LRV + "/powertrain/LRV_MLPEngine.json"; }
    virtual std::string MLPEngineCurrentJSON() const override { return LRV + "/powertrain/LRV_MLPEngine_consumption.json"; }

    virtual std::string TransmissionJSON() const override {
        if (!moon_mode) {
            return LRV + "/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json";
        } else {
            return LRV + "/powertrain/LRV_AutomaticTransmissionSimpleMap.json";
        }
    }
    std::string FenderPathRearLeft() const { return LRV + "/fender_simple_left_render.obj"; }
    std::string FenderPathRearRight() const { return LRV + "/fender_simple_right_render.obj"; }
    std::string FenderPathFrontLeft() const { return LRV + "/fender_front_left_render.obj"; }
    std::string FenderPathFrontRight() const { return LRV + "/fender_front_right_render.obj"; }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

auto vehicle_model = LRV_Model();

// ===================================================================================================================

std::shared_ptr<MyWheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
void CreateFenders(std::shared_ptr<MyWheeledVehicle> vehicle, ChSystem& sysMBS, ChSystemFsi& sysFSI);
void logData(double t,
             ChSystemFsi& sysFSI,
             ChTimer timer,
             DriverInputs driver_inputs,
             std::shared_ptr<MyWheeledVehicle> vehicle);

std::shared_ptr<fea::ChMesh> CreateMeshSPH(ChSystem& sysMBS,
                   ChSystemFsi& sysFSI,
                   std::shared_ptr<MyWheeledVehicle> vehicle,
                   std::vector<std::vector<int>>& m_2D_elementsNodes_mesh,
                   std::vector<std::vector<int>>& m_NodeNeighborElement_mesh,
                   std::vector<std::shared_ptr<fea::ChLinkPointFrame>>& m_connections,
                   std::vector<std::shared_ptr<fea::ChLinkDirFrame>>& m_connectionsD);
void CreateTires(std::shared_ptr<MyWheeledVehicle>& vehicle);
void CreateWheelBCEMarkers(std::shared_ptr<MyWheeledVehicle> vehicle, ChSystemFsi& sysFSI);

// If use flexible wheel, the following process must be followed:
// First, Create vehicle;
// Second, Create Mesh and SPH markers;
// Third, Create Tires;


std::shared_ptr<fea::ChMesh> mesh;
std::vector<std::vector<int>> _2D_elementsNodes_mesh;             ///< 2D Nodes of mesh
std::vector<std::vector<int>> NodeNeighborElement_mesh;           ///< Neighbor element of mesh
std::vector<std::shared_ptr<fea::ChLinkPointFrame>> connections;  ///< tire-wheel point connections
std::vector<std::shared_ptr<fea::ChLinkDirFrame>> connectionsD;   ///< tire-wheel direction connections
    // ===================================================================================================================

int main(int argc, char* argv[]) {
    // Set the inispacing and time step
    if (flexible_wheel) {
        // if using flexible wheel, the inispacing and time step should be smaller
        std::cout << "Simulation with flexible wheel" << std::endl;
    	inispacing = 0.01;
		step_size = 1.0e-4;
    } else {
        std::cout << "Simulation with rigid wheel" << std::endl;
    }

    // Change the parameter of soil
    double density = 1550;
    double cohesion = 0.0;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    bool visualization = true;             // run-time visualization
    double visualizationFPS = 50;          // frames rendered per second (0: every frame)
    double FSIoutputFPS = 25;              // frames output per second (0: never)
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;   // render wheel BCE markers
    bool chase_cam = true;                 // chase-cam or fixed camera

    bool verbose = true;

    // Dimension of the domain
    double smalldis = 1.0e-9;
    double bxDim = 30.0 + smalldis;        // length of the terrain
    double byDim = 3.0 + smalldis;         // width of the terrain
    double bzDim = 0.8 + smalldis;

    // Create the Chrono system
    ChSystemSMC sysMBS;
    sysMBS.SetNumThreads(8,4,2);

    // Create the SPH terrain system
    CRMTerrain terrain(sysMBS, inispacing);
    terrain.SetVerbose(verbose);
    ChSystemFsi& sysFSI = terrain.GetSystemFSI();

    // Set SPH parameters and soil material properties
    if (moon_mode) {
        gravity = ChVector<>(0, 0, -1.62);
    }
    sysFSI.Set_G_acc(gravity);
    sysMBS.Set_G_acc(gravity);
    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;             // default
    mat_props.viscosity_alpha = 0.3;  // decrease the artificial viscosity
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = inispacing / 2.0;
    mat_props.friction_angle = CH_C_PI / 10;  // default
    mat_props.dilation_angle = CH_C_PI / 10;  // default
    mat_props.cohesion_coeff = 0;             // default
    mat_props.kernel_threshold = 0.8;

    sysFSI.SetElasticSPH(mat_props);
    sysFSI.SetDensity(density);
    sysFSI.SetCohesionForce(cohesion);

    sysFSI.SetActiveDomain(active_box);
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);

    sysFSI.SetOutputLength(0);

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (blender_output) {
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            std::cout << "Error creating directory " << blender_dir << std::endl;
            return 1;
        }
    }
    if (!filesystem::create_directory(filesystem::path(SPH_dir))) {
        std::cout << "Error creating directory " << SPH_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(particles_dir))) {
        std::cout << "Error creating directory " << particles_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(img_dir))) {
        std::cout << "Error creating directory " << img_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(body_dir))) {
        std::cout << "Error creating directory " << body_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(mesh_dir))) {
		std::cout << "Error creating directory " << mesh_dir << std::endl;
		return 1;
	}

    // Construct the terrain using SPH particles and BCE markers from files
    std::cout << "Create terrain..." << std::endl;
    double height = 0.4 * bzDim;
    if (flat_terrain) {
        height = 0.0;
    }
    terrain.Construct(
        vehicle::GetDataFile("terrain/height_maps/small_height_map_1_2048_cut.bmp "),  // height map // image file
        bxDim, byDim,                                                                  // length (X) and width (Y)
        {0, height},                                                                   // height range
        0.4 * bzDim,                                                                   // depth
        3,                                                                             // number of BCE layers
        ChVector<>(0, 0, 0),                                                           // patch center
        0.0,                                                                           // patch yaw rotation
        true                                                                           // side walls?
    );
    // terrain.Construct(bxDim, byDim, 0.5 * bzDim, 3, ChVector<>(0, 0, 0), 0.0, true);

    // Create vehicle
    std::cout << "Create vehicle..." << std::endl;
    ChVector<> veh_init_pos(-bxDim / 2.0 + 3.0, 0.0, 0.4 * bzDim + 0.41 + 0.02);

    if (flat_terrain) {
        veh_init_pos = ChVector<>(-bxDim / 2.0 + 3.0, 0.0, 0.5);
    }

    // ChVector<> veh_init_pos(4.0, 2.0, 0.65);
    auto vehicle = CreateVehicle(sysMBS, ChCoordsys<>(veh_init_pos, QUNIT));
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    // vehicle.GetSuspension(0)->GetBrakeBody(VehicleSide::LEFT)->AddVisualShape(chrono_types::make_shared<ChSphereShape>(0.1));
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::NONE);
    vehicle->SetTireVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisCollide(false);


    // Fender must be created before mesh, if use flexible wheel
    if (create_fender) {
        std::cout << "Create fenders..." << std::endl;
        CreateFenders(vehicle, sysMBS, sysFSI);
    }

    // Create the Mesh and SPH markers
    if (flexible_wheel) {
        std::cout << "Create flexible wheel..."<< std::endl;
        auto mesh = CreateMeshSPH(sysMBS, sysFSI, vehicle, _2D_elementsNodes_mesh, NodeNeighborElement_mesh,
                                  connections, connectionsD);
        CreateTires(vehicle);
    } else {
        std::cout << "Create rigid wheel BCE markers..."<< std::endl;
        CreateWheelBCEMarkers(vehicle, sysFSI);
    }

    // Initialize the terrain system
    terrain.Initialize();


    // Set MKL solver for MBD system
    if (flexible_wheel) {
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sysMBS.SetSolver(mkl_solver);
    }

    ChVector<> aabb_min, aabb_max;
    terrain.GetAABB(aabb_min, aabb_max);
    std::cout << "  SPH particles:     " << sysFSI.GetNumFluidMarkers() << std::endl;
    std::cout << "  Bndry BCE markers: " << sysFSI.GetNumBoundaryMarkers() << std::endl;
    std::cout << "  AABB:              " << aabb_min << "   " << aabb_max << std::endl;

    // Create driver
    std::cout << "Create path..." << std::endl;
    auto path = CreatePath(vehicle::GetDataFile("LRV/terrain/path.txt"));
    double x_max = path->getPoint(path->getNumPoints() - 2).x() - 3.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(6.0);
    driver.GetSteeringController().SetGains(1.0, 0.05, 0.0);
    driver.GetSpeedController().SetGains(1.0, 0.1, 0.00);
    // driver.GetSpeedController().SetGains(1.5, 0.2, 0.01);
    driver.Initialize();

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // Set up vehicle output
    vehicle->SetChassisOutput(true);
    vehicle->SetSuspensionOutput(0, true);
    vehicle->SetSuspensionOutput(1, true);
    vehicle->SetSteeringOutput(0, true);
    // vehicle->SetSteeringOutput(1, true);
    vehicle->SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    vehicle->ExportComponentList(out_dir + "/component_list.json");

    postprocess::ChBlender blender_exporter(vehicle->GetSystem());
    if (blender_output) {
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.SetCamera(ChVector<>(4.0, 2, 1.0), ChVector<>(0, 0, 0), 50);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }

    // Create run-time visualization
    auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI, verbose);

    std::shared_ptr<MonitorGuiComponent> monitor =
        chrono_types::make_shared<MonitorGuiComponent>(visFSI->GetVisualSystem(), MonitorGuiComponent::MonitorData());
    visFSI->GetVisualSystem().AddGuiComponent(monitor);

    visFSI->SetImageOutput(true);
    visFSI->SetImageOutputDirectory(img_dir);
    visFSI->SetTitle("Wheeled vehicle SPH deformable terrain");
    visFSI->SetSize(1280, 720);
    // visFSI->SetSize(1920, 1080);
    visFSI->AddCamera(ChVector<>(bxDim / 8, -3, 0.4), ChVector<>(bxDim / 8, 0.0, 0.25));
    visFSI->SetCameraMoveScale(0.2f);
    visFSI->EnableFluidMarkers(visualization_sph);
    visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
    visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
    visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
    visFSI->SetSPHColorCallback(
        chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb_min.z(), aabb_max.z()));
    visFSI->AttachSystem(&sysMBS);
    visFSI->Initialize();

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    ChVector<> chassis_pos = vehicle->GetChassis()->GetPos();
    ChVector<> chassis_vel = vehicle->GetChassisBody()->GetPos_dt();
    ChVector<> chassis_acc = vehicle->GetChassisBody()->GetPos_dtdt();
    auto driveline4 = std::dynamic_pointer_cast<ChDrivelineWV>(vehicle->GetDriveline());
    std::vector<int> axles = driveline4->GetDrivenAxleIndexes();
    auto speed = vehicle->GetSuspension(axles[0])->GetAxleSpeed(LEFT) * 30 / CH_C_PI;
    std::vector<double> current_motor_speed;
    auto engine = std::dynamic_pointer_cast<MLPEngine>(vehicle->GetPowertrainAssembly()->GetEngine());

    int render_steps = (visualizationFPS > 0) ? (int)std::round((1.0 / visualizationFPS) / step_size) : 1;
    int blender_steps = render_steps * 5;
    int output_steps = (FSIoutputFPS > 0) ? (int)std::round((1.0 / FSIoutputFPS) / step_size) : 1;
    double t = 0;
    int frame = 0;
    double t_delay = 1.0;
    double t_brake = 0.5;
    ChVector<> target_cam_pos = veh_init_pos;
    if (moon_mode) {
        t_brake = 2.0;
    }

    if (debug_mode) {
        auto body_list = sysMBS.Get_bodylist();
        auto link_list = sysMBS.Get_linklist();
        for (auto& body : body_list) {
            std::cout << "body name: " << body->GetName() << std::endl;
        }
        for (auto& link : link_list) {
            std::cout << "link name: " << link->GetName() << std::endl;
        }
    }

    if (x_max < veh_init_pos.x())
        x_max = veh_init_pos.x() + 0.25;

    std::cout << "Start simulation..." << std::endl;

    vehicle->LogSubsystemTypes();

    ChTimer timer;
    timer.start();
    int count_mesh = 0;
    while (t < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Stop before end of patch
        if (veh_loc.x() > (bxDim / 2 - 4.0)) {
            break;
        }

        // Set current driver inputs
        auto driver_inputs = driver.GetInputs();
        chassis_pos = vehicle->GetChassis()->GetPos();
        chassis_vel = vehicle->GetChassisBody()->GetPos_dt();
        chassis_acc = vehicle->GetChassisBody()->GetPos_dtdt();

        if (t < t_brake) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            driver_inputs.m_steering = 0;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (t - t_brake) / t_delay);
            ChClampValue(driver_inputs.m_braking, driver_inputs.m_braking, (t - t_brake) / t_delay);
            ChClampValue(driver_inputs.m_steering, driver_inputs.m_steering, (t - t_brake) / t_delay);
        }

        // Run-time visualization
        if (visualization && frame % render_steps == 0) {
            // ChVector<> cam_loc = veh_loc + ChVector<>(0, 0, 4.5);  // ChVector<>(-6, -6, 1.5);
            target_cam_pos = ChVector<>(veh_loc.x() - 0.5, veh_loc.y(), veh_init_pos.z() - 0.41-0.4);
            ChVector<> cam_loc = target_cam_pos + ChVector<>(-8, -8, 2);  // ChVector<>(-6, -6, 1.5);
            ChVector<> cam_point = target_cam_pos;
            visFSI->UpdateCamera(cam_loc, cam_point);

            monitor->m_data.time.push_back(t);
            monitor->m_data.steering.push_back(driver_inputs.m_steering);
            monitor->m_data.throttle.push_back(driver_inputs.m_throttle);
            monitor->m_data.velocity.push_back(chassis_vel.x());
            monitor->m_data.acceleration.push_back(chassis_acc.z());

            current_motor_speed.clear();
            speed = vehicle->GetSuspension(axles[0])->GetAxleSpeed(LEFT) * 30 / CH_C_PI;
            current_motor_speed.push_back(-speed);
            speed = vehicle->GetSuspension(axles[0])->GetAxleSpeed(RIGHT) * 30 / CH_C_PI;
            current_motor_speed.push_back(-speed);
            speed = vehicle->GetSuspension(axles[1])->GetAxleSpeed(LEFT) * 30 / CH_C_PI;
            current_motor_speed.push_back(-speed);
            speed = vehicle->GetSuspension(axles[1])->GetAxleSpeed(RIGHT) * 30 / CH_C_PI;
            current_motor_speed.push_back(-speed);
            
            double current = engine->GetEngineCurrent(driver_inputs, current_motor_speed);
            monitor->m_data.consumption.push_back(current + 0.444f);             // 0.111 amp for controller for each motor

            if (!visFSI->Render())
                break;
            if (blender_output && frame % output_steps == 0) {
                blender_exporter.ExportData();
                blender_exporter.SetCamera(cam_loc, cam_point, 50);
            }
        }

        if (frame % output_steps == 0) {
            if (print_particles) {
                sysFSI.PrintParticleToFile(particles_dir);
                for (int i = 1; i < sysMBS.Get_bodylist().size(); i++) {
                    auto body = sysMBS.Get_bodylist()[i];
                    ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
                    ChVector<> pos = ref_frame.GetPos();
                    ChVector<> vel = body->GetPos_dt();
                    ChQuaternion<> rot = ref_frame.GetRot();

                    std::string delim = ",";
                    // sprintf(filename, "%s/body_pos_rot_vel%d.csv", body_dir, i);
                    // std::ofstream file;
                    std::ofstream bodyfile(body_dir + "/body_pos_rot_vel" + std::to_string(i) + ".txt",
                                           std::ios_base::app);
                    if (!bodyfile.is_open()) {
                        std::cout << "Error opening body file" << std::endl;
                    }

                    if (sysMBS.GetChTime() <= 0) {
                        bodyfile << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim
                                 << "q1" << delim << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim
                                 << "Vz" << std::endl;
                    }

                    bodyfile << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                             << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim
                             << vel.x() << delim << vel.y() << delim << vel.z() << std::endl;

                    bodyfile.close();
                }
                for (int i = 1; i < sysMBS.Get_linklist().size(); i++) {
                    auto link = sysMBS.Get_linklist()[i];
                    ChVector<> pos = link->GetLinkAbsoluteCoords().pos;
                    ChVector<> vel = ChVector<>(0, 0, 0);
                    ChQuaternion<> rot = link->GetLinkAbsoluteCoords().rot;

                    std::string delim = ",";
                    // sprintf(filename, "%s/link_pos_rot_vel%d.csv", body_dir, i);
                    // std::ofstream file;
                    std::ofstream linkfile(body_dir + "/link_pos_rot_vel" + std::to_string(i) + ".txt",
                                           std::ios_base::app);
                    if (!linkfile.is_open()) {
                        std::cout << "Error opening link file" << std::endl;
                    }
                    if (sysMBS.GetChTime() <= 0) {
                        linkfile << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim
                                 << "q1" << delim << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim
                                 << "Vz" << std::endl;
                    }

                    linkfile << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                             << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim
                             << vel.x() << delim << vel.y() << delim << vel.z() << std::endl;

                    linkfile.close();
                }
            }
            if (flexible_wheel) {
                std::string filename = mesh_dir + "/wheel." + std::to_string(count_mesh++) + ".vtk ";
                auto my_mesh = sysFSI.GetFsiMesh();
                fea::ChMeshExporter::WriteFrame(my_mesh, MESH_CONNECTIVITY, filename);
            }
            logData(t, sysFSI, timer, driver_inputs, vehicle);
        }
        if (!visualization) {
            std::cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRTF() << std::endl;
        }
        // Synchronize systems
        driver.Synchronize(t);
        vehicle->Synchronize(t, driver_inputs, terrain);
        // terrain.Synchronize(t);

        // Advance system state
        driver.Advance(step_size);
        //vehicle->Advance(step_size);
        //terrain.Advance(step_size);
        sysFSI.DoStepDynamics_FSI();
        t += step_size;

        frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;
    return 0;
}

// ===================================================================================================================

std::shared_ptr<MyWheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos) {
    // Create and initialize the vehicle
    std::shared_ptr<ChDrivelineWV> driveline;
    driveline = chrono_types::make_shared<AWDDriveline>("AWDDriveline", vehicle_model.MLPEngineJSON());
    auto vehicle = chrono_types::make_shared<MyWheeledVehicle>(&sys, vehicle_model.VehicleJSON(), driveline);

    vehicle->Initialize(init_pos);

    // driveline->Initialize(vehicle->GetChassis(), vehicle->GetAxles(), m_driven_axles);

    vehicle->GetChassis()->SetFixed(false);

    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    // auto engine = ReadEngineJSON(vehicle_model.EngineJSON());
    std::shared_ptr<ChEngine> engine =
        std::make_shared<MLPEngine>("MLPEngine", vehicle_model.MLPEngineJSON(), vehicle_model.MLPEngineCurrentJSON());
    auto transmission = ReadTransmissionJSON(vehicle_model.TransmissionJSON());
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    if (!flexible_wheel) {
        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(vehicle_model.TireJSON());
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }
    }
    return vehicle;
}

void CreateTires(std::shared_ptr<MyWheeledVehicle>& vehicle) {
    auto tire_FL = chrono_types::make_shared<ANCFToroidalTireSPH>("FEMTire_FL", 0, LEFT, connections, connectionsD);
    auto tire_FR = chrono_types::make_shared<ANCFToroidalTireSPH>("FEMTire_FR", 0, RIGHT, connections, connectionsD);
    auto tire_RL = chrono_types::make_shared<ANCFToroidalTireSPH>("FEMTire_RL", 1, LEFT, connections, connectionsD);
    auto tire_RR = chrono_types::make_shared<ANCFToroidalTireSPH>("FEMTire_RR", 1, RIGHT, connections, connectionsD);

    vehicle->InitializeTire(tire_FL, vehicle->GetAxle(0)->GetWheel(LEFT), VisualizationType::MESH);
    vehicle->InitializeTire(tire_FR, vehicle->GetAxle(0)->GetWheel(RIGHT), VisualizationType::MESH);
    vehicle->InitializeTire(tire_RL, vehicle->GetAxle(1)->GetWheel(LEFT), VisualizationType::MESH);
    vehicle->InitializeTire(tire_RR, vehicle->GetAxle(1)->GetWheel(RIGHT), VisualizationType::MESH);
}

std::shared_ptr<fea::ChMesh> CreateMeshSPH(ChSystem& sysMBS,
                   ChSystemFsi& sysFSI,
                   std::shared_ptr<MyWheeledVehicle> vehicle,
                   std::vector<std::vector<int>>& m_2D_elementsNodes_mesh,
                   std::vector<std::vector<int>>& m_NodeNeighborElement_mesh,
                   std::vector<std::shared_ptr<fea::ChLinkPointFrame>>& m_connections,
                   std::vector<std::shared_ptr<fea::ChLinkDirFrame>>& m_connectionsD) {
    // Add the tire
    auto m_mesh = chrono_types::make_shared<fea::ChMesh>();
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(2300, 1.6e7, 0.3);

    // parameters for Mesh tire
    double m_rim_radius = 0.295;
    double m_wheel_radius = 0.41;
    double m_height = 0.115;
    double m_thickness = 0.005;
    int m_div_circumference = 36;
    int m_div_width = 12;
    double m_alpha = 0.15;

    {
        int TotalNumElements = m_div_circumference * m_div_width * 4;     // my_mesh->GetNelements();
        int TotalNumNodes = m_div_circumference * (m_div_width + 1) * 4;  // my_mesh->GetNnodes();
        int TotalNumLinks = 2 * m_div_circumference * 4;

        m_2D_elementsNodes_mesh.resize(TotalNumElements);
        m_NodeNeighborElement_mesh.resize(TotalNumNodes);
        m_connections.resize(TotalNumLinks);
        m_connectionsD.resize(TotalNumLinks);

        std::cout << "TotalNumElements: " << TotalNumElements << std::endl;
        std::cout << "TotalNumNodes: " << TotalNumNodes << std::endl;
        std::cout << "TotalNumLinks: " << TotalNumLinks << std::endl;


        // Create the mesh nodes.
        // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
        // and are then transformed to the global frame.
        int num_link = 0;
        for (int num_wheel = 0; num_wheel < 4; num_wheel++) {
            std::shared_ptr<ChWheel> m_wheel;
            if (num_wheel == 0) {
                m_wheel = vehicle->GetAxle(0)->GetWheel(LEFT);
            } else if (num_wheel == 1) {
                m_wheel = vehicle->GetAxle(0)->GetWheel(RIGHT);
            } else if (num_wheel == 2) {
                m_wheel = vehicle->GetAxle(1)->GetWheel(LEFT);
            } else {
                m_wheel = vehicle->GetAxle(1)->GetWheel(RIGHT);
            }
            ChFrameMoving<> wheel_frame = *(m_wheel->GetSpindle().get());

            for (int i = 0; i < m_div_circumference; i++) {
                double phi = (CH_C_2PI * i) / m_div_circumference;
                for (int j = 0; j <= m_div_width; j++) {
                    double theta = -CH_C_PI * 2.0 / 3.0 + (CH_C_PI * 4.0 / 3.0 * j) / m_div_width;

                    double x = (m_rim_radius + m_height * cos(theta)) * cos(phi);
                    double y = m_height * sin(theta);
                    double z = (m_rim_radius + m_height * cos(theta)) * sin(phi);
                    ChVector<> loc = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));

                    double nx = cos(theta) * cos(phi);
                    double ny = sin(theta);
                    double nz = cos(theta) * sin(phi);
                    ChVector<> dir = wheel_frame.TransformDirectionLocalToParent(ChVector<>(nx, ny, nz));

                    auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
                    node->SetMass(0.0);
                    m_mesh->AddNode(node);

                    // Fix the edge node on the wheel
                    if (j == 0 || j == m_div_width) {
                        m_connections[num_link] = chrono_types::make_shared<ChLinkPointFrame>();
                        m_connections[num_link]->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyzD>(node),
                                                            m_wheel->GetSpindle());
                        sysMBS.Add(m_connections[num_link]);

                        m_connectionsD[num_link] = chrono_types::make_shared<ChLinkDirFrame>();
                        m_connectionsD[num_link]->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyzD>(node),
                                                             m_wheel->GetSpindle());
                        sysMBS.Add(m_connectionsD[num_link]);
                        num_link++;
                        
                    }
                }
            }
        }

        // Element thickness
        double dz = m_thickness;

        // Create the ANCF shell elements
        int num_elem = 0;
        for (int num_wheel = 0; num_wheel < 4; num_wheel++) {
            for (int i = 0; i < m_div_circumference; i++) {
                for (int j = 0; j < m_div_width; j++) {
                    // Adjacent nodes
                    int inode0, inode1, inode2, inode3;
                    inode1 = j + i * (m_div_width + 1);
                    inode2 = j + 1 + i * (m_div_width + 1);
                    if (i == m_div_circumference - 1) {
                        inode0 = j;
                        inode3 = j + 1;
                    } else {
                        inode0 = j + (i + 1) * (m_div_width + 1);
                        inode3 = j + 1 + (i + 1) * (m_div_width + 1);
                    }

                    inode0 += m_div_circumference * (m_div_width + 1) * num_wheel;
                    inode1 += m_div_circumference * (m_div_width + 1) * num_wheel;
                    inode2 += m_div_circumference * (m_div_width + 1) * num_wheel;
                    inode3 += m_div_circumference * (m_div_width + 1) * num_wheel;

                    auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode0));
                    auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode1));
                    auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode2));
                    auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(inode3));

                    m_2D_elementsNodes_mesh[num_elem].push_back(inode0);
                    m_2D_elementsNodes_mesh[num_elem].push_back(inode1);
                    m_2D_elementsNodes_mesh[num_elem].push_back(inode2);
                    m_2D_elementsNodes_mesh[num_elem].push_back(inode3);
                    m_NodeNeighborElement_mesh[inode0].push_back(num_elem);
                    m_NodeNeighborElement_mesh[inode1].push_back(num_elem);
                    m_NodeNeighborElement_mesh[inode2].push_back(num_elem);
                    m_NodeNeighborElement_mesh[inode3].push_back(num_elem);

                    // Create the element and set its nodes.
                    auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                    element->SetNodes(node0, node1, node2, node3);

                    // Element dimensions
                    double dx = 0.5 * ((node1->GetPos() - node0->GetPos()).Length() +
                                       (node3->GetPos() - node2->GetPos()).Length());
                    double dy = 0.5 * ((node2->GetPos() - node1->GetPos()).Length() +
                                       (node3->GetPos() - node0->GetPos()).Length());

                    // Set element dimensions
                    element->SetDimensions(dx, dy);

                    // Add a single layers with a fiber angle of 0 degrees.
                    element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

                    // Set other element properties
                    element->SetAlphaDamp(m_alpha);

                    // Add element to mesh
                    m_mesh->AddElement(element);

                    ChVector<> center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                                element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
                    std::cout << "Adding element" << num_elem << "  with center:  " << center.x() << " " << center.y()
                              << " " << center.z() << std::endl;

                    num_elem++;
                }
            }
        }
    }
    // Add the mesh to the system
    sysMBS.Add(m_mesh);

    // fluid representation of flexible bodies
    bool multilayer = true;
    bool removeMiddleLayer = true;
    sysFSI.AddFEAmeshBCE(m_mesh, m_NodeNeighborElement_mesh, std::vector<std::vector<int>>(), m_2D_elementsNodes_mesh,
                         false, true, multilayer, removeMiddleLayer, 0, 0);

    sysFSI.AddFsiMesh(m_mesh, std::vector<std::vector<int>>(), m_2D_elementsNodes_mesh);
    fea::ChMeshExporter::WriteMesh(m_mesh, MESH_CONNECTIVITY);

    return m_mesh;
}

void CreateWheelBCEMarkers(std::shared_ptr<MyWheeledVehicle> vehicle, ChSystemFsi& sysFSI) {
    // Create BCE markers for a tire
    std::string tire_coll_obj = "../data/vehicle/LRV/LRV_rigidwheel_collision.obj";

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(tire_coll_obj);
    std::vector<ChVector<>> point_cloud;
    sysFSI.CreateMeshPoints(trimesh, sysFSI.GetInitialSpacing(), point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddPointsBCE(wheel->GetSpindle(), point_cloud, ChFrame<>(), true);
        }
    }
}


void CreateFenders(std::shared_ptr<MyWheeledVehicle> vehicle, ChSystem& sysMBS, ChSystemFsi& sysFSI) {
    auto cmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create fenders
    // auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    std::vector<std::string> fenderPaths = {vehicle_model.FenderPathFrontLeft(), vehicle_model.FenderPathFrontRight(),
                                            vehicle_model.FenderPathRearLeft(), vehicle_model.FenderPathRearRight()};
    std::vector<std::string> uprightNames = {"Front_Upright_L", "Front_Upright_R", "Rear_Upright_L", "Rear_Upright_R"};

    std::vector<VehicleSide> sides = {VehicleSide::LEFT, VehicleSide::RIGHT, VehicleSide::LEFT, VehicleSide::RIGHT};

    for (int i = 0; i < 4; ++i) {
        auto fender_trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        fender_trimesh->LoadWavefrontMesh(fenderPaths[i], false, true);
        fender_trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        fender_trimesh->RepairDuplicateVertexes(1e-9);

        auto fender_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        fender_trimesh_shape->SetMesh(fender_trimesh);
        // fender_trimesh_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));

        auto fender = chrono_types::make_shared<ChBody>();
        auto m_wheel = vehicle->GetWheel(i / 2, sides[i]);
        fender->SetNameString("fender_" + std::to_string(i));
        fender->SetMass(m_wheel->GetMass() / 20.0);
        fender->SetPos(m_wheel->GetPos());
        // fender->SetRot(vehicle->GetWheel(i / 2, sides[i])->GetTransform().GetA());
        fender->SetBodyFixed(false);
        fender->GetCollisionModel()->ClearModel();
        fender->GetCollisionModel()->AddTriangleMesh(cmaterial, fender_trimesh, false, false, VNULL, ChMatrix33<>(1),
                                                     0.005);
        fender->GetCollisionModel()->BuildModel();
        fender->SetCollide(false);
        fender->AddVisualShape(fender_trimesh_shape, ChFrame<>(0));
        sysMBS.AddBody(fender);

        auto fender_lock = chrono_types::make_shared<ChLinkLockLock>();
        fender_lock->SetNameString("fender_lock_" + std::to_string(i));
        fender_lock->Initialize(
            sysMBS.SearchBody("LRV DoubleWishboneRSDA " + uprightNames[i]), fender,
            ChCoordsys<>(vehicle->GetWheel(i / 2, sides[i])->GetSpindle()->GetPos(), Q_from_AngX(CH_C_PI_2)));
        sysMBS.AddLink(fender_lock);

        // Add BCE markers
        geometry::ChTriangleMeshConnected fender_trimesh_BCE;
        fender_trimesh_BCE.LoadWavefrontMesh(fenderPaths[i]);
        std::vector<ChVector<>> fender_point_cloud;
        sysFSI.CreateMeshPoints(fender_trimesh_BCE, sysFSI.GetInitialSpacing(), fender_point_cloud);
        sysFSI.AddFsiBody(fender);
        sysFSI.AddPointsBCE(fender, fender_point_cloud, ChFrame<>(), true);
    }
}

void logData(double t,
             ChSystemFsi& sysFSI,
             ChTimer timer,
             DriverInputs driver_inputs,
             std::shared_ptr<MyWheeledVehicle> vehicle) {
    auto sysMBS = vehicle->GetSystem();
    ChVector<> chassis_pos = vehicle->GetChassis()->GetPos();
    ChVector<> chassis_vel = vehicle->GetChassisBody()->GetPos_dt();
    ChVector<> chassis_acc = vehicle->GetChassisBody()->GetPos_dtdt();
    auto driveline4 = std::dynamic_pointer_cast<ChDrivelineWV>(vehicle->GetDriveline());
    auto transimission = vehicle->GetPowertrainAssembly();
    double torque;
    std::vector<int> axles = driveline4->GetDrivenAxleIndexes();
    double torque_from_PT = transimission->GetOutputTorque();
    int current_gear = transimission->GetTransmission()->GetCurrentGear();
    double current_rpm = transimission->GetTransmission()->GetOutputMotorshaftSpeed() * 30 / CH_C_PI;
    double speed;
    std::vector<double> fender_rot_angle;
    std::vector<ChVector<>> fender_rot_axis;
    std::vector<double> spindle_rot_angle;
    std::vector<ChVector<>> spindle_rot_axis;
    std::vector<bool> fender_lock_isbroken;
    std::vector<ChVector<>> fender_lock_torque;

    if (create_fender && debug_mode) {
        for (int i = 0; i < 4; i++) {
            fender_rot_angle.push_back(sysMBS->SearchBody("fender_" + std::to_string(i))->GetRotAngle());
            fender_rot_axis.push_back(sysMBS->SearchBody("fender_" + std::to_string(i))->GetRotAxis());
            fender_lock_isbroken.push_back(sysMBS->SearchLink("fender_lock_" + std::to_string(i))->IsBroken());
            fender_lock_torque.push_back(
                sysMBS->SearchLink("fender_lock_" + std::to_string(i))
                    ->GetLinkAbsoluteCoords()
                    .TransformDirectionLocalToParent(
                        sysMBS->SearchLink("fender_lock_" + std::to_string(i))->Get_react_torque()));
        }

        spindle_rot_angle.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Front_spindle_L")->GetRotAngle());
        spindle_rot_axis.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Front_spindle_L")->GetRotAxis());
        spindle_rot_angle.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Front_spindle_R")->GetRotAngle());
        spindle_rot_axis.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Front_spindle_R")->GetRotAxis());
        spindle_rot_angle.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Rear_spindle_L")->GetRotAngle());
        spindle_rot_axis.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Rear_spindle_L")->GetRotAxis());
        spindle_rot_angle.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Rear_spindle_R")->GetRotAngle());
        spindle_rot_axis.push_back(sysMBS->SearchBody("LRV DoubleWishboneRSDA Rear_spindle_R")->GetRotAxis());
    }

    std::ofstream logfile(out_dir + "/log.txt", std::ios_base::app);
    std::ofstream outputfile(out_dir + "/chassis_output.txt", std::ios_base::app);
    std::ofstream motorfile(out_dir + "/motor_output.txt", std::ios_base::app);
    if (!logfile.is_open()) {
        std::cerr << "Failed to open log file!" << std::endl;
        return;
    }
    if (!outputfile.is_open()) {
        std::cerr << "Failed to open output file!" << std::endl;
        return;
    }
    if (!motorfile.is_open()) {
		std::cerr << "Failed to open motor file!" << std::endl;
		return;
	}

    logfile << "-----------------------------------------------------\n";
    logfile << "Sim time:     " << t << ";    Spend time: " << timer.GetTimeSecondsIntermediate()
            << "sec;    RTF: " << sysFSI.GetRTF() << std::endl;
    logfile << "Throttle:     " << driver_inputs.m_throttle << ";    Braking: " << driver_inputs.m_braking
            << ";    Steering: " << driver_inputs.m_steering << std::endl;
    logfile << "Chassis pos:  " << chassis_pos.x() << " " << chassis_pos.y() << " " << chassis_pos.z() << std::endl;
    logfile << "Chassis vel:  " << chassis_vel.x() << " " << chassis_vel.y() << " " << chassis_vel.z() << std::endl;
    logfile << "Chassis acc:  " << chassis_acc.x() << " " << chassis_acc.y() << " " << chassis_acc.z() << std::endl;
    logfile << "PT gear: " << current_gear << ";    PT torque: " << torque_from_PT
            << ";     PT Motor RPM: " << current_rpm << std::endl;

    outputfile << t << "\t" << chassis_pos.x() << "\t" << chassis_pos.y() << "\t" << chassis_pos.z() << "\t"
               << chassis_vel.x() << "\t" << chassis_vel.y() << "\t" << chassis_vel.z() << "\t" << chassis_acc.x()
               << "\t" << chassis_acc.y() << "\t" << chassis_acc.z() << std::endl;

    if (create_fender && debug_mode) {
        for (int i = 0; i < 4; i++) {
            logfile << "fender_ " << i << "; rot axis: " << fender_rot_axis[i].x() << " " << fender_rot_axis[i].y()
                    << " " << fender_rot_axis[i].z() << ";    rot angle; " << fender_rot_angle[i] << std::endl;
            logfile << "spindle_ " << i << "; rot axis: " << spindle_rot_axis[i].x() << " " << spindle_rot_axis[i].y()
                    << " " << spindle_rot_axis[i].z() << ";    rot angle; " << spindle_rot_angle[i] << std::endl;
            logfile << "fender_lock_ " << i << "; torque: " << fender_lock_torque[i].x() << " "
                    << fender_lock_torque[i].y() << " " << fender_lock_torque[i].z() << std::endl;
            logfile << "fender_lock_ " << i << "; is broken: " << fender_lock_isbroken[i] << std::endl << std::endl;
        }
    }

    motorfile << t << "\t";
    for (int i = 0; i < axles.size(); i++) {
        torque = driveline4->GetSpindleTorque(axles[i], LEFT);
        logfile << "T.axle " << i << " L: " << torque << ";    ";
        motorfile << torque << "\t";
        torque = driveline4->GetSpindleTorque(axles[i], RIGHT);
        logfile << "T.axle " << i << " R: " << torque << std::endl;
        motorfile << torque << "\t";
    }

    for (int i = 0; i < axles.size(); i++) {
        speed = vehicle->GetSuspension(axles[i])->GetAxleSpeed(LEFT) * 30 / CH_C_PI;
        logfile << "RPM.axle " << i << " L: " << -speed << ";    ";
        motorfile << -speed << "\t";
        speed = vehicle->GetSuspension(axles[i])->GetAxleSpeed(RIGHT) * 30 / CH_C_PI;
        logfile << "RPM.axle " << i << " R: " << -speed << std::endl;
        motorfile << -speed << "\t";
    }
    motorfile << std::endl;

    std::cout << "-----------------------------------------------------\n";
    std::cout << "Sim time:     " << t << ";    Spend time: " << timer.GetTimeSecondsIntermediate()
              << ";    RTF: " << sysFSI.GetRTF() << std::endl;
    std::cout << "Throttle:     " << driver_inputs.m_throttle << ";    Braking: " << driver_inputs.m_braking
              << ";    Steering: " << driver_inputs.m_steering << std::endl;
    std::cout << "Chassis pos:  " << chassis_pos.x() << " " << chassis_pos.y() << " " << chassis_pos.z() << std::endl;
    std::cout << "Chassis vel:  " << chassis_vel.x() << " " << chassis_vel.y() << " " << chassis_vel.z() << std::endl;
    std::cout << "Chassis acc:  " << chassis_acc.x() << " " << chassis_acc.y() << " " << chassis_acc.z() << std::endl;
    std::cout << "PT gear: " << current_gear << ";    PT torque: " << torque_from_PT
              << ";     PT Motor RPM: " << current_rpm << std::endl;

    if (create_fender && debug_mode) {
        for (int i = 0; i < 4; i++) {
            std::cout << "fender_ " << i << "; rot axis: " << fender_rot_axis[i].x() << " " << fender_rot_axis[i].y()
                      << " " << fender_rot_axis[i].z() << ";    rot angle: " << fender_rot_angle[i] << std::endl;
            std::cout << "spindle_ " << i << "; rot axis: " << spindle_rot_axis[i].x() << " " << spindle_rot_axis[i].y()
                      << " " << spindle_rot_axis[i].z() << ";    rot angle: " << spindle_rot_angle[i] << std::endl;
            std::cout << "fender_lock_ " << i << "; torque: " << fender_lock_torque[i].x() << " "
                      << fender_lock_torque[i].y() << " " << fender_lock_torque[i].z() << std::endl;
            std::cout << "fender_lock_ " << i << "; is broken: " << fender_lock_isbroken[i] << std::endl << std::endl;
        }
    }

    for (int i = 0; i < 2; i++) {
        torque = driveline4->GetSpindleTorque(axles[i], LEFT);
        std::cout << "T.axle " << i << " L: " << torque << ";    ";
        torque = driveline4->GetSpindleTorque(axles[i], RIGHT);
        std::cout << "T.axle " << i << " R: " << torque << std::endl;
    }

    for (int i = 0; i < axles.size(); i++) {
        speed = vehicle->GetSuspension(axles[i])->GetAxleSpeed(LEFT) * 30 / CH_C_PI;
        std::cout << "RPM.axle " << i << " L: " << -speed << ";    ";
        speed = vehicle->GetSuspension(axles[i])->GetAxleSpeed(RIGHT) * 30 / CH_C_PI;
        std::cout << "RPM.axle " << i << " R: " << -speed << std::endl;
    }

    logfile.close();
    std::filesystem::copy(out_dir + "/log.txt", out_dir + "/log_monitor.txt", std::filesystem::copy_options::overwrite_existing);
    outputfile.close();
    motorfile.close();
}

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file) {
    // Open input file
    std::ifstream ifile(path_file);
    std::string line;

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    assert(numCols == 3);

    // Read path points
    std::vector<ChVector<>> points;

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector<>(x, y, z));
    }

    // Include point beyond SPH patch
    {
        auto np = points.size();
        points.push_back(2.0 * points[np - 1] - points[np - 2]);
    }

    // Raise all path points
    for (auto& p : points)
        p.z() += 0.1;

    ifile.close();

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}

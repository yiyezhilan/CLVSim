// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// Class to create toroidal tire with ANCF shell elements.
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainer.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "ANCFToroidalTireSPH.h"

double rim_radius = 0.295;
double wheel_radius = 0.41;
double height = 0.115;
double thickness = 0.005;
int m_div_circumference = 36;
int m_div_width = 12;
double m_alpha = 0.15;

double wheel_mass = 5.0;
ChVector<> wheel_inertia = ChVector<>(0.04043125, 0.6074, 0.04043125);

int TotalNumElements = m_div_circumference * m_div_width * 4;     // my_mesh->GetNelements();
int TotalNumNodes = m_div_circumference * (m_div_width + 1) * 4;  // my_mesh->GetNnodes();
int TotalNumLinks = 2 * m_div_circumference * 4;


// -----------------------------------------------------------------------------
ANCFToroidalTireSPH::ANCFToroidalTireSPH(
    const std::string& name,
    const int axle,                                                         ///< axle of the associated wheel
    const VehicleSide side,                                                 ///< side of the associated wheel
    const std::vector<std::shared_ptr<fea::ChLinkPointFrame>> im_connections,  ///< list of point links
    const std::vector<std::shared_ptr<fea::ChLinkDirFrame>> im_connectionsD)    ///< list of directional links
    : ChTire(name),
      m_use_contact_mesh(false),
      m_trimesh(nullptr),
      m_sweep_sphere_radius(0),
      m_connections(im_connections),
      m_connectionsD(im_connectionsD),
      m_wheel_radius(wheel_radius),
      m_wheel_width(2.0f * height),
      m_wheel_mass(wheel_mass),
      m_wheel_inertia(wheel_inertia),
      m_axle(axle),
      m_side(side){}

ANCFToroidalTireSPH::~ANCFToroidalTireSPH() {}

// -----------------------------------------------------------------------------
void ANCFToroidalTireSPH::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_contact_mesh = true;
    m_contact_meshFile = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
void ANCFToroidalTireSPH::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);
    //auto wheel_body = wheel->GetSpindle();

    //CreateContactMaterial(wheel_body->GetSystem()->GetContactMethod());
    //assert(m_material && m_material->GetContactMethod() == wheel_body->GetSystem()->GetContactMethod());

    //wheel_body->SetCollide(true);

    //wheel_body->GetCollisionModel()->ClearModel();

    //wheel_body->GetCollisionModel()->SetFamily(WheeledCollisionFamily::TIRE);

    //if (m_use_contact_mesh) {
    //    // Mesh contact
    //    m_trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(m_contact_meshFile, true, false);

    //    //// RADU
    //    // Hack to deal with current limitation: cannot set offset on a trimesh collision shape!
    //    double offset = GetOffset();
    //    if (std::abs(offset) > 1e-3) {
    //        for (int i = 0; i < m_trimesh->m_vertices.size(); i++)
    //            m_trimesh->m_vertices[i].y() += offset;
    //    }

    //    wheel_body->GetCollisionModel()->AddTriangleMesh(m_material, m_trimesh, false, false, ChVector<>(0),
    //                                                     ChMatrix33<>(1), m_sweep_sphere_radius);
    //} else {
    //    // Cylinder contact
    //    wheel_body->GetCollisionModel()->AddCylinder(m_material, GetRadius(), GetWidth(), ChVector<>(0, 0, GetOffset()),
    //                                                 Q_from_AngX(CH_C_PI_2));
    //}

    //wheel_body->GetCollisionModel()->BuildModel();
}


void ANCFToroidalTireSPH::InitializeInertiaProperties() {
    m_mass = GetTireMass();
    m_inertia.setZero();
    m_inertia.diagonal() = GetTireInertia().eigen();
    m_com = ChFrame<>();
}

void ANCFToroidalTireSPH::UpdateInertiaProperties() {
    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector<>(0, GetOffset(), 0)), spindle->GetRot());
}


// -----------------------------------------------------------------------------
void ANCFToroidalTireSPH::AddVisualizationAssets(VisualizationType vis) {
    //if (vis == VisualizationType::NONE)
    //    return;

    //m_cyl_shape = ChVehicleGeometry::AddVisualizationCylinder(m_wheel->GetSpindle(),                           //
    //                                                          ChVector<>(0, GetOffset() + GetWidth() / 2, 0),  //
    //                                                          ChVector<>(0, GetOffset() - GetWidth() / 2, 0),  //
    //                                                          GetRadius());
    //m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void ANCFToroidalTireSPH::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ANCFToroidalTireSPH::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------
/// Get the tire force and moment.
/// A ChDeformableTire always returns zero forces and moments since tire forces
/// are implicitly applied to the associated wheel through the tire-wheel connections.
TerrainForce ANCFToroidalTireSPH::GetTireForce() const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

TerrainForce ANCFToroidalTireSPH::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    int num_wheel;
    if (m_axle == 0 && m_side == LEFT) {
        num_wheel = 0;
    }
    else if (m_axle == 0 && m_side == RIGHT) {
		num_wheel = 1;
    }
    else if (m_axle == 1 && m_side == LEFT) {
		num_wheel = 2;
    }
    else if (m_axle == 1 && m_side == RIGHT) {
		num_wheel = 3;
	}

    int start = 2 * m_div_circumference * num_wheel;
    int end = 2 * m_div_circumference * (num_wheel + 1);
    // Calculate and return the resultant of all reaction forces and torques in the
    // tire-wheel connections, as applied at the wheel body center of mass.
    // These encapsulate the tire-terrain interaction forces and the inertia of the tire itself.
    ChVector<> force;
    ChVector<> moment;
    for (int ic = start; ic < end; ic++) {
        ChCoordsys<> csys = m_connections[ic]->GetLinkAbsoluteCoords();
        ChVector<> react = csys.TransformDirectionLocalToParent(m_connections[ic]->GetReactionOnBody());
        m_wheel->GetSpindle()->To_abs_forcetorque(react, csys.pos, false, force, moment);
        tire_force.force += force;
        tire_force.moment += moment;
    }

    for (int ic = start; ic < end; ic++) {
        ChCoordsys<> csys = m_connectionsD[ic]->GetLinkAbsoluteCoords();
        moment = csys.TransformDirectionLocalToParent(m_connectionsD[ic]->GetReactionOnBody());
        tire_force.moment += moment;
    }

    return tire_force;
}

TerrainForce ANCFToroidalTireSPH::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ANCFToroidalTireSPH::ReportTireForceLocal not implemented." << std::endl;
    throw ChException("ANCFToroidalTireSPH::ReportTireForceLocal not implemented.");
}

// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChTriangleMeshConnected> ANCFToroidalTireSPH::GetContactMesh() const {
    assert(m_use_contact_mesh);
    return m_trimesh;
}
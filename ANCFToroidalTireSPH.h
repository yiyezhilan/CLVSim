// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// Class to create toroidal tire with ANCF shell elements.
//
// =============================================================================

#ifndef ANCFTOROIDALTIRESPH_H
#define ANCFTOROIDALTIRESPH_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::vehicle;

/// Base class for a deformable tire model.
class ANCFToroidalTireSPH : public ChTire {
  public:
    ANCFToroidalTireSPH(
        const std::string& name,
        const int axle,										    				///< axle of the associated wheel
        const VehicleSide side,										    		///< side of the associated wheel
        const std::vector<std::shared_ptr<fea::ChLinkPointFrame>> im_connections,  ///< list of point links
        const std::vector<std::shared_ptr<fea::ChLinkDirFrame>> im_connectionsD    ///< list of directional links
    );

    virtual ~ANCFToroidalTireSPH();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ANCFToroidalTireSPH"; }

    /// Set Wavefront OBJ file for contact mesh.
    void SetMeshFilename(const std::string& mesh_file,   ///< [in] name of Wavefront file
                         double sweep_sphere_radius = 0  ///< [in] radius of sweeping sphere
    );

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_wheel_radius; }

    /// Get the tire width.
    virtual double GetWidth() const override { return m_wheel_width; }

    /// Return the tire mass.
    virtual double GetTireMass() const override { return m_wheel_mass; }

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual ChVector<> GetTireInertia() const override { return m_wheel_inertia; }

    /// Check whether or not this tire uses a contact mesh.
    bool UseContactMesh() const { return m_use_contact_mesh; }

    /// Report the tire force and moment.
    /// This generalized force encapsulates the tire-terrain forces (i.e. the resultant
    /// of all contact forces acting on the tire). The force and moment are expressed
    /// in global frame, as applied to the center of the associated wheel.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Get the tire force and moment expressed in the tire frame.
    /// Currently *NOT IMPLEMENTED*.
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

    /// Get the tire contact material.
    /// Note that this is not set until after tire initialization.
    std::shared_ptr<ChMaterialSurface> GetContactMaterial() const { return m_material; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the contact mesh.
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetContactMesh() const;


  protected:
    /// Create the contact material consistent with the specified contact method.
    // virtual void CreateContactMaterial(ChContactMethod contact_method);

    std::shared_ptr<ChMaterialSurfaceSMC> m_material;  ///< contact material;

    virtual void InitializeInertiaProperties() override final;
    virtual void UpdateInertiaProperties() override final;

    /// The mass properties of a deformable tire are implicitly included through the FEA mesh.
    /// No mass and inertia are added to the associated spindle body.
    virtual double GetAddedMass() const override final { return 0; }
    virtual ChVector<> GetAddedInertia() const override final { return ChVector<>(0, 0, 0); }

    /// Get the tire force and moment.
    /// A ChDeformableTire always returns zero forces and moments since tire forces
    /// are implicitly applied to the associated wheel through the tire-wheel connections.
    virtual TerrainForce GetTireForce() const override final;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    bool m_use_contact_mesh;         ///< flag indicating use of a contact mesh
    std::string m_contact_meshFile;  ///< name of the OBJ file for contact mesh
    double m_sweep_sphere_radius;    ///< radius of sweeping sphere for mesh contact
    double m_wheel_radius;
    double m_wheel_width;
    double m_wheel_mass;
    ChVector<> m_wheel_inertia;

    std::vector<std::shared_ptr<fea::ChLinkPointFrame>> m_connections;  ///< list of point links
    std::vector<std::shared_ptr<fea::ChLinkDirFrame>> m_connectionsD;   ///< list of directional links
    std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;       ///< contact mesh
    int m_axle;                                                         ///< index of the axle to which the tire is associated
    VehicleSide m_side;                                                 ///< left/right side

    std::shared_ptr<ChVisualShape> m_cyl_shape;  ///< visualization cylinder asset
};

#endif

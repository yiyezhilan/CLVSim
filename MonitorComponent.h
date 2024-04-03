// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// The MonitorComponent class is a GUI component that can be added to the
// VSG window to monitor the state history of the vehicle.
//
// =============================================================================


#ifndef MONITOR_COMPONENT_H
#define MONITOR_COMPONENT_H

#include "chrono_vsg/ChApiVSG.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChEventHandlerVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

using namespace chrono::vsg3d;

class MonitorGuiComponent : public ChGuiComponentVSG {
  public:
    struct MonitorData {
        std::vector<double> time;
        std::vector<double> steering;
        std::vector<double> throttle;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> consumption;
        /*std::vector<double> current;*/
        MonitorData() : time(), steering(), throttle(), velocity(), acceleration(), consumption() {}
    };

    MonitorGuiComponent(ChVisualSystemVSG& app, MonitorData Data);

    void render();

    ChVisualSystemVSG& m_app;
    MonitorData m_data;
};
#endif  // !MONITOR_COMPONENT_H

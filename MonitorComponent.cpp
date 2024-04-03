// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// The MonitorComponent class is a GUI component that can be added to the
// VSG window to monitor the state history of the vehicle.
//
// =============================================================================


#include "chrono_vsg/ChApiVSG.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono/assets/ChVisualSystem.h"
#include "MonitorComponent.h"


MonitorGuiComponent::MonitorGuiComponent(ChVisualSystemVSG& app, MonitorData Data) : m_app(app), m_data(Data){};

std::vector<double> extract_axis(const std::vector<std::vector<double>>& data, size_t axisIndex) {
    std::vector<double> result;
    for (const auto& item : data) {
        if (item.size() > axisIndex) {
            result.push_back(item[axisIndex]);
        }
    }
    return result;
}

void MonitorGuiComponent::render() {
    char label[64];
    int nstr = sizeof(label) - 1;

    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowPos(ImVec2(1280.0f * 0.675, 720.0f * 0.525));
    ImGui::Begin("Monitor");

    int subplot_cols = 2;
    int subplot_rows = 2;

    if (ImPlot::BeginSubplots("", subplot_rows, subplot_cols, ImVec2(0.0f, 0.0f),
                              ImPlotSubplotFlags_LinkAllX)) {
        // 绘制驾驶员输入变化历史
        if (ImPlot::BeginPlot("")) {
            ImPlot::SetupAxes("", "Steering (%)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("", m_data.time.data(), m_data.steering.data(), static_cast<int>(m_data.time.size()));
            ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
            ImPlot::PlotShaded("", m_data.time.data(), m_data.steering.data(), static_cast<int>(m_data.time.size()));
            ImPlot::EndPlot();
        }

        // 绘制车速和竖直方向加速度变化历史
        if (ImPlot::BeginPlot("")) {
            ImPlot::SetupAxes("", "Velocity (m/s)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("", m_data.time.data(), m_data.velocity.data(), static_cast<int>(m_data.time.size()));
            ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
            ImPlot::PlotShaded("", m_data.time.data(), m_data.velocity.data(), static_cast<int>(m_data.time.size()));
            ImPlot::EndPlot();
        }

        // 绘制竖直加速度变化历史
        if (ImPlot::BeginPlot("")) {
            ImPlot::SetupAxes("", "Acceleration (m/s^2)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("", m_data.time.data(), m_data.acceleration.data(), static_cast<int>(m_data.time.size()));
            ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
            ImPlot::PlotShaded("", m_data.time.data(), m_data.acceleration.data(), static_cast<int>(m_data.time.size()));
            ImPlot::EndPlot();
        }

        // 绘制电机电流变化历史
        if (ImPlot::BeginPlot("")) {
			ImPlot::SetupAxes("", "Current(Amp)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("", m_data.time.data(), m_data.consumption.data(), static_cast<int>(m_data.time.size()));
            ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
            ImPlot::PlotShaded("", m_data.time.data(), m_data.consumption.data(), static_cast<int>(m_data.time.size()));
			ImPlot::EndPlot();
		}

        ImPlot::EndSubplots();
    }
}
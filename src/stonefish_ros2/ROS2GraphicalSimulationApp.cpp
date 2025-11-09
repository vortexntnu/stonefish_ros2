/*    
    This file is a part of stonefish_ros2.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2GraphicalSimulationApp.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2GraphicalSimulationApp.h"

// >>>> IMPORTANT: include headers from base GUI and base types
#include <Stonefish/graphics/IMGUI.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/MovingEntity.h>
#include <Stonefish/graphics/OpenGLTrackball.h> 

#include <SDL2/SDL.h>
#include <cmath>
#include <cstdio>
#include <glm/glm.hpp>
namespace sf
{

ROS2GraphicalSimulationApp::ROS2GraphicalSimulationApp(std::string title, std::string dataPath, RenderSettings s, HelperSettings h, ROS2SimulationManager* sim)
    : GraphicalSimulationApp(title, dataPath, s, h, sim)
{
}

void ROS2GraphicalSimulationApp::Startup()
{
    Init();
    StartSimulation();
}

void ROS2GraphicalSimulationApp::Tick()
{
    LoopInternal();
    if(state_ == SimulationState::FINISHED)
    {
        CleanUp();
        rclcpp::shutdown();
    }
}

// ================== NEW EXTENDED HUD ==================

void ROS2GraphicalSimulationApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();

    IMGUI* gui = getGUI();
    if(!gui) return;

    // —— Layout: Use same width as base panels (≈230 px)
    const float kLeftColX = 10.f;

    // Second column: to the right of the base column
    float startX = kLeftColX; // + kPanelW + kGap;  // ≈ 12 + 230 + 14 = 256
    float y = 755.f;  // start high, but already in another column (no overlap)

    if (showPosePanel_) {
        DrawPosePanel_(startX, y);
    }

    if (showViewPanel_) {
        DrawViewPanel_(startX, y);
    }
}


// Keyword event handler override
void ROS2GraphicalSimulationApp::KeyDown(SDL_Event* event)
{
    // Let the base class handle first (ESC, console, etc.)
    GraphicalSimulationApp::KeyDown(event);

    if (!event || event->type != SDL_KEYDOWN) return;

    const SDL_Keycode key = event->key.keysym.sym;

    if (key == SDLK_F6)
        showPosePanel_ = !showPosePanel_;
    else if (key == SDLK_F7)
        showViewPanel_ = !showViewPanel_; // today does nothing because we left it false by default
}


// ------------------ Panel Implementation ------------------

void ROS2GraphicalSimulationApp::DrawPosePanel_(float x, float& y)
{
    IMGUI* gui = getGUI();
    if(!gui) return;

    // Unique ID for widgets in this panel
    Uid id; id.owner = 90; id.item = 0;

    // Frame/panel
    const float w = 230.f;
    float yy = y;
    gui->DoPanel(x, yy, w, 86.f); yy += 6.f;
    gui->DoLabel(x + 6.f, yy, "SELECTED POSE"); yy += 16.f;

    // Retrieve current selection
    auto sel = getSelectedEntity(); // {Entity*, subIndex}
    if (sel.first)
    {
        // Try to get Transform for moving entities
        if (auto* mov = dynamic_cast<MovingEntity*>(sel.first))
        {
            Transform T = mov->getCGTransform(); // btTransform
            Vector3 p = T.getOrigin();           // btVector3

            // Euler ZYX -> yaw, pitch, roll
            Scalar yaw, pitch, roll;
            T.getBasis().getEulerZYX(yaw, pitch, roll);

            auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

            char buf[128];
            std::snprintf(buf, sizeof(buf), "XYZ: %.3f  %.3f  %.3f",
                          (double)p.x(), (double)p.y(), (double)p.z());
            gui->DoLabel(x + 8.f, yy, buf); yy += 14.f;

            std::snprintf(buf, sizeof(buf), "RPY[deg]: %.1f  %.1f  %.1f",
                          rad2deg((double)roll),
                          rad2deg((double)pitch),
                          rad2deg((double)yaw));
            gui->DoLabel(x + 8.f, yy, buf); yy += 14.f;
        }
        else
        {
            gui->DoLabel(x + 8.f, yy, "(Static/Dynamic entity)"); yy += 14.f;
            gui->DoLabel(x + 8.f, yy, "Select body for pose."); yy += 14.f;
        }
    }
    else
    {
        gui->DoLabel(x + 8.f, yy, "No selection."); yy += 14.f;
    }

    y = yy + 6.f; // return vertical cursor
}

void ROS2GraphicalSimulationApp::DrawViewPanel_(float x, float& y)
{
    IMGUI* gui = getGUI();
    if(!gui) return;

    const float w = 230.f;
    float yy = y;
    gui->DoPanel(x, yy, w, 82.f); yy += 6.f;
    gui->DoLabel(x + 6.f, yy, "FREE CAMERA"); yy += 16.f;

    auto* sim = getSimulationManager();
    auto* tb  = sim ? sim->getTrackball() : nullptr;

    if (tb)
    {
        // Public API according to OpenGLTrackball.h
        glm::vec3 eye = tb->GetEyePosition();
        glm::vec3 fwd = tb->GetLookingDirection();

        char buf[160];
        std::snprintf(buf, sizeof(buf), "Pos: %.2f  %.2f  %.2f",
                      (double)eye.x, (double)eye.y, (double)eye.z);
        gui->DoLabel(x + 8.f, yy, buf); yy += 14.f;

        std::snprintf(buf, sizeof(buf), "Dir: %.2f  %.2f  %.2f",
                      (double)fwd.x, (double)fwd.y, (double)fwd.z);
        gui->DoLabel(x + 8.f, yy, buf); yy += 14.f;
    }
    else
    {
        gui->DoLabel(x + 8.f, yy, "Trackball not available."); yy += 14.f;
    }

    y = yy + 6.f;
}

} // namespace sf

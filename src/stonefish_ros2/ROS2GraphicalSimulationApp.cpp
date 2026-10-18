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

namespace sf
{

ROS2GraphicalSimulationApp::ROS2GraphicalSimulationApp(std::string title, std::string dataPath, RenderSettings s, HelperSettings h, ROS2SimulationManager* sim)
    : GraphicalSimulationApp(title, dataPath, s, h, sim), editor_(std::make_unique<SceneEditor>(this, sim))
{
}

void ROS2GraphicalSimulationApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();
    editor_->DoHUD();
}

void ROS2GraphicalSimulationApp::KeyDown(SDL_Event* event)
{
    if(!editor_->KeyDown(event))
        GraphicalSimulationApp::KeyDown(event);
}

void ROS2GraphicalSimulationApp::MouseMove(SDL_Event* event)
{
    GraphicalSimulationApp::MouseMove(event);
    editor_->MouseMove(event);
}

void ROS2GraphicalSimulationApp::MouseUp(SDL_Event* event)
{
    GraphicalSimulationApp::MouseUp(event);
    editor_->MouseUp(event);
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

void ROS2GraphicalSimulationApp::Shutdown()
{
    CleanUp();
}

void ROS2GraphicalSimulationApp::Step()
{
    StepSimulation();
}

void ROS2GraphicalSimulationApp::Pause()
{
    StopSimulation();
}

void ROS2GraphicalSimulationApp::Resume()
{
    ResumeSimulation();
}

}
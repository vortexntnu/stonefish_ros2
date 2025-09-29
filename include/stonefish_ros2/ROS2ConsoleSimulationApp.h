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
//  ROS2ConsoleSimulationApp.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 05/10/23.
//  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROS2ConsoleSimulationApp__
#define __Stonefish_ROS2ConsoleSimulationApp__

#include <Stonefish/core/ConsoleSimulationApp.h>

namespace sf
{
    class ROS2SimulationManager;

    class ROS2ConsoleSimulationApp : public ConsoleSimulationApp
    {
    public:
        ROS2ConsoleSimulationApp(std::string title, std::string dataPath, ROS2SimulationManager* sim);
        void Startup();
        void Shutdown();
        void Step();
        void Pause();
        void Resume();
    };
}

#endif
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
//  ROS2ScenarioParser.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROS2ScenarioParser__
#define __Stonefish_ROS2ScenarioParser__

#include "rclcpp/rclcpp.hpp"
#include <Stonefish/core/ScenarioParser.h>
#include "stonefish_ros2/SceneLandmarks.h"
#include <optional>

namespace sf
{
    class ROS2SimulationManager;

    class ROS2ScenarioParser : public ScenarioParser
    {
    public:
        ROS2ScenarioParser(ROS2SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh);

        //! After parsing: move the drone (the first robot) to the spawn point named by the 'spawn'
        //! parameter, facing its heading as the odometry reports it, and record the start pose.
        //! Returns false if there is no such spawn point (the drone then stays at its default pose).
        bool PlaceDrone();

   protected:
        virtual bool PreProcess(XMLNode* root, const std::map<std::string, std::string>& args);
        virtual bool IncludeFiles(XMLNode* node);
        virtual bool ParseStatic(XMLElement* element);
        virtual bool ParseDynamic(XMLElement* element);
        virtual VelocityField* ParseVelocityField(XMLElement* element);
        virtual bool ParseRobot(XMLElement* element);
        virtual bool ParseAnimated(XMLElement* element);
        virtual Actuator* ParseActuator(XMLElement* element, const std::string& namePrefix);
        virtual Sensor* ParseSensor(XMLElement* element, const std::string& namePrefix);
        virtual Comm* ParseComm(XMLElement* element, const std::string& namePrefix);
        virtual Light* ParseLight(XMLElement* element, const std::string& namePrefix);
        virtual bool ParseContact(XMLElement* element);
        virtual FixedJoint* ParseGlue(XMLElement* element);

    private:
        std::string SubstituteROSVars(const std::string& value);
        bool ReplaceROSVars(XMLNode* node);
        bool ParseLandmarks(XMLElement* element);
        bool ParseLandmark(XMLElement* item, const std::string& objectName);
        bool ParseSpawnOrPrior(XMLElement* item, SceneLandmark::Kind kind);


        std::shared_ptr<rclcpp::Node> nh_;
        Transform robotOrigin_ = Transform::getIdentity(); //!< world_transform of the first robot
    };
}

#endif
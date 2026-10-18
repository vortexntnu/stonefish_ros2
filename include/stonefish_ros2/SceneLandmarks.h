/*
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

//
//  SceneLandmarks.h
//  stonefish_ros2
//
//  Publishes the <landmark> tags of the scenario as vortex_msgs/LandmarkArray,
//  using the live pose of the object each tag belongs to.
//

#ifndef __Stonefish_SceneLandmarks__
#define __Stonefish_SceneLandmarks__

#include "rclcpp/rclcpp.hpp"
#include "vortex_msgs/msg/landmark_array.hpp"
#include <Stonefish/StonefishCommon.h>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sf
{
    class SimulationManager;

    //! A <landmark> tag inside a <static> or <dynamic> object.
    struct SceneLandmark
    {
        std::string entityName;
        std::string typeName;
        std::string subtypeName;
        uint16_t type;
        uint16_t subtype;
        std::optional<Vector3> point;   //!< In the object's frame. Empty = centre of the object's bounding box.
        std::optional<Vector3> normal;  //!< In the object's frame. Set = publish its yaw (surface normal).
    };

    class SceneLandmarks
    {
    public:
        SceneLandmarks(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh);

        //! Configure publishing from <ros_landmarks topic="" frame_id="" rate=""/>.
        void Configure(const std::string& topic, const std::string& frameId, double rate);

        //! Adds a landmark; returns false and leaves the list unchanged if the type or subtype is unknown.
        bool Add(const SceneLandmark& landmark, std::string& error);

        //! Removes all landmarks (the scenario is being rebuilt).
        void Clear();

        //! Landmarks of one object.
        std::vector<SceneLandmark> Of(const std::string& entityName) const;

        //! Replace the landmarks of one object (used by the scene editor).
        void Set(const std::string& entityName, const std::vector<SceneLandmark>& landmarks);

        const std::vector<SceneLandmark>& All() const { return landmarks_; }

        //! Replace every landmark (undo in the scene editor).
        void SetAll(const std::vector<SceneLandmark>& landmarks) { landmarks_ = landmarks; }

        //! World position of a landmark, or false if its object does not exist.
        bool WorldPosition(const SceneLandmark& landmark, Vector3& position) const;

        //! Known type and subtype names, from vortex_msgs (e.g. "GATE", "GATE_SEARCH_RESCUE").
        static const std::map<std::string, uint16_t>& TypeNames();
        static const std::map<std::string, std::map<std::string, uint16_t>>& SubtypeNames();

    private:
        void Publish();
        bool EntityTransform(const std::string& name, Transform& T) const;

        SimulationManager* sm_;
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string frameId_;
        std::vector<SceneLandmark> landmarks_;
    };
}

#endif

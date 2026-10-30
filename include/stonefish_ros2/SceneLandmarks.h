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
#include <random>
#include <string>
#include <vector>

namespace sf
{
    class SimulationManager;
    class Entity;

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
        //! rpy="" in the object's frame: publish the full orientation of this frame, e.g. a valve with
        //! Z out of its face and X along the handle (wins over normal).
        std::optional<Quaternion> rotation;
        bool enabled = true;            //!< enabled="false" in the scene: kept, but not published.
        bool downCamera = false;        //!< camera="down": seen by the down camera (bin floors, table items).
        //! <spawn name=""> (where the drone can start) and <prior name=""> (a task of the landmark server's
        //! premap) are kept with the landmarks so they follow their object, but are not published.
        //! typeName is then "SPAWN" or "PRIOR", subtypeName the name, and normal the heading.
        enum class Kind { LANDMARK, SPAWN, PRIOR } kind = Kind::LANDMARK;
    };

    //! Detection errors, the profiles robosub_dummy_publisher had. [near, far] pairs go from a close,
    //! centred view to the edge of the field of view.
    struct DetectionProfile
    {
        std::string name;
        bool useFieldOfView = false;
        double frontRange = 8.0; // the view's width is the camera's
        double downRadius = 1.5, downMaxAltitude = 3.0;
        double nearRange = 2.0, centreHalfFovDeg = 15.0;
        double positionNoise = 0.0;
        double rangeNoiseAlong[2] = {0, 0};   //!< std [m] = base + per_m * distance
        double rangeNoiseAcross[2] = {0, 0};
        double offAxisNoiseGain = 0.0;
        double rangeBiasPerM = 0.0;
        double orientationNoiseDeg[2] = {0, 0};
        double detectionProbability[2] = {1, 1};
        double dropoutRatePerSec[2] = {0, 0};
        double dropoutDurationSec[2] = {1, 5};
        double outlierProbability[2] = {0, 0};
        double outlierStd = 1.0;
        double falsePositiveRate[2] = {0, 0};
        double falsePositiveRadius = 2.0;
        double classConfusionProbability[2] = {0, 0};
        double frameDropProbability = 0.0;
    };

    class SceneLandmarks
    {
    public:
        SceneLandmarks(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh);

        //! Configure publishing from <ros_landmarks topic="" frame_id="" rate="" profile=""
        //! front_camera="" down_camera=""/> (the cameras are sensor names, used by profiles with a field of view).
        void Configure(const std::string& topic, const std::string& frameId, double rate,
                       const std::string& profile, const std::string& frontCamera, const std::string& downCamera);

        //! none, realistic, unstable, erratic.
        static const std::vector<DetectionProfile>& Profiles();
        void setProfile(size_t index) { profile_ = std::min(index, Profiles().size() - 1); }
        size_t getProfile() const { return profile_; }

        //! Adds a landmark; returns false and leaves the list unchanged if the type or subtype is unknown.
        bool Add(const SceneLandmark& landmark, std::string& error);

        //! Removes all landmarks (the scenario is being rebuilt).
        void Clear();

        //! Landmarks of one object.
        std::vector<SceneLandmark> Of(const std::string& entityName) const;

        //! Replace the landmarks of one object (used by the scene editor).
        void Set(const std::string& entityName, const std::vector<SceneLandmark>& landmarks);

        const std::vector<SceneLandmark>& All() const { return landmarks_; }

        //! All landmark publishing on or off (on by default).
        void setPublishing(bool on) { publishing_ = on; }
        bool isPublishing() const { return publishing_; }

        //! Replace every landmark (undo in the scene editor).
        void SetAll(const std::vector<SceneLandmark>& landmarks) { landmarks_ = landmarks; }

        //! World position of a landmark, or false if its object does not exist.
        bool WorldPosition(const SceneLandmark& landmark, Vector3& position) const;

        //! A static or dynamic object, or a robot link ("robot/link", e.g. "valve1/valve1_turnable").
        Entity* FindEntity(const std::string& name) const;
        //! Pose of FindEntity(name): its world transform (origin frame for dynamic bodies and links).
        bool EntityTransform(const std::string& name, Transform& T) const;

        //! Known type and subtype names, from vortex_msgs (e.g. "GATE", "GATE_SEARCH_RESCUE").
        static const std::map<std::string, uint16_t>& TypeNames();
        static const std::map<std::string, std::map<std::string, uint16_t>>& SubtypeNames();

    private:
        void Publish();
        //! Whether the landmark is in view, and how good the view is: 0 close and centred, 1 at the edge.
        bool InView(const SceneLandmark& l, const Vector3& p, double& badness, Vector3& viewDir, double& range) const;
        double Lerp(const double v[2], double b) const { return v[0] + (v[1] - v[0]) * b; }

        SimulationManager* sm_;
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string frameId_;
        std::vector<SceneLandmark> landmarks_;
        bool publishing_ = true;
        size_t profile_ = 0;
        std::string frontCamera_, downCamera_;
        std::mt19937 rng_{std::random_device{}()};
        std::map<std::string, double> occludedUntil_; //!< dropouts, per landmark
        double timerPeriod_ = 0.1;
    };
}

#endif

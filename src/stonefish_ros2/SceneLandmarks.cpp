/*
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

//
//  SceneLandmarks.cpp
//  stonefish_ros2
//

#include "stonefish_ros2/SceneLandmarks.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
#include <Stonefish/sensors/Sensor.h>
#include <Stonefish/sensors/vision/Camera.h>
#include <vortex/utils/ros/landmark_names.hpp>
#include <algorithm>
#include <cmath>

namespace sf
{

// The names come from vortex_utils_ros, the table the landmark server and the BT nodes use too
const std::map<std::string, uint16_t>& SceneLandmarks::TypeNames()
{
    static const std::map<std::string, uint16_t> names = [] {
        std::map<std::string, uint16_t> out;
        for(const auto& t : vortex::utils::landmarks::kLandmarkTypes)
            out[std::string(t.name)] = t.value;
        return out;
    }();
    return names;
}

const std::map<std::string, std::map<std::string, uint16_t>>& SceneLandmarks::SubtypeNames()
{
    // Subtypes by the name of their type
    static const std::map<std::string, std::map<std::string, uint16_t>> names = [] {
        std::map<std::string, std::map<std::string, uint16_t>> out;
        for(const auto& s : vortex::utils::landmarks::kLandmarkSubtypes)
            for(const auto& t : vortex::utils::landmarks::kLandmarkTypes)
                if(t.value == s.type)
                    out[std::string(t.name)][std::string(s.name)] = s.value;
        return out;
    }();
    return names;
}

SceneLandmarks::SceneLandmarks(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh)
    : sm_(sm), nh_(nh)
{
}

const std::vector<DetectionProfile>& SceneLandmarks::Profiles()
{
    // The profiles of robosub_dummy_publisher (config/robosub_dummy_publisher_<name>.yaml)
    static const std::vector<DetectionProfile> profiles = [] {
        DetectionProfile none;
        none.name = "none";

        DetectionProfile realistic;
        realistic.name = "realistic";
        realistic.useFieldOfView = true;
        realistic.rangeNoiseAlong[0] = 0.03; realistic.rangeNoiseAlong[1] = 0.03;
        realistic.rangeNoiseAcross[0] = 0.005; realistic.rangeNoiseAcross[1] = 0.015;
        realistic.offAxisNoiseGain = 1.0;
        realistic.rangeBiasPerM = 0.01;
        realistic.orientationNoiseDeg[0] = 2.0; realistic.orientationNoiseDeg[1] = 15.0;
        realistic.detectionProbability[0] = 0.95; realistic.detectionProbability[1] = 0.4;
        realistic.dropoutRatePerSec[1] = 0.1;
        realistic.dropoutDurationSec[0] = 1.0; realistic.dropoutDurationSec[1] = 4.0;
        realistic.outlierProbability[1] = 0.05;
        realistic.falsePositiveRate[1] = 0.3;
        realistic.classConfusionProbability[1] = 0.1;
        realistic.frameDropProbability = 0.05;

        DetectionProfile unstable;
        unstable.name = "unstable";
        unstable.positionNoise = 0.05;
        unstable.detectionProbability[0] = unstable.detectionProbability[1] = 0.7;
        unstable.frameDropProbability = 0.05;
        unstable.dropoutRatePerSec[0] = unstable.dropoutRatePerSec[1] = 0.05;
        unstable.dropoutDurationSec[0] = 1.0; unstable.dropoutDurationSec[1] = 6.0;
        unstable.outlierProbability[0] = unstable.outlierProbability[1] = 0.02;
        unstable.outlierStd = 1.5;
        unstable.falsePositiveRate[0] = unstable.falsePositiveRate[1] = 0.1;

        DetectionProfile erratic = unstable;
        erratic.name = "erratic";
        erratic.detectionProbability[0] = erratic.detectionProbability[1] = 0.6;
        erratic.frameDropProbability = 0.1;
        erratic.dropoutRatePerSec[0] = erratic.dropoutRatePerSec[1] = 0.1;
        erratic.outlierProbability[0] = erratic.outlierProbability[1] = 0.05;
        erratic.falsePositiveRate[0] = erratic.falsePositiveRate[1] = 0.3;

        return std::vector<DetectionProfile>{none, realistic, unstable, erratic};
    }();
    return profiles;
}

void SceneLandmarks::Configure(const std::string& topic, const std::string& frameId, double rate,
                               const std::string& profile, const std::string& frontCamera, const std::string& downCamera)
{
    frameId_ = frameId;
    frontCamera_ = frontCamera;
    downCamera_ = downCamera;
    for(size_t i = 0; i < Profiles().size(); ++i)
        if(Profiles()[i].name == profile)
            profile_ = i;
    timerPeriod_ = 1.0 / std::max(rate, 0.1);
    pub_ = nh_->create_publisher<vortex_msgs::msg::LandmarkArray>(topic, 10);
    timer_ = nh_->create_wall_timer(std::chrono::duration<double>(timerPeriod_),
                                    std::bind(&SceneLandmarks::Publish, this));
    RCLCPP_INFO(nh_->get_logger(), "Scene landmarks: publishing on '%s' in frame '%s' at %.1f Hz, profile '%s'.",
                topic.c_str(), frameId.c_str(), rate, Profiles()[profile_].name.c_str());
}

bool SceneLandmarks::Add(const SceneLandmark& landmark, std::string& error)
{
    SceneLandmark l = landmark;
    if(l.kind != SceneLandmark::Kind::LANDMARK)
    {
        landmarks_.push_back(l);
        return true;
    }
    auto type = TypeNames().find(l.typeName);
    if(type == TypeNames().end())
    {
        error = "unknown landmark type '" + l.typeName + "'";
        return false;
    }
    l.type = type->second;
    l.subtype = 0;
    if(!l.subtypeName.empty())
    {
        auto subtypes = SubtypeNames().find(l.typeName);
        if(subtypes == SubtypeNames().end() || subtypes->second.count(l.subtypeName) == 0)
        {
            error = "subtype '" + l.subtypeName + "' does not belong to type '" + l.typeName + "'";
            return false;
        }
        l.subtype = subtypes->second.at(l.subtypeName);
    }
    landmarks_.push_back(l);
    return true;
}

void SceneLandmarks::Clear()
{
    landmarks_.clear();
}

std::vector<SceneLandmark> SceneLandmarks::Of(const std::string& entityName) const
{
    std::vector<SceneLandmark> out;
    std::copy_if(landmarks_.begin(), landmarks_.end(), std::back_inserter(out),
                 [&](const SceneLandmark& l) { return l.entityName == entityName; });
    return out;
}

void SceneLandmarks::Set(const std::string& entityName, const std::vector<SceneLandmark>& landmarks)
{
    landmarks_.erase(std::remove_if(landmarks_.begin(), landmarks_.end(),
                                    [&](const SceneLandmark& l) { return l.entityName == entityName; }),
                     landmarks_.end());
    landmarks_.insert(landmarks_.end(), landmarks.begin(), landmarks.end());
}

Entity* SceneLandmarks::FindEntity(const std::string& name) const
{
    if(Entity* ent = sm_->getEntity(name))
        return ent;
    // Robot links are not entities of their own: "robot/link"
    size_t slash = name.find('/');
    Robot* robot = slash == std::string::npos ? nullptr : sm_->getRobot(name.substr(0, slash));
    if(robot == nullptr)
        return nullptr;
    for(const std::string& link : {name, name.substr(slash + 1)})
        if(SolidEntity* solid = robot->getLink(link))
            return solid;
    return nullptr;
}

bool SceneLandmarks::EntityTransform(const std::string& name, Transform& T) const
{
    Entity* ent = FindEntity(name);
    if(ent == nullptr)
        return false;
    if(ent->getType() == EntityType::STATIC)
        T = ((StaticEntity*)ent)->getTransform();
    else if(ent->getType() == EntityType::SOLID)
        T = ((SolidEntity*)ent)->getOTransform();
    else
        return false;
    return true;
}

bool SceneLandmarks::WorldPosition(const SceneLandmark& landmark, Vector3& position) const
{
    Transform T;
    if(!EntityTransform(landmark.entityName, T))
        return false;
    if(landmark.point)
    {
        position = T * (*landmark.point);
    }
    else
    {
        Vector3 min, max;
        FindEntity(landmark.entityName)->getAABB(min, max);
        position = (min + max) / Scalar(2);
    }
    return true;
}

bool SceneLandmarks::InView(const SceneLandmark& l, const Vector3& p, double& badness, Vector3& viewDir, double& range) const
{
    const DetectionProfile& prof = Profiles()[profile_];
    badness = 0.0;
    viewDir = Vector3(1, 0, 0);
    range = 0.0;
    const std::string& name = l.downCamera ? downCamera_ : frontCamera_;
    Sensor* camera = name.empty() ? nullptr : sm_->getSensor(name);
    if(camera == nullptr)
    {
        if(prof.useFieldOfView)
            RCLCPP_WARN_ONCE(nh_->get_logger(), "Scene landmarks: camera sensor '%s' not found, the '%s' profile sees nothing.",
                             name.c_str(), prof.name.c_str());
        return !prof.useFieldOfView; // without the camera there is nothing to filter with
    }

    Transform C = camera->getSensorFrame();
    Vector3 d = p - C.getOrigin();
    range = d.length();
    viewDir = range > 1e-6 ? d / range : Vector3(1, 0, 0);
    if(!prof.useFieldOfView)
        return true;

    if(l.downCamera)
    {
        // Below the vehicle, within a radius
        Scalar altitude = d.getZ();
        Scalar horizontal = std::hypot(d.getX(), d.getY());
        badness = std::clamp(horizontal / prof.downRadius, 0.0, 1.0);
        return altitude > 0 && altitude <= prof.downMaxAltitude && horizontal <= prof.downRadius;
    }

    // In the camera's own image (optical frame: z forward, x right, y down),
    // so the view follows whatever camera the drone has
    Vector3 c = C.getBasis().transpose() * d;
    if(range > prof.frontRange || c.getZ() <= 0)
        return false;
    double halfH = 45.0, halfV = 45.0; // [deg], a sensor that is not a camera
    if(Camera* cam = dynamic_cast<Camera*>(camera))
    {
        unsigned int w = 1, h = 1;
        cam->getResolution(w, h);
        halfH = cam->getHorizontalFOV() / 2.0;
        halfV = std::atan(std::tan(halfH * M_PI / 180.0) * h / std::max(w, 1u)) * 180.0 / M_PI;
    }
    if(std::atan2(std::abs(c.getX()), c.getZ()) * 180.0 / M_PI > halfH
       || std::atan2(std::abs(c.getY()), c.getZ()) * 180.0 / M_PI > halfV)
        return false;
    double angle = std::atan2(std::hypot(c.getX(), c.getY()), c.getZ()) * 180.0 / M_PI;
    double bRange = std::clamp((range - prof.nearRange) / std::max(prof.frontRange - prof.nearRange, 1e-6), 0.0, 1.0);
    double bAngle = std::clamp((angle - prof.centreHalfFovDeg) / std::max(halfH - prof.centreHalfFovDeg, 1e-6), 0.0, 1.0);
    badness = std::max(bRange, bAngle);
    return true;
}

void SceneLandmarks::Publish()
{
    if(!pub_ || !publishing_)
        return;

    const DetectionProfile& prof = Profiles()[profile_];
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    if(uniform(rng_) < prof.frameDropProbability)
        return; // the whole frame is lost

    vortex_msgs::msg::LandmarkArray msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = frameId_;
    double now = nh_->get_clock()->now().seconds();

    auto entry = [&](const SceneLandmark& l, uint16_t subtype, const Vector3& p, std::optional<Quaternion> q, double angleStd) {
        vortex_msgs::msg::Landmark e;
        e.header = msg.header;
        e.id = (int32_t)msg.landmarks.size();
        e.type.value = l.type;
        e.subtype.value = subtype;
        e.pose.pose.position.x = p.getX();
        e.pose.pose.position.y = p.getY();
        e.pose.pose.position.z = p.getZ();
        double variance = prof.positionNoise > 0.0 ? prof.positionNoise * prof.positionNoise : 1e-4;
        for(size_t i : {0, 7, 14})
            e.pose.covariance[i] = variance;
        if(q)
        {
            e.pose.pose.orientation.x = q->getX();
            e.pose.pose.orientation.y = q->getY();
            e.pose.pose.orientation.z = q->getZ();
            e.pose.pose.orientation.w = q->getW();
            for(size_t i : {21, 28, 35})
                e.pose.covariance[i] = std::pow(std::max(angleStd, M_PI / 180.0), 2);
        }
        else
        {
            e.pose.pose.orientation.w = 1.0;
            for(size_t i : {21, 28, 35})
                e.pose.covariance[i] = 1000.0; // no orientation
        }
        msg.landmarks.push_back(e);
    };

    std::vector<std::pair<const SceneLandmark*, Vector3>> seen;
    for(const SceneLandmark& l : landmarks_)
    {
        Vector3 p;
        Transform T;
        if(l.kind != SceneLandmark::Kind::LANDMARK || !l.enabled || !WorldPosition(l, p) || !EntityTransform(l.entityName, T))
            continue;

        double b, range;
        Vector3 viewDir;
        if(!InView(l, p, b, viewDir, range))
            continue;

        // Occlusions that last a while, then detections that are just missed
        std::string key = l.entityName + "/" + l.typeName + "/" + l.subtypeName + "/" + std::to_string(p.getX());
        if(occludedUntil_[key] > now)
            continue;
        if(uniform(rng_) < Lerp(prof.dropoutRatePerSec, b) * timerPeriod_)
        {
            occludedUntil_[key] = now + prof.dropoutDurationSec[0] + uniform(rng_) * (prof.dropoutDurationSec[1] - prof.dropoutDurationSec[0]);
            continue;
        }
        if(uniform(rng_) > Lerp(prof.detectionProbability, b))
            continue;
        seen.push_back({&l, p});

        // Position errors: along and across the line of sight, more at the edge of the view
        Vector3 noisy = p;
        if(range > 0.0)
        {
            Vector3 across1 = viewDir.cross(std::abs(viewDir.getZ()) < 0.9 ? Vector3(0, 0, 1) : Vector3(1, 0, 0)).normalized();
            Vector3 across2 = viewDir.cross(across1);
            double gain = 1.0 + prof.offAxisNoiseGain * b;
            double along = (prof.rangeNoiseAlong[0] + prof.rangeNoiseAlong[1] * range) * gain;
            double across = (prof.rangeNoiseAcross[0] + prof.rangeNoiseAcross[1] * range) * gain;
            noisy += viewDir * (normal(rng_) * along + prof.rangeBiasPerM * range);
            noisy += across1 * (normal(rng_) * across) + across2 * (normal(rng_) * across);
        }
        if(prof.positionNoise > 0.0)
            noisy += Vector3(normal(rng_), normal(rng_), normal(rng_)) * prof.positionNoise;
        if(uniform(rng_) < Lerp(prof.outlierProbability, b))
            noisy += Vector3(normal(rng_), normal(rng_), normal(rng_)) * prof.outlierStd;

        // Class confusion: another subtype of the same type
        uint16_t subtype = l.subtype;
        auto subtypes = SubtypeNames().find(l.typeName);
        if(subtypes != SubtypeNames().end() && subtypes->second.size() > 1 && uniform(rng_) < Lerp(prof.classConfusionProbability, b))
        {
            auto other = std::next(subtypes->second.begin(), std::uniform_int_distribution<size_t>(0, subtypes->second.size() - 1)(rng_));
            subtype = other->second;
        }

        std::optional<Quaternion> q;
        double angleStd = Lerp(prof.orientationNoiseDeg, b) * M_PI / 180.0;
        if(l.rotation)
        {
            // Full orientation, turned by a small random rotation for the noise
            q = T.getRotation() * (*l.rotation);
            if(angleStd > 0.0)
            {
                Vector3 axis(normal(rng_), normal(rng_), normal(rng_));
                if(axis.length() > 1e-6)
                    q = Quaternion(axis.normalized(), normal(rng_) * angleStd) * (*q);
            }
        }
        else if(l.normal)
        {
            // Surface normal: yaw only, like robosub_dummy_publisher
            Vector3 n = T.getBasis() * (*l.normal);
            q = Quaternion(std::atan2(n.getY(), n.getX()) + (angleStd > 0.0 ? normal(rng_) * angleStd : 0.0), 0, 0);
        }
        entry(l, subtype, noisy, q, angleStd);
    }

    // False positives: clutter of the same class near landmarks in view
    if(!seen.empty())
    {
        std::poisson_distribution<int> clutter(Lerp(prof.falsePositiveRate, 0.5));
        for(int i = clutter(rng_); i > 0; --i)
        {
            const auto& [l, p] = seen[std::uniform_int_distribution<size_t>(0, seen.size() - 1)(rng_)];
            double a = uniform(rng_) * 2 * M_PI, r = uniform(rng_) * prof.falsePositiveRadius;
            entry(*l, l->subtype, p + Vector3(r * std::cos(a), r * std::sin(a), 0.0), std::nullopt, 0.0);
        }
    }

    pub_->publish(msg);
}

}

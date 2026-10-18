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

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
#include "vortex_msgs/msg/landmark_subtype.hpp"
#include "vortex_msgs/msg/landmark_type.hpp"
#include <algorithm>
#include <cmath>

namespace sf
{

using vortex_msgs::msg::LandmarkSubtype;
using vortex_msgs::msg::LandmarkType;

const std::map<std::string, uint16_t>& SceneLandmarks::TypeNames()
{
    static const std::map<std::string, uint16_t> names = {
        {"ARUCO_MARKER", LandmarkType::ARUCO_MARKER}, {"ARUCO_BOARD", LandmarkType::ARUCO_BOARD},
        {"PIPELINE_START", LandmarkType::PIPELINE_START}, {"PIPELINE_END", LandmarkType::PIPELINE_END},
        {"VALVE", LandmarkType::VALVE}, {"GATE", LandmarkType::GATE}, {"SLALOM_PIPE", LandmarkType::SLALOM_PIPE},
        {"TORPEDO_BOARD", LandmarkType::TORPEDO_BOARD}, {"BIN", LandmarkType::BIN},
        {"PATH_MARKER", LandmarkType::PATH_MARKER}, {"TABLE", LandmarkType::TABLE},
        {"OCTAGON", LandmarkType::OCTAGON}, {"PINGER", LandmarkType::PINGER},
    };
    return names;
}

const std::map<std::string, std::map<std::string, uint16_t>>& SceneLandmarks::SubtypeNames()
{
    // Subtype numbers repeat between types, so they are listed per type (as in LandmarkSubtype.msg)
    static const std::map<std::string, std::map<std::string, uint16_t>> names = {
        {"ARUCO_BOARD", {{"ARUCO_BOARD_CAMERA", LandmarkSubtype::ARUCO_BOARD_CAMERA},
                         {"ARUCO_BOARD_SONAR", LandmarkSubtype::ARUCO_BOARD_SONAR},
                         {"ARUCO_BOARD_DETECTION", LandmarkSubtype::ARUCO_BOARD_DETECTION}}},
        {"VALVE", {{"VALVE_VERTICAL", LandmarkSubtype::VALVE_VERTICAL},
                   {"VALVE_HORIZONTAL", LandmarkSubtype::VALVE_HORIZONTAL}}},
        {"PIPELINE_START", {{"PIPELINE_START_CAMERA", LandmarkSubtype::PIPELINE_START_CAMERA},
                            {"PIPELINE_START_SONAR", LandmarkSubtype::PIPELINE_START_SONAR}}},
        {"GATE", {{"GATE_SEARCH_RESCUE", LandmarkSubtype::GATE_SEARCH_RESCUE},
                  {"GATE_SURVEY_REPAIR", LandmarkSubtype::GATE_SURVEY_REPAIR}}},
        {"SLALOM_PIPE", {{"SLALOM_PIPE_WHITE", LandmarkSubtype::SLALOM_PIPE_WHITE},
                         {"SLALOM_PIPE_RED", LandmarkSubtype::SLALOM_PIPE_RED}}},
        {"TORPEDO_BOARD", {{"TORPEDO_BOARD_WHOLE", LandmarkSubtype::TORPEDO_BOARD_WHOLE},
                           {"TORPEDO_TARGET_LARGE_SEARCH_RESCUE", LandmarkSubtype::TORPEDO_TARGET_LARGE_SEARCH_RESCUE},
                           {"TORPEDO_TARGET_LARGE_SURVEY_REPAIR", LandmarkSubtype::TORPEDO_TARGET_LARGE_SURVEY_REPAIR},
                           {"TORPEDO_TARGET_SMALL_SEARCH_RESCUE", LandmarkSubtype::TORPEDO_TARGET_SMALL_SEARCH_RESCUE},
                           {"TORPEDO_TARGET_SMALL_SURVEY_REPAIR", LandmarkSubtype::TORPEDO_TARGET_SMALL_SURVEY_REPAIR},
                           {"TORPEDO_ICON_FIRE", LandmarkSubtype::TORPEDO_ICON_FIRE},
                           {"TORPEDO_ICON_BLOOD", LandmarkSubtype::TORPEDO_ICON_BLOOD},
                           {"TORPEDO_ICON_FIRETRUCK", LandmarkSubtype::TORPEDO_ICON_FIRETRUCK},
                           {"TORPEDO_ICON_AMBULANCE", LandmarkSubtype::TORPEDO_ICON_AMBULANCE}}},
        {"BIN", {{"BIN_SEARCH_RESCUE", LandmarkSubtype::BIN_SEARCH_RESCUE},
                 {"BIN_SURVEY_REPAIR", LandmarkSubtype::BIN_SURVEY_REPAIR},
                 {"BIN_UNCLASSIFIED", LandmarkSubtype::BIN_UNCLASSIFIED},
                 {"BIN_STRUCTURE", LandmarkSubtype::BIN_STRUCTURE}}},
        {"PATH_MARKER", {{"PATH_MARKER_WHOLE", LandmarkSubtype::PATH_MARKER_WHOLE}}},
        {"TABLE", {{"TABLE_WHOLE", LandmarkSubtype::TABLE_WHOLE},
                   {"TABLE_ITEM_NUTBOLT", LandmarkSubtype::TABLE_ITEM_NUTBOLT},
                   {"TABLE_ITEM_ELECTRIC", LandmarkSubtype::TABLE_ITEM_ELECTRIC},
                   {"TABLE_ITEM_PILL", LandmarkSubtype::TABLE_ITEM_PILL},
                   {"TABLE_ITEM_BANDAID", LandmarkSubtype::TABLE_ITEM_BANDAID},
                   {"TABLE_BASKET_SURVEY_REPAIR", LandmarkSubtype::TABLE_BASKET_SURVEY_REPAIR},
                   {"TABLE_BASKET_SEARCH_RESCUE", LandmarkSubtype::TABLE_BASKET_SEARCH_RESCUE}}},
        {"OCTAGON", {{"OCTAGON_WHOLE", LandmarkSubtype::OCTAGON_WHOLE},
                     {"OCTAGON_IMAGE_REPAIR", LandmarkSubtype::OCTAGON_IMAGE_REPAIR},
                     {"OCTAGON_IMAGE_RESCUE", LandmarkSubtype::OCTAGON_IMAGE_RESCUE},
                     {"OCTAGON_IMAGE_SEARCH", LandmarkSubtype::OCTAGON_IMAGE_SEARCH},
                     {"OCTAGON_IMAGE_SURVEY", LandmarkSubtype::OCTAGON_IMAGE_SURVEY}}},
        {"PINGER", {{"PINGER_DEPLOY", LandmarkSubtype::PINGER_DEPLOY},
                    {"PINGER_RESTORE", LandmarkSubtype::PINGER_RESTORE}}},
    };
    return names;
}

SceneLandmarks::SceneLandmarks(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh)
    : sm_(sm), nh_(nh)
{
}

void SceneLandmarks::Configure(const std::string& topic, const std::string& frameId, double rate)
{
    frameId_ = frameId;
    pub_ = nh_->create_publisher<vortex_msgs::msg::LandmarkArray>(topic, 10);
    timer_ = nh_->create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
                                    std::bind(&SceneLandmarks::Publish, this));
    RCLCPP_INFO(nh_->get_logger(), "Scene landmarks: publishing on '%s' in frame '%s' at %.1f Hz.",
                topic.c_str(), frameId.c_str(), rate);
}

bool SceneLandmarks::Add(const SceneLandmark& landmark, std::string& error)
{
    SceneLandmark l = landmark;
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

bool SceneLandmarks::EntityTransform(const std::string& name, Transform& T) const
{
    Entity* ent = sm_->getEntity(name);
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
        sm_->getEntity(landmark.entityName)->getAABB(min, max);
        position = (min + max) / Scalar(2);
    }
    return true;
}

void SceneLandmarks::Publish()
{
    if(!pub_)
        return;

    vortex_msgs::msg::LandmarkArray msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = frameId_;

    for(const SceneLandmark& l : landmarks_)
    {
        Vector3 p;
        Transform T;
        if(!WorldPosition(l, p) || !EntityTransform(l.entityName, T))
            continue;

        vortex_msgs::msg::Landmark entry;
        entry.header = msg.header;
        entry.id = (int32_t)msg.landmarks.size();
        entry.type.value = l.type;
        entry.subtype.value = l.subtype;
        entry.pose.pose.position.x = p.getX();
        entry.pose.pose.position.y = p.getY();
        entry.pose.pose.position.z = p.getZ();
        for(size_t i : {0, 7, 14})
            entry.pose.covariance[i] = 1e-4;

        if(l.normal)
        {
            // Surface normal: yaw only, like robosub_dummy_publisher
            Vector3 n = T.getBasis() * (*l.normal);
            double yaw = std::atan2(n.getY(), n.getX());
            entry.pose.pose.orientation.z = std::sin(yaw / 2.0);
            entry.pose.pose.orientation.w = std::cos(yaw / 2.0);
            for(size_t i : {21, 28, 35})
                entry.pose.covariance[i] = std::pow(M_PI / 180.0, 2);
        }
        else
        {
            entry.pose.pose.orientation.w = 1.0;
            for(size_t i : {21, 28, 35})
                entry.pose.covariance[i] = 1000.0; // no orientation
        }
        msg.landmarks.push_back(entry);
    }
    pub_->publish(msg);
}

}

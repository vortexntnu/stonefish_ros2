/*
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

//
//  SceneEditor.h
//  stonefish_ros2
//
//  Edit mode of the graphical simulator (Tab), working like the Unity scene
//  view: click to select, Q/W/E for the select/move/rotate tools with gizmo
//  handles, right mouse + WASD to fly. Landmarks are markers that can be
//  selected and moved like child empties. Undo, and save back into the
//  scenario files.
//

#ifndef __Stonefish_SceneEditor__
#define __Stonefish_SceneEditor__

#include "stonefish_ros2/SceneLandmarks.h"
#include <SDL2/SDL.h>
#include <Stonefish/StonefishCommon.h>
#include <deque>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace sf
{
    class GraphicalSimulationApp;
    class ROS2SimulationManager;
    class StaticEntity;

    class SceneEditor
    {
    public:
        SceneEditor(GraphicalSimulationApp* app, ROS2SimulationManager* sim);

        bool isActive() const { return active_; }

        //! Returns true if the key was used by the editor (and must not reach the camera).
        bool KeyDown(SDL_Event* event);
        void MouseMove(SDL_Event* event);
        void MouseUp(SDL_Event* event);
        void DoHUD();

        //! Writes moved objects and changed landmarks back into the scenario files.
        bool Save(std::string& message);

    private:
        enum class Tool { SELECT, MOVE, ROTATE };
        //! Gizmo handles: the three axes, the horizontal plane square, and the three rotation rings.
        enum class Handle { NONE, AXIS_X, AXIS_Y, AXIS_Z, PLANE, RING_X, RING_Y, RING_Z };

        //! A landmark by its object and its index among that object's landmarks.
        struct LandmarkRef
        {
            std::string entity;
            size_t index;
            bool operator==(const LandmarkRef& o) const { return entity == o.entity && index == o.index; }
        };

        //! Everything an edit can change, for undo.
        struct Snapshot
        {
            std::vector<std::pair<std::string, Transform>> poses;
            std::vector<SceneLandmark> landmarks;
            std::set<std::string> moved;
            std::set<std::string> landmarksChanged;
        };

        // Selection
        void UpdateSelection();
        void SelectGroup(const std::string& groupName);
        bool GroupBounds(Vector3& min, Vector3& max) const;
        bool GroupCenter(Vector3& center) const;

        // Editing
        void Translate(const Vector3& delta);
        void RotateAbout(const Vector3& axis, Scalar angle, const Vector3& pivot);
        void MoveLandmark(const LandmarkRef& ref, const Vector3& worldPosition);
        bool LandmarkWorld(const LandmarkRef& ref, Vector3& p) const;
        std::vector<LandmarkRef> AllLandmarks() const;
        void AddLandmark();
        void RemoveLandmarks();
        void RemoveSelectedLandmark();
        void PushUndo();
        void Undo();

        // Gizmo
        bool GizmoCenter(Vector3& c) const;
        Scalar GizmoSize(const Vector3& c) const;
        Vector3 HandleAxis(Handle h) const;
        Handle HandleUnder(int x, int y) const;
        void StartGizmoDrag(Handle h, int x, int y);
        void UpdateGizmoDrag(int x, int y, bool snap);
        void DrawGizmo();

        // Screen and picking
        bool ToScreen(const Vector3& p, float& sx, float& sy) const;
        bool PlaneHit(int x, int y, const Vector3& point, const Vector3& normal, Vector3& hit) const;
        void DrawOverlay();
        void DrawPanel();
        void DrawProfileMenu(float x, float y, float w);
        void DrawWaypoint();
        void SetStatus(const std::string& text);

        static std::string GroupOf(const std::string& entityName);

        GraphicalSimulationApp* app_;
        ROS2SimulationManager* sim_;
        bool active_ = false;
        Tool tool_ = Tool::MOVE;

        // Selection: all static objects that share a name prefix before "__" (one per material)
        std::string groupName_;
        std::string pickedName_;
        std::vector<StaticEntity*> group_;
        std::optional<LandmarkRef> selectedLandmark_;
        std::optional<LandmarkRef> hoverLandmark_;

        // Mouse and gizmo dragging
        bool leftWasDown_ = false;
        bool rightDown_ = false;
        Handle hover_ = Handle::NONE;
        Handle dragging_ = Handle::NONE;
        int pressX_ = 0, pressY_ = 0;
        Vector3 dragCenter_;
        Vector3 dragHitStart_;
        float dragAngleStart_ = 0.f;
        Scalar dragRotationSign_ = 1;
        std::vector<Transform> dragStartTransforms_;
        Vector3 dragLandmarkStart_;

        std::deque<Snapshot> undo_;
        std::set<std::string> moved_;
        std::set<std::string> landmarksChanged_;
        bool profileChanged_ = false;
        unsigned int typeIndex_ = 0;
        unsigned int subtypeIndex_ = 0;
        std::string status_;
        uint64_t statusTime_ = 0;
    };
}

#endif

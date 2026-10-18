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
//  Edit mode of the graphical simulator (toggle with E): select objects and
//  drag them around the pool, see and drag their landmark points (like child
//  empties in Unity), undo, and save the result back into the scenario files.
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

        //! Returns true if the key was used by the editor.
        bool KeyDown(SDL_Event* event);
        void MouseMove(SDL_Event* event);
        void MouseUp(SDL_Event* event);
        void DoHUD();

        //! Writes moved objects and changed landmarks back into the scenario files.
        bool Save(std::string& message);

    private:
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

        enum class Drag { NONE, OBJECT, LANDMARK };

        void UpdateSelection();
        void SelectGroup(const std::string& groupName);
        bool GroupBounds(Vector3& min, Vector3& max) const;
        bool GroupCenter(Vector3& center) const;
        void Translate(const Vector3& delta);
        void Rotate(Scalar yaw);
        void MoveLandmark(const LandmarkRef& ref, const Vector3& worldPosition);
        bool LandmarkWorld(const LandmarkRef& ref, Vector3& p) const;
        std::vector<LandmarkRef> AllLandmarks() const;
        bool StartDrag(int x, int y, const Vector3& through);
        bool PlaneHit(int x, int y, Vector3& hit) const;
        bool ToScreen(const Vector3& p, float& sx, float& sy) const;
        void AddLandmark();
        void RemoveLandmarks();
        void RemoveSelectedLandmark();
        void PushUndo();
        void Undo();
        void DrawOverlay();
        void SetStatus(const std::string& text);

        static std::string GroupOf(const std::string& entityName);

        GraphicalSimulationApp* app_;
        ROS2SimulationManager* sim_;
        bool active_ = false;

        // Selection: all static objects that share a name prefix before "__" (one per material)
        std::string groupName_;
        std::string pickedName_;
        std::vector<StaticEntity*> group_;
        std::optional<LandmarkRef> selectedLandmark_;
        std::optional<LandmarkRef> hoverLandmark_;

        // Mouse and dragging (on a plane: horizontal, or vertical with Shift)
        bool leftWasDown_ = false;
        Drag drag_ = Drag::NONE;
        bool pressOnLandmark_ = false;
        bool dragVertical_ = false;
        Vector3 planePoint_;
        Vector3 planeNormal_;
        Vector3 dragStart_;
        Vector3 dragLandmarkStart_;
        std::vector<Transform> dragStartTransforms_;

        std::deque<Snapshot> undo_;
        std::set<std::string> moved_;
        std::set<std::string> landmarksChanged_;
        unsigned int typeIndex_ = 0;
        unsigned int subtypeIndex_ = 0;
        std::string status_;
        uint64_t statusTime_ = 0;
    };
}

#endif

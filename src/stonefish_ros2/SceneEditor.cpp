/*
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

//
//  SceneEditor.cpp
//  stonefish_ros2
//

#include "stonefish_ros2/SceneEditor.h"
#include "stonefish_ros2/ROS2SimulationManager.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
#include <Stonefish/graphics/IMGUI.h>
#include <Stonefish/graphics/OpenGLContent.h>
#include <Stonefish/graphics/OpenGLPipeline.h>
#include <Stonefish/graphics/OpenGLPrinter.h>
#include <Stonefish/graphics/OpenGLState.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

namespace sf
{

namespace
{
    const Scalar kNudge = 0.05;       // [m], x10 with Shift
    const Scalar kYawStep = 5.0;      // [deg], /5 with Shift
    const int kOwner = 917;           // IMGUI owner id of the editor panel
    const float kPickRadius = 12.f;   // [px] for clicking a landmark marker
    const size_t kUndoDepth = 50;
    const float kHandleRadius = 9.f;  // [px] for grabbing a gizmo handle
    const int kRingSegments = 48;
    const Scalar kSnapMove = 0.1;     // [m] with Ctrl
    const Scalar kSnapAngleDeg = 15.0;
    const float kPanelHeight = 669.f;

    std::string ReadFile(const std::string& path)
    {
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    bool WriteFile(const std::string& path, const std::string& text)
    {
        std::ofstream f(path);
        f << text;
        return f.good();
    }

    std::string Format(Scalar a, Scalar b, Scalar c)
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "%.4f %.4f %.4f", (double)a, (double)b, (double)c);
        return std::string(buf);
    }

    //! "$(find pkg)/x" -> share/pkg/x; relative paths are taken from the including file's folder.
    std::string ResolveInclude(const std::string& path, const std::string& includingFile)
    {
        static const std::regex find(R"(\$\(find ([^)]+)\))");
        std::smatch m;
        std::string p = path;
        if(std::regex_search(p, m, find))
        {
            try { p = m.prefix().str() + ament_index_cpp::get_package_share_directory(m[1].str()) + m.suffix().str(); }
            catch(...) { return ""; }
        }
        if(p.find("$(") != std::string::npos)
            return ""; // depends on a launch parameter (e.g. the drone file), not part of the course
        std::filesystem::path fp(p);
        if(fp.is_relative())
            fp = std::filesystem::path(includingFile).parent_path() / fp;
        return std::filesystem::exists(fp) ? std::filesystem::weakly_canonical(fp).string() : "";
    }

    //! The scenario file and everything it includes.
    void CollectFiles(const std::string& file, std::vector<std::string>& files)
    {
        if(file.empty() || std::find(files.begin(), files.end(), file) != files.end())
            return;
        files.push_back(file);
        static const std::regex include(R"re(<include\s+file\s*=\s*"([^"]+)")re");
        std::string text = ReadFile(file);
        for(std::sregex_iterator it(text.begin(), text.end(), include), end; it != end; ++it)
            CollectFiles(ResolveInclude((*it)[1].str(), file), files);
    }

    //! [begin, end) of the <static|dynamic name="name"> element, or npos.
    std::pair<size_t, size_t> FindObject(const std::string& text, const std::string& name)
    {
        std::regex open("<(static|dynamic)\\b[^>]*\\bname\\s*=\\s*\"" + std::regex_replace(name, std::regex(R"([.^$|()\[\]{}*+?\\])"), R"(\$&)") + "\"");
        std::smatch m;
        if(!std::regex_search(text, m, open))
            return {std::string::npos, std::string::npos};
        size_t begin = m.position(0);
        size_t end = text.find("</" + m[1].str() + ">", begin);
        return {begin, end == std::string::npos ? std::string::npos : end};
    }

    std::string SetAttribute(const std::string& tag, const std::string& attr, const std::string& value)
    {
        std::regex re("\\b" + attr + "\\s*=\\s*\"[^\"]*\"");
        if(std::regex_search(tag, re))
            return std::regex_replace(tag, re, attr + "=\"" + value + "\"", std::regex_constants::format_first_only);
        size_t close = tag.rfind("/>");
        return tag.substr(0, close) + " " + attr + "=\"" + value + "\"" + tag.substr(close);
    }

    std::string LandmarkLine(const SceneLandmark& l)
    {
        if(l.kind != SceneLandmark::Kind::LANDMARK)
        {
            char yaw[32];
            snprintf(yaw, sizeof(yaw), "%.1f", std::atan2(l.normal->getY(), l.normal->getX()) * 180.0 / M_PI);
            return std::string("  <") + (l.kind == SceneLandmark::Kind::SPAWN ? "spawn" : "prior") + " name=\"" + l.subtypeName
                   + "\" object=\"" + l.entityName + "\" point=\"" + Format(l.point->getX(), l.point->getY(), l.point->getZ())
                   + "\" yaw_deg=\"" + yaw + "\"/>\n";
        }
        std::string line = "  <landmark object=\"" + l.entityName + "\" type=\"" + l.typeName + "\"";
        if(!l.subtypeName.empty())
            line += " subtype=\"" + l.subtypeName + "\"";
        if(l.point)
            line += " point=\"" + Format(l.point->getX(), l.point->getY(), l.point->getZ()) + "\"";
        if(l.normal)
            line += " normal=\"" + Format(l.normal->getX(), l.normal->getY(), l.normal->getZ()) + "\"";
        if(l.rotation)
        {
            Scalar yaw, pitch, roll;
            Matrix3(*l.rotation).getEulerYPR(yaw, pitch, roll);
            line += " rpy=\"" + Format(roll, pitch, yaw) + "\"";
        }
        if(!l.enabled)
            line += " enabled=\"false\"";
        if(l.downCamera)
            line += " camera=\"down\"";
        return line + "/>\n";
    }

    //! Removes every whole-line <landmark .../> for the object (standalone, or inline in its block).
    std::string RemoveLandmarkLines(const std::string& text, const std::string& name, bool inlineOnly)
    {
        std::string out;
        std::istringstream in(text);
        std::string line;
        auto block = FindObject(text, name);
        size_t pos = 0;
        while(std::getline(in, line))
        {
            bool isLandmark = line.find("<landmark") != std::string::npos || line.find("<spawn ") != std::string::npos
                              || line.find("<prior ") != std::string::npos;
            bool standalone = isLandmark && line.find("object=\"" + name + "\"") != std::string::npos;
            bool inBlock = isLandmark && block.first != std::string::npos && pos > block.first && pos < block.second;
            if(!((standalone && !inlineOnly) || inBlock))
                out += line + "\n";
            pos += line.size() + 1;
        }
        return out;
    }
}

SceneEditor::SceneEditor(GraphicalSimulationApp* app, ROS2SimulationManager* sim)
    : app_(app), sim_(sim)
{
}

std::string SceneEditor::GroupOf(const std::string& entityName)
{
    size_t sep = entityName.find("__");
    return sep == std::string::npos ? entityName : entityName.substr(0, sep);
}

void SceneEditor::SetStatus(const std::string& text)
{
    status_ = text;
    statusTime_ = GetTimeInMicroseconds();
}

// --- Selection ---------------------------------------------------------------

void SceneEditor::SelectGroup(const std::string& groupName)
{
    groupName_ = groupName;
    group_.clear();
    for(unsigned int i = 0; Entity* ent = sim_->getEntity(i); ++i)
        if(ent->getType() == EntityType::STATIC && GroupOf(ent->getName()) == groupName)
            group_.push_back((StaticEntity*)ent);
}

void SceneEditor::UpdateSelection()
{
    Entity* picked = app_->getSelectedEntity().first;
    std::string name = picked != nullptr ? picked->getName() : "";
    if(name == pickedName_)
        return;
    pickedName_ = name;
    if(dragging_ != Handle::NONE)
        return; // the click on a gizmo handle also picked whatever is behind it; keep the selection
    groupName_.clear();
    group_.clear();
    if(picked == nullptr)
        return;
    if(picked->getType() != EntityType::STATIC)
        SetStatus(name + " can not be moved yet (only static objects)");
    else
        SelectGroup(GroupOf(name));
}

bool SceneEditor::GroupBounds(Vector3& min, Vector3& max) const
{
    if(group_.empty())
        return false;
    Vector3 bmin, bmax;
    group_[0]->getAABB(min, max);
    for(StaticEntity* e : group_)
    {
        e->getAABB(bmin, bmax);
        min.setMin(bmin);
        max.setMax(bmax);
    }
    return true;
}

bool SceneEditor::GroupCenter(Vector3& center) const
{
    Vector3 min, max;
    if(!GroupBounds(min, max))
        return false;
    center = (min + max) / Scalar(2);
    return true;
}

// --- Landmarks -----------------------------------------------------------------

std::vector<SceneEditor::LandmarkRef> SceneEditor::AllLandmarks() const
{
    std::vector<LandmarkRef> refs;
    std::map<std::string, size_t> count;
    for(const SceneLandmark& l : sim_->getLandmarks().All())
        refs.push_back({l.entityName, count[l.entityName]++});
    return refs;
}

bool SceneEditor::LandmarkWorld(const LandmarkRef& ref, Vector3& p) const
{
    std::vector<SceneLandmark> ls = sim_->getLandmarks().Of(ref.entity);
    return ref.index < ls.size() && sim_->getLandmarks().WorldPosition(ls[ref.index], p);
}

void SceneEditor::MoveLandmark(const LandmarkRef& ref, const Vector3& worldPosition)
{
    std::vector<SceneLandmark> ls = sim_->getLandmarks().Of(ref.entity);
    Transform T;
    if(ref.index >= ls.size() || !sim_->getLandmarks().EntityTransform(ref.entity, T))
        return;
    // Stored in the object's frame, so the point keeps following its object
    ls[ref.index].point = T.inverse() * worldPosition;
    sim_->getLandmarks().Set(ref.entity, ls);
    landmarksChanged_.insert(ref.entity);
}

void SceneEditor::AddLandmark()
{
    Entity* picked = sim_->getEntity(pickedName_);
    Vector3 c;
    if(picked == nullptr || group_.empty() || !GroupCenter(c))
        return;

    auto type = std::next(SceneLandmarks::TypeNames().begin(), typeIndex_);
    SceneLandmark l;
    l.entityName = pickedName_;
    l.typeName = type->first;
    auto subtypes = SceneLandmarks::SubtypeNames().find(l.typeName);
    if(subtypes != SceneLandmarks::SubtypeNames().end() && subtypeIndex_ < subtypes->second.size())
        l.subtypeName = std::next(subtypes->second.begin(), subtypeIndex_)->first;
    // At the centre of the whole group, stored in the picked object's frame so it follows it
    l.point = ((StaticEntity*)picked)->getTransform().inverse() * c;

    PushUndo();
    std::string error;
    if(sim_->getLandmarks().Add(l, error))
    {
        landmarksChanged_.insert(pickedName_);
        selectedLandmark_ = LandmarkRef{pickedName_, sim_->getLandmarks().Of(pickedName_).size() - 1};
        tool_ = Tool::MOVE;
        SetStatus("Added " + l.typeName + " - move it with the arrows");
    }
    else
    {
        undo_.pop_back();
        SetStatus(error);
    }
}

void SceneEditor::RemoveLandmarks()
{
    PushUndo();
    size_t n = 0;
    for(StaticEntity* e : group_)
    {
        n += sim_->getLandmarks().Of(e->getName()).size();
        sim_->getLandmarks().Set(e->getName(), {});
        landmarksChanged_.insert(e->getName());
    }
    selectedLandmark_.reset();
    SetStatus("Removed " + std::to_string(n) + " landmark(s) of " + groupName_);
}

void SceneEditor::RemoveSelectedLandmark()
{
    if(!selectedLandmark_)
        return;
    std::vector<SceneLandmark> ls = sim_->getLandmarks().Of(selectedLandmark_->entity);
    if(selectedLandmark_->index >= ls.size())
        return;
    PushUndo();
    SetStatus("Removed " + ls[selectedLandmark_->index].typeName + " landmark");
    ls.erase(ls.begin() + selectedLandmark_->index);
    sim_->getLandmarks().Set(selectedLandmark_->entity, ls);
    landmarksChanged_.insert(selectedLandmark_->entity);
    selectedLandmark_.reset();
}

std::optional<SceneLandmark> SceneEditor::SelectedPose() const
{
    if(!selectedLandmark_)
        return std::nullopt;
    std::vector<SceneLandmark> ls = sim_->getLandmarks().Of(selectedLandmark_->entity);
    if(selectedLandmark_->index >= ls.size() || ls[selectedLandmark_->index].kind == SceneLandmark::Kind::LANDMARK
       || sim_->getLandmarks().FindEntity(selectedLandmark_->entity) == nullptr)
        return std::nullopt;
    return ls[selectedLandmark_->index];
}

void SceneEditor::AddSpawn()
{
    Entity* picked = sim_->getEntity(pickedName_);
    Vector3 c;
    if(picked == nullptr || group_.empty() || !GroupCenter(c))
        return;
    // 2 m from the object towards the camera, 1 m deep, facing the object
    glm::vec3 eye = sim_->getTrackball()->GetEyePosition();
    Vector3 toCamera(eye.x - c.getX(), eye.y - c.getY(), 0);
    toCamera = toCamera.length() > 1e-3 ? toCamera.normalized() : Vector3(-1, 0, 0);
    Vector3 p = c + toCamera * 2.0;
    p.setZ(1.0);

    SceneLandmark l;
    l.kind = SceneLandmark::Kind::SPAWN;
    l.entityName = pickedName_;
    l.typeName = "SPAWN";
    l.subtypeName = groupName_;
    for(int i = 2; std::any_of(sim_->getLandmarks().All().begin(), sim_->getLandmarks().All().end(),
                               [&](const SceneLandmark& o) { return o.kind == l.kind && o.subtypeName == l.subtypeName; }); ++i)
        l.subtypeName = groupName_ + "_" + std::to_string(i);
    Transform T = ((StaticEntity*)picked)->getTransform();
    l.point = T.inverse() * p;
    l.normal = T.getBasis().transpose() * -toCamera;

    PushUndo();
    std::string error;
    sim_->getLandmarks().Add(l, error);
    landmarksChanged_.insert(pickedName_);
    selectedLandmark_ = LandmarkRef{pickedName_, sim_->getLandmarks().Of(pickedName_).size() - 1};
    tool_ = Tool::MOVE;
    SetStatus("Added spawn '" + l.subtypeName + "' (spawn:=" + l.subtypeName + ") - move it, E to turn it");
}

// --- Editing -------------------------------------------------------------------

void SceneEditor::PushUndo()
{
    Snapshot s;
    for(unsigned int i = 0; Entity* ent = sim_->getEntity(i); ++i)
        if(ent->getType() == EntityType::STATIC)
            s.poses.push_back({ent->getName(), ((StaticEntity*)ent)->getTransform()});
    s.landmarks = sim_->getLandmarks().All();
    s.moved = moved_;
    s.landmarksChanged = landmarksChanged_;
    undo_.push_back(s);
    if(undo_.size() > kUndoDepth)
        undo_.pop_front();
}

void SceneEditor::Undo()
{
    if(undo_.empty())
    {
        SetStatus("Nothing to undo");
        return;
    }
    const Snapshot& s = undo_.back();
    for(const auto& [name, T] : s.poses)
        if(Entity* ent = sim_->getEntity(name))
            ((StaticEntity*)ent)->setTransform(T);
    sim_->getLandmarks().SetAll(s.landmarks);
    moved_ = s.moved;
    landmarksChanged_ = s.landmarksChanged;
    undo_.pop_back();
    selectedLandmark_.reset();
    SetStatus("Undone");
}

void SceneEditor::Translate(const Vector3& delta)
{
    for(StaticEntity* e : group_)
    {
        Transform T = e->getTransform();
        T.setOrigin(T.getOrigin() + delta);
        e->setTransform(T);
        moved_.insert(e->getName());
    }
}

void SceneEditor::RotateAbout(const Vector3& axis, Scalar angle, const Vector3& pivot)
{
    // About an axis through the pivot (the meshes' own origins can be far away)
    Transform R(Quaternion(axis, angle), Vector3(0, 0, 0));
    Transform toPivot(Quaternion::getIdentity(), pivot);
    for(size_t i = 0; i < group_.size() && i < dragStartTransforms_.size(); ++i)
    {
        group_[i]->setTransform(toPivot * R * toPivot.inverse() * dragStartTransforms_[i]);
        moved_.insert(group_[i]->getName());
    }
}

// --- Screen ----------------------------------------------------------------------

bool SceneEditor::ToScreen(const Vector3& p, float& sx, float& sy) const
{
    OpenGLTrackball* trackball = sim_->getTrackball();
    glm::vec4 clip = trackball->GetProjectionMatrix() * trackball->GetViewMatrix()
                     * glm::vec4((float)p.getX(), (float)p.getY(), (float)p.getZ(), 1.f);
    if(clip.w <= 0.f)
        return false; // behind the camera
    GLint* vp = trackball->GetViewport();
    sx = vp[0] + (clip.x / clip.w * 0.5f + 0.5f) * vp[2];
    sy = (float)app_->getWindowHeight() - (vp[1] + (clip.y / clip.w * 0.5f + 0.5f) * vp[3]);
    delete [] vp;
    return true;
}

bool SceneEditor::PlaneHit(int x, int y, const Vector3& point, const Vector3& normal, Vector3& hit) const
{
    OpenGLTrackball* trackball = sim_->getTrackball();
    glm::vec3 eye = trackball->GetEyePosition();
    glm::vec3 ray = trackball->Ray(x, y);
    Vector3 o(eye.x, eye.y, eye.z);
    Vector3 d(ray.x, ray.y, ray.z);
    Scalar denom = d.dot(normal);
    if(std::abs(denom) < 1e-6)
        return false;
    Scalar t = (point - o).dot(normal) / denom;
    if(t < 0)
        return false;
    hit = o + d * t;
    return true;
}

// --- Gizmo -----------------------------------------------------------------------

bool SceneEditor::GizmoCenter(Vector3& c) const
{
    if(selectedLandmark_)
        return LandmarkWorld(*selectedLandmark_, c);
    return ObjectPivot(c);
}

bool SceneEditor::ObjectPivot(Vector3& c) const
{
    if(pivot_ && !group_.empty())
    {
        c = group_[0]->getTransform().getOrigin(); // the parts of a group share their world_transform
        return true;
    }
    return GroupCenter(c);
}

Scalar SceneEditor::GizmoSize(const Vector3& c) const
{
    // The same size on screen at any distance, like Unity's handles
    glm::vec3 eye = sim_->getTrackball()->GetEyePosition();
    return (c - Vector3(eye.x, eye.y, eye.z)).length() * 0.15;
}

Vector3 SceneEditor::HandleAxis(Handle h) const
{
    Vector3 a;
    switch(h)
    {
        case Handle::AXIS_X: case Handle::RING_X: a = Vector3(1, 0, 0); break;  // north / the object's front
        case Handle::AXIS_Y: case Handle::RING_Y: a = Vector3(0, 1, 0); break;  // east / its right
        case Handle::AXIS_Z: case Handle::RING_Z: a = Vector3(0, 0, -1); break; // up (Stonefish z is down)
        default: return Vector3(0, 0, 0);
    }
    if(!localAxes_ || (selectedLandmark_ && h == Handle::RING_Z))
        return a; // a spawn point or prior turns about the world's up
    // Local: the object's own axes (a landmark uses its object's)
    StaticEntity* e = group_.empty() ? nullptr : group_[0];
    if(selectedLandmark_)
        e = dynamic_cast<StaticEntity*>(sim_->getEntity(selectedLandmark_->entity));
    return e != nullptr ? e->getTransform().getBasis() * a : a;
}

namespace
{
    float SegmentDistance(float px, float py, float ax, float ay, float bx, float by)
    {
        float vx = bx - ax, vy = by - ay;
        float len2 = vx * vx + vy * vy;
        float t = len2 > 0.f ? std::clamp(((px - ax) * vx + (py - ay) * vy) / len2, 0.f, 1.f) : 0.f;
        return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
    }

    //! Two unit vectors perpendicular to the axis, for drawing its ring.
    void RingBasis(const Vector3& axis, Vector3& u, Vector3& v)
    {
        u = std::abs(axis.getZ()) > 0.9 ? Vector3(1, 0, 0) : Vector3(0, 0, 1);
        u = (u - axis * u.dot(axis)).normalized();
        v = axis.cross(u);
    }
}

SceneEditor::Handle SceneEditor::HandleUnder(int x, int y) const
{
    Vector3 c;
    float cx, cy;
    if(tool_ == Tool::SELECT || !GizmoCenter(c) || !ToScreen(c, cx, cy))
        return Handle::NONE;
    Scalar L = GizmoSize(c);
    Handle best = Handle::NONE;
    float bestDist = kHandleRadius;

    if(tool_ == Tool::MOVE)
    {
        // Plane square first: it sits between the X and Y arrows
        float px, py;
        if(ToScreen(c + (HandleAxis(Handle::AXIS_X) + HandleAxis(Handle::AXIS_Y)) * (L * 0.25), px, py) && std::hypot(px - x, py - y) < bestDist)
            return Handle::PLANE;
        for(Handle h : {Handle::AXIS_X, Handle::AXIS_Y, Handle::AXIS_Z})
        {
            float ex, ey;
            if(!ToScreen(c + HandleAxis(h) * L, ex, ey))
                continue;
            float d = SegmentDistance((float)x, (float)y, cx, cy, ex, ey);
            if(d < bestDist)
            {
                bestDist = d;
                best = h;
            }
        }
    }
    else if(!selectedLandmark_ || SelectedPose())
    {
        for(Handle h : {Handle::RING_X, Handle::RING_Y, Handle::RING_Z})
        {
            if(selectedLandmark_ && h != Handle::RING_Z)
                continue; // only the heading
            Vector3 u, v;
            RingBasis(HandleAxis(h), u, v);
            float lx = 0, ly = 0;
            for(int i = 0; i <= kRingSegments; ++i)
            {
                Scalar a = 2 * M_PI * i / kRingSegments;
                float sx, sy;
                if(!ToScreen(c + (u * std::cos(a) + v * std::sin(a)) * L, sx, sy))
                    continue;
                if(i > 0)
                {
                    float d = SegmentDistance((float)x, (float)y, lx, ly, sx, sy);
                    if(d < bestDist)
                    {
                        bestDist = d;
                        best = h;
                    }
                }
                lx = sx;
                ly = sy;
            }
        }
    }
    return best;
}

void SceneEditor::StartGizmoDrag(Handle h, int x, int y)
{
    Vector3 c;
    if(!GizmoCenter(c))
        return;
    PushUndo();
    dragging_ = h;
    pressX_ = x;
    pressY_ = y;
    dragCenter_ = c;
    dragStartTransforms_.clear();
    for(StaticEntity* e : group_)
        dragStartTransforms_.push_back(e->getTransform());
    dragLandmarkStart_ = c;
    Transform poseObject;
    if(std::optional<SceneLandmark> pose = SelectedPose())
        if(sim_->getLandmarks().EntityTransform(pose->entityName, poseObject))
            dragHeadingStart_ = poseObject.getBasis() * *pose->normal;

    dragAxis_ = HandleAxis(h == Handle::PLANE ? Handle::AXIS_Z : h);
    if(h == Handle::PLANE)
        PlaneHit(x, y, c, dragAxis_, dragHitStart_);
    else if(h == Handle::RING_X || h == Handle::RING_Y || h == Handle::RING_Z)
    {
        float cx, cy;
        ToScreen(c, cx, cy);
        dragAngleStart_ = std::atan2((float)y - cy, (float)x - cx);
        // Which way a positive rotation turns on screen, seen from this camera
        Vector3 axis = HandleAxis(h), u, v;
        RingBasis(axis, u, v);
        float ax, ay, bx, by;
        ToScreen(c + u, ax, ay);
        ToScreen(c + Transform(Quaternion(axis, 0.1), Vector3(0, 0, 0)) * u, bx, by);
        float da = std::atan2(by - cy, bx - cx) - std::atan2(ay - cy, ax - cx);
        da = std::atan2(std::sin(da), std::cos(da));
        dragRotationSign_ = da >= 0.f ? 1 : -1;
    }
}

void SceneEditor::UpdateGizmoDrag(int x, int y, bool snap)
{
    Vector3 delta(0, 0, 0);
    switch(dragging_)
    {
        case Handle::AXIS_X:
        case Handle::AXIS_Y:
        case Handle::AXIS_Z:
        {
            // Mouse movement along the arrow on screen, in metres along the axis
            Vector3 axis = dragAxis_;
            float cx, cy, ex, ey;
            if(!ToScreen(dragCenter_, cx, cy) || !ToScreen(dragCenter_ + axis, ex, ey))
                return;
            float sx = ex - cx, sy = ey - cy;
            float len2 = sx * sx + sy * sy;
            if(len2 < 1e-3f)
                return;
            Scalar t = ((x - pressX_) * sx + (y - pressY_) * sy) / len2;
            if(snap)
                t = std::round(t / kSnapMove) * kSnapMove;
            delta = axis * t;
            break;
        }
        case Handle::PLANE:
        {
            Vector3 hit;
            if(!PlaneHit(x, y, dragCenter_, dragAxis_, hit))
                return;
            // In the handle's x-y plane: horizontal in Global, the object's own plane in Local
            Vector3 ax = HandleAxis(Handle::AXIS_X), ay = HandleAxis(Handle::AXIS_Y);
            Scalar dx = (hit - dragHitStart_).dot(ax), dy = (hit - dragHitStart_).dot(ay);
            if(snap)
            {
                dx = std::round(dx / kSnapMove) * kSnapMove;
                dy = std::round(dy / kSnapMove) * kSnapMove;
            }
            delta = ax * dx + ay * dy;
            break;
        }
        case Handle::RING_X:
        case Handle::RING_Y:
        case Handle::RING_Z:
        {
            float cx, cy;
            if(!ToScreen(dragCenter_, cx, cy))
                return;
            float a = std::atan2((float)y - cy, (float)x - cx) - dragAngleStart_;
            a = std::atan2(std::sin(a), std::cos(a));
            Scalar angle = a * dragRotationSign_;
            if(snap)
                angle = std::round(angle / (kSnapAngleDeg * M_PI / 180.0)) * (kSnapAngleDeg * M_PI / 180.0);
            if(std::optional<SceneLandmark> pose = SelectedPose())
            {
                // Turn the heading, stored in the object's frame like the point
                Transform T;
                if(!sim_->getLandmarks().EntityTransform(pose->entityName, T))
                    return;
                std::vector<SceneLandmark> ls = sim_->getLandmarks().Of(pose->entityName);
                ls[selectedLandmark_->index].normal = T.getBasis().transpose()
                                                      * quatRotate(Quaternion(dragAxis_, angle), dragHeadingStart_);
                sim_->getLandmarks().Set(pose->entityName, ls);
                landmarksChanged_.insert(pose->entityName);
            }
            else
                RotateAbout(dragAxis_, angle, dragCenter_);
            return;
        }
        default:
            return;
    }

    if(selectedLandmark_)
        MoveLandmark(*selectedLandmark_, dragLandmarkStart_ + delta);
    else
        for(size_t i = 0; i < group_.size() && i < dragStartTransforms_.size(); ++i)
        {
            Transform T = dragStartTransforms_[i];
            T.setOrigin(T.getOrigin() + delta);
            group_[i]->setTransform(T);
            moved_.insert(group_[i]->getName());
        }
}

void SceneEditor::DrawGizmo()
{
    Vector3 c;
    if(tool_ == Tool::SELECT || !GizmoCenter(c))
        return;
    if(tool_ == Tool::ROTATE && selectedLandmark_ && !SelectedPose())
        return; // a landmark is a point: nothing to rotate

    OpenGLContent* content = app_->getGLPipeline()->getContent();
    OpenGLTrackball* trackball = sim_->getTrackball();
    content->SetCurrentView(trackball);
    // The GUI pass (depth test off) keeps its vertex array bound for its own drawing
    GLint guiVertexArray;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &guiVertexArray);

    Scalar L = GizmoSize(c);
    glm::vec3 C((float)c.getX(), (float)c.getY(), (float)c.getZ());
    glm::vec3 up = trackball->GetUpDirection();
    glm::vec3 right = glm::normalize(glm::cross(trackball->GetLookingDirection(), up));
    float w = (float)L * 0.008f; // line thickness: a few offset copies of each line
    auto color = [&](Handle h, glm::vec4 base) {
        return (dragging_ == h || (dragging_ == Handle::NONE && hover_ == h)) ? glm::vec4(1.f, 0.95f, 0.2f, 1.f) : base;
    };
    auto thickLine = [&](PrimitiveType type, const std::vector<glm::vec3>& pts, glm::vec4 col) {
        for(glm::vec3 offset : {glm::vec3(0.f), right * w, -right * w, up * w, -up * w})
        {
            std::vector<glm::vec3> shifted;
            for(const glm::vec3& p : pts)
                shifted.push_back(p + offset);
            content->DrawPrimitives(type, &shifted, col);
        }
    };
    const glm::vec4 red(0.95f, 0.25f, 0.25f, 1.f), green(0.3f, 0.9f, 0.3f, 1.f), blue(0.3f, 0.5f, 1.f, 1.f);

    if(tool_ == Tool::MOVE)
    {
        for(auto [h, col] : {std::pair{Handle::AXIS_X, red}, std::pair{Handle::AXIS_Y, green}, std::pair{Handle::AXIS_Z, blue}})
        {
            Vector3 a = HandleAxis(h);
            glm::vec3 A((float)a.getX(), (float)a.getY(), (float)a.getZ());
            glm::vec4 hc = color(h, col);
            glm::vec3 tip = C + A * (float)L;
            thickLine(PrimitiveType::LINES, {C, tip}, hc);
            // Arrow head: a cone of triangles
            glm::vec3 base = C + A * (float)(L * 0.8);
            glm::vec3 u = glm::normalize(glm::abs(A.z) > 0.9f ? glm::cross(A, glm::vec3(1, 0, 0)) : glm::cross(A, glm::vec3(0, 0, 1)));
            glm::vec3 v = glm::cross(A, u);
            std::vector<glm::vec3> tris;
            for(int i = 0; i < 12; ++i)
            {
                float a0 = 2.f * (float)M_PI * i / 12.f, a1 = 2.f * (float)M_PI * (i + 1) / 12.f;
                float r = (float)L * 0.07f;
                tris.push_back(tip);
                tris.push_back(base + (u * std::cos(a0) + v * std::sin(a0)) * r);
                tris.push_back(base + (u * std::cos(a1) + v * std::sin(a1)) * r);
            }
            content->DrawPrimitives(PrimitiveType::TRIANGLES, &tris, hc);
        }
        // Plane square between the X and Y arrows
        auto glv = [](const Vector3& v) { return glm::vec3((float)v.getX(), (float)v.getY(), (float)v.getZ()); };
        glm::vec3 X = glv(HandleAxis(Handle::AXIS_X)), Y = glv(HandleAxis(Handle::AXIS_Y));
        float s = (float)L * 0.25f, e = (float)L * 0.12f;
        glm::vec3 q = C + (X + Y) * s;
        std::vector<glm::vec3> square = {q - X * e - Y * e, q + X * e - Y * e, q + X * e + Y * e,
                                         q - X * e - Y * e, q + X * e + Y * e, q - X * e + Y * e};
        glm::vec4 pc = color(Handle::PLANE, glm::vec4(0.3f, 0.5f, 1.f, 0.6f));
        content->DrawPrimitives(PrimitiveType::TRIANGLES, &square, pc);
    }
    else
    {
        for(auto [h, col] : {std::pair{Handle::RING_X, red}, std::pair{Handle::RING_Y, green}, std::pair{Handle::RING_Z, blue}})
        {
            if(selectedLandmark_ && h != Handle::RING_Z)
                continue;
            Vector3 u, v;
            RingBasis(HandleAxis(h), u, v);
            std::vector<glm::vec3> ring;
            for(int i = 0; i <= kRingSegments; ++i)
            {
                Scalar a = 2 * M_PI * i / kRingSegments;
                Vector3 p = c + (u * std::cos(a) + v * std::sin(a)) * L;
                ring.push_back(glm::vec3((float)p.getX(), (float)p.getY(), (float)p.getZ()));
            }
            thickLine(PrimitiveType::LINE_STRIP, ring, color(h, col));
        }
    }

    OpenGLState::BindVertexArray(guiVertexArray);
}

// --- Mouse -----------------------------------------------------------------------

void SceneEditor::MouseMove(SDL_Event* event)
{
    if(!active_)
        return;
    int x = event->motion.x, y = event->motion.y;
    bool leftDown = (event->motion.state & SDL_BUTTON_LMASK) != 0;

    if(dragging_ != Handle::NONE)
    {
        if(leftDown)
            UpdateGizmoDrag(x, y, (SDL_GetModState() & KMOD_CTRL) != 0);
        return;
    }
    if(leftDown)
        return;

    // Hover: a gizmo handle first, then the closest landmark marker
    hover_ = HandleUnder(x, y);
    hoverLandmark_.reset();
    if(hover_ != Handle::NONE)
        return;
    float best = kPickRadius;
    for(const LandmarkRef& ref : AllLandmarks())
    {
        Vector3 p;
        float sx, sy;
        if(LandmarkWorld(ref, p) && ToScreen(p, sx, sy) && std::hypot(sx - x, sy - y) < best)
        {
            best = std::hypot(sx - x, sy - y);
            hoverLandmark_ = ref;
        }
    }
}

void SceneEditor::MouseUp(SDL_Event* event)
{
    if(event->button.button == SDL_BUTTON_LEFT && dragging_ != Handle::NONE)
    {
        dragging_ = Handle::NONE;
        pickedName_ = app_->getSelectedEntity().first != nullptr ? app_->getSelectedEntity().first->getName() : "";
    }
}

// --- Keys ------------------------------------------------------------------------

bool SceneEditor::KeyDown(SDL_Event* event)
{
    SDL_Keycode key = event->key.keysym.sym;
    bool shift = (event->key.keysym.mod & KMOD_SHIFT) != 0;
    bool ctrl = (event->key.keysym.mod & KMOD_CTRL) != 0;

    if(key == SDLK_TAB)
    {
        active_ = !active_;
        dragging_ = Handle::NONE;
        SetStatus(active_ ? "Edit mode on" : "Edit mode off");
        return true;
    }
    if(!active_)
        return false;

    // Right mouse held: WASD/Q/E fly the camera, like Unity
    if(SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_RMASK)
        return false;

    if(ctrl && key == SDLK_z) { Undo(); return true; }
    if(ctrl && key == SDLK_s)
    {
        std::string message;
        Save(message);
        SetStatus(message);
        return true;
    }
    switch(key)
    {
        case SDLK_q: tool_ = Tool::SELECT; return true;
        case SDLK_w: tool_ = Tool::MOVE; return true;
        case SDLK_e: tool_ = Tool::ROTATE; return true;
        case SDLK_z: pivot_ = !pivot_; SetStatus(pivot_ ? "Pivot: the object's origin" : "Center: the middle of the object"); return true;
        case SDLK_x: localAxes_ = !localAxes_; SetStatus(localAxes_ ? "Local: the object's own axes" : "Global: north, east, up"); return true;
        case SDLK_DELETE:
        case SDLK_BACKSPACE: RemoveSelectedLandmark(); return true;
        default: break;
    }

    // Nudge the selected landmark if there is one, otherwise the selected object
    Scalar step = shift ? kNudge * 10 : kNudge;
    Vector3 delta(0, 0, 0);
    Scalar yaw = 0;
    switch(key)
    {
        // Stonefish world is NED: x north, y east, z down
        case SDLK_UP:       delta = Vector3(step, 0, 0); break;
        case SDLK_DOWN:     delta = Vector3(-step, 0, 0); break;
        case SDLK_RIGHT:    delta = Vector3(0, step, 0); break;
        case SDLK_LEFT:     delta = Vector3(0, -step, 0); break;
        case SDLK_PAGEUP:   delta = Vector3(0, 0, -step); break;
        case SDLK_PAGEDOWN: delta = Vector3(0, 0, step); break;
        case SDLK_PERIOD:   yaw = shift ? kYawStep / 5 : kYawStep; break;
        case SDLK_COMMA:    yaw = shift ? -kYawStep / 5 : -kYawStep; break;
        default:            return false;
    }

    Vector3 p, c;
    if(selectedLandmark_ && yaw == 0 && LandmarkWorld(*selectedLandmark_, p))
    {
        PushUndo();
        MoveLandmark(*selectedLandmark_, p + delta);
    }
    else if(ObjectPivot(c))
    {
        PushUndo();
        if(yaw != 0)
        {
            dragStartTransforms_.clear();
            for(StaticEntity* e : group_)
                dragStartTransforms_.push_back(e->getTransform());
            RotateAbout(Vector3(0, 0, 1), yaw * M_PI / 180.0, c);
        }
        else
            Translate(delta);
    }
    return true;
}

// --- Drawing -------------------------------------------------------------------

void SceneEditor::DrawOverlay()
{
    IMGUI* gui = app_->getGUI();
    const glm::vec4 orange(1.f, 0.6f, 0.1f, 1.f);
    const glm::vec4 cyan(0.2f, 0.9f, 1.f, 1.f);
    const glm::vec4 white(1.f, 1.f, 1.f, 1.f);
    const glm::vec4 yellow(1.f, 1.f, 0.2f, 1.f);
    const glm::vec4 grey(0.55f, 0.55f, 0.55f, 1.f);
    float sx, sy;

    // Selected object: its bounding box corners and its name above it
    Vector3 min, max;
    if(GroupBounds(min, max))
    {
        for(int i = 0; i < 8; ++i)
        {
            Vector3 corner(i & 1 ? max.getX() : min.getX(), i & 2 ? max.getY() : min.getY(), i & 4 ? max.getZ() : min.getZ());
            if(ToScreen(corner, sx, sy))
                gui->DoLabel(sx - 4.f, sy - 7.f, "+", orange, 1.3f);
        }
        Vector3 top((min.getX() + max.getX()) / 2, (min.getY() + max.getY()) / 2, min.getZ());
        if(ToScreen(top, sx, sy))
            gui->DoLabel(sx - 4.f * groupName_.size(), sy - 22.f, groupName_, orange, 1.1f);
    }

    // Landmarks: a marker each (like a child empty), labelled when hovered, selected or on the selection
    for(const LandmarkRef& ref : AllLandmarks())
    {
        Vector3 p;
        if(!LandmarkWorld(ref, p) || !ToScreen(p, sx, sy))
            continue;
        bool selected = selectedLandmark_ && *selectedLandmark_ == ref;
        bool hovered = hoverLandmark_ && *hoverLandmark_ == ref;
        bool onSelection = !groupName_.empty() && GroupOf(ref.entity) == groupName_;
        SceneLandmark l = sim_->getLandmarks().Of(ref.entity)[ref.index]; // a copy: Of() returns a temporary
        // Grey: not published (this landmark, or all of them, switched off)
        bool published = l.enabled && sim_->getLandmarks().isPublishing();
        if(l.kind != SceneLandmark::Kind::LANDMARK)
        {
            // Spawn points (green) and premap priors (orange): always labelled with their name
            glm::vec4 c = selected ? yellow : (hovered ? white : (l.kind == SceneLandmark::Kind::SPAWN ? glm::vec4(0.3f, 1.f, 0.4f, 1.f) : orange));
            gui->DoLabel(sx - 5.f, sy - 8.f, selected ? "[o]" : "o", c, selected ? 1.2f : 1.f);
            gui->DoLabel(sx + 10.f, sy - 6.f, (l.kind == SceneLandmark::Kind::SPAWN ? "spawn " : "prior ") + l.subtypeName, c, 0.9f);
            continue;
        }
        glm::vec4 color = selected ? yellow : (hovered ? white : (published ? cyan : grey));
        gui->DoLabel(sx - 5.f, sy - 8.f, selected ? "[o]" : "o", color, selected ? 1.2f : 1.f);
        if(selected || hovered || onSelection)
        {
            std::string label = l.subtypeName.empty() ? l.typeName : l.typeName + " / " + l.subtypeName;
            gui->DoLabel(sx + 10.f, sy - 6.f, l.enabled ? label : label + " (off)", selected ? yellow : (published ? white : grey), 0.9f);
        }
    }
}

void SceneEditor::DrawPoses()
{
    // Heading arrows of the spawn points and priors, with a line up to the surface for the depth
    auto gl = [](const Vector3& v) { return glm::vec3((float)v.getX(), (float)v.getY(), (float)v.getZ()); };
    std::vector<glm::vec3> spawns, priors, axes[3];
    for(const SceneLandmark& l : sim_->getLandmarks().All())
    {
        Vector3 p;
        Transform T;
        if(l.kind == SceneLandmark::Kind::LANDMARK && l.rotation && sim_->getLandmarks().WorldPosition(l, p)
           && sim_->getLandmarks().EntityTransform(l.entityName, T))
        {
            // The published orientation: x red, y green, z blue
            Matrix3 R = T.getBasis() * Matrix3(*l.rotation);
            for(int i = 0; i < 3; ++i)
            {
                axes[i].push_back(gl(p));
                axes[i].push_back(gl(p + R.getColumn(i) * 0.15));
            }
            continue;
        }
        if(l.kind == SceneLandmark::Kind::LANDMARK || !sim_->getLandmarks().WorldPosition(l, p)
           || !sim_->getLandmarks().EntityTransform(l.entityName, T))
            continue;
        Vector3 f = T.getBasis() * *l.normal;
        f.setZ(0);
        f = f.length() > 1e-6 ? f.normalized() : Vector3(1, 0, 0);
        Vector3 side(-f.getY(), f.getX(), 0);
        const Scalar L = 0.7; // [m], about the drone's length
        std::vector<glm::vec3>& v = l.kind == SceneLandmark::Kind::SPAWN ? spawns : priors;
        for(const auto& [a, b] : {std::pair{p - f * (L / 2), p + f * (L / 2)},
                                  std::pair{p + f * (L / 2), p + f * (L * 0.2) + side * (L * 0.2)},
                                  std::pair{p + f * (L / 2), p + f * (L * 0.2) - side * (L * 0.2)},
                                  std::pair{Vector3(p.getX(), p.getY(), 0), p}})
        {
            v.push_back(gl(a));
            v.push_back(gl(b));
        }
    }
    if(spawns.empty() && priors.empty() && axes[0].empty())
        return;
    OpenGLContent* content = app_->getGLPipeline()->getContent();
    content->SetCurrentView(sim_->getTrackball());
    GLint guiVertexArray;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &guiVertexArray);
    if(!spawns.empty())
        content->DrawPrimitives(PrimitiveType::LINES, &spawns, glm::vec4(0.3f, 1.f, 0.4f, 1.f));
    if(!priors.empty())
        content->DrawPrimitives(PrimitiveType::LINES, &priors, glm::vec4(1.f, 0.6f, 0.1f, 1.f));
    const glm::vec4 axisColors[3] = {glm::vec4(1.f, 0.2f, 0.2f, 1.f), glm::vec4(0.2f, 1.f, 0.2f, 1.f), glm::vec4(0.3f, 0.5f, 1.f, 1.f)};
    for(int i = 0; i < 3; ++i)
        if(!axes[i].empty())
            content->DrawPrimitives(PrimitiveType::LINES, &axes[i], axisColors[i]);
    OpenGLState::BindVertexArray(guiVertexArray);
}

void SceneEditor::DrawWaypoint()
{
    // Where the vehicle is going: an arrow along the waypoint's heading (its x axis), and a line down
    // from the surface to show its depth. Main window only, so it never shows in the camera images.
    Transform pose;
    float sx, sy;
    if(!sim_->getWaypoint(pose) || !ToScreen(pose.getOrigin(), sx, sy))
        return;
    Vector3 w = pose.getOrigin();
    Vector3 forward = pose.getBasis().getColumn(0);
    Vector3 side = pose.getBasis().getColumn(1);
    auto gl = [](const Vector3& v) { return glm::vec3((float)v.getX(), (float)v.getY(), (float)v.getZ()); };
    const Scalar L = 0.6; // [m] arrow length
    std::vector<glm::vec3> lines = {
        gl(Vector3(w.getX(), w.getY(), 0)), gl(w),                               // depth line
        gl(w), gl(w + forward * L),                                                // shaft
        gl(w + forward * L), gl(w + forward * (L * 0.75) + side * (L * 0.15)),    // head
        gl(w + forward * L), gl(w + forward * (L * 0.75) - side * (L * 0.15)),
    };

    OpenGLContent* content = app_->getGLPipeline()->getContent();
    content->SetCurrentView(sim_->getTrackball());
    GLint guiVertexArray;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &guiVertexArray);
    content->DrawPrimitives(PrimitiveType::LINES, &lines, glm::vec4(1.f, 0.3f, 1.f, 1.f));
    OpenGLState::BindVertexArray(guiVertexArray);

    Scalar yaw, pitch, roll;
    pose.getBasis().getEulerYPR(yaw, pitch, roll);
    char buf[64];
    snprintf(buf, sizeof(buf), "waypoint (depth %.2f, yaw %.0f)", (double)w.getZ(), yaw * 180.0 / M_PI);
    app_->getGUI()->DoLabel(sx - 6.f, sy - 9.f, "X", glm::vec4(1.f, 0.3f, 1.f, 1.f), 1.4f);
    app_->getGUI()->DoLabel(sx - 20.f, sy + 10.f, buf, glm::vec4(1.f, 0.6f, 1.f, 1.f), 0.9f); // below, clear of the arrow
}

void SceneEditor::DrawLandmarkFrames()
{
    // The landmark server's frames (best landmark of each class, gate targets, start, priors) as
    // red/green/blue x/y/z axes where the stack thinks they are. Targets (<parent>/<name> offsets,
    // not /measured) are all yellow. Names in black on white. Main window only, like the waypoint.
    static OpenGLPrinter labelPrinter(GetShaderPath() + std::string(STANDARD_FONT_NAME), STANDARD_FONT_SIZE); // label widths
    // A plain white rectangle behind a label (screen coordinates from the top left)
    auto WhiteBox = [this](float x, float y, float w, float h) {
        GLfloat clear[4];
        glGetFloatv(GL_COLOR_CLEAR_VALUE, clear);
        GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);
        glEnable(GL_SCISSOR_TEST);
        glScissor((GLint)x, app_->getWindowHeight() - (GLint)(y + h), (GLsizei)std::ceil(w), (GLsizei)std::ceil(h));
        glClearColor(1.f, 1.f, 1.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        glClearColor(clear[0], clear[1], clear[2], clear[3]);
        if(!scissor)
            glDisable(GL_SCISSOR_TEST);
    };
    std::map<std::string, Transform> frames = sim_->getLandmarkFrames();
    for(auto it = frames.begin(); it != frames.end();) // /live is single raw detections: it only jumps around
        it = it->first.size() > 5 && it->first.compare(it->first.size() - 5, 5, "/live") == 0 ? frames.erase(it) : std::next(it);
    if(frames.empty())
        return;
    auto gl = [](const Vector3& v) { return glm::vec3((float)v.getX(), (float)v.getY(), (float)v.getZ()); };
    auto targetName = [](const std::string& name) {
        size_t slash = name.rfind('/');
        std::string last = slash == std::string::npos ? "" : name.substr(slash + 1);
        return last == "measured" ? std::string() : last;
    };
    std::vector<glm::vec3> axes[3], priors, targets;
    for(const auto& [name, T] : frames)
    {
        bool prior = name.rfind("prior_", 0) == 0;
        std::string target = targetName(name);
        Scalar L = prior ? 0.15 : !target.empty() ? 0.25 : 0.35; // [m]
        for(int i = 0; i < 3; ++i)
        {
            std::vector<glm::vec3>& v = prior ? priors : !target.empty() ? targets : axes[i];
            v.push_back(gl(T.getOrigin()));
            v.push_back(gl(T.getOrigin() + T.getBasis().getColumn(i) * L));
        }
    }

    OpenGLContent* content = app_->getGLPipeline()->getContent();
    content->SetCurrentView(sim_->getTrackball());
    GLint guiVertexArray;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &guiVertexArray);
    const glm::vec4 colors[3] = {glm::vec4(1.f, 0.2f, 0.2f, 1.f), glm::vec4(0.2f, 1.f, 0.2f, 1.f), glm::vec4(0.3f, 0.5f, 1.f, 1.f)};
    for(int i = 0; i < 3; ++i)
        if(!axes[i].empty())
            content->DrawPrimitives(PrimitiveType::LINES, &axes[i], colors[i]);
    if(!priors.empty())
        content->DrawPrimitives(PrimitiveType::LINES, &priors, glm::vec4(0.6f, 0.6f, 0.6f, 1.f));
    if(!targets.empty())
        content->DrawPrimitives(PrimitiveType::LINES, &targets, glm::vec4(1.f, 0.85f, 0.1f, 1.f));
    OpenGLState::BindVertexArray(guiVertexArray);

    for(const auto& [name, T] : frames)
    {
        float sx, sy;
        if(name.rfind("prior_", 0) != 0 && ToScreen(T.getOrigin(), sx, sy))
        {
            const float scale = 0.85f, x = sx + 6.f, y = sy + 3.f;
            WhiteBox(x, y, labelPrinter.TextLength(name) * scale, STANDARD_FONT_SIZE * scale);
            app_->getGUI()->DoLabel(x, y, name, glm::vec4(0.f, 0.f, 0.f, 1.f), scale);
        }
    }
}

void SceneEditor::DoHUD()
{
    if(debugView_)
    {
        DrawLandmarkFrames();
        DrawWaypoint();
    }
    IMGUI* gui = app_->getGUI();
    SceneLandmarks& landmarks = sim_->getLandmarks();
    GLfloat w = 270.f;
    GLfloat x = (GLfloat)app_->getWindowWidth() - w - 10.f;
    Uid id;
    id.owner = kOwner;

    if(!active_)
    {
        gui->DoPanel(x, 10.f, w, 68.f);
        gui->DoLabel(x + 5.f, 18.f, "Press Tab to edit the scene");
        id.item = 10;
        landmarks.setPublishing(gui->DoCheckBox(id, x + 5.f, 35.f, w - 10.f, landmarks.isPublishing(), "Publish landmarks"));
        id.item = 13;
        debugView_ = gui->DoCheckBox(id, x + 5.f, 55.f, w - 10.f, debugView_, "Debug view (waypoint, frames)");
        DrawProfileMenu(x, 83.f, w);
        return;
    }

    // A new left click: on a gizmo handle it starts a drag, on a landmark marker it selects
    // that landmark, elsewhere it leaves the selection to the simulator's picking
    int mx, my;
    bool leftDown = (SDL_GetMouseState(&mx, &my) & SDL_BUTTON_LMASK) != 0;
    bool overPanel = mx >= x && mx <= x + w && my <= (int)kPanelHeight + 10;
    if(leftDown && !leftWasDown_ && !gui->isAnyActive() && !overPanel)
    {
        Handle h = HandleUnder(mx, my);
        if(h != Handle::NONE)
            StartGizmoDrag(h, mx, my);
        else if(hoverLandmark_)
            selectedLandmark_ = hoverLandmark_;
        else
            selectedLandmark_.reset();
    }
    leftWasDown_ = leftDown;

    UpdateSelection();
    DrawPoses();
    DrawGizmo();
    DrawOverlay();
    DrawPanel();
}

void SceneEditor::DrawPanel()
{
    IMGUI* gui = app_->getGUI();
    SceneLandmarks& landmarks = sim_->getLandmarks();
    GLfloat w = 270.f;
    GLfloat x = (GLfloat)app_->getWindowWidth() - w - 10.f;
    GLfloat y = 10.f;
    Uid id;
    id.owner = kOwner;
    char buf[256];

    size_t changes = moved_.size() + landmarksChanged_.size();
    gui->DoPanel(x, y, w, kPanelHeight);
    y += 5.f;
    gui->DoLabel(x + 5.f, y, changes ? "SCENE EDITOR  (unsaved)" : "SCENE EDITOR");
    gui->DoLabel(x + w - 75.f, y, "Tab: exit");
    y += 20.f;

    // Tools, like Unity's Q/W/E
    const char* tools[] = {"Q Select", "W Move", "E Rotate"};
    GLfloat bw = (w - 20.f) / 3.f;
    for(int i = 0; i < 3; ++i)
    {
        id.item = 20 + i;
        std::string title = (int)tool_ == i ? std::string("[") + tools[i] + "]" : tools[i];
        if(gui->DoButton(id, x + 5.f + i * (bw + 5.f), y, bw, 22.f, title))
            tool_ = (Tool)i;
    }
    y += 27.f;
    // Unity's Pivot/Center (Z) and Local/Global (X) toggles
    GLfloat hw = (w - 15.f) / 2.f;
    id.item = 23;
    if(gui->DoButton(id, x + 5.f, y, hw, 22.f, pivot_ ? "Z Pivot" : "Z Center"))
        pivot_ = !pivot_;
    id.item = 24;
    if(gui->DoButton(id, x + 10.f + hw, y, hw, 22.f, localAxes_ ? "X Local" : "X Global"))
        localAxes_ = !localAxes_;
    y += 30.f;

    Vector3 c;
    if(std::optional<SceneLandmark> pose = SelectedPose())
    {
        bool spawn = pose->kind == SceneLandmark::Kind::SPAWN;
        snprintf(buf, sizeof(buf), spawn ? "Spawn point: %s" : "Premap prior: %s", pose->subtypeName.c_str());
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        Vector3 p;
        LandmarkWorld(*selectedLandmark_, p);
        Transform T;
        sim_->getLandmarks().EntityTransform(pose->entityName, T);
        Vector3 f = T.getBasis() * *pose->normal;
        snprintf(buf, sizeof(buf), "N %.2f E %.2f D %.2f  heading %.0f", (double)p.getX(), (double)p.getY(), (double)p.getZ(),
                 std::atan2(f.getY(), f.getX()) * 180.0 / M_PI);
        gui->DoLabel(x + 5.f, y, buf, glm::vec4(-1.f), 0.9f);
        y += 15.f;
        snprintf(buf, sizeof(buf), spawn ? "on %s, start with spawn:=%s" : "on %s, the landmark server's '%s'",
                 GroupOf(pose->entityName).c_str(), pose->subtypeName.c_str());
        gui->DoLabel(x + 5.f, y, buf, glm::vec4(0.75f, 0.75f, 0.75f, 1.f), 0.85f);
        y += 30.f;
    }
    else if(selectedLandmark_)
    {
        SceneLandmark l = landmarks.Of(selectedLandmark_->entity)[selectedLandmark_->index]; // a copy: Of() returns a temporary
        snprintf(buf, sizeof(buf), "Landmark type: %s", l.typeName.c_str());
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        snprintf(buf, sizeof(buf), "Subtype: %s", l.subtypeName.empty() ? "(none)" : l.subtypeName.c_str());
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        Vector3 p;
        LandmarkWorld(*selectedLandmark_, p);
        snprintf(buf, sizeof(buf), "on %s   N %.2f E %.2f D %.2f", GroupOf(selectedLandmark_->entity).c_str(),
                 (double)p.getX(), (double)p.getY(), (double)p.getZ());
        gui->DoLabel(x + 5.f, y, buf, glm::vec4(-1.f), 0.9f);
        y += 17.f;
        id.item = 11;
        bool enabled = gui->DoCheckBox(id, x + 5.f, y, w - 10.f, l.enabled, "Publish this landmark");
        if(enabled != l.enabled)
        {
            PushUndo();
            std::vector<SceneLandmark> ls = landmarks.Of(selectedLandmark_->entity);
            ls[selectedLandmark_->index].enabled = enabled;
            landmarks.Set(selectedLandmark_->entity, ls);
            landmarksChanged_.insert(selectedLandmark_->entity);
        }
        y += 28.f;
    }
    else if(GroupCenter(c))
    {
        size_t n = 0;
        for(StaticEntity* e : group_)
            n += landmarks.Of(e->getName()).size();
        snprintf(buf, sizeof(buf), "Object: %s (%zu landmark%s)", groupName_.c_str(), n, n == 1 ? "" : "s");
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        snprintf(buf, sizeof(buf), "N %.2f  E %.2f  depth %.2f", (double)c.getX(), (double)c.getY(), (double)c.getZ());
        gui->DoLabel(x + 5.f, y, buf, glm::vec4(-1.f), 0.9f);
        y += 15.f;
        // Its landmarks: type / subtype (click a marker to edit one)
        size_t shown = 0;
        for(StaticEntity* e : group_)
            for(const SceneLandmark& l : landmarks.Of(e->getName()))
                if(shown++ < 4)
                {
                    std::string line = "  " + l.typeName + (l.subtypeName.empty() ? "" : " / " + l.subtypeName) + (l.enabled ? "" : " (off)");
                    gui->DoLabel(x + 5.f, y, line, glm::vec4(0.2f, 0.9f, 1.f, 1.f), 0.85f);
                    y += 13.f;
                }
        if(shown > 4)
        {
            gui->DoLabel(x + 5.f, y, "  ... " + std::to_string(shown - 4) + " more", glm::vec4(0.2f, 0.9f, 1.f, 1.f), 0.85f);
            y += 13.f;
        }
        y += 8.f;
    }
    else
    {
        gui->DoLabel(x + 5.f, y, "Click an object or a landmark (o)");
        y += 52.f;
    }

    const char* help[] = {
        "Drag the arrows / rings (Ctrl: snap)",
        "Right mouse: look, + WASD: fly",
        "Del: remove landmark/spawn   Ctrl+Z: undo",
    };
    for(const char* line : help)
    {
        gui->DoLabel(x + 5.f, y, line, glm::vec4(0.75f, 0.75f, 0.75f, 1.f), 0.9f);
        y += 13.f;
    }
    y += 8.f;

    // Type and subtype: of the selected landmark (like an inspector), otherwise of a new landmark
    std::vector<std::string> types;
    for(const auto& [name, value] : SceneLandmarks::TypeNames())
        types.push_back(name);
    auto subtypesOf = [](const std::string& type) {
        std::vector<std::string> names;
        auto sub = SceneLandmarks::SubtypeNames().find(type);
        if(sub != SceneLandmarks::SubtypeNames().end())
            for(const auto& [name, value] : sub->second)
                names.push_back(name);
        return names;
    };
    std::optional<SceneLandmark> editing;
    if(selectedLandmark_ && !SelectedPose())
    {
        editing = landmarks.Of(selectedLandmark_->entity)[selectedLandmark_->index];
        typeIndex_ = (unsigned int)(std::find(types.begin(), types.end(), editing->typeName) - types.begin());
        std::vector<std::string> subs = subtypesOf(editing->typeName);
        subtypeIndex_ = (unsigned int)(std::find(subs.begin(), subs.end(), editing->subtypeName) - subs.begin());
    }
    id.item = 0;
    unsigned int newType = gui->DoComboBox(id, x + 5.f, y, w - 10.f, types, typeIndex_, editing ? "Type of this landmark" : "New landmark type");
    bool changed = newType != typeIndex_;
    if(changed)
    {
        typeIndex_ = newType;
        subtypeIndex_ = 0;
    }
    y += 45.f;

    std::vector<std::string> subtypes = subtypesOf(types[typeIndex_]);
    if(subtypes.empty())
        subtypes.push_back("(none)");
    id.item = 1;
    unsigned int newSubtype = std::min(gui->DoComboBox(id, x + 5.f, y, w - 10.f, subtypes, std::min(subtypeIndex_, (unsigned int)subtypes.size() - 1), "Subtype"),
                                       (unsigned int)subtypes.size() - 1);
    changed = changed || newSubtype != subtypeIndex_;
    subtypeIndex_ = newSubtype;
    y += 45.f;

    if(editing && changed)
    {
        PushUndo();
        std::vector<SceneLandmark> ls = landmarks.Of(selectedLandmark_->entity);
        SceneLandmark& l = ls[selectedLandmark_->index];
        l.typeName = types[typeIndex_];
        l.type = SceneLandmarks::TypeNames().at(l.typeName);
        l.subtypeName = subtypes[subtypeIndex_] == "(none)" ? "" : subtypes[subtypeIndex_];
        l.subtype = l.subtypeName.empty() ? 0 : SceneLandmarks::SubtypeNames().at(l.typeName).at(l.subtypeName);
        landmarks.Set(selectedLandmark_->entity, ls);
        landmarksChanged_.insert(selectedLandmark_->entity);
    }

    GLfloat half = (w - 15.f) / 2.f;
    id.item = 2;
    if(gui->DoButton(id, x + 5.f, y, half, 25.f, "Add to object"))
    {
        if(group_.empty())
            SetStatus("Select an object first");
        else
            AddLandmark();
    }
    id.item = 3;
    if(gui->DoButton(id, x + 10.f + half, y, half, 25.f, selectedLandmark_ ? "Remove landmark" : "Remove all"))
    {
        if(selectedLandmark_)
            RemoveSelectedLandmark();
        else if(!group_.empty())
            RemoveLandmarks();
    }
    y += 32.f;

    id.item = 25;
    if(gui->DoButton(id, x + 5.f, y, w - 10.f, 25.f, "Add spawn point to object"))
    {
        if(group_.empty())
            SetStatus("Select an object first");
        else
            AddSpawn();
    }
    y += 32.f;

    id.item = 4;
    if(gui->DoButton(id, x + 5.f, y, half, 25.f, "Undo"))
        Undo();
    snprintf(buf, sizeof(buf), changes ? "Save (%zu)" : "Save", changes);
    id.item = 5;
    if(gui->DoButton(id, x + 10.f + half, y, half, 25.f, buf))
    {
        std::string message;
        Save(message);
        SetStatus(message);
    }
    y += 32.f;

    id.item = 10;
    landmarks.setPublishing(gui->DoCheckBox(id, x + 5.f, y, w - 10.f, landmarks.isPublishing(), "Publish landmarks"));
    y += 20.f;
    id.item = 13;
    debugView_ = gui->DoCheckBox(id, x + 5.f, y, w - 10.f, debugView_, "Debug view (waypoint, frames)");
    y += 20.f;
    DrawProfileMenu(x, y, w);
    y += 45.f;

    if(!status_.empty() && GetTimeInMicroseconds() - statusTime_ < 6000000)
        gui->DoLabel(x + 5.f, y, status_, glm::vec4(1.f, 0.85f, 0.4f, 1.f), 0.9f);
}

void SceneEditor::DrawProfileMenu(float x, float y, float w)
{
    // Detection errors of the published landmarks, the profiles robosub_dummy_publisher had
    std::vector<std::string> names;
    for(const DetectionProfile& p : SceneLandmarks::Profiles())
        names.push_back(p.name);
    Uid id;
    id.owner = kOwner;
    id.item = 12;
    SceneLandmarks& landmarks = sim_->getLandmarks();
    unsigned int chosen = app_->getGUI()->DoComboBox(id, x + 5.f, y, w - 10.f, names, (unsigned int)landmarks.getProfile(), "Detection profile");
    if(chosen != landmarks.getProfile())
    {
        landmarks.setProfile(chosen);
        profileChanged_ = true;
        SetStatus("Detection profile: " + names[chosen] + (active_ ? " (save to keep)" : ""));
    }
}

bool SceneEditor::Save(std::string& message)
{
    std::vector<std::string> files;
    CollectFiles(std::filesystem::weakly_canonical(sim_->getScenarioPath()).string(), files);
    std::map<std::string, std::string> texts;
    for(const std::string& f : files)
        texts[f] = ReadFile(f);

    // 1. Object poses: world_transform of each moved object, in the file that defines it
    size_t savedPoses = 0;
    for(const std::string& name : moved_)
    {
        StaticEntity* e = (StaticEntity*)sim_->getEntity(name);
        if(e == nullptr)
            continue;
        Transform T = e->getTransform();
        Scalar yaw, pitch, roll;
        T.getBasis().getEulerYPR(yaw, pitch, roll);
        for(auto& [file, text] : texts)
        {
            auto block = FindObject(text, name);
            if(block.first == std::string::npos)
                continue;
            size_t wt = text.rfind("<world_transform", block.second);
            if(wt == std::string::npos || wt < block.first)
                break;
            size_t wtEnd = text.find("/>", wt) + 2;
            std::string tag = text.substr(wt, wtEnd - wt);
            tag = SetAttribute(tag, "xyz", Format(T.getOrigin().getX(), T.getOrigin().getY(), T.getOrigin().getZ()));
            tag = SetAttribute(tag, "rpy", Format(roll, pitch, yaw));
            text.replace(wt, wtEnd - wt, tag);
            ++savedPoses;
            break;
        }
    }

    // 2. Landmarks: rewrite those of changed objects in the scene's landmark file
    size_t savedLandmarks = 0;
    if(!landmarksChanged_.empty())
    {
        std::string landmarkFile;
        for(const auto& [file, text] : texts)
            if(text.find("<landmark object=") != std::string::npos || text.find("<spawn ") != std::string::npos)
                landmarkFile = file;
        if(landmarkFile.empty())
        {
            // First landmarks of this scenario: a new file next to it, included from it
            std::filesystem::path scn(files.front());
            landmarkFile = (scn.parent_path() / (scn.stem().string() + "_landmarks.scn")).string();
            texts[landmarkFile] = "<?xml version=\"1.0\"?>\n<scenario>\n</scenario>\n";
            std::string& top = texts[files.front()];
            top.insert(top.rfind("</scenario>"), "  <include file=\"" + landmarkFile + "\"/>\n");
        }

        for(const std::string& name : landmarksChanged_)
        {
            for(auto& [file, text] : texts)
                text = RemoveLandmarkLines(text, name, file != landmarkFile);
            std::string& text = texts[landmarkFile];
            for(const SceneLandmark& l : sim_->getLandmarks().Of(name))
            {
                text.insert(text.rfind("</scenario>"), LandmarkLine(l));
                ++savedLandmarks;
            }
        }
    }

    // 3. Detection profile: the profile attribute of <ros_landmarks>
    if(profileChanged_)
        for(auto& [file, text] : texts)
        {
            size_t tag = text.find("<ros_landmarks");
            if(tag == std::string::npos)
                continue;
            size_t end = text.find("/>", tag) + 2;
            text.replace(tag, end - tag, SetAttribute(text.substr(tag, end - tag), "profile",
                                                      SceneLandmarks::Profiles()[sim_->getLandmarks().getProfile()].name));
            break;
        }

    for(const auto& [file, text] : texts)
        if(text != ReadFile(file) && !WriteFile(file, text))
        {
            message = "Could not write " + file;
            return false;
        }

    moved_.clear();
    landmarksChanged_.clear();
    profileChanged_ = false;
    message = "Saved " + std::to_string(savedPoses) + " object pose(s), " + std::to_string(savedLandmarks) + " landmark(s)";
    return true;
}

}

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
#include <Stonefish/entities/StaticEntity.h>
#include <Stonefish/graphics/IMGUI.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <Stonefish/utils/SystemUtil.hpp>
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
        std::string line = "  <landmark object=\"" + l.entityName + "\" type=\"" + l.typeName + "\"";
        if(!l.subtypeName.empty())
            line += " subtype=\"" + l.subtypeName + "\"";
        if(l.point)
            line += " point=\"" + Format(l.point->getX(), l.point->getY(), l.point->getZ()) + "\"";
        if(l.normal)
            line += " normal=\"" + Format(l.normal->getX(), l.normal->getY(), l.normal->getZ()) + "\"";
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
            bool isLandmark = line.find("<landmark") != std::string::npos;
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
    if(picked == nullptr)
    {
        groupName_.clear();
        group_.clear();
    }
    else if(picked->getType() != EntityType::STATIC)
    {
        groupName_.clear();
        group_.clear();
        SetStatus("Only static objects can be moved");
    }
    else
        SelectGroup(GroupOf(name));
}

bool SceneEditor::GroupCenter(Vector3& center) const
{
    if(group_.empty())
        return false;
    Vector3 min, max, bmin, bmax;
    group_[0]->getAABB(min, max);
    for(StaticEntity* e : group_)
    {
        e->getAABB(bmin, bmax);
        min.setMin(bmin);
        max.setMax(bmax);
    }
    center = (min + max) / Scalar(2);
    return true;
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

void SceneEditor::Rotate(Scalar yawDeg)
{
    Vector3 c;
    if(!GroupCenter(c))
        return;
    // About the vertical axis through the group's centre (the meshes' own origins can be far away)
    Transform R(Quaternion(Vector3(0, 0, 1), yawDeg * M_PI / 180.0), Vector3(0, 0, 0));
    for(StaticEntity* e : group_)
    {
        Transform T = e->getTransform();
        Transform toCenter(Quaternion::getIdentity(), c);
        e->setTransform(toCenter * R * toCenter.inverse() * T);
        moved_.insert(e->getName());
    }
}

bool SceneEditor::PlaneHit(int x, int y, Vector3& hit) const
{
    OpenGLTrackball* trackball = sim_->getTrackball();
    glm::vec3 eye = trackball->GetEyePosition();
    glm::vec3 ray = trackball->Ray(x, y);
    Vector3 o(eye.x, eye.y, eye.z);
    Vector3 d(ray.x, ray.y, ray.z);
    Scalar denom = d.dot(planeNormal_);
    if(std::abs(denom) < 1e-6)
        return false;
    Scalar t = (planePoint_ - o).dot(planeNormal_) / denom;
    if(t < 0)
        return false;
    hit = o + d * t;
    return true;
}

bool SceneEditor::KeyDown(SDL_Event* event)
{
    SDL_Keycode key = event->key.keysym.sym;
    if(key == SDLK_e)
    {
        active_ = !active_;
        dragging_ = false;
        SetStatus(active_ ? "Edit mode: click an object, drag to move (Shift: up/down)" : "Edit mode off");
        return true;
    }
    if(!active_ || group_.empty())
        return false;

    bool shift = (event->key.keysym.mod & KMOD_SHIFT) != 0;
    Scalar step = shift ? kNudge * 10 : kNudge;
    switch(key)
    {
        // Stonefish world is NED: x north, y east, z down
        case SDLK_UP:       Translate(Vector3(step, 0, 0)); return true;
        case SDLK_DOWN:     Translate(Vector3(-step, 0, 0)); return true;
        case SDLK_RIGHT:    Translate(Vector3(0, step, 0)); return true;
        case SDLK_LEFT:     Translate(Vector3(0, -step, 0)); return true;
        case SDLK_PAGEUP:   Translate(Vector3(0, 0, -step)); return true;
        case SDLK_PAGEDOWN: Translate(Vector3(0, 0, step)); return true;
        case SDLK_PERIOD:   Rotate(shift ? kYawStep / 5 : kYawStep); return true;
        case SDLK_COMMA:    Rotate(shift ? -kYawStep / 5 : -kYawStep); return true;
        default:            return false;
    }
}

void SceneEditor::MouseMove(SDL_Event* event)
{
    if(!active_)
        return;
    UpdateSelection();

    bool leftDown = (event->motion.state & SDL_BUTTON_LMASK) != 0;
    if(!leftDown || group_.empty() || app_->getGUI()->isAnyActive())
    {
        dragging_ = false;
        return;
    }

    if(!dragging_)
    {
        // Start dragging on a plane through the group's centre
        Vector3 c;
        GroupCenter(c);
        dragVertical_ = (SDL_GetModState() & KMOD_SHIFT) != 0;
        planePoint_ = c;
        if(dragVertical_)
        {
            glm::vec3 look = sim_->getTrackball()->GetLookingDirection();
            planeNormal_ = Vector3(look.x, look.y, 0);
            if(planeNormal_.length2() < 1e-6)
                planeNormal_ = Vector3(1, 0, 0);
            planeNormal_.normalize();
        }
        else
            planeNormal_ = Vector3(0, 0, 1);

        if(!PlaneHit(event->motion.x, event->motion.y, dragStart_))
            return;
        dragStartTransforms_.clear();
        for(StaticEntity* e : group_)
            dragStartTransforms_.push_back(e->getTransform());
        dragging_ = true;
        return;
    }

    Vector3 hit;
    if(!PlaneHit(event->motion.x, event->motion.y, hit))
        return;
    Vector3 delta = hit - dragStart_;
    if(dragVertical_)
        delta = Vector3(0, 0, delta.getZ());
    else
        delta.setZ(0);
    for(size_t i = 0; i < group_.size(); ++i)
    {
        Transform T = dragStartTransforms_[i];
        T.setOrigin(T.getOrigin() + delta);
        group_[i]->setTransform(T);
        moved_.insert(group_[i]->getName());
    }
}

void SceneEditor::MouseUp(SDL_Event* event)
{
    if(event->button.button == SDL_BUTTON_LEFT)
        dragging_ = false;
}

void SceneEditor::AddLandmark()
{
    Entity* picked = sim_->getEntity(pickedName_);
    Vector3 c;
    if(picked == nullptr || group_.empty() || !GroupCenter(c))
        return;

    auto types = SceneLandmarks::TypeNames();
    auto type = std::next(types.begin(), typeIndex_);
    SceneLandmark l;
    l.entityName = pickedName_;
    l.typeName = type->first;
    auto subtypes = SceneLandmarks::SubtypeNames().find(l.typeName);
    if(subtypes != SceneLandmarks::SubtypeNames().end() && subtypeIndex_ < subtypes->second.size())
        l.subtypeName = std::next(subtypes->second.begin(), subtypeIndex_)->first;
    // At the centre of the whole group, stored in the picked object's frame so it follows it
    l.point = ((StaticEntity*)picked)->getTransform().inverse() * c;

    std::string error;
    if(sim_->getLandmarks().Add(l, error))
    {
        landmarksChanged_.insert(pickedName_);
        SetStatus("Added " + l.typeName + (l.subtypeName.empty() ? "" : " / " + l.subtypeName) + " (save to keep)");
    }
    else
        SetStatus(error);
}

void SceneEditor::RemoveLandmarks()
{
    size_t n = 0;
    for(StaticEntity* e : group_)
    {
        n += sim_->getLandmarks().Of(e->getName()).size();
        sim_->getLandmarks().Set(e->getName(), {});
        landmarksChanged_.insert(e->getName());
    }
    SetStatus("Removed " + std::to_string(n) + " landmark(s) of " + groupName_ + " (save to keep)");
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
            if(text.find("<landmark object=") != std::string::npos)
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

    for(const auto& [file, text] : texts)
        if(text != ReadFile(file) && !WriteFile(file, text))
        {
            message = "Could not write " + file;
            return false;
        }

    moved_.clear();
    landmarksChanged_.clear();
    message = "Saved " + std::to_string(savedPoses) + " object pose(s), " + std::to_string(savedLandmarks) + " landmark(s)";
    return true;
}

void SceneEditor::DoHUD()
{
    IMGUI* gui = app_->getGUI();
    GLfloat w = 250.f;
    GLfloat x = (GLfloat)app_->getWindowWidth() - w - 10.f;
    GLfloat y = 10.f;
    Uid id;
    id.owner = kOwner;

    if(!active_)
    {
        gui->DoPanel(x, y, w, 28.f);
        gui->DoLabel(x + 5.f, y + 8.f, "Press E to edit the scene");
        return;
    }
    UpdateSelection();

    gui->DoPanel(x, y, w, 330.f);
    y += 5.f;
    gui->DoLabel(x + 5.f, y, "SCENE EDITOR (E to exit)");
    y += 20.f;

    char buf[256];
    Vector3 c;
    if(GroupCenter(c))
    {
        snprintf(buf, sizeof(buf), "Selected: %s (%zu part%s)", groupName_.c_str(), group_.size(), group_.size() == 1 ? "" : "s");
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        snprintf(buf, sizeof(buf), "Centre N %.2f  E %.2f  D %.2f", (double)c.getX(), (double)c.getY(), (double)c.getZ());
        gui->DoLabel(x + 5.f, y, buf);
        y += 15.f;
        size_t n = 0;
        for(StaticEntity* e : group_)
            n += sim_->getLandmarks().Of(e->getName()).size();
        snprintf(buf, sizeof(buf), "Landmarks: %zu", n);
        gui->DoLabel(x + 5.f, y, buf);
    }
    else
        gui->DoLabel(x + 5.f, y, "Click an object to select it");
    y += 20.f;

    gui->DoLabel(x + 5.f, y, "Drag: move   Shift+drag: up/down");
    y += 13.f;
    gui->DoLabel(x + 5.f, y, "Arrows/PgUp/PgDn: nudge   , . : rotate");
    y += 22.f;

    // Landmark type and subtype
    std::vector<std::string> types;
    for(const auto& [name, value] : SceneLandmarks::TypeNames())
        types.push_back(name);
    id.item = 0;
    unsigned int newType = gui->DoComboBox(id, x + 5.f, y, w - 10.f, types, typeIndex_, "Landmark type");
    if(newType != typeIndex_)
    {
        typeIndex_ = newType;
        subtypeIndex_ = 0;
    }
    y += 45.f;

    std::vector<std::string> subtypes;
    auto sub = SceneLandmarks::SubtypeNames().find(types[typeIndex_]);
    if(sub != SceneLandmarks::SubtypeNames().end())
        for(const auto& [name, value] : sub->second)
            subtypes.push_back(name);
    if(subtypes.empty())
        subtypes.push_back("(none)");
    id.item = 1;
    subtypeIndex_ = std::min(gui->DoComboBox(id, x + 5.f, y, w - 10.f, subtypes, subtypeIndex_, "Subtype"), (unsigned int)subtypes.size() - 1);
    y += 45.f;

    id.item = 2;
    if(gui->DoButton(id, x + 5.f, y, (w - 15.f) / 2.f, 25.f, "Add landmark") && !group_.empty())
        AddLandmark();
    id.item = 3;
    if(gui->DoButton(id, x + 10.f + (w - 15.f) / 2.f, y, (w - 15.f) / 2.f, 25.f, "Remove landmarks") && !group_.empty())
        RemoveLandmarks();
    y += 35.f;

    snprintf(buf, sizeof(buf), "Save scene (%zu change%s)", moved_.size() + landmarksChanged_.size(),
             moved_.size() + landmarksChanged_.size() == 1 ? "" : "s");
    id.item = 4;
    if(gui->DoButton(id, x + 5.f, y, w - 10.f, 25.f, buf))
    {
        std::string message;
        Save(message);
        SetStatus(message);
    }
    y += 35.f;

    if(!status_.empty() && GetTimeInMicroseconds() - statusTime_ < 6000000)
        gui->DoLabel(x + 5.f, y, status_);
}

}

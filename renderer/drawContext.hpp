#pragma once
#include "pathfinding.hpp"
#include "pathopt.hpp"
#include "textureGen.hpp"
namespace sdpf::renderer {

struct context {
    navmesh::navmesh* mesh = nullptr;
    kdtree::tree* tree = nullptr;
    double minPathWith = 8;
    bool showSDFMode = false;
    bool showVsdfMap = false;
    bool showNavFlow = false;
    bool showNodes = true;
    bool showWays = true;
    bool showPathFindingWays = false;
    bool showOptWays = true;
    std::vector<std::vector<double>> points{};
    ivec2 point_begin = ivec2(-1, -1);
    ivec2 point_target = ivec2(-1, -1);
    std::vector<ivec2> way_begin{};
    std::vector<ivec2> way_target{};
    std::vector<ivec2> way_pathfinding{};
    std::vector<vec2> way_pathopt{};
    ~context() {
        if (mesh) {
            delete mesh;
        }
        if (tree) {
            delete tree;
        }
    }
    enum viewMode_e {
        VIEW = 0,
        ADD_POINT = 1,
        SET_BEGIN = 2,
        SET_TARGET = 3
    };
    int viewMode = VIEW;
    inline void addPoint(double x, double y) {
        points.push_back(std::vector<double>({x / 5., y / 5.}));
    }
    inline void updatePath() {
        pathfinding::buildNodePath(
            *mesh,
            vec2(point_begin.x, point_begin.y),
            vec2(point_target.x, point_target.y), way_pathfinding);
        std::vector<vec2> inPath;
        for (auto& it : way_pathfinding) {
            inPath.push_back(vec2(it.x, it.y));
        }
        pathopt::optPath(inPath, mesh->sdfMap, 8, way_pathopt);
    }
    inline void setBegin(double x, double y) {
        point_begin = ivec2(x / 5., y / 5.);
        ivec2 target;
        navmesh::toRoad(*mesh, point_begin, way_begin, target);
        if (point_begin.x >= 0 && point_begin.x < mesh->width &&
            point_begin.y >= 0 && point_begin.y < mesh->height) {
            updatePath();
        }
    }
    inline void setTarget(double x, double y) {
        point_target = ivec2(x / 5., y / 5.);
        ivec2 target;
        navmesh::toRoad(*mesh, point_target, way_target, target);
        if (point_begin.x >= 0 && point_begin.x < mesh->width &&
            point_begin.y >= 0 && point_begin.y < mesh->height) {
            updatePath();
        }
    }
    inline void updateMesh() {
        if (mesh) {
            delete mesh;
            mesh = nullptr;
        }
        if (tree) {
            delete tree;
            tree = nullptr;
        }
        if (!points.empty()) {
            tree = new kdtree::tree(points, 2);
            //for (auto& it : points) {
            //    printf("point:(%lf,%lf)\n", it.at(0), it.at(1));
            //}
            //tree->printKdTree();
            mesh = new navmesh::navmesh(128, 128);
            navmesh::buildSdfMap(*mesh, *tree);
            std::vector<ivec2> starts;
            navmesh::buildIdMap(*mesh, starts, minPathWith);
            navmesh::removeWaste(*mesh, starts);

            //for (auto& it : starts) {
            //    printf("tops:(%d,%d)\n", it.x, it.y);
            //}
            navmesh::buildNodeBlock(*mesh, starts);
            navmesh::buildNavFlowField(*mesh, minPathWith);
        }
    }
    inline void render() {
        window_map();
        window_tool();
    }
    inline void window_map() {
        ImGui::SetNextWindowSize(ImVec2(640, 640), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("地图", nullptr, 0)) {
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            if (mesh) {
                if (showSDFMode) {
                    drawSDF(*mesh, p0);
                } else {
                    drawRoad(*mesh, p0);
                }
                if (showVsdfMap) {
                    drawFlow(mesh->vsdfMap, ImColor(ImVec4(1.0f, 0.0f, 0.0f, 1.0f)), p0);
                }
                if (showNavFlow) {
                    drawFlow(mesh->pathNavMap, ImColor(ImVec4(1.0f, 0.0f, 1.0f, 1.0f)), p0);
                }
                if (showNodes) {
                    drawNodes(*mesh, p0);
                }
                if (showWays) {
                    drawWays(*mesh, p0);
                }
                if (showPathFindingWays) {
                    drawPath(way_pathfinding, p0, ImColor(ImVec4(0.0f, 1.0f, 1.0f, 1.0f)));
                    drawPath(way_begin, p0, ImColor(ImVec4(1.0f, 0.0f, 0.0f, 1.0f)), 2.0);
                    drawPath(way_target, p0, ImColor(ImVec4(0.0f, 0.0f, 1.0f, 1.0f)), 2.0);
                }
                if (showOptWays) {
                    drawPath(way_pathopt, p0, ImColor(ImVec4(1.0f, 0.0f, 1.0f, 1.0f)));
                }
                if (point_begin.x >= 0 && point_begin.y >= 0) {
                    draw_list->AddCircleFilled(
                        ImVec2(point_begin.x * 5 + p0.x + 2, point_begin.y * 5 + p0.y + 2),
                        16, ImColor(ImVec4(1.f, 0, 0, 1.0f)));
                }
                if (point_target.x >= 0 && point_target.y >= 0) {
                    draw_list->AddCircleFilled(
                        ImVec2(point_target.x * 5 + p0.x + 2, point_target.y * 5 + p0.y + 2),
                        16, ImColor(ImVec4(0, 0, 1.f, 1.0f)));
                }
            }
            drawPoints(points, ImColor(ImVec4(1.0f, 1.0f, 0.0f, 1.0f)), p0);
            if (ImGui::IsWindowHovered()) {
                auto mpos = ImGui::GetMousePos();
                ImVec2 wpos(mpos.x - p0.x, mpos.y - p0.y);
                if (wpos.x >= 0 && wpos.y >= 0) {
                    if (viewMode == ADD_POINT) {
                        if (ImGui::IsMouseClicked(0)) {
                            addPoint(wpos.x, wpos.y);
                        }
                    }
                    if (mesh && wpos.x < mesh->width * 5 && wpos.y < mesh->height * 5) {
                        if (ImGui::IsMouseClicked(0)) {
                            if (viewMode == SET_BEGIN) {
                                setBegin(wpos.x, wpos.y);
                            }
                            if (viewMode == SET_TARGET) {
                                setTarget(wpos.x, wpos.y);
                            }
                        }
                    }
                }
            }
        }
        ImGui::End();
    }
    inline void window_tool() {
        if (ImGui::Begin("操作", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Checkbox("显示SDF", &showSDFMode);
            ImGui::Checkbox("显示SDF方向", &showVsdfMap);
            ImGui::Checkbox("显示导航流场", &showNavFlow);
            ImGui::Checkbox("显示节点", &showNodes);
            ImGui::Checkbox("显示路线", &showWays);
            ImGui::Checkbox("显示规划的路线", &showPathFindingWays);
            ImGui::Checkbox("显示优化的路线", &showOptWays);
            if (ImGui::Button("生成地图")) {
                updateMesh();
            }
            if (ImGui::Button("清空点云")) {
                points.clear();
            }

            int v = viewMode;
            ImGui::RadioButton("浏览模式", &v, VIEW);
            ImGui::RadioButton("设置点云", &v, ADD_POINT);
            ImGui::RadioButton("设置起点", &v, SET_BEGIN);
            ImGui::RadioButton("设置终点", &v, SET_TARGET);
            viewMode = v;
        }
        ImGui::End();
    }
};

}  // namespace sdpf::renderer
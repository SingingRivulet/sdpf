#pragma once
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
    std::vector<std::vector<double>> points{};
    ~context() {
        if (mesh) {
            delete mesh;
        }
        if (tree) {
            delete tree;
        }
    }
    enum viewMode_e {
        NONE,
        ADD_POINT
    } viewMode = ADD_POINT;
    inline void addPoint(double x, double y) {
        points.push_back(std::vector<double>({x / 5., y / 5.}));
    }
    inline void updateMesh() {
        if (mesh) {
            delete mesh;
        }
        if (tree) {
            delete tree;
        }
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
        navmesh::buildNavFlowField(*mesh, starts, minPathWith);
        navmesh::buildNodeBlock(*mesh, starts);
        navmesh::buildConnect(*mesh);
    }
    inline void render() {
        window_map();
        window_tool();
    }
    inline void window_map() {
        ImGui::SetNextWindowSize(ImVec2(640, 640), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("地图", nullptr, 0)) {
            ImVec2 p0 = ImGui::GetCursorScreenPos();
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
            if (ImGui::Button("生成地图")) {
                updateMesh();
            }
            if (ImGui::Button("清空点云")) {
                points.clear();
            }
        }
        ImGui::End();
    }
};

}  // namespace sdpf::renderer
#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_impl_sdlrenderer.h>
#include <filesystem>
#include <iostream>
#include "navmesh.hpp"
namespace sdpf::renderer {

template <class T>
concept vsdfMap = requires(T& map) {
    sdpf::vec2(std::get<0>(map.at(int(), int())));
    sdpf::vec2(std::get<1>(map.at(int(), int())));
};

template <class T>
concept pathNavMap = requires(T& map) {
    sdpf::ivec2(std::get<0>(map.at(int(), int())));
};

template <class T>
concept pointsArray = requires(T& points) {
    //例如std::vector<std::vector<double> > d；
    double((*points.begin())[0]);
    points.end();
};

inline void drawSDF(navmesh::navmesh& map, const ImVec2& p0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImU32 col_w = ImColor(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    for (int x = 0; x < map.width; ++x) {
        for (int y = 0; y < map.height; ++y) {
            int px = x * 5;
            int py = y * 5;
            auto val = map.sdfMap.at(x, y);
            if (val > 256) {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + 5), col_w);
            } else {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + p0.y + 5),
                                         ImColor(ImVec4(val / 256, val / 256, val / 256, 1.0f)));
            }
        }
    }
}
inline void drawRoad(navmesh::navmesh& map, const ImVec2& p0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImU32 col_b = ImColor(ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
    const ImU32 col_1 = ImColor(ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
    const ImU32 col_2 = ImColor(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    const ImU32 col_3 = ImColor(ImVec4(1.0f, 0.7f, 0.0f, 1.0f));
    for (int x = 0; x < map.width; ++x) {
        for (int y = 0; y < map.height; ++y) {
            int px = x * 5;
            int py = y * 5;
            if (map.idMap.at(x, y) == -1) {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + p0.y + 5), col_1);
            } else if (map.idMap.at(x, y) == -2) {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + p0.y + 5), col_2);
            } else if (map.idMap.at(x, y) > 0) {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + p0.y + 5), col_3);
            } else {
                draw_list->AddRectFilled(ImVec2(px + p0.x, py + p0.y),
                                         ImVec2(px + p0.x + 5, py + p0.y + 5), col_b);
            }
        }
    }
}

inline void drawNodes(navmesh::navmesh& map, const ImVec2& p0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImU32 col = ImColor(ImVec4(0.5f, 0.7f, 0.5f, 1.0f));
    for (auto& it : map.nodes) {
        draw_list->AddCircleFilled(
            ImVec2(it->position.x * 5 + p0.x + 2, it->position.y * 5 + p0.y + 2),
            16, col);
    }
}

template <typename T>
inline void drawPath(const T& path, const ImVec2& p0, ImU32 col) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 last;
    bool first = true;
    for (auto& point : path) {
        ImVec2 now(point.x * 5 + p0.x + 2, point.y * 5 + p0.y + 2);
        if (!first) {
            draw_list->AddLine(last, now, col, 4.0);
        }
        first = false;
        last = now;
    }
}

inline void drawWays(navmesh::navmesh& map, const ImVec2& p0) {
    const ImU32 col = ImColor(ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
    for (auto& way_it : map.ways) {
        drawPath(way_it.second->maxPath, p0, col);
    }
}

template <pointsArray T>
inline void drawPoints(T& array, ImU32 col, const ImVec2& p0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    for (auto& it : array) {
        ImVec2 begin(it[0] * 5 + p0.x + 2, it[1] * 5 + p0.y + 2);
        ImVec2 end(begin.x + 2, begin.y + 2);
        draw_list->AddRectFilled(begin, end, col);
    }
}

template <vsdfMap T>
constexpr bool getDir(T& map, int x, int y, ivec2& out) {
    auto o = std::get<0>(map.at(x, y));
    out = ivec2(o.x, o.y);
    return true;
}

template <pathNavMap T>
constexpr bool getDir(T& map, int x, int y, ivec2& out) {
    auto pos = std::get<0>(map.at(x, y));
    out = pos - ivec2(x, y);
    return (pos.x >= 0 && pos.y >= 0);
}

template <class T>
inline void drawFlow(T& map, ImU32 col, const ImVec2& p0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    for (int x = 0; x < map.width; ++x) {
        for (int y = 0; y < map.height; ++y) {
            ivec2 dir;
            if (getDir(map, x, y, dir)) {
                float len = sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len != 0) {
                    int dx = round(dir.x * 4 / len);
                    int dy = round(dir.y * 4 / len);
                    draw_list->AddLine(
                        ImVec2(x * 5 + p0.x + 2,
                               y * 5 + p0.y + 2),
                        ImVec2(x * 5 + p0.x + 2 + dx,
                               y * 5 + p0.y + 2 + dy),
                        col);

                    draw_list->AddRectFilled(
                        ImVec2(x * 5 + p0.x + 2,
                               y * 5 + p0.y + 2),

                        ImVec2(x * 5 + p0.x + 2 + 1,
                               y * 5 + p0.y + 2 + 1),
                        0xffffffff);
                }
            }
        }
    }
}

}  // namespace sdpf::renderer
#pragma once
#include <math.h>
#include <stdexcept>
#include "astar.hpp"
#include "navmesh.hpp"

//寻路
namespace sdpf::pathfinding {
struct context {
    vec2 start, target;
    ivec2 pathStart, pathEnd;
};
struct nodePath {
    std::vector<navmesh::node*> nodes;
    double length;
};

inline void buildNodePath(nodePath& ctx,           //路线
                          navmesh::navmesh& mesh,  //mesh
                          navmesh::node* begin,    //起点
                          navmesh::node* target,   //终点
                          int it_count = 512       //迭代次数
) {
    astar_node::context atx;
    astar_node::start(atx, begin, target, it_count);
    ctx.nodes.clear();
    ctx.length = -1;
    astar_node::buildRoad(atx, [&](navmesh::node* n) {
        ctx.nodes.push_back(n);
    });
    std::reverse(ctx.nodes.begin(), ctx.nodes.end());
    if (atx.result) {
        ctx.length = atx.result->h;
    }
}

inline void buildTmpWay(navmesh::navmesh& mesh,
                        navmesh::way& way,
                        const ivec2& start,
                        int targetId) {
    bool rev = false;
    bool lengthRev = false;
    way.reverse = false;
    auto targetNode = mesh.nodes.at(targetId - 1).get();
    auto targetBlock = mesh.pathDisMap.at(start.x, start.y);
    int a = std::get<0>(targetBlock);
    int b = std::get<1>(targetBlock);
    int startIndex = std::get<3>(targetBlock);
    if (a == targetId) {
        rev = true;
        lengthRev = true;
    } else if (b == targetId) {
    } else {
        return;
    }
    if (a > b) {
        std::swap(a, b);
        rev = !rev;
    }
    auto pair = std::make_pair(a, b);
    auto it = mesh.ways.find(pair);
    if (it != mesh.ways.end()) {
        if (it->second->reverse) {
            rev = !rev;
        }
        auto& path = it->second->maxPath;
        auto len = path.size();
        way.minWidth = it->second->minWidth;
        way.maxPath.clear();
        if (rev) {
            for (int i = startIndex; i >= 0; --i) {
                way.maxPath.push_back(path[i]);
            }
        } else {
            for (int i = startIndex; i < len; ++i) {
                way.maxPath.push_back(path[i]);
            }
        }
        if (lengthRev) {
            way.minWidth = std::get<2>(targetBlock);
        } else {
            way.minWidth = it->second->minWidth - std::get<2>(targetBlock);
        }
    }
}

inline void buildPathPos(navmesh::navmesh& mesh,
                         navmesh::node* a,
                         navmesh::node* b,
                         std::vector<ivec2>& path) {
    int32_t id1 = a->id;
    int32_t id2 = b->id;
    bool rev = false;
    if (id1 > id2) {
        std::swap(id1, id2);
        rev = true;
    }
    auto pair = std::make_pair(id1, id2);
    auto it = mesh.ways.find(pair);
    if (it != mesh.ways.end()) {
        if (it->second->reverse) {
            rev = !rev;
        }
        path = it->second->maxPath;
        if (rev) {
            std::reverse(path.begin(), path.end());
        }
    }
}

inline void buildNodePath(navmesh::navmesh& mesh,    //mesh
                          vec2 begin,                //起点
                          vec2 target,               //终点
                          std::vector<ivec2>& path,  //最终路线
                          int it_count = 512         //迭代次数
) {
    std::vector<ivec2> pathWayStart, pathWayTarget;
    ivec2 wayStart, wayEnd;
    //利用流场求解道路上的起止点
    if (!navmesh::toRoad(mesh, ivec2(begin.x, begin.y), pathWayStart, wayStart)) {
        return;
    }
    if (!navmesh::toRoad(mesh, ivec2(target.x, target.y), pathWayTarget, wayEnd)) {
        return;
    }

    auto& dStart = mesh.pathDisMap.at(wayStart.x, wayStart.y);
    //构造临时节点
    navmesh::node dStart_node_tmp;
    navmesh::way dStart_way1, dStart_way2;
    dStart_node_tmp.ways.insert(&dStart_way1);
    dStart_node_tmp.ways.insert(&dStart_way2);
    dStart_way1.p1 = &dStart_node_tmp;
    dStart_way2.p1 = &dStart_node_tmp;
    //构造临时路线
    int dStart_id1 = std::get<0>(dStart);
    int dStart_id2 = std::get<1>(dStart);
    auto dStart_node1 = mesh.nodes.at(dStart_id1 - 1).get();
    auto dStart_node2 = mesh.nodes.at(dStart_id2 - 1).get();
    dStart_way1.p2 = dStart_node1;
    dStart_way2.p2 = dStart_node2;
    buildTmpWay(mesh, dStart_way1, wayEnd, dStart_id1);
    buildTmpWay(mesh, dStart_way2, wayEnd, dStart_id2);

    auto& dEnd = mesh.pathDisMap.at(wayEnd.x, wayEnd.y);
    //构造临时节点
    navmesh::node dTarget_node_tmp;
    navmesh::way dTarget_way1, dTarget_way2;
    dTarget_node_tmp.ways.insert(&dTarget_way1);
    dTarget_node_tmp.ways.insert(&dTarget_way2);
    dTarget_way1.p1 = &dTarget_node_tmp;
    dTarget_way2.p1 = &dTarget_node_tmp;
    //构造临时路线
    int dEnd_id1 = std::get<0>(dEnd);
    int dEnd_id2 = std::get<1>(dEnd);
    auto dEnd_node1 = mesh.nodes.at(dEnd_id1 - 1).get();
    auto dEnd_node2 = mesh.nodes.at(dEnd_id2 - 1).get();
    dTarget_way1.p2 = dStart_node1;
    dTarget_way2.p2 = dStart_node2;
    buildTmpWay(mesh, dTarget_way1, wayEnd, dEnd_id1);
    buildTmpWay(mesh, dTarget_way2, wayEnd, dEnd_id2);

    //构造路线
    nodePath nodePath;
    buildNodePath(nodePath, mesh, &dStart_node_tmp, &dTarget_node_tmp, it_count);

    path.clear();
    for (auto& it : pathWayStart) {
        path.push_back(it);
    }
    navmesh::node* last = nullptr;
    for (auto& it : nodePath.nodes) {
        if (last != nullptr) {
            std::vector<ivec2> tmp;
            buildPathPos(mesh, last, it, tmp);
            for (auto& it : tmp) {
                path.push_back(it);
            }
        }
        last = it;
    }
    for (auto it = pathWayTarget.rbegin(); it != pathWayTarget.rend(); ++it) {
        path.push_back(*it);
    }
}
}  // namespace sdpf::pathfinding
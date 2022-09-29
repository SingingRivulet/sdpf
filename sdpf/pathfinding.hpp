#pragma once
#include <math.h>
#include <stdexcept>
#include "astar.hpp"
#include "navmesh.hpp"

//寻路
namespace sdpf::pathfinding {

inline void buildTmpWay(navmesh::navmesh& mesh,
                        navmesh::way& way,
                        const ivec2& start,
                        int targetId) {
    bool rev = false;
    bool lengthRev = false;
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
    //printf("buildTmpWay %d %d\n", a, b);
    auto pair = std::make_pair(a, b);
    auto it = mesh.ways.find(pair);
    if (it != mesh.ways.end()) {
        //printf("set path\n");
        auto& path = it->second->maxPath;
        auto len = path.size();
        way.minWidth = it->second->minWidth;
        ivec2 last;
        bool first = true;
        double lenSum = 0;
        if (rev) {
            for (int i = startIndex; i >= 0; --i) {
                way.maxPath.push_back(path[i]);
                way.minWidth = std::min(way.minWidth,
                                        mesh.sdfMap.at(path[i].x, path[i].y));
                if (!first) {
                    lenSum += path[i].length(last);
                }
                last = path[i];
                first = false;
            }
        } else {
            for (int i = startIndex; i < len; ++i) {
                way.maxPath.push_back(path[i]);
                way.minWidth = std::min(way.minWidth,
                                        mesh.sdfMap.at(path[i].x, path[i].y));
                if (!first) {
                    lenSum += path[i].length(last);
                }
                last = path[i];
                first = false;
            }
        }
        //printf("lenSum=%lf\n", lenSum);
        way.length += lenSum;
    }
}

inline void buildNodePath(navmesh::navmesh& mesh,    //mesh
                          vec2 begin,                //起点
                          vec2 target,               //终点
                          std::vector<ivec2>& path,  //最终路线
                          int it_count = 512,        //迭代次数
                          double minPathWidth = 8) {
    std::vector<ivec2> pathWayStart, pathWayTarget;
    ivec2 wayStart, wayEnd;
    //利用流场求解道路上的起止点
    if (!navmesh::toRoad(mesh, ivec2(begin.x, begin.y), pathWayStart, wayStart)) {
        return;
    }
    if (!navmesh::toRoad(mesh, ivec2(target.x, target.y), pathWayTarget, wayEnd)) {
        return;
    }

    //计算宽度和路线长度
    double wayStartLen = 0;
    double wayStartMinWidth = INFINITY;
    {
        ivec2 last;
        bool first = true;
        for (auto& it : pathWayStart) {
            if (!first) {
                wayStartLen += it.length(last);
            }
            wayStartMinWidth = std::min(wayStartMinWidth, mesh.sdfMap.at(it.x, it.y));
            first = false;
            last = it;
        }
    }
    double wayTargetLen = 0;
    double wayTargetMinWidth = INFINITY;
    {
        ivec2 last;
        bool first = true;
        for (auto& it : pathWayTarget) {
            if (!first) {
                wayTargetLen += it.length(last);
            }
            wayTargetMinWidth = std::min(wayTargetMinWidth, mesh.sdfMap.at(it.x, it.y));
            first = false;
            last = it;
        }
    }
    //printf("wayStartLen=%lf wayTargetLen=%lf\n", wayStartLen, wayTargetLen);

    //printf("wayStart=(%d,%d) wayEnd=(%d,%d)\n", wayStart.x, wayStart.y, wayEnd.x, wayEnd.y);

    auto& dStart = mesh.pathDisMap.at(wayStart.x, wayStart.y);
    auto& dEnd = mesh.pathDisMap.at(wayEnd.x, wayEnd.y);
    //构造临时节点
    navmesh::node dStart_node_tmp;
    dStart_node_tmp.flowFieldFlag = 0;
    dStart_node_tmp.id = -1;
    navmesh::way dStart_way1, dStart_way2;
    dStart_way1.maxPath = pathWayStart;
    dStart_way1.length = wayStartLen;
    dStart_way1.minWidth = wayStartMinWidth;
    dStart_way1.p1 = &dStart_node_tmp;
    dStart_way2.maxPath = pathWayStart;
    dStart_way2.length = wayStartLen;
    dStart_way2.minWidth = wayStartMinWidth;
    dStart_way2.p1 = &dStart_node_tmp;

    navmesh::node dTarget_node_tmp;
    dTarget_node_tmp.flowFieldFlag = 0;
    dTarget_node_tmp.id = -2;
    navmesh::way dTarget_way1, dTarget_way2;
    dTarget_way1.maxPath = pathWayTarget;
    dTarget_way1.length = wayTargetLen;
    dTarget_way1.minWidth = wayTargetMinWidth;
    dTarget_way1.p1 = &dTarget_node_tmp;
    dTarget_way2.maxPath = pathWayTarget;
    dTarget_way2.length = wayTargetLen;
    dTarget_way2.minWidth = wayTargetMinWidth;
    dTarget_way2.p1 = &dTarget_node_tmp;

    std::vector<navmesh::node*> nodeClearList;

    //构造临时路线
    int dStart_id1 = std::get<0>(dStart);
    int dStart_id2 = std::get<1>(dStart);
    int dEnd_id1 = std::get<0>(dEnd);
    int dEnd_id2 = std::get<1>(dEnd);
    if (dStart_id1 <= 0) {
        return;
    }
    if (dEnd_id1 <= 0) {
        return;
    }

    if (dStart_id2 <= 0) {  //直达
        auto dStart_node1 = mesh.nodes.at(dStart_id1 - 1).get();
        dStart_way1.p2 = dStart_node1;
        dStart_node_tmp.ways.insert(&dStart_way1);
        nodeClearList.push_back(dStart_node1);
    } else {
        auto dStart_node1 = mesh.nodes.at(dStart_id1 - 1).get();
        dStart_way1.p2 = dStart_node1;
        buildTmpWay(mesh, dStart_way1, wayStart, dStart_id1);
        dStart_node_tmp.ways.insert(&dStart_way1);
        nodeClearList.push_back(dStart_node1);

        auto dStart_node2 = mesh.nodes.at(dStart_id2 - 1).get();
        dStart_way2.p2 = dStart_node2;
        buildTmpWay(mesh, dStart_way2, wayStart, dStart_id2);
        dStart_node_tmp.ways.insert(&dStart_way2);
        dStart_node2->tmpways.insert(&dStart_way2);
        nodeClearList.push_back(dStart_node2);
    }

    if (dEnd_id2 <= 0) {  //直达
        auto dEnd_node1 = mesh.nodes.at(dEnd_id1 - 1).get();
        dTarget_way1.p2 = dEnd_node1;
        dTarget_node_tmp.ways.insert(&dTarget_way1);
        dEnd_node1->tmpways.insert(&dTarget_way1);
        nodeClearList.push_back(dEnd_node1);
    } else {
        auto dEnd_node1 = mesh.nodes.at(dEnd_id1 - 1).get();
        dTarget_way1.p2 = dEnd_node1;
        buildTmpWay(mesh, dTarget_way1, wayEnd, dEnd_id1);
        dTarget_node_tmp.ways.insert(&dTarget_way1);
        dEnd_node1->tmpways.insert(&dTarget_way1);
        nodeClearList.push_back(dEnd_node1);

        auto dEnd_node2 = mesh.nodes.at(dEnd_id2 - 1).get();
        dTarget_way2.p2 = dEnd_node2;
        buildTmpWay(mesh, dTarget_way2, wayEnd, dEnd_id2);
        dTarget_node_tmp.ways.insert(&dTarget_way2);
        dEnd_node2->tmpways.insert(&dTarget_way2);
        nodeClearList.push_back(dEnd_node2);
    }

    //构造路线
    path.clear();
    //使用流场的寻路方式
    printf("buildMeshFlowField\n");
    navmesh::buildMeshFlowField(mesh, &dTarget_node_tmp);  //流场寻路只需要终点
    //printf("dStart_node_tmp.id=%d\n", dStart_node_tmp.id);
    navmesh::node* targetNavNode = &dStart_node_tmp;
    navmesh::node* last = nullptr;
    int count = 0;
    while (targetNavNode && targetNavNode->flowFieldFlag == mesh.searchMap_id) {
        auto w = targetNavNode->flowDir;
        if (w) {
            auto& nodepath = w->maxPath;
            if (w->p1 == targetNavNode) {
                targetNavNode = w->p2;
                for (auto itp = nodepath.begin(); itp != nodepath.end(); ++itp) {
                    path.push_back(*itp);
                }
                printf("%d->%d length=%lf\n", w->p1->id, w->p2->id, w->length);
            } else {
                targetNavNode = w->p1;
                for (auto itp = nodepath.rbegin(); itp != nodepath.rend(); ++itp) {
                    path.push_back(*itp);
                }
                printf("%d->%d length=%lf\n", w->p2->id, w->p1->id, w->length);
            }
            last = targetNavNode;
        } else {
            break;
        }
        ++count;
        if (count > 512) {
            break;
        }
    }
    for (auto it : nodeClearList) {
        it->tmpways.clear();
    }
    /*
    //使用a*的寻路方式
    astar_node::context atx;
    astar_node::start(
        atx, &dStart_node_tmp, &dTarget_node_tmp, [&](const auto& node, auto callback) {
            //printf("find path id:%d\n", node->id);
            for (auto it : node->ways) {
                //printf("way:%d %d minWidth=%lf\n", it->p1->id, it->p2->id, it->minWidth);
                navmesh::node* targetNavNode;
                if (it->minWidth > minPathWidth) {  //可以通过
                    if (it->p1 == node) {
                        targetNavNode = it->p2;
                    } else {
                        targetNavNode = it->p1;
                    }
                    //printf("con:%d\n", targetNavNode->id);
                    callback(targetNavNode, it->length);
                }
            }
            if (node->id == dEnd_id1) {
                //printf("con:end1\n");
                callback(&dTarget_node_tmp, dTarget_way1.length);
            }
            if (node->id == dEnd_id2) {
                //printf("con:end2\n");
                callback(&dTarget_node_tmp, dTarget_way2.length);
            }
        },
        it_count);
    int lastNode = 0;
    astar_node::buildRoad(atx, [&](navmesh::node* n) {
        //printf("id:%d\n", n->id);
        bool rev = 0;
        if (lastNode == -2) {
            if (dTarget_way1.p2 == n) {
                auto& nodepath = dTarget_way1.maxPath;
                for (auto itp = nodepath.begin(); itp != nodepath.end(); ++itp) {
                    path.push_back(*itp);
                }
            } else if (dTarget_way2.p2 == n) {
                auto& nodepath = dTarget_way2.maxPath;
                for (auto itp = nodepath.begin(); itp != nodepath.end(); ++itp) {
                    path.push_back(*itp);
                }
            }
        } else if (lastNode > 0) {
            if (n->id == -1) {
                if (dStart_way1.p2->id == lastNode) {
                    auto& nodepath = dStart_way1.maxPath;
                    for (auto itp = nodepath.rbegin(); itp != nodepath.rend(); ++itp) {
                        path.push_back(*itp);
                    }
                } else if (dStart_way2.p2->id == lastNode) {
                    auto& nodepath = dStart_way2.maxPath;
                    for (auto itp = nodepath.rbegin(); itp != nodepath.rend(); ++itp) {
                        path.push_back(*itp);
                    }
                }
            } else if (n->id > 0) {
                int a = lastNode;
                int b = n->id;
                if (a > b) {
                    std::swap(a, b);
                    rev = !rev;
                }
                auto pair = std::make_pair(a, b);
                auto it = mesh.ways.find(pair);
                if (it != mesh.ways.end()) {
                    auto& nodepath = it->second->maxPath;
                    if (!rev) {
                        for (auto itp = nodepath.begin(); itp != nodepath.end(); ++itp) {
                            path.push_back(*itp);
                        }
                    } else {
                        for (auto itp = nodepath.rbegin(); itp != nodepath.rend(); ++itp) {
                            path.push_back(*itp);
                        }
                    }
                }
            }
        }
        lastNode = n->id;
    });
    */
    std::reverse(path.begin(), path.end());
}
}  // namespace sdpf::pathfinding
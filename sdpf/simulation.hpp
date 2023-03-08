#pragma once
#include "navmesh.hpp"
#include "pathopt.hpp"
namespace sdpf {

inline void simulation(activeNav::activeContext& ctx,
                       navmesh::navmesh& mesh,
                       std::vector<std::unique_ptr<activeNav::activeNode>>& activeNodes,
                       vec2 target,                     //终点
                       int pathfinding_it_count = 512,  //迭代次数
                       double minPathWidth = 8) {
    for (auto& it : activeNodes) {
        it->simulationPath.clear();
        it->currentPos = it->startPos;
    }
    int step = 0;
    while (1) {
        int processCount = 0;
        pathfinding::buildNodePath(mesh, activeNodes, target, pathfinding_it_count, minPathWidth);
        for (auto& node_pathfinding : activeNodes) {
            std::vector<vec2> inPath;
            for (auto& it : node_pathfinding->path) {
                inPath.push_back(vec2(it.x, it.y));
            }
            sdpf::vec2 tmp;
            bool res = pathopt::nextPos(inPath, mesh.sdfMap, ctx,
                                        node_pathfinding.get(), 8, tmp);
            node_pathfinding->currentPos = tmp;
            if (res) {
                ++processCount;
            }
            node_pathfinding->simulationPath.push_back(node_pathfinding->currentPos);
            printf("node_pathfinding->currentPos:(%f %f)\n", node_pathfinding->currentPos.x, node_pathfinding->currentPos.y);
        }
        for (auto& node_pathfinding : activeNodes) {
            node_pathfinding->update();
        }
        printf("step:%d processCount:%d\n", step, processCount);
        ++step;
        if (processCount <= 0 || step > 4096) {
            break;
        }
    }
    for (auto& it : activeNodes) {
        it->currentPos = it->startPos;
    }
}

}  // namespace sdpf
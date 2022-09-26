#pragma once
#include "kdtree.hpp"
#include "navmesh.hpp"

//点云处理
namespace sdpf::pointcloud {

void getPointDis(kdtree::tree& tree, const vec2& pos, vec2& dis, vec2& target) {
    auto res = tree.searchKdTree(std::vector<double>({pos.x, pos.y}));
    if (res) {
        //printf("search:(%lf,%lf)=>(%lf,%lf)\n", pos.x, pos.y, res->val.at(0), res->val.at(1));
        target = vec2(res->val.at(0), res->val.at(1));
        dis = target - pos;
    } else {
        target = vec2(0, 0);
        dis = vec2(0, 0);
    }
}

}  // namespace sdpf::pointcloud
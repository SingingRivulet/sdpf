#pragma once
#include "kdtree.hpp"
#include "navmesh.hpp"

//点云处理
namespace sdpf::pointcloud {

vec2 getPointDis(kdtree::tree& tree, const vec2& pos) {
    auto res = tree.searchKdTree(std::vector<double>({pos.x, pos.y}));
    if (res) {
        return vec2(res->val.at(0), res->val.at(1));
    } else {
        return vec2(0, 0);
    }
}

}  // namespace sdpf::pointcloud
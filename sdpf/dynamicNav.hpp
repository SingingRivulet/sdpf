#pragma once
#include <algorithm>
#include <iostream>
#include <vector>
#include "hbb.h"
#include "sdf.hpp"
namespace sdpf::dynamicNav {

struct dynamicContext {
    HBB indexer;
};

struct dynamicNode {
    vec2 startPos;
    vec2 currentPos;
    ivec2 wayStart;
    std::vector<ivec2> path;
    std::vector<ivec2> pathWayStart;
    double pathLength;
    std::vector<vec2> pathOpt;
    std::vector<vec2> pathLogger;
    std::vector<vec2> simulationPath;
    HBB::boundCircle* box = nullptr;
    bool active;
    double r = 8;
    dynamicContext* context = nullptr;
    inline ~dynamicNode() {
        disconnect();
    }
    inline void disconnect() {
        if (box) {
            box->autodrop();
            box = nullptr;
            context = nullptr;
        }
    }
    inline void connect(dynamicContext* ctx) {
        if (context) {
            return;
        }
        context = ctx;
        update();
    }
    inline void update() {
        if (box) {
            box->autodrop();
        }
        if (context) {
            box = context->indexer.add(currentPos, r, this);
        } else {
            box = nullptr;
            context = nullptr;
        }
    }
};

//计算路线长度
inline double getPathLength(std::vector<ivec2>& path) {
    ivec2 last;
    bool first = true;
    double pathLength = 0;
    for (auto& it : path) {
        if (!first) {
            pathLength += it.length(last);
        }
        first = false;
        last = it;
    }
    return pathLength;
}

//动态物体的光线测试
inline bool dynamicNavRayTest(dynamicContext& map,                //导航地图
                              const vec2& begin,                  //起点
                              const vec2& end,                    //终点
                              dynamicNav::dynamicNode* selfNode,  //自己的节点
                              double range) {
    struct arg_t {
        bool res = false;
        vec2 begin, end;
        dynamicNav::dynamicNode* selfNode;
    } arg;
    arg.begin = begin;
    auto dir = end - begin;
    arg.end = begin + dir * range / dir.norm();
    arg.selfNode = selfNode;
    map.indexer.fetchByRay(
        begin, end,
        [](HBB::boundCircle* box, void* arg) {
            auto self = (arg_t*)arg;
            if (box->data == self->selfNode) {
                return;
            }
            int status = 0;
            if (box->rayDist(self->begin, self->end, &status) < box->r) {
                if (status != 1) {
                    self->res = true;
                }
            }
        },
        &arg);
    return arg.res;
}

}  // namespace sdpf::dynamicNav
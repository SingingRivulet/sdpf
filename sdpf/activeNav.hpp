#pragma once
#include <algorithm>
#include <vector>
#include "hbb.h"
#include "sdf.hpp"
namespace sdpf::activeNav {

struct activeContext {
    HBB indexer;
};

struct activeNode {
    vec2 currentPos;
    ivec2 wayStart;
    std::vector<ivec2> path;
    std::vector<ivec2> pathWayStart;
    double pathLength;
    std::vector<vec2> pathOpt;
    std::vector<vec2> pathLogger;
    HBB::boundCircle* box = nullptr;
    bool active;
    double r = 8;
    activeContext* context = nullptr;
    inline ~activeNode() {
        disconnect();
    }
    inline void disconnect() {
        if (box) {
            box->autodrop();
            box = nullptr;
            context = nullptr;
        }
    }
    inline void connect(activeContext* ctx) {
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
inline bool activeNavRayTest(activeContext& map,              //导航地图
                             const vec2& begin,               //起点
                             const vec2& end,                 //终点
                             activeNav::activeNode* selfNode  //自己的节点
) {
    struct arg_t {
        bool res = false;
        vec2 begin, end;
        activeNav::activeNode* selfNode;
    } arg;
    arg.begin = begin;
    arg.end = end;
    arg.selfNode = selfNode;
    map.indexer.fetchByRay(
        begin, end,
        [](HBB::boundCircle* box, void* arg) {
            auto self = (arg_t*)arg;
            if (box->data == self->selfNode) {
                return;
            }
            if (box->intersects(self->begin, self->end) < box->r) {
                self->res = true;
            }
        },
        &arg);
    return arg.res;
}

}  // namespace sdpf::activeNav
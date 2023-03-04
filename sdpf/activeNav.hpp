#pragma once
#include <algorithm>
#include <vector>
#include "sdf.hpp"
namespace sdpf::activeNav {

struct activeNode {
    vec2 startPos;
    //vec2 nowPos;
    ivec2 wayStart;
    std::vector<ivec2> path;
    std::vector<ivec2> pathWayStart;
    std::vector<vec2> pathOpt;
    bool active;
};

}  // namespace sdpf::activeNav
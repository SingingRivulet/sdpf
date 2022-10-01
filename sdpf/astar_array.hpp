#pragma once
#include <map>
#include <vector>
#include "vec2.hpp"
namespace sdpf::astar_array {

struct node {  //寻路节点
    ivec2 position;
    double g = 0;  //起始点到当前点实际代价
    double h = 0;  //当前节点到目标节点最佳路径的估计代价
    double f = 0;  //估计值
    node* parent;  //父节点
};

struct context {  //寻路context
    std::map<ivec2, node*> openlist{}, closelist{};
    node* processing = nullptr;
    node* result = nullptr;
    ivec2 target;
    bool failed = false;
    double minPathWidth = 0;
    std::vector<node*> nodes;  //内存管理用的
    ~context() {
        for (auto it : nodes) {
            delete it;
        }
        nodes.clear();
    }
};

template <class callback_c>
inline void start(context& ctx,
                  const ivec2& begin,
                  const ivec2& target,
                  const callback_c& callback,
                  int it_count = 512) {
    auto st = new node;
    st->f = 0;
    st->g = 0;
    st->h = 0;
    st->parent = NULL;
    st->position = begin;
    ctx.nodes.push_back(st);
    ctx.processing = st;
    ctx.result = NULL;
    ctx.failed = false;
    ctx.target = target;

    int num = 0;
    while (ctx.processing) {
        step(ctx, callback);
        ++num;
        if (num > it_count)
            break;
    }
}
template <class callback_c>
inline void buildRoad(context& ctx, const callback_c& callback) {
    if (ctx.result) {
        auto p = ctx.result;
        while (p) {
            callback(p->position);
            p = p->parent;
        }
    }
}
inline int heuristic(const ivec2& p1, const ivec2& p2) {
    int x = p1.x - p2.x;
    int y = p1.y - p2.y;
    return sqrt(x * x + y * y);
}
template <class callback_c>
inline void step(context& ctx, const callback_c& callback) {
    if (ctx.processing == NULL)
        return;
    ctx.closelist[ctx.processing->position] = ctx.processing;

    std::vector<node*> ns;
    callback(ctx.processing->position, [&](const ivec2& pos) {
        if (ctx.openlist.find(pos) != ctx.openlist.end())
            return;
        if (ctx.closelist.find(pos) != ctx.closelist.end())
            return;
        auto p = new node;
        ctx.nodes.push_back(p);
        p->parent = ctx.processing;
        p->position = pos;

        //启发
        p->g = ctx.processing->g + (pos - ctx.processing->position).norm();
        p->h = heuristic(pos, ctx.target);
        p->f = p->g + p->h;

        ctx.openlist[pos] = p;
        ns.push_back(p);
    });
    if (ns.empty()) {
        if (ctx.openlist.empty()) {
            ctx.failed = true;  //搜索失败
            ctx.processing = NULL;
            return;
        } else {
            double minf = -1;
            node* min = nullptr;
            //openlist中找一个f最小的
            for (auto it : ctx.openlist) {
                if (minf == -1) {
                    minf = it.second->f;
                    min = it.second;
                } else {
                    if (it.second->f < minf) {
                        minf = it.second->f;
                        min = it.second;
                    }
                }
            }
            if (min) {
                ctx.processing = min;
                ctx.openlist.erase(min->position);
            }
        }
    } else {
        double minf = -1;
        node* min = nullptr;
        //ns中找一个f最小的
        for (auto it : ns) {
            if (heuristic(it->position, ctx.target) <= 1) {
                ctx.processing = NULL;
                ctx.result = it;
                return;
            }
            if (minf == -1) {
                minf = it->f;
                min = it;
            } else {
                if (it->f < minf) {
                    minf = it->f;
                    min = it;
                }
            }
        }
        if (min) {
            ctx.processing = min;
            ctx.openlist.erase(min->position);
        }
    }
}
}  // namespace sdpf::astar_array
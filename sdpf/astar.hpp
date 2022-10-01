#pragma once
#include "navmesh.hpp"
//寻路
namespace sdpf {
namespace astar_node {

struct node {  //寻路节点
    ivec2 position;
    double g = 0;            //起始点到当前点实际代价
    double h = 0;            //当前节点到目标节点最佳路径的估计代价
    double f = 0;            //估计值
    navmesh::node* navNode;  //导航节点
    node* parent;            //父节点
};

struct context {  //寻路context
    std::map<navmesh::node*, node*> openlist{}, closelist{};
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
                  navmesh::node* begin,
                  navmesh::node* target,
                  const callback_c& callback,
                  int it_count = 512) {
    auto st = new node;
    st->f = 0;
    st->g = 0;
    st->h = 0;
    st->parent = NULL;
    st->position = begin->position;
    st->navNode = begin;
    ctx.nodes.push_back(st);
    ctx.processing = st;
    ctx.result = NULL;
    ctx.failed = false;
    ctx.target = target->position;

    int num = 0;
    while (ctx.processing) {
        step(ctx, callback);
        ++num;
        if (num > it_count)
            break;
    }
}

inline void buildRoad(context& ctx, std::function<void(navmesh::node*)> callback) {
    if (ctx.result) {
        auto p = ctx.result;
        while (p) {
            callback(p->navNode);
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
    ctx.closelist[ctx.processing->navNode] = ctx.processing;

    std::vector<node*> ns;
    //for (auto it : ctx.processing->navNode->ways) {
    //navmesh::node* targetNavNode;
    //if (it->minWidth > ctx.minPathWidth) {  //可以通过
    callback(ctx.processing->navNode, [&](navmesh::node* targetNavNode, double length) {
        //if (it->p1 == ctx.processing->navNode) {
        //    targetNavNode = it->p2;
        //} else {
        //    targetNavNode = it->p1;
        //}
        if (ctx.openlist.find(targetNavNode) != ctx.openlist.end())
            return;
        if (ctx.closelist.find(targetNavNode) != ctx.closelist.end())
            return;
        auto p = new node;
        ctx.nodes.push_back(p);
        p->parent = ctx.processing;
        p->position = targetNavNode->position;
        p->navNode = targetNavNode;

        //启发
        p->g = ctx.processing->g + length;
        p->h = heuristic(targetNavNode->position, ctx.target);
        p->f = p->g + p->h;

        ctx.openlist[targetNavNode] = p;
        ns.push_back(p);
    });
    //}
    //}
    if (ns.empty()) {
        if (ctx.openlist.empty()) {
            ctx.failed = true;  //搜索失败
            ctx.processing = NULL;
            return;
        } else {
            double minf = -1;
            node* min;
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
            ctx.processing = min;
            ctx.openlist.erase(min->navNode);
        }
    } else {
        double minf = -1;
        node* min;
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
        ctx.processing = min;
        ctx.openlist.erase(min->navNode);
    }
}

}  // namespace astar_node

}  // namespace sdpf
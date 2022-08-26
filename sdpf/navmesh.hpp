#pragma once
#include <map>
#include <memory>
#include <set>
#include <vec2.hpp>
#include <vector>
#include "pointcloud.hpp"
#include "sdf.hpp"
//导航网络
namespace sdpf::navmesh {

struct node {               //节点
    ivec2 position;         //位置
    int32_t id;             //节点id
    std::set<way*> ways{};  //相连
};
struct way {                            //连线
    node *p1 = nullptr, *p2 = nullptr;  //两个端点(id较小的排前面)
    std::vector<ivec2> maxPath{};       //值最大的路线（sdf极值线）
    double minWidth;                    //最小路宽，小于说明物体无法通过
    bool reverse = false;               //路线翻转
    double length = 0;                  //路线长度
};
struct navmesh {
    field<vec2> vsdfMap;                                               //向量距离场
    sdf::sdf sdfMap;                                                   //sdf
    std::vector<std::unique_ptr<node>> nodes{};                        //节点
    std::map<std::pair<int32_t, int32_t>, std::unique_ptr<way>> ways;  //相连(id较小的排前面)
    field<int32_t> idMap;                                              //地图上的节点id及道路信息
    field<int32_t> searchMap;                                          //搜索标识
    field<std::tuple<int32_t, int32_t, double, int>> pathDisMap;       //路线离端点距离
    field<std::tuple<int32_t, ivec2, double>> pathMap;                 //路线流场
    field<std::tuple<ivec2, double>> pathNavMap;                       //导航至路上的流场
    int width, height;
    double minItemSize = 2;  //最小物体的半径
    inline navmesh(int width, int height)
        : vsdfMap(width, height),
          sdfMap(width, height),
          ways(width, height),
          idMap(width, height),
          searchMap(width, height),
          pathDisMap(width, height),
          pathMap(width, height),
          pathNavMap(width, height) {
        this->width = width;
        this->height = height;
    }
};

inline void buildSdfMap(navmesh& mesh, kdtree::tree& tree) {
#pragma omp parallel for
    for (int i = 0; i < mesh.width; ++i) {
        for (int j = 0; j < mesh.height; ++j) {
            auto v = pointcloud::getPointDis(tree, vec2(i, j));
            mesh.vsdfMap.at(i, j) = v;
            mesh.sdfMap.at(i, j) = v.norm();
        }
    }
}

inline double getCosPointDirDeg(navmesh& mesh, const ivec2& p1, const ivec2& p2) {
    auto d1 = mesh.vsdfMap.at(p1.x, p1.y);
    auto d2 = mesh.vsdfMap.at(p2.x, p2.y);
    auto l1 = d1.norm();
    auto l2 = d2.norm();
    if (l1 <= 0 || l2 <= 0) {
        return 99999999;
    }
    d1 /= l1;
    d2 /= l2;
    return d1.dot(d2);
}

inline bool isRidge(navmesh& mesh, const ivec2& p, double isRidgeMinCosD = 0.866025403784438) {
    if (p.x <= 0 || p.y <= 0 || p.x >= mesh.width - 1 || p.y >= mesh.height - 1) {
        //地图边缘不能检测
        return false;
    }

    if (mesh.sdfMap.at(p.x, p.y) > mesh.minItemSize) {
        return false;
    }

    return (getCosPointDirDeg(mesh, p + ivec2(1, 0), p + ivec2(-1, 0)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(1, 1), p + ivec2(-1, -1)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(0, 1), p + ivec2(0, -1)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(1, -1), p + ivec2(-1, 1)) < isRidgeMinCosD);
}

inline void buildIdMap(navmesh& mesh, std::vector<ivec2>& tops) {
    omp_lock_t locker;
    omp_init_lock(&locker);
#pragma omp parallel for
    for (int i = 0; i < mesh.sdfMap.width; ++i) {
        for (int j = 0; j < mesh.sdfMap.height; ++j) {
            if (mesh.sdfMap.isTop(ivec2(i, j))) {
                mesh.idMap.at(i, j) = -1;
                omp_set_lock(&locker);
                tops.push_back(ivec2(i, j));
                omp_unset_lock(&locker);
            } else if (isRidge(mesh, ivec2(i, j))) {
                mesh.idMap.at(i, j) = -1;
            } else {
                mesh.idMap.at(i, j) = 0;
            }
        }
    }
    omp_destroy_lock(&locker);
}

inline bool isNode(navmesh& mesh, const ivec2& pos, int area = 2) {
    bool beginValue = false;
    bool endValue = false;
    bool lastValue = false;
    bool first = true;
    int count = 0;
    for (int i = -area + 1; i <= area; ++i) {
        int x = pos.x + i;
        int y = pos.y + area;
        bool val = (mesh.idMap.at(x, y) == -1);
        if (first) {
            first = false;
            beginValue = val;
        } else {
            if ((!lastValue) && val) {
                ++count;
            }
        }
        lastValue = val;
    }
    for (int i = area - 1; i >= -area; ++i) {
        int x = pos.x + area;
        int y = pos.y + i;
        bool val = (mesh.idMap.at(x, y) == -1);
        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    for (int i = area - 1; i >= -area; ++i) {
        int x = pos.x + i;
        int y = pos.y - area;
        bool val = (mesh.idMap.at(x, y) == -1);
        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    for (int i = -area + 1; i <= area; ++i) {
        int x = pos.x - area;
        int y = pos.y + i;
        bool val = (mesh.idMap.at(x, y) == -1);
        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    if ((!lastValue) && beginValue) {
        ++count;
    }
    return count >= 3;
}

inline void buildNodeBlock(navmesh& mesh, const std::vector<ivec2>& tops, int topSize = 2) {
    int index = 1;
    for (auto& p : tops) {
        if (isNode(mesh, p)) {
            std::unique_ptr<node> n(new node);
            n->id = index;
            n->position.init(p.x, p.y);

            for (int i = -topSize; i <= topSize; ++i) {
                for (int j = -topSize; j <= topSize; ++j) {
                    int x = i + n->position.x;
                    int y = j + n->position.y;
                    if (x >= 0 && y >= 0 && x < mesh.idMap.width && y < mesh.idMap.width) {
                        mesh.idMap.at(x, y) = index;
                        mesh.pathMap.at(x, y) = std::tuple<int, ivec2, double>(index, ivec2(0, 0), 0);
                    }
                }
            }

            mesh.nodes.push_back(std::move(n));
            ++index;
        }
    }
}

inline void buildConnect(navmesh& mesh, int pathSize = 2) {  //识别连接关系
    std::queue<ivec2> que;
    for (auto& root_node : mesh.nodes) {
        //广搜
        const int index = root_node->id;
        const auto& root_pos = root_node->position;
        que.push(root_pos);
        while (!que.empty()) {
            const auto& pos = que.front();                       //当前的位置
            const auto& val = mesh.idMap.at(pos.x, pos.y);       //格子性质标识
            auto& searchFlag = mesh.searchMap.at(pos.x, pos.y);  //搜索标识
            auto& pathFlag = mesh.pathMap.at(pos.x, pos.y);      //路线方向标识
            if (searchFlag != 1) {                               //未遍历，开始处理
                searchFlag = 1;                                  //标记已经遍历过
                bool needSearchNear = false;
                if (val > 0) {  //大于0表示是节点
                    if (val == index) {
                        //在自己节点内部
                        needSearchNear = true;
                    } else {
                        //发现其他节点
                        //标记节点相连
                        //不再继续遍历
                        int32_t a = val;
                        int32_t b = index;
                        if (a > b) {
                            std::swap(a, b);
                        }
                        auto pair = std::make_pair(a, b);
                        if (mesh.ways.find(pair) == mesh.ways.end()) {
                            //扫描附近的节点，回溯路线
                            ivec2 minConn_pos;
                            double minConn_w = -1;
                            for (int i = -pathSize; i <= pathSize; ++i) {
                                for (int j = -pathSize; j <= pathSize; ++j) {
                                    if (i != 0 && j != 0) {
                                        int x = i + pos.x;
                                        int y = j + pos.y;
                                        auto& pathMapVal = mesh.pathMap.at(x, y);
                                        if (std::get<0>(pathMapVal) == index) {
                                            auto w = std::get<2>(pathMapVal) + ivec2(x - pos.x, y - pos.y).norm();
                                            if (minConn_w < 0 || w < minConn_w) {
                                                minConn_w = w;
                                                minConn_pos.init(x, y);
                                            }
                                        }
                                    }
                                }
                            }
                            if (minConn_w > 0) {
                                //回溯
                                std::unique_ptr<way> l(new way);
                                l->p1 = mesh.nodes.at(val - 1).get();
                                l->p2 = mesh.nodes.at(index - 1).get();
                                l->maxPath.push_back(pos);
                                int maxPath_index = 1;
                                l->minWidth = std::max(mesh.sdfMap.width, mesh.sdfMap.height);
                                l->length = minConn_w;

                                while (minConn_pos.x != 0 || minConn_pos.y != 0) {
                                    l->maxPath.push_back(minConn_pos);
                                    auto& pathMapValue = mesh.pathMap.at(minConn_pos.x, minConn_pos.y);
                                    //计算到两端距离
                                    double delta_p1 = std::get<2>(pathMapValue);
                                    double delta_p2 = minConn_w - delta_p1;
                                    if (delta_p1 > delta_p2) {
                                        mesh.pathDisMap.at(minConn_pos.x, minConn_pos.y) =
                                            std::tuple<int32_t, int32_t, double, int>(index, val, delta_p2, maxPath_index);
                                    } else {
                                        mesh.pathDisMap.at(minConn_pos.x, minConn_pos.y) =
                                            std::tuple<int32_t, int32_t, double, int>(val, index, delta_p1, maxPath_index);
                                    }
                                    ++maxPath_index;
                                    //设置路线宽度
                                    double pathWidth = mesh.sdfMap.at(minConn_pos.x, minConn_pos.y);
                                    if (pathWidth < l->minWidth) {
                                        l->minWidth = pathWidth;
                                    }
                                    minConn_pos = std::get<1>(pathMapValue);
                                }
                                l->maxPath.push_back(l->p2->position);

                                l->p1->ways.insert(l.get());
                                l->p2->ways.insert(l.get());

                                if (l->p1->id > l->p2->id) {
                                    std::swap(l->p1, l->p2);
                                    //std::reverse(l->maxPath.begin(), l->maxPath.end());
                                    l->reverse = true;
                                } else {
                                    l->reverse = false;
                                }
                                mesh.ways[pair] = std::move(l);
                            }
                        }
                    }
                } else if (val == -1) {
                    //为-1表示是路线
                    //searchFlag!= 1表示未搜索
                    needSearchNear = true;
                    if (std::get<0>(pathFlag) == -1) {
                        //扫描附近的点，有其他路线或者自身节点则与它连接
                        ivec2 minConn_pos;
                        double minConn_w = -1;
                        for (int i = -pathSize; i <= pathSize; ++i) {
                            for (int j = -pathSize; j <= pathSize; ++j) {
                                if (i != 0 && j != 0) {
                                    int x = i + pos.x;
                                    int y = j + pos.y;
                                    auto& pathMapVal = mesh.pathMap.at(x, y);
                                    if (mesh.searchMap.at(x, y) == 1 &&   //必须已经处理过
                                        std::get<0>(pathMapVal) == index  //和自身相连
                                    ) {
                                        //建立连接
                                        ivec2 dir(x - pos.x, y - pos.y);                                           //向量
                                        double w = sqrt(dir.x * dir.x + dir.y * dir.y) + std::get<2>(pathMapVal);  //计算代价
                                        if (minConn_w < 0 || w < minConn_w) {
                                            minConn_w = w;
                                            minConn_pos.init(x, y);
                                        }
                                    }
                                }
                            }
                        }
                        pathFlag = std::tuple<int, ivec2, double>(index, minConn_pos, minConn_w);
                    }
                }
                if (needSearchNear) {  //扫描周围的格子，作为继续遍历的对象
                    for (int i = -pathSize; i <= pathSize; ++i) {
                        for (int j = -pathSize; j <= pathSize; ++j) {
                            if (i != 0 && j != 0) {
                                int x = i + pos.x;
                                int y = j + pos.y;
                                if (mesh.searchMap.at(x, y) != 1) {  //未搜索的格子
                                    que.push(ivec2(x, y));
                                }
                            }
                        }
                    }
                }
            }
            que.pop();
        }
    }
}

//构建上路流场
inline void buildNavFlowField(navmesh& mesh, double minPathWith) {
    mesh.pathNavMap.setAll(std::tuple<ivec2, double>(ivec2(0, 0), -1));
    std::vector<ivec2> startPoints;
    for (auto& it : mesh.nodes) {
        startPoints.push_back(it->position);
    }
    for (auto& it : mesh.ways) {
        for (auto& p : it.second->maxPath) {
            startPoints.push_back(p);
        }
    }
    std::queue<ivec2> que;
    for (auto& p : startPoints) {
        mesh.searchMap.at(p.x, p.y) = 2;
        mesh.pathNavMap.at(p.x, p.y) = std::tuple<ivec2, double>(ivec2(0, 0), 0);
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i != 0 && j != 0) {
                    int x = i + p.x;
                    int y = j + p.y;
                    que.push(ivec2(x, y));
                }
            }
        }
    }
    //广搜
    while (!que.empty()) {
        const auto& pos = que.front();
        auto& searchFlag = mesh.searchMap.at(pos.x, pos.y);    //搜索标识
        if (searchFlag != 2) {                                 //未遍历，开始处理
            if (mesh.sdfMap.at(pos.x, pos.y) > minPathWith) {  //宽度足够大，目标可达
                searchFlag = 2;                                //标记已经遍历过

                ivec2 minConn_pos;
                double minConn_w = -1;
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        if (i != 0 && j != 0) {
                            int x = i + pos.x;
                            int y = j + pos.y;
                            que.push(ivec2(x, y));
                            if (mesh.searchMap.at(pos.x, pos.y) == 2) {
                                auto& pathNavMapVal = mesh.pathNavMap.at(x, y);
                                auto w = std::get<1>(pathNavMapVal) +
                                         ivec2(x - pos.x, y - pos.y).norm();
                                if (minConn_w < 0 || w < minConn_w) {
                                    minConn_w = w;
                                    minConn_pos.init(x, y);
                                }
                            }
                        }
                    }
                }
                if (minConn_w > 0) {
                    mesh.pathNavMap.at(pos.x, pos.y) =
                        std::tuple<ivec2, double>(minConn_pos, minConn_w);
                }
            }
        }
        que.pop();
    }
}

//导航至路上
inline bool toRoad(navmesh& mesh,
                   const ivec2& pos,             //起点（输入）
                   std::vector<ivec2>& pathPos,  //途经
                   ivec2& target                 //终点
) {
    pathPos.clear();
    ivec2 conn_pos = pos;
    while (conn_pos.x != 0 || conn_pos.y != 0) {
        pathPos.push_back(conn_pos);
        auto& pathMapValue = mesh.pathMap.at(conn_pos.x, conn_pos.y);
        conn_pos = std::get<1>(pathMapValue);
    }
    if (pathPos.empty()) {
        return false;
    }
    auto endPos = pathPos.rbegin();
    if (mesh.idMap.at(endPos->x, endPos->y) != 0) {
        target = *endPos;
        return true;
    } else {
        return false;
    }
}

}  // namespace sdpf::navmesh

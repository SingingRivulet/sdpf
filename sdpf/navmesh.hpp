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

struct way;
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
    std::vector<std::unique_ptr<node>> nodes{};                        //节点
    std::map<std::pair<int32_t, int32_t>, std::unique_ptr<way>> ways;  //相连(id较小的排前面)
    field<std::tuple<vec2, vec2>> vsdfMap;                             //向量距离场
    sdf::sdf sdfMap;                                                   //sdf
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
            std::tuple<vec2, vec2> sdfp;
            pointcloud::getPointDis(tree, vec2(i, j), std::get<0>(sdfp), std::get<1>(sdfp));
            //printf("(%d,%d)=>%f\n", i, j, v.norm());
            mesh.vsdfMap.at(i, j) = sdfp;
            mesh.sdfMap.at(i, j) = std::get<0>(sdfp).norm();
        }
    }
}

inline double getCosPointDirDeg(navmesh& mesh, const ivec2& p1, const ivec2& p2) {
    auto pt1 = mesh.vsdfMap.at(p1.x, p1.y);
    auto pt2 = mesh.vsdfMap.at(p2.x, p2.y);

    auto pos1 = std::get<1>(pt1);
    auto pos2 = std::get<1>(pt2);
    auto delta = pos1 - pos2;
    if (fabs(delta.x) + fabs(delta.y) < 0.0001) {  //是同一个点
        return INFINITY;
    }

    auto d1 = std::get<0>(pt1);
    auto d2 = std::get<0>(pt2);

    auto l1 = d1.norm();
    auto l2 = d2.norm();
    if (l1 <= 0 || l2 <= 0) {
        return INFINITY;
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

    if (mesh.sdfMap.at(p.x, p.y) < mesh.minItemSize) {
        return false;
    }

    //printf("deg=%lf\n", getCosPointDirDeg(mesh, p + ivec2(1, 0), p + ivec2(-1, 0)));

    return (getCosPointDirDeg(mesh, p + ivec2(1, 0), p + ivec2(0, 0)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(1, 1), p + ivec2(0, 0)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(0, 1), p + ivec2(0, 0)) < isRidgeMinCosD ||
            getCosPointDirDeg(mesh, p + ivec2(1, 0), p + ivec2(1, 1)) < isRidgeMinCosD);
}

inline void buildIdMap(navmesh& mesh, std::vector<ivec2>& startPoints, double minPathWith) {
    omp_lock_t locker;
    omp_init_lock(&locker);
#pragma omp parallel for
    for (int i = 0; i < mesh.sdfMap.width; ++i) {
        for (int j = 0; j < mesh.sdfMap.height; ++j) {
            if (mesh.sdfMap.at(i, j) > minPathWith && isRidge(mesh, ivec2(i, j))) {
                omp_set_lock(&locker);
                startPoints.push_back(ivec2(i, j));
                omp_unset_lock(&locker);
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
    //printf("isNode(%d,%d)\n", pos.x, pos.y);
    for (int i = -area + 1; i <= area; ++i) {
        int x = pos.x + i;
        int y = pos.y + area;
        double k = ((double)i) / area;

        bool val = true;
        for (int vi = 0; vi <= area; ++vi) {
            int vx = pos.x + k * vi;
            int vy = pos.y + vi;

            if (vx >= 0 && vy >= 0 &&
                vx < mesh.width && vy < mesh.height) {
                if (mesh.idMap.at(vx, vy) != -2) {
                    val = false;
                    break;
                }
            } else {
                val = false;
                break;
            }
        }

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
    for (int i = area - 1; i >= -area; --i) {
        int x = pos.x + area;
        int y = pos.y + i;
        double k = ((double)i) / area;

        bool val = true;
        for (int vi = 0; vi <= area; ++vi) {
            int vx = pos.x + vi;
            int vy = pos.y + k * vi;

            if (vx >= 0 && vy >= 0 &&
                vx < mesh.width && vy < mesh.height) {
                if (mesh.idMap.at(vx, vy) != -2) {
                    val = false;
                    break;
                }
            } else {
                val = false;
                break;
            }
        }
        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    for (int i = area - 1; i >= -area; --i) {
        int x = pos.x + i;
        int y = pos.y - area;
        double k = ((double)i) / area;

        bool val = true;
        for (int vi = 0; vi <= area; ++vi) {
            int vx = pos.x + k * vi;
            int vy = pos.y - vi;

            if (vx >= 0 && vy >= 0 &&
                vx < mesh.width && vy < mesh.height) {
                if (mesh.idMap.at(vx, vy) != -2) {
                    val = false;
                    break;
                }
            } else {
                val = false;
                break;
            }
        }

        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    for (int i = -area + 1; i <= area; ++i) {
        int x = pos.x - area;
        int y = pos.y + i;
        double k = ((double)i) / area;

        bool val = true;
        for (int vi = 0; vi <= area; ++vi) {
            int vx = pos.x - vi;
            int vy = pos.y + k * vi;

            if (vx >= 0 && vy >= 0 &&
                vx < mesh.width && vy < mesh.height) {
                if (mesh.idMap.at(vx, vy) != -2) {
                    val = false;
                    break;
                }
            } else {
                val = false;
                break;
            }
        }

        if ((!lastValue) && val) {
            ++count;
        }
        lastValue = val;
    }
    if ((!lastValue) && beginValue) {
        ++count;
    }
    return count >= 3 ;
}

inline void getIsland(navmesh& mesh,
                      std::vector<ivec2>& points,
                      std::vector<std::vector<ivec2>>& islands) {
    islands.clear();
    std::set<ivec2> points_nosearch;
    for (auto& p : points) {
        points_nosearch.insert(p);
    }
    while (!points_nosearch.empty()) {
        auto sit = points_nosearch.begin();
        ivec2 begin = *sit;
        points_nosearch.erase(sit);

        std::vector<ivec2> points_buffer;

        std::queue<ivec2> que;
        que.push(begin);
        while (!que.empty()) {
            const auto& pos = que.front();
            points_buffer.push_back(pos);

            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (!(i == 0 && j == 0)) {
                        int x = i + pos.x;
                        int y = j + pos.y;
                        if (x >= 0 && y >= 0 &&
                            x < mesh.width && y < mesh.height) {
                            auto it = points_nosearch.find(ivec2(x, y));
                            if (it != points_nosearch.end()) {
                                points_nosearch.erase(it);
                                que.push(ivec2(x, y));
                            }
                        }
                    }
                }
            }

            que.pop();
        }

        islands.push_back(std::move(points_buffer));
    }
}

inline void buildNodeBlock(navmesh& mesh, const std::vector<ivec2>& tops, int topSize = 2) {
    int index = 1;
    std::vector<ivec2> points;
    std::vector<std::vector<ivec2>> blocks;
    for (auto& p : tops) {
        if (isNode(mesh, p)) {
            for (int i = -topSize; i <= topSize; ++i) {
                for (int j = -topSize; j <= topSize; ++j) {
                    int x = i + p.x;
                    int y = j + p.y;
                    if (x >= 0 && y >= 0 &&
                        x < mesh.idMap.width && y < mesh.idMap.width &&
                        mesh.idMap.at(x, y) == -2) {
                        //mesh.idMap.at(x, y) = -3;
                        points.push_back(ivec2(x, y));
                    }
                }
            }
        }
    }
    getIsland(mesh, points, blocks);
    for (auto& block : blocks) {
        //求中心点
        ivec2 sum(0, 0);
        int count = 0;
        for (auto& point : block) {
            sum += point;
            ++count;
        }
        if (count > 0) {
            double cx = (double)sum.x / count;
            double cy = (double)sum.y / count;
            //找最靠近中心的点
            ivec2 center;
            double center_len = INFINITY;
            for (auto& point : block) {
                if (point.x == (int)cx && point.y == (int)cy) {
                    //确认
                    center = point;
                    goto createNode;
                }
                auto dx = point.x - cx;
                auto dy = point.y - cy;
                auto len = dx * dx + dy * dy;
                if (len < center_len) {
                    center = point;
                    center_len = len;
                }
            }
        createNode:
            std::unique_ptr<node> n(new node);
            n->id = index;
            n->position.init(center.x, center.y);

            for (auto& point : block) {
                auto x = point.x;
                auto y = point.y;
                if (x >= 0 && y >= 0 && x < mesh.idMap.width && y < mesh.idMap.width) {
                    mesh.idMap.at(x, y) = index;
                    mesh.pathMap.at(x, y) = std::tuple<int, ivec2, double>(index, ivec2(0, 0), 0);
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
inline void buildNavFlowField(navmesh& mesh, const std::vector<ivec2>& startPoints, double minPathWith) {
    mesh.pathNavMap.setAll(std::tuple<ivec2, double>(ivec2(-1, -1), -1));
    mesh.searchMap.setAll(0);
    std::queue<ivec2> que;
    for (auto& p : startPoints) {
        mesh.searchMap.at(p.x, p.y) = 2;
        mesh.pathNavMap.at(p.x, p.y) = std::tuple<ivec2, double>(ivec2(-1, -1), 0);
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
        auto& searchFlag = mesh.searchMap.at(pos.x, pos.y);  //搜索标识
        if (searchFlag != 2) {                               //未遍历，开始处理
            if (pos.x >= 0 && pos.y >= 0 &&
                pos.x < mesh.width && pos.y < mesh.height &&
                mesh.sdfMap.at(pos.x, pos.y) > minPathWith) {  //宽度足够大，目标可达
                searchFlag = 2;                                //标记已经遍历过

                ivec2 minConn_pos;
                double minConn_w = -1;
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        if (!(i == 0 && j == 0)) {
                            int x = i + pos.x;
                            int y = j + pos.y;
                            if (x >= 0 && y >= 0 &&
                                x < mesh.width && y < mesh.height) {
                                que.push(ivec2(x, y));
                                if (mesh.searchMap.at(x, y) == 2) {
                                    auto& pathNavMapVal = mesh.pathNavMap.at(x, y);
                                    auto w = std::get<1>(pathNavMapVal) +
                                             vec2(x - pos.x, y - pos.y).norm();

                                    //printf("%d %d w=%lf\n", pos.x, pos.y, w);
                                    if (minConn_w < 0 || w < minConn_w) {
                                        minConn_w = w;
                                        minConn_pos.init(x, y);
                                    }
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

//删除孤立的路线
inline void removeWaste(navmesh& mesh, std::vector<ivec2>& points) {
    auto len = points.size();
    if (len > 1) {
        std::vector<std::vector<ivec2>> paths;
        getIsland(mesh, points, paths);

        int maxLen = -1;
        int maxLen_id = -1;
        int paths_len = paths.size();
        for (int i = 0; i < paths_len; ++i) {
            int nlen = paths[i].size();
            if (maxLen < 0 || nlen > maxLen) {
                maxLen = nlen;
                maxLen_id = i;
            }
            //printf("removeWaste %d %d\n", i, nlen);
        }
        if (maxLen_id >= 0) {
            points = paths[maxLen_id];
        }
    }
    for (auto& it : points) {
        mesh.idMap.at(it.x, it.y) = -2;
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

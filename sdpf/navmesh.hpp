#pragma once
#include <list>
#include <map>
#include <memory>
#include <set>
#include <vec2.hpp>
#include <vector>
#include "astar_array.hpp"
#include "pointcloud.hpp"
#include "sdf.hpp"
//导航网络
namespace sdpf::navmesh {

struct way;
struct node {                   //节点
    ivec2 position;             //位置
    int32_t id;                 //节点id
    double flowValue;           //流场值
    int32_t flowFieldFlag = 0;  //流场寻路标识
    way* flowDir = nullptr;     //流场方向
    std::set<way*> ways{};      //相连
    std::set<way*> tmpways{};   //临时路线
};
struct way {                            //连线
    node *p1 = nullptr, *p2 = nullptr;  //两个端点(id较小的排前面)
    std::vector<ivec2> maxPath{};       //值最大的路线（sdf极值线）
    double minWidth;                    //最小路宽，小于说明物体无法通过
    double length = 0;                  //路线长度
};

struct pathNav {
    ivec2 target{};
    double cost = 0;
    pathNav() = default;
    pathNav(const pathNav&) = default;
    inline pathNav(const ivec2& t, double c) {
        target = t;
        cost = c;
    }
};

struct vectorDis {
    vec2 dir{};
    vec2 pos{};
};

struct pathDis {
    int32_t firstNode = 0;
    int32_t secondNode = 0;
    int32_t pointIndex = 0;
    double distance = 0;
    pathDis() = default;
    pathDis(const pathDis&) = default;
    inline pathDis(int32_t f, int32_t s, double d, int32_t i) {
        firstNode = f;
        secondNode = s;
        distance = d;
        pointIndex = i;
    }
};

struct navmesh {
    std::vector<std::unique_ptr<node>> nodes{};                        //节点
    std::map<std::pair<int32_t, int32_t>, std::unique_ptr<way>> ways;  //相连(id较小的排前面)
    sdf::sdf sdfMap;                                                   //sdf
    field<vectorDis> vsdfMap;                                          //向量距离场
    field<int32_t> idMap;                                              //地图上的节点id及道路信息
    field<int32_t> searchMap;                                          //搜索标识
    field<pathDis> pathDisMap;                                         //路线离端点距离
    field<pathNav> pathNavMap;                                         //导航至路上的流场
    int32_t searchMap_id = 1;
    int width, height;
    double minItemSize = 2;  //最小物体的半径
    inline navmesh(int width, int height)
        : sdfMap(width, height),
          vsdfMap(width, height),
          idMap(width, height),
          searchMap(width, height),
          pathDisMap(width, height),
          pathNavMap(width, height) {
        this->width = width;
        this->height = height;
        searchMap.setAll(0);
    }
};

inline void buildMeshFlowField(navmesh& mesh, node* target) {
    ++mesh.searchMap_id;
    target->flowValue = 0;
    target->flowDir = nullptr;
    std::queue<node*> que{};

    que.push(target);
    std::set<int> connect;
    while (!que.empty()) {
        const auto& node_search = que.front();
        if (node_search->flowFieldFlag != mesh.searchMap_id) {
            node_search->flowFieldFlag = mesh.searchMap_id;
            //printf("node_search->id=%d\n", node_search->id);

            node* targetNavNode;
            double minLen = INFINITY;
            node_search->flowDir = nullptr;
            for (auto& it : node_search->ways) {
                if (it->p1 == node_search) {
                    targetNavNode = it->p2;
                } else {
                    targetNavNode = it->p1;
                }
                if (targetNavNode->flowFieldFlag == mesh.searchMap_id) {
                    if (targetNavNode->flowValue < minLen) {
                        minLen = targetNavNode->flowValue;
                        node_search->flowDir = it;
                    }
                } else {
                    que.push(targetNavNode);
                }
            }
            for (auto& it : node_search->tmpways) {
                if (it->p1 == node_search) {
                    targetNavNode = it->p2;
                } else {
                    targetNavNode = it->p1;
                }
                if (targetNavNode->flowFieldFlag == mesh.searchMap_id) {
                    if (targetNavNode->flowValue < minLen) {
                        minLen = targetNavNode->flowValue;
                        node_search->flowDir = it;
                    }
                } else {
                    que.push(targetNavNode);
                }
            }
            if (node_search == target) {
                node_search->flowValue = 0;
            } else {
                if (node_search->flowDir) {
                    node_search->flowValue = node_search->flowDir->length + minLen;
                } else {
                    node_search->flowValue = INFINITY;
                }
            }
        }

        que.pop();
    }
}

inline vectorDis vsdf_box(const vec2& pos, int width, int height) {
    vectorDis res;
    double lens[] = {pos.y, height - pos.y, pos.x, width - pos.x};
    vec2 poss[] = {vec2(pos.x, 0), vec2(pos.x, height),
                   vec2(0, pos.y), vec2(width, pos.y)};
    double minLen = INFINITY;
    for (int i = 0; i < 4; ++i) {
        if (lens[i] < minLen) {
            minLen = lens[i];
            res.pos = poss[i];
            res.dir = res.pos - pos;
        }
    }
    return res;
}

inline void buildSdfMap(navmesh& mesh, KDTree& tree) {
#pragma omp parallel for
    for (int i = 0; i < mesh.width; ++i) {
        for (int j = 0; j < mesh.height; ++j) {
            //点的位置
            vectorDis sdfp;
            pointcloud::getPointDis(tree, vec2(i, j), sdfp.dir, sdfp.pos);
            //边缘
            auto boxsdf = vsdf_box(vec2(i, j), mesh.width, mesh.height);
            //选距离最短的
            if (sdfp.dir.norm() < boxsdf.dir.norm()) {
                mesh.vsdfMap.at(i, j) = sdfp;
                mesh.sdfMap.at(i, j) = sdfp.dir.norm();
            } else {
                mesh.vsdfMap.at(i, j) = boxsdf;
                mesh.sdfMap.at(i, j) = boxsdf.dir.norm();
            }
        }
    }
}

inline double getCosPointDirDeg(navmesh& mesh, const ivec2& p1, const ivec2& p2) {
    auto pt1 = mesh.vsdfMap.at(p1.x, p1.y);
    auto pt2 = mesh.vsdfMap.at(p2.x, p2.y);

    auto pos1 = pt1.pos;
    auto pos2 = pt2.pos;
    auto delta = pos1 - pos2;
    if (fabs(delta.x) + fabs(delta.y) < 0.0001) {  //是同一个点
        return INFINITY;
    }

    auto d1 = pt1.dir;
    auto d2 = pt2.dir;

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
    bool lastValue = false;
    bool first = true;
    int count = 0;
    //printf("isNode(%d,%d)\n", pos.x, pos.y);
    for (int i = -area + 1; i <= area; ++i) {
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
    return count >= 3;
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

inline void buildPath(navmesh& mesh,
                      int begin_id,
                      const ivec2& begin,
                      int target_id,
                      const ivec2& target,
                      int it_count = 1024) {
    astar_array::context atx;
    astar_array::start(
        atx, begin, target, [&](const ivec2& pos, auto callback) {
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (!(i == 0 && j == 0)) {
                        int x = i + pos.x;
                        int y = j + pos.y;
                        if (x >= 0 && y >= 0 &&
                            x < mesh.width && y < mesh.height) {
                            auto id = mesh.idMap.at(x, y);
                            if (id == -2 || id == begin_id || id == target_id) {
                                callback(ivec2(x, y));
                            }
                        }
                    }
                }
            }
        },
        it_count);
    std::vector<std::tuple<ivec2, double, double>> path{};  //位置，宽度，距离
    astar_array::buildRoad(atx, [&](const ivec2& pos) {
        path.push_back(std::make_tuple(pos, 0., 0.));
    });

    if (path.size() > 1) {
        //printf("path:");
        path.pop_back();
        std::reverse(path.begin(), path.end());
        ivec2 last = begin;
        double lenSum = 0.;
        double minWidth = INFINITY;
        for (auto& point : path) {
            auto& pos = std::get<0>(point);
            auto& width = std::get<1>(point);
            auto& len = std::get<2>(point);
            auto deltaLen = pos.length(last);
            lenSum += deltaLen;
            len = lenSum;
            width = mesh.sdfMap.at(pos.x, pos.y);
            if (width < minWidth) {
                minWidth = width;
            }
            last = pos;
            //printf("(%d,%d,%lf,%lf) ", pos.x, pos.y, len, width);
        }
        lenSum += target.length(last);
        //printf("\n");

        //计算到两端距离
        int maxPath_index = 0;
        for (auto& point : path) {
            auto& pos = std::get<0>(point);
            double delta_p1 = std::get<2>(point);
            double delta_p2 = lenSum - delta_p1;
            if (delta_p1 > delta_p2) {
                mesh.pathDisMap.at(pos.x, pos.y) =
                    pathDis(begin_id, target_id, delta_p2, maxPath_index);
            } else {
                mesh.pathDisMap.at(pos.x, pos.y) =
                    pathDis(target_id, begin_id, delta_p1, maxPath_index);
            }
            ++maxPath_index;
        }

        //创建路线
        auto way_key = std::make_pair(begin_id, target_id);
        if (mesh.ways.find(way_key) == mesh.ways.end()) {
            std::unique_ptr<way> l(new way);
            l->p1 = mesh.nodes.at(begin_id - 1).get();
            l->p2 = mesh.nodes.at(target_id - 1).get();

            l->length = lenSum;

            l->minWidth = minWidth;
            for (auto& point : path) {
                auto& pos = std::get<0>(point);
                l->maxPath.push_back(pos);
            }

            mesh.nodes.at(begin_id - 1)->ways.insert(l.get());
            mesh.nodes.at(target_id - 1)->ways.insert(l.get());

            mesh.ways[way_key] = std::move(l);
        }
    }
}

inline void buildConnect(navmesh& mesh, std::set<ivec2>& points_nosearch) {
    while (!points_nosearch.empty()) {
        auto sit = points_nosearch.begin();
        ivec2 begin = *sit;
        points_nosearch.erase(sit);

        std::vector<ivec2> points_buffer;

        std::queue<ivec2> que{};
        que.push(begin);
        std::set<int> connect;
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
                            int id = mesh.idMap.at(x, y);
                            if (id > 0) {
                                connect.insert(id);
                            }
                        }
                    }
                }
            }

            que.pop();
        }

        if (connect.size() == 2) {
            std::vector<ivec2> con_pos;
            std::vector<int> con_id;
            printf("connect:");
            for (auto it : connect) {
                auto pos = mesh.nodes.at(it - 1)->position;
                con_pos.push_back(pos);
                con_id.push_back(it);
                printf("%d(%d,%d) ", it, pos.x, pos.y);
            }
            printf("\n");
            buildPath(mesh, con_id.at(0), con_pos.at(0), con_id.at(1), con_pos.at(1));
        }
    }
}

inline void buildNodeBlock(navmesh& mesh, const std::vector<ivec2>& points_block, int topSize = 2) {
    int index = 1;
    std::vector<ivec2> points;
    std::set<ivec2> points_way;
    std::vector<std::vector<ivec2>> blocks;
    for (auto& p : points_block) {
        points_way.insert(p);
    }
    for (auto& p : points_block) {
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
                        points_way.erase(ivec2(x, y));
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
            mesh.pathDisMap.at(center.x, center.y) =
                pathDis(n->id, 0, 0, 0);
            for (auto& point : block) {
                auto x = point.x;
                auto y = point.y;
                if (x >= 0 && y >= 0 && x < mesh.idMap.width && y < mesh.idMap.width) {
                    mesh.idMap.at(x, y) = index;
                }
            }

            mesh.nodes.push_back(std::move(n));
            ++index;
        }
    }
    buildConnect(mesh, points_way);
}

//构建上路流场
inline void buildNavFlowField(navmesh& mesh, double minPathWith) {
    mesh.pathNavMap.setAll(pathNav(ivec2(-1, -1), -1));
    ++mesh.searchMap_id;
    std::queue<ivec2> que;
#define processPoint                                          \
    mesh.searchMap.at(p.x, p.y) = mesh.searchMap_id;          \
    mesh.pathNavMap.at(p.x, p.y) = pathNav(ivec2(-1, -1), 0); \
    for (int i = -1; i <= 1; ++i) {                           \
        for (int j = -1; j <= 1; ++j) {                       \
            if (i != 0 && j != 0) {                           \
                int x = i + p.x;                              \
                int y = j + p.y;                              \
                que.push(ivec2(x, y));                        \
            }                                                 \
        }                                                     \
    }
    for (auto& it : mesh.ways) {
        for (auto& p : it.second->maxPath) {
            processPoint;
        }
    }
    for (auto& it : mesh.nodes) {
        auto& p = it->position;
        processPoint;
    }
#undef processPoint
    //广搜
    while (!que.empty()) {
        const auto& pos = que.front();
        auto& searchFlag = mesh.searchMap.at(pos.x, pos.y);  //搜索标识
        if (searchFlag != mesh.searchMap_id) {               //未遍历，开始处理
            if (pos.x >= 0 && pos.y >= 0 &&
                pos.x < mesh.width && pos.y < mesh.height) {
                searchFlag = mesh.searchMap_id;  //标记已经遍历过

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
                                if (mesh.searchMap.at(x, y) == mesh.searchMap_id) {
                                    auto& pathNavMapVal = mesh.pathNavMap.at(x, y);
                                    auto wdelta = vec2(x - pos.x, y - pos.y).norm();
                                    auto pathWidth = mesh.sdfMap.at(pos.x, pos.y);
                                    if (pathWidth == 0) {
                                        pathWidth = 0.000001;
                                    }
                                    if (pathWidth <= minPathWith) {
                                        wdelta += 1000. / pathWidth;  //太窄，逃离
                                    }
                                    auto w = pathNavMapVal.cost + wdelta;

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
                        pathNav(minConn_pos, minConn_w);
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
    while (conn_pos.x >= 0 || conn_pos.y >= 0) {
        pathPos.push_back(conn_pos);
        auto& pathNavMapValue = mesh.pathNavMap.at(conn_pos.x, conn_pos.y);
        conn_pos = pathNavMapValue.target;
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

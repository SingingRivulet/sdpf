#pragma once
#include <algorithm>
#include <vector>
#include "activeNav.hpp"
#include "sdf.hpp"
namespace sdpf::pathopt {

//发射光线
inline bool rayMarch(sdf::sdf& map,       //导航地图
                     const vec2& begin,   //起点
                     const vec2& end,     //终点
                     double path_width,   //线宽
                     vec2& nearestPoint,  //距离边缘最近的点
                     bool skip = false) {
    auto beginDis = map[begin];
    if (beginDis < path_width) {
        return true;
    }
    auto dirLen = begin.length(end);
    if (dirLen <= 0.00001) {
        //原地放线
        if (beginDis > path_width) {
            nearestPoint = begin;
            return true;
        }
    }
    auto beginVecLen = std::min(beginDis, dirLen);  //有向距离场性质：这个范围内一定没有物体

    vec2 dir = end - begin;            //方向
    vec2 dir_norm = dir / dir.norm();  //方向的单位向量
    vec2 currentPos = begin;           //起点
    if (skip) {
        currentPos += dir_norm * beginVecLen / 2;
    }
    auto nearestPoint_mindis = map[currentPos];                  //初始化最短距离
    nearestPoint = currentPos;                                   //最短距离的位置
    while (currentPos.length2(end) > path_width * path_width) {  //大于路宽的平方说明没到目的地
        double area_r = map[currentPos];                         //空白的半径
        if (area_r < nearestPoint_mindis) {
            nearestPoint_mindis = area_r;
            nearestPoint = currentPos;
        }
        if (path_width > area_r) {
            //发生碰撞
            return true;
        }
        double lds = currentPos.length(end);  //当前位置到目的地的距离
        auto rayLen = std::min(area_r, lds);
        currentPos += dir_norm * rayLen;
    }
    return false;
}

//发射一系列光线扫描，获取最远的点
inline int getFarPoint(const std::vector<vec2>& path_in,  //原始路线
                       sdf::sdf& map,                     //导航地图
                       double path_width,                 //路线宽度
                       int nowPathId,                     //当前id
                       const vec2& nowPoint,              //当前位置
                       vec2& newPoint                     //更新位置
) {
    int path_len = path_in.size();
    const int search_left = nowPathId;
    const int search_right = path_len;
    int left = search_left;
    int right = search_right;
    int newPathId = -1;
    while (left < right - 1) {  //二分搜索
        int mid = round((left + right) / 2.);
        int id = mid;
        if (id > path_len - 1) {
            id = path_len - 1;
        }
        vec2 nearestPoint;
        //发射光线
        if (rayMarch(map,
                     nowPoint, path_in.at(id),
                     path_width, nearestPoint)) {
            //如果发生碰撞，nearestPoint为无效值，区间往前
            right = mid;
        } else {
            //未发生碰撞，区间往后，同时更新位置
            left = mid;
            newPoint = nearestPoint;
            newPathId = mid;
        }
    }
    if (newPathId == -1 || newPathId == nowPathId) {
        auto targetPoint = path_in.at(search_left + 1);
        newPoint = targetPoint;
        auto startPoint = path_in.at(search_left);
        auto dir = targetPoint - startPoint;
        double left = 1.0, right = 0.0;
        for (int i = 0; i < 8; ++i) {
            double mid = (left + right) / 2;
            vec2 nearestPoint;
            if (rayMarch(map,
                         startPoint + (dir * mid), targetPoint,
                         path_width * 2, nearestPoint)) {
                left = mid;
            } else {
                right = mid;
                newPoint = startPoint + (dir * mid);
            }
        }
        return search_left + 1;
    }
    return newPathId;
}

inline void optPath(const std::vector<vec2>& path_in,  //原始路线
                    sdf::sdf& map,                     //导航地图
                    double path_width,                 //路线宽度
                    std::vector<vec2>& path_out        //输出路线
) {
    path_out.clear();
    if (path_in.empty()) {
        return;
    }
    const int path_len = path_in.size();
    if (path_len <= 3) {  //路线太短，无须优化
        path_out = path_in;
        return;
    }
    //从第一个点开始搜索
    int nowPathId = 1;  //在当前位置能看见的最远点
    vec2 nowPoint = path_in.at(0);
    path_out.push_back(nowPoint);
    while (nowPathId < path_len - 1) {
        vec2 tmpPoint;
        nowPathId = getFarPoint(path_in, map, path_width,
                                nowPathId, nowPoint, tmpPoint);
        path_out.push_back(tmpPoint);
        nowPoint = tmpPoint;
    }
    path_out.push_back(path_in.at(path_len - 1));  //终点
}

//发射一系列光线扫描，获取最远的点
inline int getFarPoint(const std::vector<vec2>& path_in,    //原始路线
                       sdf::sdf& map,                       //导航地图
                       activeNav::activeContext& actNodes,  //动态导航索引
                       activeNav::activeNode* selfNode,     //自己的节点
                       double path_width,                   //路线宽度
                       int nowPathId,                       //当前id
                       const vec2& nowPoint,                //当前位置
                       vec2& newPoint                       //更新位置
) {
    int path_len = path_in.size();
    const int search_left = nowPathId;
    const int search_right = path_len;
    int left = search_left;
    int right = search_right;
    int newPathId = -1;
    while (left < right - 1) {  //二分搜索
        int mid = round((left + right) / 2.);
        int id = mid;
        if (id > path_len - 1) {
            id = path_len - 1;
        }
        vec2 nearestPoint;
        //发射光线
        if (rayMarch(map,
                     nowPoint, path_in.at(id),
                     path_width, nearestPoint) ||
            activeNav::activeNavRayTest(actNodes,
                                        nowPoint,
                                        path_in.at(id),
                                        selfNode)) {
            //如果发生碰撞，nearestPoint为无效值，区间往前
            right = mid;
        } else {
            //未发生碰撞，区间往后，同时更新位置
            left = mid;
            newPoint = nearestPoint;
            newPathId = mid;
        }
    }
    if (newPathId == -1 || newPathId == nowPathId) {
        auto targetPoint = path_in.at(search_left + 1);
        newPoint = targetPoint;
        auto startPoint = path_in.at(search_left);
        auto dir = targetPoint - startPoint;
        double left = 1.0, right = 0.0;
        for (int i = 0; i < 8; ++i) {
            double mid = (left + right) / 2;
            vec2 nearestPoint;
            auto rayBeginPoint = startPoint + (dir * mid);
            if (activeNav::activeNavRayTest(actNodes,
                                            rayBeginPoint,
                                            targetPoint,
                                            selfNode)) {
                return -1;
            }
            if (rayMarch(map,
                         rayBeginPoint, targetPoint,
                         path_width * 2, nearestPoint)) {
                left = mid;
            } else {
                right = mid;
                newPoint = rayBeginPoint;
            }
        }
        return search_left + 1;
    }
    return newPathId;
}
inline void optPath(const std::vector<vec2>& path_in,    //原始路线
                    sdf::sdf& map,                       //导航地图
                    activeNav::activeContext& actNodes,  //动态导航索引
                    activeNav::activeNode* selfNode,     //自己的节点
                    double path_width,                   //路线宽度
                    std::vector<vec2>& path_out,         //输出路线
                    double minLen = -1) {
    path_out.clear();
    if (path_in.empty()) {
        return;
    }
    const int path_len = path_in.size();
    if (path_len <= 3) {  //路线太短，无须优化
        path_out = path_in;
        return;
    }
    //从第一个点开始搜索
    int nowPathId = 1;  //在当前位置能看见的最远点
    vec2 nowPoint = path_in.at(0);
    path_out.push_back(nowPoint);
    double lenSum = 0;
    while (nowPathId < path_len - 1) {
        vec2 tmpPoint;
        nowPathId = getFarPoint(path_in, map, actNodes, selfNode, path_width,
                                nowPathId, nowPoint, tmpPoint);
        path_out.push_back(tmpPoint);
        lenSum += (tmpPoint - nowPoint).norm();
        if (minLen > 0 && lenSum > minLen) {
            return;
        }
        if (nowPathId == -1) {
            return;
        }
        nowPoint = tmpPoint;
    }
    path_out.push_back(path_in.at(path_len - 1));  //终点
}

inline bool nextPos(const std::vector<vec2>& path_in,    //原始路线
                    sdf::sdf& map,                       //导航地图
                    activeNav::activeContext& actNodes,  //动态导航索引
                    activeNav::activeNode* selfNode,     //自己的节点
                    double path_width,                   //路线宽度
                    vec2& path_out,                      //输出路线
                    double vel = 4) {
    std::vector<vec2> path_tmp;
    optPath(path_in, map, actNodes, selfNode, path_width, path_tmp, 1);
    path_out = selfNode->currentPos;
    for (auto& it : path_tmp) {
        auto delta = it - selfNode->currentPos;
        if (delta.norm() > 0) {
            if (delta.norm() > vel) {
                path_out = selfNode->currentPos + delta * vel / delta.norm();
                return true;
            } else {
                path_out = it;
            }
        }
    }
    return false;
}

}  // namespace sdpf::pathopt
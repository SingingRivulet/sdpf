#pragma once
#include <algorithm>
#include <iostream>
#include <vector>
#include "dynamicNav.hpp"
#include "sdf.hpp"
namespace sdpf::pathopt {

//发射光线
inline bool rayMarch(sdf::sdf& map,       //导航地图
                     const vec2& begin,   //起点
                     const vec2& end,     //终点
                     double path_width,   //线宽
                     vec2& nearestPoint,  //距离边缘最近的点
                     bool skip = false,
                     bool escape = false) {
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
    newPoint = nowPoint;
    while (left < right - 1) {  //二分搜索
        int mid = floor((left + right) / 2.);
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
    if (newPathId == -1) {
        return -1;
    }
    if (newPathId >= (int)path_in.size() - 1) {
        //最后一个点
        newPoint = *path_in.rbegin();
    } else {
        const double search_left = 0.0;
        const double search_right = 1.0;
        double left = search_left;
        double right = search_right;
        auto target = path_in.at(newPathId + 1);
        auto dir = path_in.at(newPathId) - nowPoint;
        for (int i = 0; i < 8; ++i) {
            auto mid = (left + right) / 2.;
            auto movePoint = nowPoint + dir * mid;
            vec2 nearestPoint;
            //发射光线
            if (rayMarch(map,
                         movePoint, target,
                         path_width, nearestPoint)) {
                //如果发生碰撞，nearestPoint为无效值，区间往前
                left = mid;
            } else {
                //未发生碰撞，区间往后，同时更新位置
                right = mid;
                newPoint = movePoint;
            }
        }
    }
    return newPathId;
}
inline bool optPath(const std::vector<vec2>& path_in,  //原始路线
                    sdf::sdf& map,                     //导航地图
                    double path_width,                 //路线宽度
                    std::vector<vec2>& path_out,       //输出路线
                    double minLen = -1) {
    path_out.clear();
    if (path_in.empty()) {
        return false;
    }
    const int path_len = path_in.size();
    if (path_len <= 3) {  //路线太短，无须优化
        path_out = path_in;
        return false;
    }
    //如果卡在障碍物里面，逃离障碍物
    int startPathId = 0;
    while (1) {
        if (startPathId >= (int)path_in.size()) {
            return false;
        }
        auto pos = path_in.at(startPathId);
        if (map.at(pos.x, pos.y) > path_width) {
            break;
        }
        ++startPathId;
    }
    //从第一个点开始搜索
    int nowPathId = startPathId + 1;  //在当前位置能看见的最远点
    int nowPos = startPathId;
    if (startPathId != 0) {
        path_out.push_back(path_in.at(0));
    }
    vec2 nowPoint = path_in.at(nowPos);
    path_out.push_back(nowPoint);
    double lenSum = 0;
    while (nowPathId < path_len - 1) {
        vec2 tmpPoint;
        nowPathId = getFarPoint(path_in, map, path_width,
                                nowPathId, nowPoint, tmpPoint);
        path_out.push_back(tmpPoint);
        lenSum += (tmpPoint - nowPoint).norm();
        if (nowPathId == -1) {
            return false;
        }
        nowPoint = tmpPoint;
    }
    path_out.push_back(path_in.at(path_len - 1));  //终点
    return true;
}

constexpr double degree2rad(double degree) {
    return degree * M_PI / 180;
}

inline bool moveRayTest(dynamicNav::dynamicContext& map_dynamic,  //动态地图
                        sdf::sdf& map_static,                     //静态地图
                        const vec2& end,                          //终点
                        dynamicNav::dynamicNode* selfNode,        //自己的节点
                        double range,
                        double path_width) {
    if (dynamicNav::dynamicNavRayTest(map_dynamic,
                                      selfNode->currentPos,
                                      end, selfNode, range)) {
        //std::cout << "coll:dynamic" << std::endl;
        return true;
    }
    //vec2 tmp;
    //if (rayMarch(map_static,
    //             selfNode->currentPos,
    //             end, path_width, tmp)) {
    //    std::cout << "coll:static" << std::endl;
    //    return true;
    //}
    return false;
}

inline void avoidRotate(sdf::sdf& map_static,
                        dynamicNav::dynamicContext& map_dynamic,  //动态导航索引
                        dynamicNav::dynamicNode* selfNode,        //自己的节点
                        double path_width,                        //路线宽度
                        vec2& path_out,                           //输出路线
                        double vel,
                        double rotBegin,
                        double rotEnd,
                        double range) {
    //TODO
    const double search_left = rotBegin;
    const double search_right = rotEnd;
    double left = search_left;
    double right = search_right;

    auto dir = path_out - selfNode->currentPos;

    {
        //初始状态
        vec2 rotDir = dir.rotate(rotBegin);
        path_out = rotDir + selfNode->currentPos;
    }

    for (int i = 0; i < 8; ++i) {
        //迭代8次
        auto mid = (left + right) / 2.;
        vec2 rotDir = dir.rotate(mid);
        if (moveRayTest(map_dynamic, map_static,
                        selfNode->currentPos + rotDir, selfNode,
                        range, path_width)) {
            //如果发生碰撞，区间往前
            left = mid;
        } else {
            //未发生碰撞，区间往后，同时更新位置
            right = mid;
            path_out = rotDir + selfNode->currentPos;
        }
    }
}

inline bool avoid(sdf::sdf& map_static,
                  dynamicNav::dynamicContext& map_dynamic,  //动态导航索引
                  dynamicNav::dynamicNode* selfNode,        //自己的节点
                  double path_width,                        //路线宽度
                  vec2& path_out,                           //输出路线
                  double vel = 4) {
    //避让处理
    auto dir = path_out - selfNode->currentPos;
    //检查距离
    double len = dir.norm();
    if (len <= 0) {
        return false;
    }
    //先检查直接往前，无阻挡的话直接返回true
    if (!moveRayTest(map_dynamic, map_static,
                     path_out, selfNode, len, path_width)) {
        //std::cout << "front" << std::endl;
        return true;
    }
    //检查向左旋转60度，无遮挡的话往0度二分搜索，直到遮挡后返回true
    vec2 rt_left_dir = dir.rotate(degree2rad(60));
    vec2 rt_left = selfNode->currentPos + rt_left_dir;
    if (!moveRayTest(map_dynamic, map_static,
                     rt_left, selfNode, len, path_width)) {
        //std::cout << "left" << std::endl;
        avoidRotate(map_static, map_dynamic, selfNode, path_width, path_out, vel,
                    degree2rad(60), degree2rad(0), len);
        return true;
    }
    //若左边第一次存在遮挡，右边同理
    vec2 rt_right_dir = dir.rotate(degree2rad(-60));
    vec2 rt_right = selfNode->currentPos + rt_right_dir;
    if (!moveRayTest(map_dynamic, map_static,
                     rt_right, selfNode, len, path_width)) {
        //std::cout << "right" << std::endl;
        avoidRotate(map_static, map_dynamic, selfNode, path_width, path_out, vel,
                    degree2rad(-60), degree2rad(0), len);
        return true;
    }
    //std::cout << "fail" << std::endl;
    //左右均被遮挡，返回false
    return false;
}

inline bool nextPos(const std::vector<vec2>& path_in,      //原始路线
                    sdf::sdf& map,                         //导航地图
                    dynamicNav::dynamicContext& actNodes,  //动态导航索引
                    dynamicNav::dynamicNode* selfNode,     //自己的节点
                    double path_width,                     //路线宽度
                    vec2& path_out,                        //输出路线
                    double vel = 4) {
    std::vector<vec2> path_tmp;
    optPath(path_in, map, path_width, path_tmp);
    path_out = selfNode->currentPos;
    for (auto& it : path_tmp) {
        auto delta = it - selfNode->currentPos;
        if (delta.norm() > 0) {
            if (delta.norm() > vel) {
                path_out = selfNode->currentPos + delta * vel / delta.norm();
                return avoid(map, actNodes, selfNode, path_width, path_out, vel);
            } else {
                path_out = it;
            }
        }
    }
    if (!avoid(map, actNodes, selfNode, path_width, path_out, vel)) {
        return false;
    }
    if (path_tmp.empty()) {
        return false;
    }
    return (*path_tmp.rbegin() - selfNode->currentPos).norm() > vel;
}

}  // namespace sdpf::pathopt
#pragma once
#include <algorithm>
#include <vector>
#include "sdf.hpp"
namespace sdpf::pathopt {

//发射光线
inline bool rayMarch(sdf::sdf& map,      //导航地图
                     const vec2& begin,  //起点
                     const vec2& end,    //终点
                     double path_width,  //线宽
                     vec2& nearestPoint  //距离边缘最近的点
) {
    auto beginVecLen = std::min(map[begin], begin.length(end));  //有向距离场性质：这个范围内一定没有物体
    vec2 dir = end - begin;                                      //方向
    vec2 dir_norm = dir / dir.norm();                            //方向的单位向量
    vec2 nowPos = begin + dir_norm * beginVecLen;                //起点
    auto nearestPoint_mindis = map[nowPos];                      //初始化最短距离
    nearestPoint = nowPos;                                       //最短距离的位置
    while (nowPos.length2(end) > path_width * path_width) {      //大于路宽的平方说明没到目的地
        double area_r = map[nowPos];                             //空白的半径
        if (area_r < nearestPoint_mindis) {
            nearestPoint_mindis = area_r;
            nearestPoint = nowPos;
        }
        if (path_width > area_r) {
            //发生碰撞
            return true;
        }
        double lds = nowPos.length(end);  //当前位置到目的地的距离
        auto rayLen = std::min(area_r, lds);
        nowPos += dir_norm * rayLen;
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
    auto path_len = path_in.size();
    const int search_left = nowPathId;
    const int search_right = path_len - 1;
    int left = search_left;
    int right = search_right;
    int newPathId = -1;
    while (left < right) {  //二分搜索
        int mid = (left + right) / 2;
        vec2 nearestPoint;
        //发射光线
        if (rayMarch(map,
                     nowPoint, path_in.at(mid),
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
        //保底方案
        newPathId = search_left;
        vec2 nearestPoint;
        if (!rayMarch(map,
                      nowPoint, path_in.at(newPathId),
                      path_width, nearestPoint)) {
            newPoint = nearestPoint;
        } else {
            //直接去下一个点
            newPoint = path_in.at(search_left + 1);
            return search_left + 1;
        }
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
    const auto path_len = path_in.size();
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
        nowPoint = tmpPoint;
    }
    path_out.push_back(path_in.at(path_len - 1));  //终点
}

}  // namespace sdpf::pathopt
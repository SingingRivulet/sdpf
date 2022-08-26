#pragma once
#include <math.h>
#include <omp.h>
#include <cmath>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>
#include "field.hpp"
#include "vec2.hpp"
//sdf（有向距离场）
namespace sdpf::sdf {

struct sdf : field<double> {  //运行阶段为只读数据结构
    inline sdf(int w, int h)
        : field(w, h) {}
    double* data = nullptr;
    int width = 0;
    int height = 0;
    double scale = 1.0;  //缩放（sdf有缩放不失真的特性）

    constexpr double operator()(double x, double y) {  //查找（自带插值）
        return getInterpBilinear(x * scale, y * scale) * scale;
    }
    constexpr double operator[](const vec2& i) {  //查找（自带插值）
        return getInterpBilinear(i.x * scale, i.y * scale) * scale;
    }

    constexpr double getInterpBilinear(double x, double y) {
        //四个临近点的坐标 (x1,y1)、(x1,y2),(x2,y1)，(x2,y2)
        int x1, x2;
        int y1, y2;

        //两个差值的中值
        double f12, f34;
        double epsilon = 0.0001;

        //四个临近像素坐标x像素值
        double f1, f2, f3, f4;

        //计算四个临近坐标
        x1 = (int)x;
        x2 = x1 + 1;
        y1 = (int)y;
        y2 = y1 + 1;

        //不在图片的范围内
        if ((x < 0) || (x > width - 1) || (y < 0) || (y > height - 1)) {
            return 0;
        } else {
            if (fabs(x - width + 1) <= epsilon)  //如果计算点在右测边缘
            {
                //如果差值点在图像的最右下角
                if (fabs(y - height + 1) <= epsilon) {
                    f1 = at(x1, y1);
                    return f1;
                } else {
                    f1 = at(x1, y1);
                    f3 = at(x1, y2);

                    //图像右方的插值
                    return ((f1 + (y - y1) * (f3 - f1)));
                }
            }
            //如果插入点在图像的下方
            else if (fabs(y - height + 1) <= epsilon) {
                f1 = at(x1, y1);
                f2 = at(x2, y1);

                //图像下方的插值
                return ((f1 + (x - x1) * (f2 - f1)));
            } else {
                //得计算四个临近点像素值
                f1 = at(x1, y1);
                f2 = at(x2, y1);
                f3 = at(x1, y2);
                f4 = at(x2, y2);

                //第一次插值
                f12 = f1 + (x - x1) * (f2 - f1);  //f(x,0)

                //第二次插值
                f34 = f3 + (x - x1) * (f4 - f3);  //f(x,1)

                //最终插值
                return (f12 + (y - y1) * (f34 - f12));
            }
        }
    }
    //通过三个点获取法线
    inline vec2 getDir(const ivec2& p1, const ivec2& p2, const ivec2& p3) {
        double p1_z = at(p1.x, p1.y);
        double x1 = p2.x - p1.x;
        double y1 = p2.y - p1.y;
        double z1 = at(p2.x, p2.y) - p1_z;
        double x2 = p3.x - p1.x;
        double y2 = p3.y - p1.y;
        double z2 = at(p3.x, p3.y) - p1_z;
        double x0 = y1 * z2 - y2 * z1;
        double y0 = x1 * z2 - x2 * z1;
        double z0 = x1 * y2 - x2 * y1;
        if (z0 < 0) {
            x0 = -x0;
            y0 = -y0;
        }
        vec2 res(x0, y0);
        double len = res.norm();
        if (len <= 0.0) {
            return vec2(0, 0);
        } else {
            res /= len;
            return res;
        }
    }
    //判断峰顶
    inline bool isTop(const ivec2& p) {
        if (p.x <= 0 || p.y <= 0 || p.x >= width - 1 || p.y >= height - 1) {
            //地图边缘不能检测
            return false;
        }
        auto& center_v = at(p.x, p.y);
        int count = 0;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i != 0 && j != 0) {
                    int x = i + p.x;
                    int y = j + p.y;
                    if (at(x, y) <= center_v) {
                        ++count;
                    }
                }
            }
        }
        return count >= 6;
    }
    //判断山脊
    //原理：最大圆盘算法
    //sdf本身就是圆盘，同时与两个边相切为中心
    //（两侧法线夹角大于isRidgeMinCosD为山脊）
    inline bool isRidge(const ivec2& p, double isRidgeMinCosD = 0.866025403784438) {  //默认30度
        if (p.x <= 0 || p.y <= 0 || p.x >= width - 1 || p.y >= height - 1) {
            //地图边缘不能检测
            return false;
        }
        //定义中心周围的8个点
        ivec2 p1(p.x, p.y + 1);      //90
        ivec2 p2(p.x + 1, p.y + 1);  //45
        ivec2 p3(p.x + 1, p.y);      //0
        ivec2 p4(p.x + 1, p.y - 1);  //-45
        ivec2 p5(p.x, p.y - 1);      //-90
        ivec2 p6(p.x - 1, p.y - 1);  //-135
        ivec2 p7(p.x - 1, p.y);      //180
        ivec2 p8(p.x - 1, p.y + 1);  //135
        //计算梯度并转换为单位向量
        vec2 h1 = getDir(p, p1, p2);
        vec2 h2 = getDir(p, p3, p4);
        vec2 h3 = getDir(p, p5, p6);
        vec2 h4 = getDir(p, p7, p8);
        //计算对侧夹角
        return (std::abs(h1.dot(h3)) < isRidgeMinCosD ||
                std::abs(h2.dot(h4)) < isRidgeMinCosD);
    }
};
}  // namespace sdpf::sdf

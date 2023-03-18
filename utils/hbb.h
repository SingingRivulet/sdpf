#ifndef SDPF_HBB
#define SDPF_HBB
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include "vec2.hpp"
namespace sdpf {

class HBB {
   public:
    using vec = vec2;
    class boundCircle {
       public:
        boundCircle *left,
            *right,
            *parent,
            *next;

        HBB* hbb;

        vec center;
        double r;

        void* data;

        inline bool isDataNode() {
            return data != NULL;
        }

        inline void setLeft(boundCircle* in) {
            left = in;
            in->parent = this;
        }
        inline void setRight(boundCircle* in) {
            right = in;
            in->parent = this;
        }

        inline float getMergeSizeSq(const boundCircle* other) const {
            //圆心距离加各自半径除以2
            auto delta = other->center - center;
            auto mdelta = sqrt(delta.x * delta.x + delta.y * delta.y);
            return (mdelta + r + other->r) / 2;
        }

        inline void merge(const boundCircle* other, boundCircle* out) const {
            //求外接圆
            auto cline = other->center - center;                        //圆心连线
            auto mcline = sqrt(cline.x * cline.x + cline.y * cline.y);  //连线长度

            if (mcline <= 0) {
                out->center = center;
                out->r = std::max(r, other->r);
                return;
            }

            auto p1 = center;         //左下角
            auto p2 = other->center;  //右上角
            //cline为坐下到右上的连线

            auto cldir = cline / mcline;  //单位向量（连线方向）
            auto b1 = p1 - cldir * r;
            auto b2 = p2 + cldir * other->r;

            out->center = (b1 + b2) / 2;

            //计算半径
            auto delta = b1 - b2;
            out->r = sqrt(delta.x * delta.x + delta.y * delta.y);
        }

        inline bool isEmpty() const {
            return r < 0;
        }

        inline bool inBox(const vec& point) const {
            auto delta = point - center;
            auto mdelta = sqrt(delta.x * delta.x + delta.y * delta.y);
            return mdelta < r;
        }

        inline bool intersects(const boundCircle* in) const {
            auto delta = in->center - center;
            auto mdelta = delta.x * delta.x + delta.y * delta.y;
            auto theta = in->r + r;
            return theta * theta > mdelta;
        }
        inline double rayDist(const vec2& p1, const vec2& p2) const {
            //如果两点相同，则输出一个点的坐标为垂足
            if (p1.x == p2.x && p1.y == p2.y) {
                return (p1 - center).norm();
            }
            // 根据向量外积计算有向面积，用来判断夹角。
            double s = (center.x - p1.x) * (p2.y - p1.y) - (center.y - p1.y) * (p2.x - p1.x);

            // 如果夹角为钝角
            if (s > 0) {
                return (center - p1).norm();
            } else if ((center.x - p2.x) * (p1.y - p2.y) - (center.y - p2.y) * (p1.x - p2.x) > 0) {
                return (center - p2).norm();
            } else {
                //计算直线上两点之间的距离
                double d = (p2 - p1).norm();
                return fabs(s / d);
            }
        }
        inline bool intersects(const vec2& p1, const vec2& p2) const {
            return rayDist(p1, p2) < r;
        }

        inline bool inBox(const boundCircle* in) const {
            auto delta = in->center - center;
            auto mdelta = sqrt(delta.x * delta.x + delta.y * delta.y);
            return mdelta + in->r < r;
        }

        inline vec getCenter() const {
            return center;
        }

        void construct() {
            left = NULL;
            right = NULL;
            parent = NULL;
            data = NULL;
            center = vec2(0, 0);
            r = 0;
        }

        void collisionTest(
            const boundCircle* in,
            void (*callback)(boundCircle*, void*),
            void* arg = NULL);

        void fetchByPoint(
            const vec& point,
            void (*callback)(boundCircle*, void*),
            void* arg = NULL);

        void fetchByRay(
            const vec& p0,
            const vec& p1,
            void (*callback)(boundCircle*, void*),
            void* arg = NULL);

        void autoclean();
        void autodrop();
        void add(boundCircle* in);
        void remove();
        void drop();
    };

    boundCircle* root;

    boundCircle* createAABB();
    void delAABB(boundCircle*);

    void add(boundCircle* in);
    void remove(boundCircle* in);
    boundCircle* add(const vec& center, double r, void* data);

    inline void collisionTest(
        const boundCircle* in,
        void (*callback)(boundCircle*, void*),
        void* arg = NULL) {
        root->collisionTest(in, callback, arg);
    }
    inline void fetchByPoint(
        const vec& point,
        void (*callback)(boundCircle*, void*),
        void* arg = NULL) {
        root->fetchByPoint(point, callback, arg);
    }
    inline void fetchByRay(
        const vec& p1,
        const vec& p2,
        void (*callback)(boundCircle*, void*),
        void* arg = NULL) {
        root->fetchByRay(p1, p2, callback, arg);
    }
    HBB();
    ~HBB();

   private:
    void poolInit();
    void poolDestroy();
    void* pool;
};
}  // namespace sdpf
#endif
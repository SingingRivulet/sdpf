#pragma once
#include <math.h>
#include "mat.hpp"

namespace sdpf {
namespace vec {

//向量
template <typename T>
class vector2d {
   public:
    T x;
    T y;
    inline vector2d() {
        x = 0;
        y = 0;
    }
    inline vector2d(T xt, T yt) {
        x = xt;
        y = yt;
    }
    inline void init(T xt, T yt) {
        x = xt;
        y = yt;
    }
    inline bool operator==(const vector2d<T>& i) const {
        if (x != i.x) {
            return false;
        }
        if (y != i.y) {
            return false;
        }
        return true;
    }
    inline bool operator<(const vector2d<T>& i) const {
        if (x < i.x)
            return true;
        else if (x == i.x) {
            if (y < i.y)
                return true;
        }
        return false;
    }
    inline void operator()(T xt, T yt) {
        init(xt, yt);
    }
    inline vector2d<T>& operator=(const vector2d<T>* p) {
        x = p->x;
        y = p->y;
        return *this;
    }
    inline vector2d<T>& operator=(const vector2d<T>& p) {
        x = p.x;
        y = p.y;
        return *this;
    }
    inline vector2d<T> operator+(const vector2d<T>& p) const {
        vector2d<T> b;
        b = this;
        b.x += p.x;
        b.y += p.y;
        return b;
    }
    inline vector2d<T>& operator+=(const vector2d<T>& p) {
        x += p.x;
        y += p.y;
        return *this;
    }
    inline vector2d<T>& operator-=(const vector2d<T>& p) {
        x -= p.x;
        y -= p.y;
        return *this;
    }
    inline vector2d<T> operator-(const vector2d<T>& p) const {
        vector2d<T> b;
        b = this;
        b.x -= p.x;
        b.y -= p.y;
        return b;
    }
    inline vector2d<T> operator-() const {
        vector2d<T> b;
        b.x = -x;
        b.y = -y;
        return b;
    }
    inline vector2d<T>& operator*=(T p) {
        x *= p;
        y *= p;
        return *this;
    }
    inline vector2d<T>& operator/=(T p) {
        x /= p;
        y /= p;
        return *this;
    }
    inline vector2d<T> operator*(const T& p) const {
        return vector2d<T>(p * x, p * y);
    }
    inline vector2d<T> operator/(const T& p) const {
        return vector2d<T>(x / p, y / p);
    }
    inline auto norm() const {
        return sqrt((x * x) + (y * y));
    }
    inline auto invnorm() const {
        return 1 / sqrt((x * x) + (y * y));
    }
    inline auto length2(const vector2d<T>& p) const {
        auto t = p - (*this);
        return ((t.x * t.x) + (t.y * t.y));
    }
    inline auto length(const vector2d<T>& p) const {
        return sqrt(length2(p));
    }
    inline auto dot(const vector2d<T>& p) const {
        return (
            (x * p.x) +
            (y * p.y));
    }
    inline auto rotate(double rt) const {
        T rotMat[4];
        T tmpVec[2], tmpVec_out[2];
        tmpVec[0] = x;
        tmpVec[1] = y;
        mat::buildRotateMat2d(rt, rotMat);
        mat::vec2xMat2(tmpVec, rotMat, tmpVec_out);
        return vector2d<T>(tmpVec_out[0], tmpVec_out[1]);
    }
};

}  // namespace vec
using vec2 = vec::vector2d<double>;
using ivec2 = vec::vector2d<int32_t>;
}  // namespace sdpf
#pragma once
#include <math.h>
#include <stdexcept>
#include "vec2.hpp"

//离散场
namespace sdpf {

template <typename T>
struct field {  //运行阶段为只读数据结构
    T* data = nullptr;
    int width = 0;
    int height = 0;

    inline field(int w, int h) {
        data = new T[w * h];
        width = w;
        height = h;
    }
    inline ~field() {
        if (data) {
            delete[] data;
        }
        data = nullptr;
        width = 0;
        height = 0;
    }

    inline void setAll(T v) {
        if (data) {
            int len = width * height;
            for (int i = 0; i < len; ++i) {
                data[i] = v;
            }
        }
    }

    inline field(field&& i) {  //移动构造函数
        if (data) {
            delete[] data;
        }
        data = i.data;
        width = i.width;
        height = i.height;
        i.data = nullptr;
        i.width = 0;
        i.height = 0;
    }
    inline void operator=(field&& i) {  //移动构造函数
        if (data) {
            delete[] data;
        }
        data = i.data;
        width = i.width;
        height = i.height;
        i.data = nullptr;
        i.width = 0;
        i.height = 0;
    }

    inline T& at(int ix, int iy) {  //非只读成员，谨慎使用
        if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
            throw std::out_of_range("坐标超出");
        }
        return data[iy * width + ix];
    }
};

}  // namespace sdpf
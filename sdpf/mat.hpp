#pragma once
#include <math.h>

namespace sdpf::mat {

template <typename T>
inline void vec4xMat4(const T* vec, const T* mat, T* out) {
    /*
     * |0   1   2   3 |
     * |              |
     * |4   5   6   7 |
     * |              |
     * |8   9   10  11|
     * |              |
     * |12  13  14  15|
     * 
     */
    out[0] = vec[0] * mat[0] + vec[1] * mat[4] + vec[2] * mat[8] + vec[3] * mat[12];
    out[1] = vec[0] * mat[1] + vec[1] * mat[5] + vec[2] * mat[9] + vec[3] * mat[13];
    out[2] = vec[0] * mat[2] + vec[1] * mat[6] + vec[2] * mat[10] + vec[3] * mat[14];
    out[3] = vec[0] * mat[3] + vec[1] * mat[7] + vec[2] * mat[11] + vec[3] * mat[15];
}

template <typename T>
inline void vec3xMat3(const T* vec, const T* mat, T* out) {
    /*
     * |0   1   2 |
     * |          |
     * |3   4   5 |
     * |          |
     * |6   7   8 |
     * 
     */
    out[0] = vec[0] * mat[0] + vec[1] * mat[3] + vec[2] * mat[6];
    out[1] = vec[0] * mat[1] + vec[1] * mat[4] + vec[2] * mat[7];
    out[2] = vec[0] * mat[2] + vec[1] * mat[5] + vec[2] * mat[8];
}

template <typename T>
inline void vec2xMat2(const T* vec, const T* mat, T* out) {
    /*
     * |0   1 |
     * |      |
     * |2   3 |
     * 
     */
    out[0] = vec[0] * mat[0] + vec[1] * mat[2];
    out[1] = vec[0] * mat[1] + vec[1] * mat[3];
}

template <typename T>
inline void buildRotateMat2d(T θ, T* out) {
    //|cosθ  sinθ|
    //|-sinθ cosθ|
    out[0] = cos(θ);
    out[1] = sin(θ);
    out[2] = cos(θ);
    out[3] = -sin(θ);
}

}  // namespace sdpf::mat
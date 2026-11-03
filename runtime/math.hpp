// 数学常量、折角、反对称、SO(3) 对数、偏航、范数饱和
#pragma once

#include "types/types.hpp"

namespace T1
{
    constexpr double kPi = 3.14159265358979323846;      // rad
    constexpr double kSqrt2 = 1.41421356237309504880;

    /**
     * 把角度折进 (-π, π]
     *
     * 说明
     * - 用 atan2(sin, cos)，不要 fmod
     * - −π 会变成 π（同一条射线）
     *
     * 参数
     * - angle：任意弧度
     *
     * 返回：(-π, π] 内的等价角，单位 rad
     */
    double wrap_angle(double angle);

    /**
     * 三维向量的反对称矩阵
     *
     * 说明
     * - 满足 skew(a) * b == a.cross(b)
     *
     * 参数
     * - v：三维向量
     *
     * 返回：3×3，[0 −z y; z 0 −x; −y x 0]
     */
    Mat3 skew(const Vec3& v);

    /**
     * 旋转矩阵的轴角向量
     *
     * 流程：AngleAxis(R) → angle * axis，angle 在 [0, π]
     *
     * 说明
     * - 角度接近 0 时轴无意义，返回零向量
     * - 角度为 π 时轴可以翻号
     *
     * 参数
     * - rotation：旋转矩阵
     *
     * 返回：ω = θ n，单位 rad
     */
    Vec3 so3_log(const Mat3& rotation);

    /**
     * 旋转的偏航角
     *
     * 说明
     * - 把 x 轴投到世界 XY 平面，atan2(R(1,0), R(0,0))
     *
     * 参数
     * - rotation：旋转矩阵
     *
     * 返回：yaw，rad，世界系
     */
    double yaw_of(const Mat3& rotation);

    /**
     * 把向量长度限制在上限以内
     *
     * 流程：未超限原样返回；超限则等比缩放，方向不变
     *
     * 参数
     * - v：任意维向量
     * - max_norm：长度上限，调用方保证 ≥ 0
     *
     * 返回：与 v 同维
     */
    VecX saturate_norm(const VecX& v, double max_norm);

}  // namespace T1

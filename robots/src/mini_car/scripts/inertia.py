#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算 URDF 用的转动惯量，直接输出能贴进 <inertial> 的那一行。

用法：
    # 长方体：<box size="X Y Z"/> —— 依次给三个方向的尺寸、质量
    python3 inertia.py box 0.7 0.7 0.5 5.0

    # 圆柱：<cylinder radius length/> —— 依次给半径、长度、质量
    #   默认圆柱轴沿 Z（URDF 的默认朝向）
    python3 inertia.py cyl 0.06 0.01 0.4

    # 圆柱已经被转过了（轴不沿 Z）—— 用 --axis 说明轴现在指向哪
    python3 inertia.py cyl 0.05 0.02 0.3 --axis y

公式（实心、均匀密度）：
    长方体  Ixx = m(Y²+Z²)/12   Iyy = m(X²+Z²)/12   Izz = m(X²+Y²)/12
    圆柱    Ixx = Iyy = m(3r²+L²)/12                Izz = m·r²/2
            （上面这组是"圆柱轴沿 Z"；轴换了方向，三个值就跟着换位置）

记忆法：绕哪根轴转，公式里就偏偏不看那根轴的尺寸。
"""

import argparse


def box(m, x, y, z):
    return (m * (y * y + z * z) / 12.0,
            m * (x * x + z * z) / 12.0,
            m * (x * x + y * y) / 12.0)


def cyl_z(m, r, L):
    """圆柱轴沿 Z（URDF 默认朝向）"""
    perp = m * (3 * r * r + L * L) / 12.0
    return (perp, perp, m * r * r / 2.0)


def main():
    ap = argparse.ArgumentParser(description="算 URDF 的转动惯量")
    ap.add_argument("shape", choices=["box", "cyl"])
    ap.add_argument("dims", type=float, nargs="+", help="box: X Y Z；cyl: r L")
    ap.add_argument("mass", type=float)
    ap.add_argument("--axis", default="z", choices=["x", "y", "z"],
                    help="圆柱轴当前指向哪根轴（默认 z；几何体转过就用转后的轴）")
    a = ap.parse_args()

    if a.shape == "box":
        if len(a.dims) != 3:
            ap.error("box 需要三个尺寸：X Y Z")
        ixx, iyy, izz = box(a.mass, *a.dims)
    else:
        if len(a.dims) != 2:
            ap.error("cyl 需要两个尺寸：radius length")
        r, L = a.dims
        if r <= 0 or L <= 0:
            ap.error("半径和长度必须为正")
        perp, _, along = cyl_z(a.mass, r, L)      # perp=绕垂直轴, along=绕自身轴
        # 圆柱轴指向哪根轴，哪根轴就拿"绕自身轴"的那个值，另两个拿 perp
        ixx, iyy, izz = perp, perp, perp
        if a.axis == "x":
            ixx = along
        elif a.axis == "y":
            iyy = along
        else:
            izz = along

    # 有效数字裁到 4 位，惯量不需要更多精度
    def fmt(v):
        return ("%.4g" % v)

    print("  <inertial>")
    print("    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>")
    print("    <mass value=\"%s\"/>" % fmt(a.mass))
    print("    <inertia ixx=\"%s\" ixy=\"0\" ixz=\"0\"" % fmt(ixx))
    print("             iyy=\"%s\" iyz=\"0\" izz=\"%s\"/>" % (fmt(iyy), fmt(izz)))
    print("  </inertial>")


if __name__ == "__main__":
    main()

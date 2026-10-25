# Mid-360 扫描轨迹数据

`mid360.csv` 来自 Livox 官方仿真仓库 `Livox-SDK/livox_laser_simulation` 的
`scan_mode/mid360.csv`（commit `1cce1073633a062b92e30243a4c2920e45551bb5`），
MIT License，版权归 Livox 所有，许可证原文见 `LICENSE.livox`。

格式：

```text
Time/s,Azimuth/deg,Zenith/deg
1,268.99,37.838
...
```

共 **800,000** 行，对应 Mid-360 在 200,000 点/秒下 **4 秒**的真实点序列
（40 个 10 Hz 帧，每帧 20,000 点）。

插件对两列角度按标准球坐标解释：

```text
elevation = 90° - Zenith      # 得到 -7.2° .. +52.2°，与 Mid-360 手册一致
x = cos(elevation) * cos(azimuth)
y = cos(elevation) * sin(azimuth)
z = sin(elevation)
```

> 注意：Gazebo Classic 版官方插件用的是 `zenith - 90°`，那会把点云上下镜像。
> 本插件没有沿用这个错误。

每帧取连续的 20,000 行，逐点在“该点自己的时刻”做物理射线投射。
文件末尾回绕时，插件会给整帧加一个黄金角旋转（默认 137.507764°），避免
4 秒后轨迹精确重复；可用 `<azimuth_cycle_offset_deg>` 改为 0 关闭。

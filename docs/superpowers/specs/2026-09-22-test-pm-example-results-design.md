# test_pm 节点具体示例与结果图片设计说明

## 目标

直接修改当前文件：

```text
docs/patents/point-mass-trajectory-reconstruction-technical-disclosure-no-figures.md
```

将其“具体实施例”改写为基于 `point_mass_model/test_pm` ROS 节点的真实运行示例，将“有益效果”更名为“算法增益效果”，并在具体示例下加入一张 RViz 运行截图和一张由同次运行数据生成的轨迹曲线图。

本次不修改或重新生成 DOCX。

## 真实示例来源

示例以以下节点源码为唯一参数来源：

```text
swarm-playground/main_ws/src/planner/point_mass_trajectory_generation/src/test_point_mass_trajectory.cpp
```

文档中写入以下源码确定的配置：

- ROS 节点名称：`pmps_test_node`；
- 输出话题：`/generated_trajectory`；
- 消息类型：`nav_msgs/Path`；
- 坐标系：`map`；
- 三个坐标轴的加速度上界均为 (20\,\mathrm{m/s^2})，下界均为 (-20\,\mathrm{m/s^2})；
- 圆形航点构造半径为 (25\,\mathrm m)；
- 循环生成七个航点，并以初始位置作为起始航点；
- 恒定高度为 (2\,\mathrm m)；
- 初始位置为 ((0,0,2)\,\mathrm m)；
- 初始速度为 ((0,0,0)\,\mathrm{m/s})；
- 初始姿态四元数为 ((1,0,0,0))；
- 中间参考方向由 ((\sin\theta_i,\cos\theta_i,1)) 归一化得到；
- `fix_replan=false`，执行完整搜索；
- 轨迹采样周期为 (0.03\,\mathrm s)。

轨迹点数、轨迹计算耗时、采样总时长和任何其他结果只能从实际运行中取得，不得根据源码猜测或人工编造。

## ROS 运行与数据采集

使用当前已编译的 ROS Noetic 工作空间运行：

```text
roscore
rosrun point_mass_model test_pm
```

订阅节点发布的 `/generated_trajectory` 消息并保存实际 `nav_msgs/Path` 数据。由于节点一次性发布锁存消息，采集程序应等待一条非空 Path 后退出。

数据曲线的时间轴按源码中的固定采样周期构造：

[
t_k=0.03k,qquad k=0,1,\ldots,N-1.
]

其中 (N) 为实际 Path 中的位置点数量。文档中将采样总时长表述为 ((N-1)\times0.03\,\mathrm s)。不使用消息内相同的 Pose 时间戳推断逐点时间。

## RViz 运行截图

输出文件：

```text
docs/patents/figures/test-pm-rviz-result.png
```

截图要求：

- RViz Fixed Frame 为 `map`；
- 添加 Path 显示并订阅 `/generated_trajectory`；
- 完整轨迹位于视野内；
- 保留坐标网格、世界坐标轴和左侧显示列表，以证明话题和显示配置；
- 去除与示例无关的面板或错误提示；
- 图像文字和轨迹线清晰，无其他窗口遮挡；
- 截图来自实际节点运行后的 RViz 画面，不使用生成式图像替代。

## 数据曲线图

输出文件：

```text
docs/patents/figures/test-pm-trajectory-curves.png
```

曲线图由同一次节点运行采集的 Path 数据生成，至少包含：

- 三维轨迹曲线；
- 按源码公式生成并标出的起始点及七个圆形航点；
- (x(t))、(y(t))、(z(t)) 三条位置曲线；
- 轴名称、单位、图例和网格；
- 实际轨迹点数与按 (0.03\,\mathrm s) 计算的采样总时长。

不通过位置差分生成并宣称为算法输出的速度或加速度曲线，因为当前 ROS 节点只在 `nav_msgs/Path` 中发布位置。

## Markdown 修改

### 标题修改

```text
## 具体实施例  ->  ## 具体示例
## 有益效果    ->  ## 算法增益效果
```

### 具体示例内容

“具体示例”应依次说明：

1. 节点用途和运行环境；
2. 航点构造公式与初始状态；
3. 加速度边界、完整搜索模式和采样周期；
4. 节点的求解、采样、消息转换和发布过程；
5. 实测的计算时间、轨迹点数和采样总时长；
6. RViz 画面说明及图片；
7. 数据曲线说明及图片；
8. 仅依据实际画面和数据能够支持的结果分析。

图片采用相对于 Markdown 文件的路径：

```markdown
![test_pm节点的RViz运行结果](figures/test-pm-rviz-result.png)

![test_pm节点输出轨迹的数据曲线](figures/test-pm-trajectory-curves.png)
```

不恢复已删除的附图说明章节、仿真预留章节或理论证明章节。

## 算法增益效果

保留原“有益效果”中的技术内容，仅将标题改为“算法增益效果”。如需结合示例添加一句总结，只能陈述由实测数据直接支持的轨迹生成、输出和可视化效果，不宣称未测量的性能增益。

## 验收标准

- 两张图片均来自真实节点运行或真实 Path 数据，不含虚构结果。
- RViz 截图能清楚辨认轨迹、Fixed Frame、Path 话题和显示项。
- 数据图中的点数、时间轴和航点与实际节点及源码一致。
- Markdown 中的实测数字与运行日志和采集数据一致。
- 两个标题修改正确，图片位于“具体示例”下方。
- 原始完整交底书、DOCX、Figure 1 文件和无关目录不被修改。

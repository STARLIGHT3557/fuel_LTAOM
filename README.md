<img src="https://z3.ax1x.com/2021/05/05/gKVnHJ.png" alt="prometheus logo" align="right" height="70" />

# 更改： 相较于官网Prometheus， 删去了无关的代码， 加入super代码，并进行基于gazebo的仿真。 需要配套prometheus_px4进行使用

### 1. 使用方法： 
首次运行前，安装 OctoMap 建图节点的运行依赖（ROS Noetic）：
```bash
sudo apt-get update
sudo apt-get install ros-noetic-octomap-server
```
仅安装 `ros-noetic-octomap` 或 `ros-noetic-octomap-msgs` 不包含 `octomap_server_node`。

```
   source source.sh
```
#### 注： 修改source.sh中对应路径, 并将其放在~/.bashrc中

### 2. 在终端中启动base_simulation.sh 

### 3. 在打开的rviz中通过3d nav goal 给目标点

### 4. FUEL 自主探索（独立启动）

```bash
bash compile_fuel.sh
bash fuel_simulation.sh
```

起飞悬停并进入 `COMMAND_CONTROL` 后，通过 `rosservice call /fuel/start "{}"` 或 RViz 的 **2D Nav Goal** 启动探索；`rosservice call /fuel/stop "{}"` 停止并请求悬停。不要与 `base_simulation.sh` 同时运行。

依赖安装、探索边界、MID360 外参、数据接口和测试方法见 [FUEL 接入说明](Modules/fuel_bridge/README.md)。

OctoMap、RViz、点云坐标系、探索范围和边界绕圈等问题的排查过程见 [仿真问题与解决记录](Modules/fuel_bridge/TROUBLESHOOTING.md)。

# Prometheus - 自主无人机开源项目

[[English Readme]](https://github.com/amov-lab/Prometheus/blob/master/README_EN.md)

**Prometheus**是希腊神话中最具智慧的神明之一，希望本项目能为无人机研发带来无限的智慧与光明。

## 项目总览

Prometheus是一套开源的**自主无人机软件系统平台**，为无人机的智能与自主飞行提供**全套解决方案**。本项目基于[PX4开源飞控固件](https://docs.px4.io/main/zh/index.html)和[ROS机器人操作系统](https://wiki.ros.org/)，旨在为无人机开发者配套成熟可用的**机载电脑端**软件系统，提供更加简洁快速的开发体验。目前已集成**控制**、**规划**及**目标检测**等研究方向，提供多个功能demo，并配套有Amovlab仿真组件。

 - Github：https://github.com/amov-lab/Prometheus
 - Gitee：https://gitee.com/amovlab/Prometheus
 - **开源项目，维护不易，还烦请点一个star收藏，谢谢支持！**

## 快速入门

 - 安装及使用：[Prometheus使用手册](https://docs.amovlab.com/prometheus-wiki/#/src/Prometheus-%E8%87%AA%E4%B8%BB%E6%97%A0%E4%BA%BA%E6%9C%BA%E5%BC%80%E6%BA%90%E9%A1%B9%E7%9B%AE/Prometheus-%E8%87%AA%E4%B8%BB%E6%97%A0%E4%BA%BA%E6%9C%BA%E5%BC%80%E6%BA%90%E9%A1%B9%E7%9B%AE)
    - 需掌握C语言基础（大部分程序为C语言，部分模块有少量C++和python的语法）。
    - 纯新手入门者建议先自学ROS官网教程。
    - PX4飞控代码可不掌握，但需要掌握基本概念及基本操作。

 - 答疑及交流：
    - 答疑论坛（官方定期答疑，推荐）：[阿木社区-Prometheus问答专区](https://bbs.amovlab.com/forum.php?mod=forumdisplay&fid=101)
    - 添加微信jiayue199506（备注消息：Prometheus）进入Prometheus自主无人机交流群。
    - B站搜索并关注“阿木社区”，开发团队定期直播答疑。

## 进阶学习

 - Promehteus自主无人机二次开发课程： 本课程偏重本项目中的基础知识和操作实践，适合本项目入门者。课程报名[请戳这里](https://bbs.amovlab.com/plugin.php?id=zhanmishu_video:video&mod=video&cid=43)。
 - 更多进阶学习资料正在紧张筹备中。

## 硬件产品

 - 仿真必备遥控器：[点击购买](https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22617251051.13.2ffa3e75uvfxuB&id=612837659406)

 - **Prometheus二次开发平台：**   [Prometheus 450第三代正式发布](https://mp.weixin.qq.com/s/LdtmLQ2eYUwg-pRklMXL8w)！
    - 丰富的文档资料，详情[请戳这里](https://wiki.amovlab.com/public/prometheuswiki/) 。
    - 售后服务与技术指导。
    - 免费赠送 成都线下工程实训课程 及 Promehteus自主无人机二次开发课程购买折扣。
 
 - 其他配套硬件：机架、机载电脑、双目、激光雷达等无人机二次开发硬件请关注 [阿木实验室淘宝店](https://shop142114972.taobao.com/?spm=a230r.7195193.1997079397.2.67d03d8dJQgFRW)　或　[阿木实验室京东旗舰店](https://mall.jd.com/index-10260560.html?from=pc)。
 


## Prometheus校园赞助计划
 - 奖励使用Prometheus进行实验并发表相关论文的学生科研工作者。
 
 - 奖励为Prometheus提供新功能或帮忙进行测试的学生开发者。

- 详情[请戳这里](https://mp.weixin.qq.com/s/i9CpIctu56LfQFIciwzImA) ！

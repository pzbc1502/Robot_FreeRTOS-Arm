# 主工程与目录职责

主工程为本仓库根目录的 `Robot_FreeRTOS.uvprojx`，目标 `Target_1`。
本机对应 `E:/Renesas/RA6M5-Robot/Robot_FreeRTOS - Arm`；其他同名Robot_FreeRTOS、嵌赛副本和聊天默认目录不自动等同于本工程。

| 目录 | 职责与维护规则 |
| --- | --- |
| APP | 机械臂运动、采集、定靶与业务状态机 |
| Middle / USER_BSP | 协议、运动学、设备接口与板级驱动 |
| src | FreeRTOS任务入口；故障钩子不执行依赖调度的操作 |
| configuration.xml / ra_cfg / ra_gen / RTE | FSP配置与生成文件；优先改配置入口并核对生成差异 |
| ra | 厂商SDK；不为目录美观移动或重写 |
| UpperComputer | PC协议模拟器；不包含真实Jetson/Qt应用 |
| DEBUG/test_*.py | 明确列入离线入口的检查；不得按目录盲跑所有脚本 |
| DEBUG下的demo、serial、采集脚本 | 实机工具，须显式端口及操作确认；历史工具不代表当前主流程 |
| script | 离线检查入口与版本快照工具 |
| doc | 当前技术合同、维护记录；history为明确过时的参考 |
| 文档 | 仅本地保留的报告、申报资料及图片；整个目录不纳入Git，不作为线上协议定义 |
| Objects / Listings / UpperComputer/build* / dist* | 可再生构建产物，本地保留、退出源码索引 |
| artifacts | 新检查、版本快照与实机日志的默认输出，Git忽略 |

现有DEBUG历史日志和原始验收证据保留，不以“清理缓存”为名删除。历史Jetson整理参考位于`doc/history/项目整理说明_旧Jetson参考.md`。

日常入口：`python -B script/check_project.py`；版本记录：`python -B script/project_snapshot.py`。
修改提交、构建、烧录、实机运动和输出验收应分别记录；本轮整理不自动提交或推送。

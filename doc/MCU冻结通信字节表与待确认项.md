# MCU 冻结通信字节表与待确认项

核对日期：2026-10-09。交付对象：Jetson 协议实现方。

## 1. 本次交付范围

**已冻结的是扫描扩展 `SCAN_WIRE_20261009`，不是完整的新选痣执行协议。**

权威来源：

- [MCU 通信头文件](../Middle/jetson_vision.h)：TYPE、扫描 LEN、枚举和错误码。
- [通信收发实现](../Middle/jetson_vision.c)：实际载荷序列化和解析。
- [MCU 流程实现](../APP/robot_workflow.c)：扫描到位、返回和就绪失效行为。
- [双方通信协议](Jetson↔RA6M5%20通信协议.md)：第 6 章是已冻结扫描，第 7 章是新目标业务逻辑规格。

本表依据当前源码，不代表已经烧录到现场开发板或完成实机验收。不得将 C 结构体直接作为载荷发送：`jetson_scan_control_t` 包含内部 SEQ 和可能的对齐填充，线上布局以下表为准。

| 接口 | 当前交付状态 |
| --- | --- |
| SCAN_CTRL / SCAN_STATUS，含最终固定正视通知 | 已冻结、MCU 和 PC 模拟器已接入 |
| 旧 TARGET_CTRL / VISION_ERROR | 字节基线已有；不含目标身份或医生许可 |
| TARGET_BIND / TARGET_REVERIFY / TARGET_VISION | 仅逻辑规格，TYPE、LEN、偏移及动作枚举未冻结，MCU 未接入 |
| TARGET_SESSION_CTRL / DOCTOR_PERMIT / TARGET_STATUS | 同上；医生许可有效期数值未确定 |

**现阶段 Jetson 可以实现扫描握手，不能据此认为新选痣和激光执行链路已接通。**扫描最终保持在 `FLOW_SCAN_READY`，不会自动转入旧选视图定靶流程，旧 SELECT / TARGET_START 不能用来绕过新目标业务缺口。

## 2. 公共帧格式

所有编号均为十六进制；偏移从 0 开始。多字节整数小端。

```text
A5 5A VER TYPE SEQ LEN PAYLOAD[LEN] CRC_LO CRC_HI
```

| 整帧偏移 | 字段 | 说明 |
| --- | --- | --- |
| 0 / 1 | SOF | A5 / 5A |
| 2 | VER | 01，本次不更改 |
| 3 | TYPE | 消息类型 |
| 4 | SEQ | 请求使用 01~FF；00 保留，不用于扫描请求 |
| 5 | LEN | 仅载荷字节数，最大 32 |
| 6 起 | PAYLOAD | 按消息表逐字段编码 |
| 6+LEN / 7+LEN | CRC | CRC16/MODBUS，低字节在前 |

CRC 初值 `FFFF`，反射多项式 `A001`，覆盖整帧偏移 2 至 `5+LEN`，不含帧头和 CRC。总长 `8+LEN`。

## 3. 扫描控制：TYPE=07，LEN=6

Jetson -> MCU，总帧长 14 字节，CRC 位于整帧偏移 12、13。

| 载荷偏移 | 整帧偏移 | 字段 | 类型 / 编码 |
| --- | --- | --- | --- |
| 0 | 6 | action | uint8：00 QUERY，01 START，02 PHOTO_DONE |
| 1 | 7 | value | uint8：QUERY / START 必须为 00；PHOTO_DONE 为 01 保存成功 / 00 保存失败 |
| 2~5 | 8~11 | scan_id | uint32 LE，本轮扫描编号 |

START / PHOTO_DONE 的 scan_id 必须非零。QUERY 可用 0 查询当前或最近上下文；业务推进仍必须核对实际返回的本轮 ID。

Jetson 使用持久化单调递增的非零 scan_id；新轮递增，重发沿用原 ID、SEQ 和载荷。MCU 当前启动内的新 START 必须大于已接受 ID 高水位，不能用固定示例 ID 反复开启新轮。重启 / 断连后清旧待发动作并重新测距，不自动重放旧启动。

## 4. 扫描状态：TYPE=82，LEN=10

MCU -> Jetson，总帧长 18 字节，CRC 位于整帧偏移 16、17。

| 载荷偏移 | 整帧偏移 | 字段 | 类型 / 编码 |
| --- | --- | --- | --- |
| 0 | 6 | event | uint8，见事件表 |
| 1 | 7 | state | uint8，见状态表 |
| 2 | 8 | flags | uint8，见完成位表 |
| 3 | 9 | request_action | uint8，对应 SCAN_CTRL 的 action |
| 4 | 10 | result | uint8：01 正常 / 接受，00 失败 / 拒绝 |
| 5 | 11 | error_code | uint8：00 无错误，其他见第 7 节 |
| 6~9 | 12~15 | scan_id | uint32 LE |

### 4.1 事件枚举

| event | 名称 | SEQ / request_action | 含义 |
| --- | --- | --- | --- |
| 01 | ACK | 当前请求 / 当前动作 | 接受或拒绝，不代表到位 |
| 02 | SNAPSHOT | 当前请求 / 对应动作 | 只读状态；QUERY 对应 action=00 |
| 03 | FRONT_PHOTO_READY | 最初 START 的 SEQ / 01 | 左到正停稳，等待新照片保存确认 |
| 04 | PHOTO_ACCEPTED | PHOTO_DONE 的 SEQ / 02 | 保存确认接受，继续正到右 |
| 05 | SCAN_DONE | 最初 START 的 SEQ / 01 | 右端真实停稳，随后自动 HOME；不放行选痣 |
| 06 | HOME_READY | 最初 START 的 SEQ / 01 | 扫后 HOME 校验通过，随后自动固定正视；不放行选痣 |
| 07 | TERMINATED | 最初 START 的 SEQ / 01 | 中止 / 故障，停止保持，不自动 HOME |
| 08 | FRONT_OBSERVE_READY | 最初 START 的 SEQ / 01 | 最终固定正视真实到位并保持，输出关闭 |

### 4.2 状态枚举

| state | 名称 | 含义 |
| --- | --- | --- |
| 00 | NONE | 无上下文 |
| 01 | PREPARING | 准备基础位置及左视位 |
| 02 | LEFT_TO_FRONT | 第一段扫描 |
| 03 | WAIT_PHOTO | 中途正视保持，等待保存确认 |
| 04 | FRONT_TO_RIGHT | 第二段扫描 |
| 05 | RETURNING_HOME | 右扫结束，准备 / 执行 HOME |
| 06 / 07 | 保留 | 不使用旧 HOME_HOLD / SELECT_CONSUMED 语义 |
| 08 | ABORTED_HOLD | 取消保持 |
| 09 | FAULT_HOLD | 故障保持 |
| 0A | RETURNING_FRONT | 扫后 HOME 通过，正在自动进入固定正视 |
| 0B | FRONT_OBSERVE_HOLD | 当前最终固定正视到位并保持；不是输出许可 |
| 0C | SCAN_CHAIN_COMPLETED | 初始正视就绪资格已撤销，不能重放为可选就绪 |

**0C 的当前代码边界：**从最终正视保持接受 `WORKFLOW_CTRL / FINISH_RETURN_HOME` 时设置 `0C`、清 flags，然后执行返回。因此 `0C` 不证明本次结束返回已到 HOME，也不证明目标绑定、许可消费或输出完成。目前尚无新目标接口将状态转入 `0C`；后续扩展不能冒充已实现。

正常结束后的 HOME 完成仍看旧状态帧：`TYPE=81 / LEN=3 / event=20 / value=05 / error=00`，关联 FINISH_RETURN_HOME 的 SEQ；失败则按工作流错误处理。

### 4.3 完成位与最终就绪判断

| flags | 含义 |
| --- | --- |
| bit0 / 01 | PHOTO_SAVED，本轮保存确认已接受 |
| bit1 / 02 | SCAN_FINISHED，两段扫描及右端停稳已完成 |
| bit2 / 04 | HOME_VERIFIED，本轮扫后 HOME 曾验证，不能证明当前仍在 HOME |
| bit3~7 | 保留，必须为 0 |

WAIT_PHOTO 为 `00`；第二段为 `01`；返回 HOME 为 `03`；自动回正视及最终正视保持为 `07`。`0F` 不合法，`07` 不能单独证明最终正视到位。

最终就绪事件的必要组合：

```text
TYPE=82 LEN=10
event=08 state=0B flags=07 request_action=01 result=01 error_code=00
scan_id=本轮编号，SEQ=本轮最初 START 的 SEQ
```

最终事件丢失可 QUERY 本轮。合法当前就绪快照为 `event=02 / state=0B / flags=07 / request_action=00 / result=01 / error=00`，关联 QUERY 的 SEQ 和本轮 ID。ACK、SCAN_DONE、HOME_READY、历史成功包均不能替代当前就绪。取消 / 故障 / 断连后，迟到成功包不能重新放行。

## 5. Jetson 需要实现的扫描应答

```text
原 WORKFLOW START_MEASURE -> 测距位到位 -> 新 SAFE_DISTANCE 合格并锁存
SCAN START -> ACK -> 左到正 -> FRONT_PHOTO_READY
Jetson 取新图并保存 -> PHOTO_DONE -> ACK / PHOTO_ACCEPTED
MCU 正到右 -> SCAN_DONE -> 自动 HOME -> HOME_READY
MCU 自动固定正视 -> FRONT_OBSERVE_READY -> 保持
```

右端 -> HOME -> 固定正视由 MCU 自行完成。Jetson 不再发送额外 HOME、SELECT、GOTO 或数据归档许可来推进这一段，也不用应答 HOME_READY / FRONT_OBSERVE_READY 才能继续运动。候选尚未可用时仍保持、输出关闭。

心跳独立发送，推荐 200 ms，MCU 超时 600 ms；其他业务帧不续心跳。扫描 ACK 等待 300 ms，最多额外重发 3 次，重发相同请求。准备 / 单段 / 保存 / HOME / 最终正视的 MCU 等待上限分别为 60 s / 30 s / 15 s / 45 s / 60 s；不能把所有 ACK 和心跳超时统一改为 45 s。

目前未新增扫描连续深度保护，仍沿用起始测距逻辑。这不等于整条路径已安全；固定模型、限位 / 急停和实际路径验收仍需落实。

### 5.1 中止接受与停止确认（2026-10-09 补充）

本次不改变 `VER=01`、TYPE、LEN 或错误编号；修正旧固件“发出停止即报告停止成功”的行为。双方联调须人工确认板上固件包含这次修复，不能只凭帧版本判断兼容。

| 方向 / 阶段 | 字节内容 | 含义 |
| --- | --- | --- |
| Jetson -> MCU：中止 | `TYPE=06 LEN=1 payload[0]=03` | ABORT_HOLD，关输出、中止并保持，不自动 HOME |
| MCU -> Jetson：接受 | `TYPE=81 LEN=3 payload=21 01 00`，SEQ 匹配中止请求 | 只表示接受请求，不能显示为“已停止” |
| MCU -> Jetson：确认通过 | `TYPE=81 LEN=3 payload=20 06 00`，SEQ 匹配中止请求 | WORKFLOW / ABORTED_HOLD / NONE，软件停止确认完成 |
| MCU -> Jetson：确认失败 | `TYPE=81 LEN=3 payload=20 07 06`，SEQ 匹配中止请求 | WORKFLOW / FAULT_HOLD / MOTION_FAILED，停止未确认；保持关输出和中止锁存 |

确认条件：本次五轴 CAN 停止应答通过现有总线检查；原执行任务已经退出且事件队列为空；每次重新读取完整 J1～J5，连续至少三次反馈相对固定首样本的角度变化均不超过 `0.10°`，样本至少间隔 `100 ms`，覆盖至少 `200 ms`。任一缺帧、非有限值或仍在执行都会打断稳定证据。总确认预算 `2000 ms`，到期不再接受迟到的成功样本。

这些是软件观测条件，不是绝对零速度或功能安全认证，也不能排除编码器下游的机械松动、自重变形。现场仍需硬件急停、输出禁止和实物验收。扫描的 `TYPE=82 / TERMINATED`、扫描状态 `08/09` 及 flags 清零仅表示本轮作废，不能代替以上匹配中止 SEQ 的停止确认。

同一中止请求重发不重新执行、不延长确认期限；确认后在命令缓存有效期内重放原结果。等待中收到不同 SEQ 的中止请求会并入当前确认，各自回传匹配结果，不重置预算。缓存满时拒绝新请求为 BUSY，既有中止继续执行。停止等待及确认失败期间拒绝 START_MEASURE；故障解除后须用新的显式 ABORT 请求重新取得停止确认，之后才能另发 START_MEASURE。恢复心跳不会续跑，也不会自动回 HOME。

**Jetson 仍必须独立补齐发送侧。**接收错误、协议校验失败、保存失败或其他本轮失效时，串口控制层应撤销旧成功资格、优先发送 ABORT，并在有限期限内重发同一请求和等待匹配确认，不依赖 Qt 事件循环。正常推进保持锁定。ACK 或扫描 TERMINATED 不算停止成功；发送不可信、收到失败或确认超时，应停止心跳并显示“停止未确认”，要求现场按既定硬件急停策略处理。停止心跳仅是健康 MCU 的超时保护后备，不能保证失控 MCU 或失效驱动器停止。

Jetson 等待停止完成建议上限 `3 s`，从首次发送中止开始计时，不因重发、ACK 或其他业务帧续期；收到匹配成功确认后仍不能恢复旧扫描。ACK 仍使用原 `300 ms / 最多额外三次重发` 策略，接受后等待业务完成，不能把接受丢包或超时当成停止成功。发送侧独立任务与重发尚须队友实现、故障注入验证；本次 MCU 离线通过不代表双端闭环已验收。

## 6. 目标、视觉、医生许可：不能提供虚构字节表

### 6.1 现有旧视觉接口

| TYPE / LEN | 载荷偏移 | 实际字段 |
| --- | --- | --- |
| 02 / 1 | 0 | TARGET_CTRL enable：00 STOP_HOLD，01 START |
| 03 / 5 | 0~1 | VISION_ERROR dcx：int16 LE |
| 03 / 5 | 2~3 | VISION_ERROR dcy：int16 LE |
| 03 / 5 | 4 | valid：uint8，00 无效 / 01 有效 |

旧误差定义仍是目标相对图像中心，右 / 下为正。上述接口没有 scan_id、candidate_id、target_session_id、frame_id 或 permit_id，不能通过换 SEQ 获得身份绑定，也不能扩大旧载荷假装兼容。

### 6.2 尚待双方冻结的新接口

下面是双方协议第 7 章的逻辑名称，**不是已分配的 TYPE 或线上枚举**。

| 逻辑接口 | 必要语义 | TYPE / LEN / 字段偏移 / 数值枚举 |
| --- | --- | --- |
| TARGET_BIND | 绑定 / 清除本轮所选目标，清旧视觉和许可；仅绑定，不运动 | 未冻结；MCU 未实现 |
| TARGET_REVERIFY | 当前新图确认同一颗 / 歧义 / 丢失，携带目标与图像关联 | 未冻结；MCU 未实现 |
| TARGET_VISION | 目标关联、新样本编号、dcx/dcy、valid，必要质量 / 年龄 | 未冻结；MCU 未实现 |
| TARGET_SESSION_CTRL | 绑定目标的 START_ALIGN / STOP_HOLD / QUERY | 动作数值及字节布局未冻结；MCU 未实现 |
| DOCTOR_PERMIT | 目标关联、permit_id、授予 / 撤销、有效期 | 字节布局、动作数值、TTL 数值及范围未冻结；MCU 未实现 |
| TARGET_STATUS / QUERY | 带目标身份的复核、对准、许可、输出开关与失败状态 | 未冻结；MCU 未实现 |

建议目标键由 `scan_session_id / candidate_id / target_session_id / target_revision` 组成，其中 scan_session_id 与扫描 scan_id 是同一轮编号；数据类型和偏移仍待冻结。显示字符串 R01 不直接作为线上姿态编号。新绑定视觉误差应相对实测激光瞄准参考，不能暗改旧 TYPE=03 的含义。

本轮不要求痣的基座 XYZ、GLB 坐标、FK 遥测或相机安装矩阵来推进固定演示扫描。选择目标后从 MCU 已到位的固定正视进行同一颗复核和像素对准，不新增任意空间到靶接口。

### 6.3 单次许可的已确认逻辑与未定数值

已确认业务方向是“一次医生确认，只允许一次输出”。协议逻辑规格要求：首次接受时 MCU 本地单调计时；同一 permit_id 重发不续期；实际开启输出时消费；目标 / 对准丢失、视觉过期、换痣、重扫、取消 / 复位 / 故障、心跳失效、松开 P000 或输出结束撤销旧许可，恢复后不能自动复用。

**许可 TTL 目前没有冻结值，也没有当前 MCU 实现。**`TARGET_VISION_VALID_MS=500` 是视觉有效期，心跳 `600 ms` 是链路超时，`TARGET_OUTPUT_MAX_MS=10000` 是单次输出硬上限，三者都不是医生许可有效期；10 秒也不是默认治疗时长。P000 再武装边沿、许可失效后的迟到包拒绝和重连处理应一并冻结。

## 7. 0x10 错误码及冲突处理

### 7.1 当前 MCU 错误表

| error_code | 名称 |
| --- | --- |
| 00 | NONE |
| 01 | VERSION_MISMATCH |
| 02 | UNKNOWN_TYPE |
| 03 | INVALID_PARAM |
| 04 | POSE_INVALID |
| 05 | BUSY |
| 06 | MOTION_FAILED |
| 07 | HEARTBEAT_TIMEOUT |
| 08 | SAFETY |
| 09 | SAFE_DISTANCE_TOO_CLOSE |
| 0A | VISION_LOST |
| 0B | SOFT_RESET_FAILED |
| 0C | INVALID_STATE |
| 0D | SEQ_CONFLICT |
| 0E | TARGET_GATE_DENIED |
| 0F | MOTION_ABORTED |
| **10** | **SCAN_ID_MISMATCH** |
| 11 | PHOTO_SAVE_FAILED |
| 12 | PHOTO_CONFIRM_TIMEOUT |
| 13 | SCAN_STAGE_TIMEOUT |

error_code 在以下载荷中共用本表：`TYPE=FE / LEN=1` 的偏移 0，`TYPE=81 / LEN=3` 的偏移 2，`TYPE=82 / LEN=10` 的偏移 5。

`TYPE=81` 的 **event=10** 是旧 CAPTURE_POINT 拍摄点到位事件，与 **error_code=10** 属于不同字段，允许相同数值，不构成错误码冲突。

### 7.2 Jetson 已确认的冲突

2026-10-09 收到 Jetson 队友的核对反馈：当前 `VER=01` 下，Jetson 的 `error_code=10` 定义为 `SAFE_DISTANCE_TIMEOUT`，与 MCU 的 `SCAN_ID_MISMATCH` 确实冲突，**不是混淆 event 字段**。

以下位置与验证结果来自队友提供的代码摘录和离线检查，本机没有访问或重新运行 Jetson 工程：

- `/home/jetson/jixiebi/jetson_nano_vision/jetson_vision/vision_protocol.py:113`：`ERROR_SAFE_DISTANCE_TIMEOUT = 0x10`。
- 同文件第 547 行：错误名称映射将该编号显示为 `SAFE_DISTANCE_TIMEOUT`。
- 同文件第 57 行：`STATUS_CAPTURE_POINT_REACHED = 0x10` 属于 event，可保留，不与错误字段冲突。
- Jetson 的 TYPE=81 路径接受 error_code=10；TYPE=FE 路径的校验只允许至 0F，导致合法的 MCU 错误帧被报为“未知错误码”。

结论：既存在错误语义冲突，也存在不同 TYPE 的错误校验不一致。只改显示名称，或只放宽 FE 的数值上限，都不能完成兼容。

### 7.3 本轮统一方案与 Jetson 修改清单

1. 本轮 `SCAN_WIRE_20261009` 保持 MCU 头文件不变：`error_code=10 -> SCAN_ID_MISMATCH`，11~13 仍按第 7.1 节，VER 仍为 01。
2. Jetson 同步修改常量、错误名称映射、业务判断、日志 / UI 和测试预期；检查所有引用 `ERROR_SAFE_DISTANCE_TIMEOUT` 的位置，不能仅将旧常量重命名却保留“测距超时”的业务处理。
3. FE、81、82 中的 error_code 必须使用同一冻结枚举集合，完整接受本表 00~13。不要保留 FE 只允许至 0F 的限制，也不要按 TYPE 为同一错误编号赋不同含义。
4. event=10 的旧拍摄点到位含义保持；state、event、error_code 分别解码，不能共用一张名称表。
5. Jetson 如仍需 `SAFE_DISTANCE_TIMEOUT`，保留测距超时检测及安全停止行为，但可先使用独立的本地错误标识，不占用线上 error_code=10。若需要传输该错误，双方另行冻结新编号和字段；本次不自动分配 14，也不将其冒充扫描阶段超时 13。
6. 面向本轮协议禁用旧的 10=测距超时解释。若确需兼容另一个已经部署的协议版本，应显式区分版本 / 能力；不能仅凭同一个 VER=01 自动判断两种语义。当前尚无线上能力协商，联调前先核对双方构建版本。
7. 未知错误保留原始编号供诊断，不回退为 NONE，不据此放行下一步；错误处理不能为了兼容而关闭心跳、测距或停止保护。

**状态：已查清并确定本轮适配方向；Jetson 修改、回归及双端验收尚未证实完成。**本次只更新本文，没有修改 MCU 代码、Jetson 代码或现场设备。

### 7.4 必须补充的回归用例

| 用例 | 预期 |
| --- | --- |
| FE / LEN=1 / error=10 | 正常解码为 SCAN_ID_MISMATCH，不报未知错误 |
| 81 / LEN=3 / 载荷偏移 2 为 10 | 同上，不显示 SAFE_DISTANCE_TIMEOUT |
| 82 / LEN=10 / 载荷偏移 5 为 10 | 拒绝原因是扫描编号不匹配，不作为成功或最终到位 |
| FE、81、82 的 error=11 / 12 / 13 | 分别按保存失败、保存确认超时、扫描阶段超时处理 |
| 81 / event=10 / error=00 | 仍是旧拍摄点到位事件，无错误 |
| 旧 00~0F 错误 | 含义及处理不变 |
| 未分配错误码、错 LEN、错 CRC | 不推进业务，不降级为无错误 |
| 本地安全测距超时 | 超时检测与安全停止仍有效，不显示或发送为 SCAN_ID_MISMATCH |

## 8. 离线核对报文

以下 scan_id=100，仅用于编码 / CRC 核对，不用于重复启动现场机械臂。

```text
START SEQ=14:
A5 5A 01 07 14 06 01 00 64 00 00 00 7F EC

PHOTO_DONE SEQ=15:
A5 5A 01 07 15 06 02 01 64 00 00 00 83 D3

QUERY SEQ=16:
A5 5A 01 07 16 06 00 00 64 00 00 00 FF E4

FRONT_OBSERVE_READY SEQ=14:
A5 5A 01 82 14 0A 08 0B 07 01 01 00 64 00 00 00 FF B6
```

以下用于错误字段解码回归，均为 SEQ=14 的合成离线样例；不代表 MCU 会在每个业务阶段发送全部组合，也不是运动命令：

```text
FE error=10:
A5 5A 01 FE 14 01 10 68 70

81 COMMAND_ACK 拒绝，error=10:
A5 5A 01 81 14 03 21 00 10 74 50

82 SCAN_ACK 拒绝，error=10，scan_id=100:
A5 5A 01 82 14 0A 01 00 00 01 00 10 64 00 00 00 DD 5D

81 CAPTURE_POINT，event=10 / value=01 / error=00:
A5 5A 01 81 14 03 10 01 00 25 C3
```

上述四份错误 / 事件样例的 CRC、长度及错误字段已用当前 PC 协议解析器离线核验通过；这不等于 Jetson 错误名称映射已经修复，Jetson 仍需自行执行第 7.4 节回归。

## 9. 双方下一步

- Jetson 先按第 2~5 节实现扫描、保存应答和最终正视门控，使用本轮 ID / SEQ 关联，拒绝迟到包恢复已取消流程。
- Jetson 的 error_code=10 冲突已确认，按第 7.3 节完成常量、显示、业务判断、FE / 81 / 82 解码和测试修改，并回传第 7.4 节回归结果。
- 目标业务另冻结一张包含 TYPE、LEN、偏移、枚举、应答、重发和超时的表；同时确定医生许可 TTL、单次消费及 P000 再武装规则，再修改 MCU 和 Jetson 代码。
- PC 离线测试、上板握手和实际路径验收分别记录。未完成新目标 / 许可双端实现及验收前，不开放扫描后的新激光执行入口。

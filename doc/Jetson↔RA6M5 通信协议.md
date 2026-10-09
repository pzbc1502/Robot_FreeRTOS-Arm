# Jetson <-> RA6M5 扫描、选痣与安全执行通信协议

修订日期：2026-10-09。传输：UART 115200 8N1。现有帧版本：`VER=0x01`。

业务修订名称：`FACE_SCAN_TARGET_V2`；本次流程修订标记：`AUTO_HOME_FRONT_20261009`。两者仅用于区分文档，不是已实现的线上能力标识或 schema 校验值。

本文为“旧字节基线 + 已冻结并接入的扫描协议 + 选痣业务待冻结规格”。扫描布局标记为 `SCAN_WIRE_20261009`，用户已确认采用本地 `07/82`；`VER=01` 不变。消息、动作、状态和错误表中的两位编号均为十六进制，例如 `82` 表示 `0x82`；第 7 章的逻辑名称尚无线上编码。

## 1. 范围与实现状态

### 1.1 本次业务依据

以用户确认的[项目演示.md](项目演示.md)第 9 章及[固定扫描规划](弧线连续扫描需求与实施规划.md)为同一业务依据：

```text
开始 -> HOME / 公共测距位 -> 本轮安全距离确认
  -> 左到正扫描 -> 中途正视真实停稳，FRONT_PHOTO_READY
  -> Jetson 取新图并实际保存 -> 本轮 PHOTO_DONE 被 MCU 接受
  -> 正到右扫描 -> 右端真实停稳，可报告 SCAN_DONE
  -> MCU 自动 HOME，真实校验后可报告 HOME_READY
  -> MCU 自动进入固定正式正视位，不等待 Jetson 新运动请求
  -> 最终真实到位并停稳，FRONT_OBSERVE_READY，保持且输出关闭
  -> Jetson 候选可用后开放数字人选痣，绑定具体目标，不产生运动
  -> 当前新画面复核所选同一颗 -> 原视觉状态机稳定对准
  -> 医生再次确认本目标 -> MCU 检查许可及 P000 -> 输出
  -> 输出关闭 -> 下一颗重新绑定 / 正常完成返回
```

左 / 正 / 右是扫描或观察姿态，不是候选身份。R01 等是界面短编号，不能写成 `point_id=1` 代替目标选择。

预置 GLB 只承担展示和选择，不要求每轮重新生成整脸网格。供执行选择的标记须与真实目标可靠对应；预置展示与本轮检测如实区分。GLB 顶点、显示坐标和扫描进度不能直接作为运动坐标。

**右端 → HOME → 固定正视由 MCU 在原扫描请求内自动完成。**Jetson 不再发送 HOME / GOTO / SELECT / APPROACH_REQUEST 推进这段运动，也不需要对中间到位逐步授权。照片保存成功确认是继续第二段的必要握手；最终正视通知是进入选痣阶段的运动放行依据，不是输出许可。

途中视频、检测和归档可由 Jetson 保留，但全脸分析完成、三维重建、逐帧 FK、手眼外参或痣基座 XYZ 均不是这条固定运动链的前置条件。MCU 内部仍复用原运动执行和安全检查。

### 1.2 当前事实与待实现能力

| 内容 | 当前状态 | 本文处理 |
| --- | --- | --- |
| TYPE `01~06`、`81`、`FE`，旧逐点 / SELECT / 定靶 | 已有固件实现 | 原长度和旧模式字节含义保持 |
| `robot_send_scan_segment_event()`、`robot_scan_result_get()` | 两段运动层已实现并通过离线检查 | 不是串口命令，不含保存握手、自动 HOME → 正视或目标选择 |
| `SCAN_CTRL=07/LEN=6`、`SCAN_STATUS=82/LEN=10` | 已冻结、接入 MCU 和 PC 源码；离线验证通过，未烧录联调 | 最终正视事件 `08`，新增状态 `0A/0B/0C`，完整表见第 6 章 |
| 目标绑定 / 复核、绑定视觉、医生单次许可 | 新接口及 MCU 门控未实现 | 第 7 章规定逻辑语义，TYPE / LEN / 偏移待双方冻结 |
| 真实关节 / 位姿遥测和图像时间关联 | 未形成已验收新接口 | 不作为固定扫描与固定正视的前置，后续研究另行立项 |
| FK / 相机安装变换 | 按队友说明未验收，`calibrated=false`、`T_tool_camera=null` | 继续如实记录，不虚报标定；本轮不实现 XYZ 自动到靶 |
| Gemini 新选痣入口输出 | 按队友文档仍禁用 | 双端实现并验收全部条件后才可启用 |

本次修改 MCU 和 PC 调试源码，接入扫描、保存等待、自动 HOME → 正视以及查询 / 去重。未修改 Jetson、FK/DH、视觉参数或相机标定；未烧录、连接端口、授权运动或输出。新扫描在独立 `FLOW_SCAN_READY` 保持，旧 SELECT/TARGET 启动被拒绝；第 7 章未完成前不能进入新输出链。旧独立模式保留，不代表其具有新目标身份或医生许可。

### 1.3 旧扫描草案撤回范围

原 `SCAN_DEMO_V1` 的 `HOME_READY + flags=0F -> 旧 SELECT -> 旧 TARGET / P000` 不适用于本版。

- 新 HOME 到位只是中间节点；MCU 自动继续固定正视，不等待候选分析或 Jetson 新指令，不放行选痣或 TARGET 输出。
- 正视保存成功只允许第二段，不证明侧脸候选已识别、去重和归档。
- 原 `SELECT_READY`、`SELECT_ALLOWED`、`SELECT_CONSUMED` 的扫描业务解释撤回；第 6 章为修订规格。
- HOME 后等待 SELECT / 选痣后再伸出，以及必须先提交 TARGET_POSITION / POSITION_STAGED / 基座 XYZ 的方案撤回。
- 旧草案 FRONT_READY 明确为中途拍照就绪，逻辑名统一为 FRONT_PHOTO_READY；最终观察位另用 FRONT_OBSERVE_READY，禁止混用。
- 旧 07 / 82 HEX 示例撤下，避免另一端继续按 `flags=0F` 实现。
- 双方须确认同一业务版本；只看到 `VER=01` 或 ACK 不证明兼容。
- 本次确认采用 07 / 82；队友旧草案若占用这些值，须按本表更新，不混发两套布局。
- 新接口未冻结前，不发送自拟 TYPE，不扩展旧五字节视觉，不借用 `CAPTURE action=04`。

## 2. 传输与收发规则

### 2.1 数据表示

| 项目 | 值 |
| --- | --- |
| UART | `115200`、`8N1`、无流控 |
| 多字节整数 | 小端 |
| 最大 payload | `32 bytes` |
| 心跳周期 / MCU 超时 | `100~200 ms / 600 ms` |
| 当前视觉有效期 | `500 ms`，建议新结果每 `100~200 ms` |
| 相机采集设置 | 对齐 RGB / Depth `848x530`，设置 `30 FPS` |

FPS 设置不等于可靠目标结果频率，须记录新帧频率及采集到发送延迟。推理、保存和 Qt 切页不能阻塞独立心跳。

### 2.2 帧格式与 CRC

```text
SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
A5   5A   01  xx   xx  nn  ...          xx       xx
```

总长 `8+LEN`，LEN 为 0~32。Jetson 请求 / 样本使用 SEQ=01~FF，00 保留给 MCU 无请求关联的异步故障。

CRC16/MODBUS：初值 FFFF，多项式 A001，从 VER 到 payload 末尾，不含帧头及 CRC；低字节先发。

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if not 0 <= len(payload) <= 32:
        raise ValueError("payload too long")
    if not 0 <= msg_type <= 255 or not 0 <= seq <= 255:
        raise ValueError("invalid header")
    body = bytes([1, msg_type, seq, len(payload)]) + payload
    return b"\xA5\x5A" + body + crc16_modbus(body).to_bytes(2, "little")
```

接收：帧头 -> 版本 / 长度 -> 收齐 -> CRC -> TYPE / 固定长度 / 参数 -> 业务门控。CRC 错误或不完整直接丢弃，不动作；通过 CRC 的非法命令按错误规则拒绝。支持分包、粘包和连续多帧。

### 2.3 SEQ、幂等与重连

- 所有发送通过同一 TX 队列 / 锁；线程安全生成非零全局 SEQ，新样本用新 SEQ，跳过活动控制请求保留的编号。
- 同一请求重发原 TYPE、SEQ、payload、CRC；不能换 SEQ 试图重启动作。
- ACK 等待 300 ms，最多额外重发 3 次。接受后停止重发 START，等待业务状态；状态丢失用只读查询。
- 旧控制缓存为 8 项 / 3000 ms，不足以保护长扫描、目标和许可。新增业务保留活动上下文、已接受决定及必要终态，不能因旧缓存淘汰重复执行。
- SEQ 只有 8 位，仅关联请求，不等于扫描、候选、目标、帧或许可编号。
- 同 TYPE+SEQ 改 payload 返回 SEQ_CONFLICT。同业务编号重发不得重新运动、重新消费许可或续期。
- MCU 重启、重连 / 切换客户端、重新开始、取消或故障均撤销旧执行上下文；清旧 TX 队列，先查询能力 / 状态，再显式建立新上下文，不自动续跑。
- 长动作状态带完整业务编号和请求关联，不拿心跳 SEQ 关联运动完成。
- ACK 不证明到位、保存完成、身份一致、对准或输出许可；不能靠延时放行。
- 安全停止不受普通 BUSY 限制；查询无运动，查询接受也不代表任务恢复。

## 3. 已实现消息基线：字节含义保持

| TYPE | 方向 | 名称 | LEN | payload |
| --- | --- | --- | ---: | --- |
| `01` | Jetson -> MCU | HEARTBEAT | 4 | `uint32 tick_ms` |
| `02` | Jetson -> MCU | TARGET_CTRL | 1 | `uint8 enable` |
| `03` | Jetson -> MCU | VISION_ERROR | 5 | `int16 dcx + int16 dcy + uint8 valid` |
| `04` | Jetson -> MCU | CAPTURE_CTRL | 2 | `uint8 action + uint8 point_id` |
| `05` | Jetson -> MCU | SAFE_DISTANCE | 3 | `uint16 distance_mm + uint8 valid` |
| `06` | Jetson -> MCU | WORKFLOW_CTRL | 1 | `uint8 action` |
| `81` | MCU -> Jetson | STATUS | 3 | `uint8 event + uint8 value + uint8 error_code` |
| `FE` | MCU -> Jetson | ERROR | 1 | `uint8 error_code` |

旧模式供独立回归。新 Gemini 选痣模式必须显式区分，不能借旧 SELECT / TARGET / VISION 绕过新目标绑定和医生许可。

### 3.1 HEARTBEAT / WORKFLOW_CTRL

只有合法 HEARTBEAT 刷新全局心跳；视觉、测距或业务命令不能代替。tick_ms 为 Jetson 单调计时，不天然等于 MCU 时间。

| WORKFLOW action | 名称 | 原行为 |
| --- | --- | --- |
| `01` | START_MEASURE | 关输出、清上轮、soft_reset、公共测距位 |
| `02` | FINISH_RETURN_HOME | 关定靶和输出、soft_reset 校验后回 IDLE |
| `03` | ABORT_HOLD | 关输出、中止并保持，不自动 HOME |

新业务仍复用原 START / FINISH / ABORT，但后续 MCU 实现须同步失效扫描、目标及医生许可。扫后内部 HOME 不使用串口 FINISH_RETURN_HOME 来推进，不进入整轮 IDLE，也不清掉自动正视所需的有效本轮上下文。整轮结束返回和扫描中间 HOME 是不同语义。

### 3.2 SAFE_DISTANCE

valid=01 表示有效实测毫米值；00 忽略 distance。保留“一次测距、稳定后锁存”，本阶段不单方增加持续测距。

1. 等 MEASURE_POSITION_READY 后取新深度，每 100~200 ms、新非零 SEQ 发送。
2. 当前源码阈值 100 mm，连续 3 个不同非零 SEQ 的有效合格样本才锁存；无效 / 过近清计数。
3. 以 WORKFLOW DISTANCE_SAFE_LATCHED 放行；单条 SAFE_DISTANCE value=1 不等于锁存。
4. 过近时 +Y 每步 20 mm，最多 8 步、累计位移 160 mm。等 RETREAT_STEP_READY 后重新实测，不复用撤离前缓存、不按位移推算距离。
5. 每步等测距 2000 ms；撤离后合格只报 RETREAT_DONE_WAIT_RESTART，用户重新开始；无有效结果或超限进入 FAULT_HOLD。

需考虑相机 / 工具偏置、外形、盲区、曲面和深度质量；轴向深度不等于最小三维净距。初始锁存不证明整段安全，固定模型扫描 / HOME / 接近路径仍须验证。模型 / 底座移动、相机失效或场景不符时撤销本轮并停止。

数值来自当前 APP/robot_workflow.h，不是本次调参，须核对烧录构建；不据此宣称真人 / 医疗安全验收。

### 3.3 CAPTURE_CTRL：旧观察姿态接口

| action | point_id | 原含义 |
| --- | --- | --- |
| `01` | `1/2/3` | GOTO：左 / 正 / 右逐点采集 |
| `02` | `0` | 旧采集 HOME |
| `03` | `1/2/3` | SELECT：固定观察姿态 |
| `04` | `0` | 旧调试 CURRENT，新正式业务禁用 |

旧 SELECT 没有候选身份和目标会话，不能称为新“选痣”。

新模式由 MCU 内部复用正式 HOME 和正视 point_id=2 执行器。正视包含原基础定位及全部关节步骤，不能只设置三个角度。Jetson 不发旧 SELECT 推进这段链，也不能先发旧 SELECT 再旧 TARGET 绕过新门控。新扫描不得伪造旧 `capture_done_mask=07`。

### 3.4 TARGET_CTRL / VISION_ERROR：旧定靶接口

TARGET enable：01=START，00=STOP_HOLD。START 等 ACK、TARGET_CTRL_ON、TARGET_READY；STOP 关输出并保持，不自动 HOME。

旧 VISION 定义不暗改：

```text
dcx = target_center_x - image_center_x
dcy = target_center_y - image_center_y
```

右 / 下为正；valid=0 忽略误差、按视觉无效处理。机械方向由 MCU 映射，Jetson 不临时反号；不能将 (0,0,valid=0) 当对准，也不能无目标发送 (0,0,valid=1)。

此格式不含目标身份，仅用于旧模式。新选痣采用第 7.5 节绑定视觉，不能自动跟画面中另一颗痣。原图像中心误差不一定等于新激光瞄准参考误差，二者不能静默互换 TYPE=03 语义。

## 4. 已实现状态与错误

### 4.1 STATUS：TYPE=81，LEN=3

| event | 名称 | value |
| --- | --- | --- |
| `01` | TARGET_READY | 01 等待视觉 |
| `02` | ALIGN_DONE | 01 对准和二次确认通过 |
| `03` | OUTPUT | 01 / 00 开 / 关 |
| `04` | TARGET_CTRL | 01 / 00 启用 / 关闭或拒绝 |
| `05` | HEARTBEAT_ALIVE | 01 可选响应 |
| `06` | SAFE_DISTANCE | 01 / 00 合格 / 过近，非锁存证明 |
| `07` | VISION_STATE | 01 / 00 有效 / 丢失 |
| `10` | CAPTURE_POINT_READY | 1 / 2 / 3 旧拍摄点真实到位 |
| `11` | CAPTURE_HOME_READY | 01 旧采集 HOME 校验通过 |
| `12` | SELECTED_VIEW_READY | 1 / 2 / 3 旧选中姿态到位 |
| `20` | WORKFLOW | 见下表 |
| `21` | COMMAND_ACK | 01 / 00 接受 / 拒绝 |

| WORKFLOW value | 名称 |
| --- | --- |
| `00` | IDLE |
| `01` | START_ACCEPTED |
| `02` | MEASURE_POSITION_READY |
| `03` | DISTANCE_SAFE_LATCHED |
| `04` | RETREAT_DONE_WAIT_RESTART |
| `05` | RETURN_HOME_DONE |
| `06` | ABORTED_HOLD |
| `07` | FAULT_HOLD |
| `08` | RETREAT_STEP_READY |

旧 COMMAND_ACK 用于 WORKFLOW / CAPTURE / TARGET，关联原请求 SEQ。接受 value=1/error=0，拒绝 value=0/error!=0，必须同时检查 event/value/error。

旧 ALIGN_DONE / OUTPUT / SELECTED_VIEW_READY 不含完整新目标身份，新模式不能单独据此放行，须有第 7.7 节绑定状态。

### 4.2 ERROR：TYPE=FE，LEN=1

异步故障 SEQ=00；未知 TYPE / 非法参数也可原请求 SEQ 拒绝。旧固件收到新 TYPE 的错误不算能力探测成功。

| code | 名称 |
| --- | --- |
| `00` | NONE |
| `01` | VERSION_MISMATCH |
| `02` | UNKNOWN_TYPE |
| `03` | INVALID_PARAM |
| `04` | POSE_INVALID |
| `05` | BUSY |
| `06` | MOTION_FAILED |
| `07` | HEARTBEAT_TIMEOUT |
| `08` | SAFETY_INPUT |
| `09` | SAFE_DISTANCE_TOO_CLOSE |
| `0A` | VISION_LOST |
| `0B` | SOFT_RESET_FAILED |
| `0C` | INVALID_STATE |
| `0D` | SEQ_CONFLICT |
| `0E` | TARGET_GATE_DENIED |
| `0F` | MOTION_ABORTED |

扫描错误已冻结并加入头文件：10=SCAN_ID_MISMATCH，11=PHOTO_SAVE_FAILED，12=PHOTO_CONFIRM_TIMEOUT，13=SCAN_STAGE_TIMEOUT；旧 00~0F 不变。

新业务还须区分最终正视未就绪、目标不匹配、身份歧义、许可过期 / 撤销 / 消费；编号待冻结，不挪用旧含义。本轮不把 XYZ 标定和逐帧位姿配对错误加入必需链路。

## 5. 新业务身份与坐标约束

### 5.1 身份层次

| 逻辑字段 | 建议类型 | 含义 |
| --- | --- | --- |
| scan_session_id / 扫描报文 scan_id | uint32 非零 | 同一轮扫描，不建两套编号 |
| candidate_id | uint32 非零 | 本轮真实候选；R01 字符串只在界面 |
| target_session_id | uint32 非零 | 一次选痣 / 对准；换痣或重新定位建新会话 |
| target_revision | uint32 非零 | 目标记录版本；身份等关键关联变化更新 |
| observe_pose_id | uint8 | 本轮固定正视，可作为日志元信息；不是候选编号或运动请求 |
| frame_id / vision_sample_id | uint32 | 新图像 / 视觉样本关联，不是 SEQ |
| permit_id | uint32 非零 | 单次医生许可及撤销代次 |

以上类型是建议，不是已冻结 payload。目标控制、视觉、许可和状态均关联：

```text
target_key = (scan_session_id, candidate_id, target_session_id, target_revision)
```

一次一个活动目标。换痣先关输出、停止旧定靶并确认保持，撤销旧许可，再绑定新目标。重扫不能复用旧候选。目标绑定不产生新的运动；重复只重放接受结果，同编号改内容拒绝。

### 5.2 固定路线与像素控制边界

- 正式配方来自 APP/robot_capture.c：基础定位 auto 0 -130 -15；左 J1/J4/J5=70/330/86，正=90/0/83，右=115/35/85，单位为度。J2/J3 由基础定位确定，不用 HOME 角度代填。
- 右扫后 MCU 自动 HOME，再按同版正式正视配方伸出。当前完整五轴反馈和原到位结果是成功依据，不能靠命令角、延时或 ACK。
- Jetson 只提交当前所选目标的身份、有效性和像素误差，不提交痣基座 XYZ；模型点不能驱动运动。
- 固定路线不要求先做 FK 圆弧求解或手眼外参；MCU 原内部运动学、安全范围和限位仍有效，不改 DH / J1 映射。
- 固定正视无法可靠观察的目标不执行；侧脸回访、任意接近、真实三维到靶均不在首版。
- 真实固定路径、工作距离、像素瞄准参考和同目标复核仍须验收。无手眼矩阵不等于可以解除 Gemini 执行禁用或伪造 calibrated=true。

## 6. 扫描握手：07 / 82 冻结规格

2026-10-09 用户确认采用以下布局，已接入 MCU 和 PC 源码。Jetson 按此表实现保存应答及最终到位门控；不要使用旧 flags=0F。编译和离线测试不代替现场路径、真实拍照与双端联调验收。

### 6.1 SCAN_CTRL：TYPE=07，LEN=6

| payload 偏移 | 字段 | 类型 | 含义 |
| ---: | --- | --- | --- |
| 0 | action | uint8 | QUERY=00、START=01、PHOTO_DONE=02 |
| 1 | value | uint8 | QUERY/START 为 00；PHOTO_DONE 为 01 成功 / 00 失败 |
| 2~5 | scan_id | uint32 LE | 非零本轮；仅 QUERY 可为 0 |

- QUERY 只读，先 ACK 再 SNAPSHOT；0 查询当前 / 最近，未知非零编号拒绝，不套用另一轮证据。
- START 前 Jetson 已建立本轮、采集 / 记录就绪；MCU 查测距、心跳、故障 / 忙、起点及路径版本，接受后准备左视并执行第一段，不是即刻到位。
- PHOTO_DONE=1 只在匹配本轮 WAIT_PHOTO 首次接受，表示新正视关键帧及观测实际保存；再查正视真实反馈 / 安全，启动第二段一次。
- PHOTO_DONE=0 接受失败通知后终止保持，不继续右扫、不自动 HOME。
- LEN!=6 用原 ERROR / 请求 SEQ / INVALID_PARAM；合法长度非法参数用扫描 ACK 拒绝，不改变已有任务。
- 取消复用 WORKFLOW ABORT_HOLD，不增加任意关节或激光开命令。

START 前应保证中途取图、保存和应答任务可用。扫描中可展示、记录 RGB-D 和候选，不强制完成全脸分析。PHOTO_DONE 只确认中途新正视图实际保存；帧号、文件和质量留在 Jetson，不要求逐帧 FK。

### 6.2 SCAN_STATUS：TYPE=82，LEN=10

| payload 偏移 | 字段 | 类型 |
| ---: | --- | --- |
| 0 | event | uint8 |
| 1 | state | uint8 |
| 2 | flags | uint8 |
| 3 | request_action | uint8 |
| 4 | result | uint8，01 正常 / 接受，00 失败 / 拒绝 |
| 5 | error_code | uint8，正常 00 |
| 6~9 | scan_id | uint32 LE |

| event | 名称 | SEQ / action | 含义 |
| --- | --- | --- | --- |
| `01` | ACK | 当前请求 / 当前 action | 接受或拒绝，不是到位 |
| `02` | SNAPSHOT | 查询或重发请求 / 对应 action | 当前上下文，不运动 |
| `03` | FRONT_PHOTO_READY | 最初 START / 01 | 第一段正视真实停稳，等待照片保存 |
| `04` | PHOTO_ACCEPTED | PHOTO_DONE / 02 | 保存接受，第二段即将 / 正在执行 |
| `05` | SCAN_DONE | 最初 START / 01 | 可选进度：右端真实停稳，MCU 自动 HOME，不放行选痣 |
| `06` | HOME_READY | 最初 START / 01 | 可选进度：扫后 HOME 真实校验通过，MCU 自动继续正视，不要求应答 |
| `07` | TERMINATED | 最初 START / 01 | 中止 / 故障，不自动返回 |
| `08` | FRONT_OBSERVE_READY | 最初 START / 01 | 必需最终通知：本轮扫后 HOME 已验证，当前固定正视真实到位、停稳保持且输出关闭 |

| state | 名称 | 含义 |
| --- | --- | --- |
| `00` | NONE | 无扫描上下文 |
| `01` | PREPARING | 基础位置 / 左视准备 |
| `02` | LEFT_TO_FRONT | 第一段 |
| `03` | WAIT_PHOTO | 正视保持等待本轮保存确认 |
| `04` | FRONT_TO_RIGHT | 第二段 |
| `05` | RETURNING_HOME | 右端完成，准备 / 执行正常 HOME |
| `06` | 保留 | 旧 HOME_HOLD 等待选痣分支撤回，不能作为本轮正常终态 |
| `07` | 保留 | 原 SELECT_CONSUMED 撤回，不用于新目标选择 |
| `08` | ABORTED_HOLD | 已取消 |
| `09` | FAULT_HOLD | 故障保持 |
| `0A` | RETURNING_FRONT | HOME 成功后执行正式正视；尚不能选痣 |
| `0B` | FRONT_OBSERVE_HOLD | 当前最终正视已验证并保持；不是输出许可 |
| `0C` | SCAN_CHAIN_COMPLETED | 已正常结束或后续转入目标业务，初始正视就绪不再代表当前位置；查询不得重放为可选就绪。当前代码在接受 FINISH_RETURN_HOME 时设置 |

| flags | 名称 | 含义 |
| --- | --- | --- |
| bit0 / `01` | PHOTO_SAVED | 本轮正视保存确认被接受 |
| bit1 / `02` | SCAN_FINISHED | 两段完成、右端真实停稳 |
| bit2 / `04` | HOME_VERIFIED | 本轮扫后 HOME 曾校验，不代表后续仍在 HOME |
| bit3~7 | 保留，必须为 0 | 原 SELECT_ALLOWED 撤回，不是目标 / 输出许可 |

历史完成位组合：WAIT_PHOTO=00，第二段=01，返回 HOME=03，HOME 成功后的自动正视阶段及最终就绪=07。**flags=07 不能单独证明最终正视到位，必须同时检查当前 event / state、result、error 和本轮关联。**不把 bit3 偷换为最终到位；旧 flags=0F 不合法。

取消 / 故障清 flags 和当前正视就绪资格；终态快照 result=0 并给原因。查询接受不等于恢复；拒绝错误编号回显请求编号，state=NONE/flags=0，不挂另一轮证据。正常最终就绪要求 result=1/error=0、state=FRONT_OBSERVE_HOLD 且当前资格有效。

HOME_READY 如发送，其 state 应表达正在转入 RETURNING_FRONT，不表示 HOME_HOLD 等待。中途 FRONT_PHOTO_READY 与最终 FRONT_OBSERVE_READY 不得共用事件编号。ROBOT_SCAN_RESULT_FRONT_READY 是现有 C 运动层结果名，不是最终观察就绪报文。

### 6.3 幂等与完成边界

- Jetson 持久化单调递增的非零 uint32 scan_id；新轮增加，重发沿用原 ID / SEQ。MCU 在本次启动内保存最近 ID 和已接受 ID 高水位，新 START 必须大于高水位；历史较小 ID 拒绝。MCU 断电不持久化上下文，重启 / 断连后清客户端待发动作并重新测距，禁止自动重发旧启动。ID 耗尽须停止并协商，不回绕。
- 同本轮重复 START 回接受记录 / 当前快照，不再准备左视，不更改 start_seq。
- PHOTO_DONE 已接受后重复只重放记录 / 当前快照，不再右扫、HOME 或进入正视；改写已接受结果拒绝。
- 保存等待不阻塞心跳、反馈、取消和安全处理；位置偏离或反馈失效不能旧姿态续扫。
- SCAN_DONE 只证明两段运动完成，HOME_READY 只证明曾经过 HOME；两者不是最终正视就绪，不放行选痣或定靶。
- 右端 → HOME → 固定正视由 MCU 自动串接，每步成功后才能下一步；不等待额外运动命令、分析结果或对中间通知的确认。心跳、安全中止和真实反馈继续独立处理。
- 最终 FRONT_OBSERVE_READY 不要求 Jetson 回一个运动许可。丢失时查询本轮当前状态；只读快照不重复 HOME、伸出或恢复失效资格。
- 返回当前就绪快照前，MCU 仍须确认本轮未失效、当前阶段与反馈合格；保存历史终态供诊断不等于仍具备执行就绪资格。Jetson 发现阶段回退或错误即清本地就绪，禁止靠晚到包重新放行。
- 开始视觉伺服、其他运动、取消、复位或故障后，初始正视资格失效，查询报告当前阶段，不把历史最终成功当作仍在固定正视。
- 正常采集 HOME 保留必要本轮测距上下文，不进入整轮 IDLE；整轮 FINISH 撤销本轮。
- 取消 / 故障 / 超时后迟到保存、HOME 或最终正视消息不恢复流程。
- 运动 API 本身不自动返回；现已由 workflow 消费结果，自动串接 HOME 和正式正视。不向 Jetson 索要额外 HOME / SELECT。HOME 执行经原 capture 层清除基础姿态缓存，回正视时重新基础定位，避免从 HOME 跳过伸出。
- START 和已接受 PHOTO_DONE 的去重记录保留到本轮结束，不使用旧命令 3 秒缓存。对同轮改写保存结果返回 SEQ_CONFLICT；查询不会重复运动。事件归属仍用最初 START SEQ，照片接受使用 PHOTO_DONE SEQ。
- WAIT_PHOTO / 最终保持每 300 ms 刷新五轴反馈；当前就绪查询也刷新反馈。缺失、偏离、忙、限位 / 心跳 / 中止均不复用旧到位资格。该项是关节反馈检查，不是新增连续深度测距。
- 当前 MCU RX 为 8 条扫描命令 FIFO；满时回扫描 ACK/result=0/error=BUSY，回显请求 ID、state=NONE/flags=0。控制不覆盖尚未消费的扫描请求，也不刷新心跳。
- 新扫描最终保持不转入旧 FLOW_SELECTED_VIEW；目标绑定 / 复核和医生许可未接入前，旧 CAPTURE SELECT 和 TARGET_START 均拒绝。允许显式 WORKFLOW FINISH_RETURN_HOME 正常结束，或 ABORT_HOLD 停止保持。

### 6.4 字节核对示例

以下只用于离线核对，`scan_id=100`，不能反复拿固定编号启动新轮。CRC 覆盖 VER 至 payload，小端附加：

```text
START       SEQ=14: A5 5A 01 07 14 06 01 00 64 00 00 00 7F EC
PHOTO_DONE  SEQ=15: A5 5A 01 07 15 06 02 01 64 00 00 00 83 D3
QUERY       SEQ=16: A5 5A 01 07 16 06 00 00 64 00 00 00 FF E4
最终正视     SEQ=14: A5 5A 01 82 14 0A 08 0B 07 01 01 00 64 00 00 00 FF B6
```

负例：LEN 不等于 6、SEQ=0、START ID=0、保留 state=06/07、flags=0F、错 CRC、旧轮保存或改写成功为失败，不得放行。实际 C UART / workflow 和 PC 测试见第 10 章。

## 7. 新选痣接口：逻辑规格，字节待冻结

**本章名称不是已分配 TYPE。**必要语义可在共同冻结时合理合并，但不能擅自扩展旧 02/03/04/81 长度。目标关联和单次许可用于防错目标、错会话，不引入 XYZ 自动到靶。MCU 复用原视觉状态机与安全实现，不重写视觉控制算法。

### 7.1 能力 / 业务版本查询

分别报告业务版本、扫描及自动 HOME → 正视支持、目标关联 / 复核、绑定视觉、医生许可支持和启动 / 链路代次。按实际能力逐项报告，不要求 features 恰好为某个包含未实现三维模块的全量掩码。

支持 07/82 只能证明扫描扩展，不能推断选痣和许可支持；新旧模式显式协商，新模式拒绝旧 TARGET START / 旧 VISION 驱动输出。旧固件未知 TYPE、无能力回应或业务版本不符时禁用新入口，不自动降级运动。

重启 / 链路代次改变使旧上下文失效。能力接口未实现前使用构建版本核对并禁用新执行，不能声明已有线上协商。

### 7.2 候选可用性：Jetson 本地门控

完整候选档案、图片、已有 GLB 和模型标记对应关系留在 Jetson。可执行候选必须能在当前固定正视新画面可靠复核；全脸重建、侧脸融合及所有候选分析完成不成为 MCU 的扫描 / 返回前置。

旧草案的 SCAN_RESULT / DATA_READY 不作为本轮必需消息。若双方另保留归档诊断，关联本轮且只记录事实，不要求 MCU 等它才 HOME、进入正视或报告最终到位，也不自动触发运动。

候选身份更新不能静默替换已绑定目标；先撤销许可并停止旧定靶，再更新版本 / 新建会话。只扫描过某区域不等于该区域没有痣，预置标记不冒充本轮检测。

Qt 开放本轮首次执行选痣的最低条件：

```text
可选候选与模型 / 列表对应可靠，候选证据可用
&& 匹配本轮 FRONT_OBSERVE_READY 或有效的当前就绪快照
&& MCU 当前 state=FRONT_OBSERVE_HOLD，result=1，error=0
&& 未取消 / 故障 / 换场景
&& 当前链路和固定路径验收条件有效
```

模型可提前浏览，但不提前提交执行目标。MCU 已到正视而候选尚未就绪时保持、输出关闭；无候选不启动定靶，可显式正常结束返回。候选就绪也不能代替真实正视到位。

### 7.3 到位证据与后续遥测边界

本轮由 MCU 内部使用完整、新鲜的 J1~J5 反馈及原到位检查完成运动验证，再发送业务状态。Jetson 不需要每帧接收 FK、工具矩阵或相机位姿；文本 read_all 可用于台架诊断，不作为自动业务协议。

- 正视中途拍照通知后机械臂保持，Jetson 丢弃旧缓存并采集新图；到位通知不是曝光时刻，不假称已实现时钟同步。
- 最终正视成功须完成 HOME 后的基础定位和正视关节步骤，五轴检查通过且停稳，恢复原姿态有效状态及正视标识。
- MCU 定靶启动仍独立检查当前反馈、距离锁存、心跳、限位 / 急停、故障和运动忙；过去的到位包不是永久许可。
- 若以后做真实多视角融合 / XYZ 回访，再单独定义 POSE_FEEDBACK、可靠 FK、手眼变换、采样窗口和曝光关联，不将该研究协议带入本轮必需链路。

未验收 FK 或外参继续标记未验收，不能因为本轮不上传矩阵就声称标定完成。

### 7.4 TARGET_BIND：最终正视到位后绑定，不运动

必要逻辑信息为 target_key、绑定 / 清除动作和请求关联。固定正视是已完成的业务阶段，不是选痣报文里的任意运动模式；不带痣 XYZ、calibration_id、position_payload_crc32 或姿态数组。

本轮唯一模式是“MCU 自动固定正视已就绪 → 目标绑定 → 同一颗复核 → 像素对准”。APPROACH_REQUEST、TARGET_POSITION、POSITION_STAGED、VERIFIED_BASE_TARGET 均不纳入本轮，也不要求医生选左 / 正 / 右观察位。

MCU 接受首次绑定须独立检查本轮保存、两段扫描、HOME 已验证和当前最终正视就绪；不信任 Qt 按钮，不伪造 capture_done_mask=07。绑定成功仅记录所选目标、清旧视觉 / 对准 / 许可，不启动移动或输出。

绑定应答只表达接受 / 拒绝和对应 target_key；机械臂实际到位已经由此前 FRONT_OBSERVE_READY 证明，不再等待一个由选痣触发的 TARGET_POSITION_READY。最终就绪关联 scan_id 和扫描 START，而目标状态从绑定时才关联 target_key。

同一绑定重发不清掉正在进行的合法业务，也不重新授权或运动；同一编号改内容拒绝。换痣先确认旧输出关闭及旧定靶停止。视觉修正后不能拿初始正视通知当当前位置；若需要恢复正视，应另行明确已验证的返回 / 再准备流程，本次首次自动链不等于换痣自动复位。

MCU 不从候选编号猜空间位置，不接收 GLB 点、任意低层关节 / 速度 / GPIO 来替代此业务。

### 7.5 TARGET_REVERIFY / TARGET_VISION：同目标复核和新视觉

必要字段：

- REVERIFY：target_key、新图像编号、同一目标确认 / 歧义 / 丢失、有效性。
- VISION：target_key、新帧编号或可校验样本关联、dcx/dcy、valid、必要质量 / 采集年龄。

顺序：FRONT_OBSERVE_READY → 数字人选痣 / 绑定 → 当前新 RGB-D 确认同一颗 → 复核应答 → 定靶启动 → ACK / 定靶开启 / TARGET_READY → 新视觉 → ALIGN_DONE。

新定靶控制逻辑名称为 TARGET_SESSION_CTRL，动作至少包括 START_ALIGN、STOP_HOLD 和只读 QUERY，全部绑定 target_key；TYPE / action 编码待冻结，不把 target_key 追加到旧 TYPE=02。新模式的安全 STOP_HOLD 仍先关输出、撤销许可并停止保持。

Jetson 负责视觉身份证据；MCU 校验键、阶段、新鲜度及安全条件。仅编号相同不能证明识别确实是同一颗。

新误差：

```text
dcx = 所选同一颗痣中心 x - 实测标定的激光瞄准参考 x
dcy = 所选同一颗痣中心 y - 实测标定的激光瞄准参考 y
```

统一处理图坐标，右 / 下为正；注明原始 / 显示方向、分辨率、缩放、距离 / 安装适用范围。180° 显示须恢复正确坐标关系，不临时反号。瞄准参考不一定是图像中心，不暗改旧 TYPE=03。

仅匹配当前键且有效的新帧计数。重复帧、新 SEQ 包旧图、错目标或迟到帧不续视觉有效期。身份不明 valid=0，撤销许可、停止 / 保持，不自动切到最近或最稳定另一颗。

对准及输出期间持续监测，P801 开时不运动。视觉有效期、容差、稳定帧数、增益和速度按实际烧录固件的 robot_target.h 执行，本次不改参数。ALIGN_DONE 不是医生许可，对准丢失后旧许可不恢复。

### 7.6 DOCTOR_PERMIT：单次确认、撤销与消费

必要字段：target_key、permit_id、授予 / 撤销、有效期；类型 / 范围 / 偏移待冻结。Qt 不控制 P801。

只在当前目标复核 / 稳定对准 / 安全成立后接受。MCU 从首次接受开始本地单调倒计时，重复不续期；有效期数值待定，不拿 Jetson 墙钟直接比较。

1. 同 permit_id 重复只重放决定，不重新授予 / 输出 / 延长 TTL。
2. 一次许可一次输出，在实际输出开启时原子消费；未满足 P000 不输出。
3. 医生确认和 P000 均不能单独开光。建议再武装需先松开再按下；边沿规则待双方 / 现场确认，不能长按导致重对准后自动输出。
4. 目标 / 对准丢失、视觉过期、换痣、重新定位、重扫、场景变化、重连 / 心跳超时、取消 / 复位 / 故障、松开 P000 或输出结束，旧许可失效。
5. 撤销 / 消费 / 过期后迟到授予不恢复。保留许可代次 / 失效记录，不能仅删变量后接受旧帧。
6. 撤销优先关输出，不因 BUSY 忽略；撤销 ACK 不代替 OUTPUT_OFF。无可靠回应执行安全停止和现场策略。
7. 视觉恢复可重对准，但必须医生新确认、新许可和硬件条件才能再输出。
8. 现单次连续输出硬上限 10000 ms 保留，不是默认治疗时长，不提供治疗剂量。

### 7.7 TARGET_STATUS / QUERY：带身份的状态

分别表达绑定接受、等待复核、复核接受 / 失败、定靶就绪、对准、许可有效 / 撤销 / 过期 / 消费、输出开 / 关、保持和失败原因。本轮选痣没有新的接近运动，不报虚构的移动中或 XYZ 到靶。

相关状态带 target_key、请求关联；许可状态带 permit_id。未知状态 / 身份不符不放行。查询 ACK 与快照分开，不重启运动或许可。

旧 81 若同步回传，仅作为兼容诊断；新 Qt 不单靠裸旧状态推进目标或输出。

## 8. 完整业务握手

新名称是逻辑时序，不是可发送 HEX；旧帧已实现，新业务待实施 / 冻结。

### 8.1 开始与测距

```text
独立心跳持续运行
Jetson -> MCU: 原 WORKFLOW START_MEASURE
MCU -> Jetson: COMMAND_ACK + START_ACCEPTED
MCU: 关输出、清旧轮，安全复位 / 到测距位
MCU -> Jetson: MEASURE_POSITION_READY
Jetson -> MCU: 到位后新 SAFE_DISTANCE，至少三份合格样本
MCU -> Jetson: DISTANCE_SAFE_LATCHED
```

过近按 RETREAT_STEP_READY / 新实测 / 等待重启，不直接扫描。

### 8.2 扫描、保存、自动 HOME 与自动正视

```text
Jetson: 建立 D，中途取图 / 保存 / 应答任务就绪
Jetson -> MCU: SCAN START(D)，SEQ=s
MCU -> Jetson: ACK(s)
MCU: 左视准备真实校验 -> 左到正
Jetson: 展示扫描，可记录 RGB-D、候选及来源

MCU -> Jetson: FRONT_PHOTO_READY(D,s)，中途正视真实停稳
Jetson: 保存新正视关键帧及观测
Jetson -> MCU: PHOTO_DONE(D,成功)，SEQ=p
MCU -> Jetson: ACK(p) + PHOTO_ACCEPTED(D,p)
MCU: 再查正视反馈，正到右
Jetson: 继续展示 / 记录，独立心跳不停

MCU -> Jetson: 可报告 SCAN_DONE(D,s)，右端真实停稳，只是进度
MCU: 经验证路线正常 HOME
MCU: 校验 HOME 真实到位
MCU -> Jetson: 可报告 HOME_READY(D,s)，转入 RETURNING_FRONT，只是进度
MCU: 不等待 Jetson 新命令，重建基础定位并执行正式正视配方
MCU: 校验最终正视真实到位、停稳、运动结束，恢复正确观察上下文
MCU -> Jetson: FRONT_OBSERVE_READY(D,s)，state=FRONT_OBSERVE_HOLD
               本轮保存 / 扫描 / HOME 完成，当前正视就绪，result=1/error=0
MCU: 保持且输出关闭，等待目标绑定与复核
Jetson: 核对本轮最终到位及候选可用性，开放数字人选痣
```

除持续心跳和原安全机制外，PHOTO_DONE 是本段推进运动所必需的业务回传。右端 → HOME → 正视不等待新 HOME / SELECT、归档通知或对进度的确认。最终通知不要求再回一个运动许可；丢失时查询当前状态，绝不重复伸出。保存失败 / 超时、任一运动故障、取消或心跳超时均关输出并停止保持，不继续自动链。

### 8.3 已到正视后的选痣、复核与对准

```text
前提: 已收到匹配本轮的最终正视就绪，且当前资格有效
Qt: 数字人 / 列表选具体一颗，不是选左 / 正 / 右
Jetson: 建立 K=target_key，保留原始证据
Jetson -> MCU: TARGET_BIND(K)
MCU -> Jetson: 绑定接受 / 拒绝，关联 K；绑定不产生运动
Jetson: 当前新 RGB-D 确认仍是所选同一颗
Jetson -> MCU: TARGET_REVERIFY(K,新帧,有效)
MCU -> Jetson: 复核接受 / 拒绝，关联 K
Jetson -> MCU: 新目标定靶请求(K)
MCU -> Jetson: ACK -> 定靶开启 -> TARGET_READY(K)
Jetson -> MCU: TARGET_VISION(K,新帧,误差,有效)，持续发送
MCU -> Jetson: ALIGN_DONE(K)
```

当前正视无效、目标不可见、身份有歧义或原安全条件不足时不启用定靶 / 输出，不自动找另一颗。无需 TARGET_POSITION、位置暂存 CRC、痣 XYZ 或手眼标定完成消息。

### 8.4 应答与放行判据

| 交互 | 谁需要应答 | 可以推进什么 | 不能推导什么 |
| --- | --- | --- | --- |
| SCAN START | MCU ACK，关联请求 | 接受后 MCU 准备左视和第一段 | 已到位、已保存或可以选痣 |
| FRONT_PHOTO_READY | Jetson 取新图保存后回 PHOTO_DONE | 匹配且接受成功后只执行一次第二段 | 最终观察位就绪 |
| PHOTO_DONE | MCU ACK / PHOTO_ACCEPTED，关联保存请求 | 第二段及之后的正常自动链 | 右端、HOME 或最终正视已经完成 |
| SCAN_DONE / HOME_READY | 不要求 Jetson 逐步授权 | MCU 自行继续下一内部阶段；Qt 更新进度 | 开放执行选痣或启动定靶 |
| FRONT_OBSERVE_READY | 无额外运动许可应答；丢失用只读 QUERY | 候选也可用时，Qt 开放选痣 / 绑定 | 目标身份已确认、已对准或可以输出 |
| TARGET_BIND / REVERIFY / START_ALIGN | MCU 分别应答，身份与阶段一致 | 复核通过后启用原视觉状态机 | 绑定 ACK 已对准、医生已经确认 |
| DOCTOR_PERMIT | MCU 应答并记录单次许可 | 所有原安全条件及 P000 通过后消费输出 | Qt 单方面直接开 P801 |

START 的 ACK 和 PHOTO_DONE 的 ACK 分别关联各自 SEQ；中途就绪、右端 / HOME 进度和最终正视通知关联最初 scan_id / start_seq。定靶 / 许可从选痣后才关联 target_key，不拿心跳 SEQ 或尚不存在的目标编号关联扫描到位。

### 8.5 医生确认、输出与结束

```text
医生核对当前末端真实画面，再次确认本颗
Jetson -> MCU: DOCTOR_PERMIT(K,新许可编号,有效期)
MCU -> Jetson: 接受 / 拒绝 + 许可状态
MCU: 当前目标 / 对准 / 通信 / 安全 / P000 / 运动冻结均通过
MCU: 原子消费许可，打开输出
MCU -> Jetson: OUTPUT_ON(K,permit_id)
Jetson: 持续监测和发送绑定视觉

松 P000 / 视觉失效 / 撤销 / 超时 / 故障 / 输出结束
MCU: 立即关输出、旧许可失效
MCU -> Jetson: OUTPUT_OFF(K,permit_id,原因)
```

下一颗先确认关输出、旧定靶停止保持，再建新目标 / 许可，不连续自动发射。正常结束用 FINISH_RETURN_HOME，等真实 RETURN_HOME_DONE；异常不自动 HOME。

### 8.6 心跳超时与中止

600 ms 无合法心跳：关 P801、中止、清距离 / 扫描资格 / 目标 / 许可、FAULT_HOLD，不自动 HOME。扫描保留 ID 和故障快照供查询，flags 清零；旧 ERROR / WORKFLOW 和扫描 TERMINATED 分别回传。恢复心跳不续跑。ABORT_HOLD 同样不自动恢复。

2026-10-09 停止确认修复：`WORKFLOW_CTRL=06 / LEN=1 / action=03` 接受后，先回匹配 SEQ 的 `81 / LEN=3 / 21 01 00`，仅表示请求接受。只有 CAN 停止应答、执行任务退出、队列清空和新五轴反馈稳定检查全部通过，才回同 SEQ 的 `20 06 00`；失败或超时回 `20 07 06`（FAULT_HOLD / MOTION_FAILED），不能假报 ABORTED_HOLD。具体采样、重发及恢复合同见[冻结字节表第5.1节](MCU冻结通信字节表与待确认项.md)。扫描 TERMINATED 或状态 ABORTED_HOLD 仅使扫描作废，不是该确认的替代品。

停止确认 MCU 预算2秒，新反馈每100毫秒采样，至少3次、覆盖200毫秒、相对首样本变化不超过0.10度；不是物理零速度认证。确认等待或失败时拒绝重新启动测距；普通故障恢复也须先用新 ABORT 请求取得成功确认，再显式开始新轮。重复中止不续期，不自动 HOME。

Jetson 必须由串口控制层独立完成“本轮失效 -> 优先发送 ABORT -> 有界重发 -> 等待匹配停止确认”，不能依赖 Qt 正常响应或只清本地资格。建议从首次中止发送起最多等待3秒，ACK不续期。发送不可信、失败或超时须停心跳、锁住重新开始并显示“停止未确认”，不得宣布已经停稳；硬件急停仍是独立保护。该 Jetson 修改和双端验收尚未完成，本次 MCU 修改不能代替它。

## 9. 超时、并发和数据拆分

### 9.1 分阶段预算

| 阶段 | 现状 / 建议 |
| --- | --- |
| ACK | 300 ms，最多额外重发 3 次 |
| ABORT 停止确认 | MCU2秒；Jetson建议自首次中止发送起3秒，重发 / ACK不续期；失败停心跳并显示停止未确认 |
| 旧 HOME / soft_reset | MCU 上限 45 s |
| 公共测距位移动 | 上限 15 s；含复位 START 总等待建议 62 s |
| 扫描准备 | MCU 60 s；包含基础 AUTO 和 J1/J4/J5 三步；PC 等 62 s |
| 每段扫描 | 默认运动 12 s、收敛 2 s、预算另留 4 s；workflow 30 s，PC 等 32 s |
| 正视保存等待 | MCU 15 s；Jetson 实际保存应在 10 s 内完成，PC 等 17 s |
| 扫后 HOME | MCU 45 s；PC 等 47 s |
| HOME 后自动固定正视 | MCU 60 s；包含基础 AUTO 与 J1/J4/J5；PC 等 62 s |
| Jetson 等最终就绪 | 覆盖准备、两段、保存、HOME 和自动正视；按阶段进度跟踪有限预算，收到 SCAN_DONE / HOME_READY 不结束等待 |
| QUERY 快照 | 建议 2 s，每秒最多一次，心跳不停 |
| 候选识别 / 页面准备 | Jetson 独立有限预算；不阻塞 MCU 自动 HOME / 正视 |
| 目标绑定 / 同一颗复核 | 独立预算，无新增接近运动；失败不默认换痣 |
| 医生确认 / TTL | 独立配置，数值待定；等待关输出并监测 |
| 视觉 / 输出上限 | 当前 500 ms / 10000 ms，不随其他等待放宽 |

超时停止推进并查询 / 安全停止，不当成功或无限延长。保存、HOME、最终正视、候选准备和许可不能共用一个定时器；最终通知超时也不能用 HOME 曾成功替代。

### 9.2 并发门控

一次一个运动业务；扫描准备至最终正视到位期间仅处理匹配保存、查询、心跳、安全中止，不插旧 GOTO/SELECT/TARGET。目标绑定本身不运动，开始视觉控制后初始正视就绪资格失效；其他运动不能混入目标业务，输出期间冻结运动。

视觉 / 遥测使用新样本策略，不积压旧误差。保存与分析后台执行，串口服务不阻塞。取消 / 新轮失效旧回调，迟到消息不能恢复按钮、流程或输出。

### 9.3 32 字节与分帧事务

建议 target_key 已占 16 字节，叠加样本编号、有效性、误差或许可前实际算长度，不擅自追加；本轮不叠加痣 XYZ。

拆帧需统一事务 / 会话编号、片序 / 片数、版本、完整提交；单片 CRC 不证明齐套。缺片、旧片、冲突、超时清暂存，完整校验后才改变运动 / 许可。图像、点云、模型、完整候选档案留 Jetson。

## 10. 双方冻结与实施

### 10.1 必须确认

| 决策 | 状态 |
| --- | --- |
| 两段扫描、中途正视保存后右扫 | MCU / PC 源码已实现、离线验证通过；Jetson / 真机未联调 |
| 右端 → HOME → 固定正视 | MCU 自动完成已确认，不等待 Jetson 新运动命令 |
| 最终 FRONT_OBSERVE_READY 后选痣 | 用户已确认；两种正视事件必须不同，中间进度不放行 |
| 选痣绑定不运动，当前同一颗复核后像素对准 | 已确认；移除 XYZ 暂存、手眼矩阵和观察姿态选择前置 |
| 正式姿态、工具与完整路径版本 | MCU 正式配方为候选，实物全程仍需验收 |
| 能力 / 业务版本、启动 / 链路代次 | 必须定义，编码待冻结 |
| 新 TYPE / 固定 LEN / 偏移 / 枚举 / 错误 | 扫描 07/82 及最终 08/0A/0B/0C 已冻结；第 7 章其余业务编码仍待确认 |
| 激光参考、图方向和距离适用范围 | 待标定验证，不暗改旧 TYPE=03 |
| 单次 TTL、撤销 / 消费 / 代次、P000 再武装 | 单次方向明确，数值 / 编码 / 边沿待确认 |
| 扫描长上下文、查询 / 重发和预算 | 已按第 6 / 9 章实现，字节向量和离线用例已提供；目标业务另行冻结 |

本轮固定正视 + 像素对准不承诺任意模型点 / 侧脸目标 XYZ 直达。真实三维控制和同步位姿遥测不在本轮冻结清单；以后另行规划，不重新加入本次硬性前置。

### 10.2 分工

| MCU / RA6M5 | Jetson / Qt |
| --- | --- |
| 安全准备、两段运动、中途保持、自动 HOME → 正视、最终到位 | 本轮启动、独立心跳、中途新图保存并回结果、处理最终通知 |
| 内部新鲜五轴反馈、完整配方结果、正视标识及有效状态 | 候选 / 模型对应及新画面复核，不要求接收逐帧 FK |
| 目标键与当前阶段检查，绑定不运动 | 最终到位后选具体痣，不再发送 HOME / SELECT / XYZ 接近 |
| 新视觉计数、许可消费 / 撤销、硬件最终裁决 | 同一颗复核、绑定视觉、再次确认 / 撤销 |
| 故障关输出保持、终态 / 查询和幂等 | 独立心跳、异常提示、清旧请求 / 回调 |

MCU 不替代视觉识别；Qt 页面确认不替代硬件门。分别记录构建、烧录、标定及整机日志。

### 10.3 验收

1. 冻结字节表及正 / 负编码向量，验证 CRC / 长度 / 小端 / 身份，不运动。
2. 验证候选、模型、所选 target_key 一致的软件链路，无运动 / 输出。
3. 核对正式固定配方及当前五轴反馈；不混用 PC 旧点，不假报 FK / 手眼已标定。
4. 固定模型无输出低速测两段、保存等待、自动 HOME → 正视；不发送额外运动命令也能最终就绪，中间状态不放行。
5. 无输出测最终到位丢包后 QUERY 不重复运动、重复 PHOTO_DONE 不续跑第二次、HOME 成功而正视失败不放行；中止 / 断链全阶段停止。
6. 无输出测选痣绑定不运动、所选同一颗复核和像素对准；错目标、不可见、视觉失效均拒绝。
7. 测许可重发不续期、一次消费、撤销后迟到授予、丢对准不自动恢复、SEQ 回绕、断链 / 重启。
8. 隔离实际输出，用指示 / 负载验证门控；实际激光另需授权、防护和硬件验收，不因前七项自动开启。

代码、构建、烧录、真机路径和身份 / 输出安全分别验收，不互相替代。

## 11. 已实现旧报文编码示例

仅格式验证 / 独立旧模式回归，不是新选痣脚本。测距必须真实测量，不照抄示例作安全证据。

| 场景 | HEX |
| --- | --- |
| HEARTBEAT，SEQ=01，tick=1234 | `A5 5A 01 01 01 04 D2 04 00 00 19 AF` |
| START_MEASURE，SEQ=10 | `A5 5A 01 06 10 01 01 D8 DD` |
| COMMAND_ACK 接受，SEQ=10 | `A5 5A 01 81 10 03 21 01 00 85 CC` |
| MEASURE_POSITION_READY，SEQ=10 | `A5 5A 01 81 10 03 20 02 00 D4 FC` |
| 测距 100 mm，SEQ=11 | `A5 5A 01 05 11 03 64 00 01 77 CD` |
| 测距 100 mm，SEQ=12 | `A5 5A 01 05 12 03 64 00 01 33 CD` |
| 测距 100 mm，SEQ=13 | `A5 5A 01 05 13 03 64 00 01 0E 0D` |
| DISTANCE_SAFE_LATCHED，SEQ=10 | `A5 5A 01 81 10 03 20 03 00 D5 6C` |
| 旧 SELECT 右视，SEQ=43 | `A5 5A 01 04 43 02 03 03 04 BF` |
| 旧 SELECTED_VIEW_READY，SEQ=43 | `A5 5A 01 81 43 03 12 03 00 F0 AF` |
| 旧 TARGET START，SEQ=44 | `A5 5A 01 02 44 01 01 98 3D` |
| 旧 TARGET_READY，SEQ=44 | `A5 5A 01 81 44 03 01 01 00 B5 CA` |
| 旧视觉 dcx=-7,dcy=-17，SEQ=45 | `A5 5A 01 03 45 05 F9 FF EF FF 01 4E 21` |
| ABORT_HOLD，SEQ=48 | `A5 5A 01 06 48 01 03 D8 CF` |
| FINISH_RETURN_HOME，SEQ=49 | `A5 5A 01 06 49 01 02 48 CF` |
| RETURN_HOME_DONE，SEQ=49 | `A5 5A 01 81 49 03 20 05 00 CA C1` |

旧“扫描 → SELECT”的整段脚本已撤下，表中旧 SELECT 仅供独立旧模式回归，不插入新自动链。扫描新版向量见 6.4；第 7 章目标 / 许可消息仍不可自拟发送。测试入口：`DEBUG/test_scan_uart.py`、`test_scan_workflow.py`、`test_scan_protocol.py`、`test_scan_console.py`；均为无端口离线测试。禁止重复固定示例启动新动作。

## 附录 A：旧调试协议

当前 `JETSON_LEGACY_PROTOCOL_ENABLE=0`，正式链路不混发：

```text
旧视觉：FF 05 03 dcx_lo dcx_hi dcy_lo dcy_hi checksum FE
checksum = (05 + 03 + DATA...) & FF
旧启停：AA 01 01 BB / AA 01 00 BB
旧状态：CC func value DD
```

PC 可复用串口框架及 `DEBUG/formal_workflow_demo.py` 验证旧帧；现 PC 不因本文新增名称而自动支持新接口。模拟距离、照片或身份只证明协议逻辑，不是实际采集、标定或安全证据。

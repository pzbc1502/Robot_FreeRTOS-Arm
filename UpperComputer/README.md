# RA6M5 机械臂 PC 上位机

这是用于 RA6M5 机械臂调试和比赛演示的 Windows/PySide6 上位机。上位机可以同时读取机械臂调试日志，并通过正式 Jetson 串口协议模拟完整比赛控制流程。

## 串口连接

- “ARM控制口”：默认使用 COM7，接收 RA6M5 文本日志并发送调试命令；完整比赛流程不依赖该串口。
- “Jetson模拟口”：默认使用 COM14，与 RA6M5 Jetson UART 双向连接，负责正式 A5 5A + CRC16/MODBUS 协议。
- 两个串口默认均为 115200。
- PC 上位机运行时，不要让真实 Jetson 或 formal_workflow_demo.py 同时打开 Jetson 串口。

## 运行日志显示

运行日志栏新增“显示 UART 统计”复选框，默认不勾选，屏蔽 ARM 接收的 `[JETSON_UART] rx_bytes=...` 周期统计。勾选后显示，取消勾选后隐藏，选择会保存到下次启动。过滤只作用于界面，不影响串口接收、状态机或这些统计在日志文件中的记录；错误和安全提示不受该过滤影响。

切换时重新显示最近 10000 条缓存中的匹配日志。“清空日志”清除界面和这份缓存，不删除日志文件，也不会在重新勾选时恢复已经清空的内容。

离线回归：`DEBUG/test_console_log_filter.py`，使用隔离设置和临时日志，不打开串口。

## 一键比赛全流程

本节为旧逐点比赛回归模式；新的固定扫描请使用独立“扫描调试”页，不要混用。当前固件安全阈值为100 mm，旧一键脚本仍采用不少于150 mm的保守模拟输入。

1. 打开 Jetson模拟口。
2. 协议选择“新协议 A5 5A + CRC16”。
3. 在“比赛全流程模拟”中选择医生使用的视图，默认正视图。
4. 旧一键模拟安全距离默认160 mm，脚本仍不接受低于150 mm的固定模拟值；不要把默认模拟数据当作现场测距证据。
5. 视觉序列格式为“dcx,dcy:持续秒数”，例如“20,-15:2;8,-6:2;0,0:0”；最后的 0 表示持续发送到收到 `OUTPUT_OFF` 或流程结束。若写成 `0,0:8`，8 秒后会停止发送，RA6M5 将因 500 ms 无新视觉帧而报“视觉数据丢失”。
6. 点击“开始一键比赛全流程”。

上位机会自动执行：

```
启动正式状态机
    -> 回 HOME
    -> 公共测距位
    -> 连续 3 帧安全距离
    -> HOME/左视图/正视图/右视图/HOME
    -> 前往医生选择的视图点
    -> 启动视觉定靶
    -> 等待 ALIGN_DONE
    -> 提示按住 P000
    -> 激光开启
    -> 提示松开 P000
    -> 激光关闭
    -> 完成并回 HOME
```

界面会用中文显示当前阶段、人工操作、超时和 RA6M5 拒绝原因。正式流程固定使用 200 ms 心跳，每条控制命令都等待同 SEQ 的 COMMAND_ACK 和业务完成状态。

三视图拍摄采用逐步驱动：每到一个拍摄点先等待 1 秒，再模拟 Jetson 完成拍照并下发下一点。当前 PC 一键流程使用固定安全距离，只验证“距离安全”的主流程；它不模拟距离过近后 RGB-D 重新测距和机械臂逐步退让。该异常分支必须由真实 Jetson 在每次退让后发送新的距离数据进行联调。

P000 是真实激光输出许可，上位机不会模拟或绕过该按键。视觉对准完成后必须按住 P000 才可能输出，松开后激光关闭。

板载状态灯用于现场确认定靶阶段：P003 点亮表示已经完成视觉对准、可以按住 P000；P503 点亮表示全部安全条件通过且 P801 已经实际输出。P003/P503 均为低电平点亮，视觉丢失、误差超限、松开 P000、流程终止或故障时会随状态熄灭。

点击“终止并保持”、关闭 Jetson 串口、关闭上位机、切换回旧协议或发生通信错误时，上位机会请求 WORKFLOW ABORT。该操作关闭激光并让机械臂保持当前位置，不自动发送 soft_reset。恢复安全后重新点击“开始一键比赛全流程”，固件会先回 HOME。

正式比赛流程运行期间，上位机会拒绝 ARM 控制口的手动运动和复位命令，防止与状态机抢占机械臂；仅保留 `laser_off` 作为关激光兜底。其他串口终端和 MQTT 发送端不受 PC 上位机互锁保护，比赛时必须关闭。

原“一键定靶演示”继续保留，只用于独立调试 TARGET_CTRL、视觉对准、P000 和激光输出。它与“一键比赛全流程”不能同时运行。

## 一键三视图弧线演示

该功能是独立运动演示，不参与正式三视图拍摄和比赛状态机。打开 ARM 控制口后，在机械臂控制页设置轨迹时间（4000～8000 ms，默认 8000 ms），点击“一键三视图弧线”。

上位机会依次等待并执行：

```text
soft_reset
    -> auto 0 -130 -15
    -> view_arc 8000
```

`view_arc` 由 RA6M5 以 10 ms 周期同步控制 J0/J3/J4，先平滑进入左视图，再连续经过正视图到达右视图。它不是严格的笛卡尔圆，也不会修改正式三视图状态机。

动作结束后机械臂保持右视图。由于这是关节空间演示轨迹，RA6M5 会主动把笛卡尔位姿标记为无效；执行下一条 `auto` 前必须先 `soft_reset`。

点击“停止弧线”会发送 `motion_abort` 和 `laser_off`，立即锁存停止并关闭激光。停止后同样必须执行 `soft_reset` 才能恢复运动。首次上板必须使用人脸模型或空载区域验证轨迹包络，不得直接贴近真人测试。

## 正式固定扫描调试

2026-10-09 已重新打包 `dist/RA6M5_Robot_Console.exe`，包含独立“扫描调试”页和冻结的07/82扫描协议。可直接运行新版 EXE，或运行 `UpperComputer/ra6m5_upper_console.py`。程序启动不自动开端口或运动。

1. 现场确认：固定模型、运动范围无人、激光关闭、急停可用；禁止真实 Jetson / 其他上位机同时占用协议口。串口号按当前设备实际选择，不使用文档默认号猜测。
2. 打开 Jetson 模拟口，选择新协议。点击“开始测距”，MCU 按原流程 HOME → 公共测距位；PC 保持心跳，停止默认距离循环。
3. 公共测距位到位后，在 Jetson 页“安全距离mm”输入现场实测值、勾有效；返回扫描页点“提交3帧实测距离”并确认。此按钮重复同一实测数值用于协议台架测试，不是连续测距或三个独立深度样本。
4. 收到本轮安全许可后点“启动扫描”；PC 生成持久化递增 scan_id。只有 FIRST FRONT_PHOTO_READY / WAIT_PHOTO 后，“保存成功（模拟）”按钮才可用。
5. “保存成功（模拟）”仅用于验证 PHOTO_DONE 握手，PC 不会实际拍照；真实 Jetson 必须新取图并保存成功后才发确认。点“保存失败”则 MCU 中止保持，不继续右扫。
6. MCU 自动执行正到右 → HOME → 固定正视；PC 不另发 HOME / SELECT。SCAN_DONE、HOME_READY、ACK 均不显示最终就绪；最终事件08 / state0B / flags07才显示正视到位，仍不允许激光。
7. PC 控制ACK等待300 ms、额外重发最多3次，重发完全相同 ID/SEQ/字节；每秒只读查询一次。阶段超时请求ABORT，不能假定已停止，现场仍须核查。心跳不因等待照片 / 最终通知停止。
8. 测试结束可点“结束回HOME”，只在最终正视就绪时可用；须收到正常 HOME 完成后才解除操作锁。异常用“终止并保持”，不自动复位返回。关闭端口 / 切换旧协议会请求中止、清就绪；不能当作已经收到停止确认。

当前仅扫描链已接通。目标身份 / 同一颗复核 / 医生单次许可还未接入，旧定靶启动和旧选视图不能接到新扫描之后；旧全流程页只用于独立回归。其他串口终端、MQTT 或独立调试器不受本PC互锁约束，现场必须关闭。

2026-10-09 停止确认修复：扫描 TERMINATED、FAULT_HOLD、控制 ACK 均不会解除运动控制锁。只有匹配本次 ABORT SEQ、`event=20 / value=06 / error=00` 的停止完成应答才能解锁。中止等待最多3秒；失败、发送不可信或超时会停心跳，显示“停止未确认”，禁止重新测距；晚到成功不能恢复资格。确认失败后，现场先处理故障，再显式点击中止产生新的请求，取得新确认后才能重新开始。连续点击等待中的中止不会改SEQ或延长预算。

上述 PC 功能是模拟器的界面门控，失效处理和超时检查仍依赖 Qt 定时任务，不等同于已经实现独立串口安全任务。Jetson 队友必须按冻结表第5.1节另行实现“不依赖Qt的主动ABORT、有界重发、确认失败后停心跳”；PC和MCU的离线检查不能代替这项双端验收。

离线检查：`DEBUG/test_scan_uart.py`、`test_scan_workflow.py`、`test_scan_protocol.py`、`test_scan_console.py`。前三项不需要Qt；最后一项使用本机PySide6并注入模拟串口。均不连接机械臂。

## 从源码运行

```
python "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py"
```

自检：

```
python "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py" --self-test
    python "Robot_FreeRTOS - Arm\UpperComputer\ra6m5_upper_console.py" --gui-self-test
```

## 打包 EXE

在 `Robot_FreeRTOS - Arm\UpperComputer` 下执行：

本机使用安装了 PySide6、pyserial 和 PyInstaller 的 Python 3.11。打包时临时清理 DLL 搜索路径，避免其他图像工具的同名 `icuuc.dll` 被收集进 EXE，导致 Qt 加载失败；完成后恢复当前终端的原路径，不修改系统环境变量。

```powershell
$python = Join-Path $env:LOCALAPPDATA 'Programs/Python/Python311/python.exe'
$originalPath = $env:PATH
try {
    $env:PATH = "$env:SystemRoot\System32;$env:SystemRoot;$(Split-Path -Parent $python)"
    & $python -B -m PyInstaller --noconfirm --clean RA6M5_Robot_Console.spec
    if ($LASTEXITCODE -ne 0) { throw 'EXE packaging failed' }
} finally {
    $env:PATH = $originalPath
}
```

产物位于 UpperComputer\dist\RA6M5_Robot_Console.exe。

扫描版 EXE 打包时的旧版备份为 `dist/RA6M5_Robot_Console_before_scan_20261009_124946.exe.bak`。

2026-10-09 已再次打包日志显示开关版并更新原 `dist/RA6M5_Robot_Console.exe`；更新前的扫描版备份为 `dist/RA6M5_Robot_Console_before_log_filter_20261009_154524721.exe.bak`。新版通过 7 项日志过滤测试、扫描界面模拟回归、协议 / Qt 界面自检和打包模块检查，SHA256 为 `7C3ADEF26AEA7CFD9FA190DDC3EBD15EACDDA3721FA3A976D9CC1DF928E35EAA`。检查不打开串口，不修改 MCU 固件，也不代替上板及现场路径验收。

该日志开关版现已被停止确认修复版替代，以上哈希仅标识历史产物。2026-10-09 19:36 在用户确认安全停止、关闭上位机并核对进程退出后，已更新原 `dist/RA6M5_Robot_Console.exe`，保留日志开关。新文件44,766,942字节，SHA256=`B524732286291897748C06A6B4026BBAD0F1FE995B7D6D812ADAFBA8CB577CBD`；旧版备份为 `dist/RA6M5_Robot_Console_before_stop_confirmation_20261009_193642419.exe.bak`。模拟串口检查、7项日志过滤测试、源码与EXE的协议 / GUI自检均通过；另检查冻结EXE确实包含停止确认门控。未打开串口或执行运动。

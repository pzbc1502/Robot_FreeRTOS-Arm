# 机械臂点痣装置 Qt 界面

这是 Jetson Nano B01 上使用的离线 Qt 桌面界面。它不依赖微信小程序，也不依赖网络。

## 功能

- 显示本机摄像头实时画面。
- 使用现有 `jetson_nano_vision` 里的 OpenCV 黑痣检测函数。
- 在画面上标出识别点。
- 显示像素坐标、视觉误差 `dcx/dcy`、半径、面积、对比度、目标数量。
- 提供 `开始识别`、`单次识别`、`停止识别`、`预定位`、`急停` 按钮。
- 可通过串口向 RA6M5 发送视觉误差帧；`急停` 仍只提示使用独立硬件急停。

## 推荐安装方式

在 Jetson Nano 上优先使用系统包安装 Qt 和 OpenCV：

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-opencv python3-numpy python3-serial
```

如果你使用虚拟环境，也可以参考：

```bash
python3 -m pip install -r requirements.txt
```

Jetson Nano 上 `PyQt5` 用 `apt` 安装通常比 `pip` 更稳。

## 运行

在当前目录运行：

```bash
python3 -m app.main
```

常用参数：

```bash
python3 -m app.main --camera 0 --width 640 --height 480 --fps 30
python3 -m app.main --camera csi --width 640 --height 480 --fps 30
python3 -m app.main --demo
python3 -m app.main --camera 0 --serial-port /dev/ttyUSB0 --baudrate 115200
```

`--demo` 不打开真实摄像头，会生成模拟画面，方便先检查界面。
`--serial-port` 为空时不连接 RA6M5，只显示识别结果；配置串口后，点击 `预定位` 会发送当前首个目标的视觉误差帧。

## 桌面图标

可以在 Nano 上运行：

```bash
bash install_desktop_launcher.sh
```

它会在桌面创建 `机械臂点痣装置.desktop`。之后点击图标即可打开界面。

## RA6M5 通信

当前 Qt 侧实现的是根目录 `视觉协议.md` 中的视觉误差帧：

```text
[0xFF][0x05][0x03][dcx_lo][dcx_hi][dcy_lo][dcy_hi][checksum][0xFE]
```

`dcx = target_x - frame_width / 2`，`dcy = target_y - frame_height / 2`。RA6M5 回传 `[0xCC][FUNC][VALUE][0xDD]` 状态帧时，界面会轮询并显示最近状态。

建议保持安全流程：

1. `识别` 只负责找目标点。
2. `预定位` 只发送视觉误差帧，由下位机侧决定是否运动。
3. 真正治疗动作不要放成单击即执行。
4. 急停最好同时保留独立硬件按钮。

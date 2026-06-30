# Jetson Nano Vision

这个新工程只保留和摄像头相关的可复用部分：相机打开、相机测试、棋盘格采集、相机标定、去畸变、矩形靶检测、黑痣检测，以及已有的 `calib_images` 标定图片。

原来的树莓派按键、OLED、UART7、心电图控制、云台控制都没有带进来，后面可以按 Jetson Nano 的新项目需求重新接。

## 目录

```text
jetson_nano_vision/
  calib_images/              # 从旧工程复制来的 15 张 640x480 标定图
  jetson_vision/
    camera_utils.py          # V4L2/Picamera2 相机打开工具
    camera_test.py           # 摄像头测试
    snapshot.py              # 抓帧保存（拍人脸样本喂给离线调参用）
    reference_capture.py     # 采集无黑痣多帧中值基准图
    calib_capture.py         # 采集棋盘格标定图
    calib_solve.py           # 求解相机内参和畸变参数
    undistort.py             # 运行时去畸变
    detect_preview.py        # 相机 + 去畸变 + 矩形检测预览
    mole_preview.py          # 相机 + 去畸变 + 黑痣检测 + 串口发送
    mole_visual_follow.py    # 黑痣视觉跟随测试，只发送视觉误差，不触发激光
    reference_mole_preview.py # 基准差分黑痣检测与多帧稳定性预览
    mole_image_test.py       # 单张图片黑痣检测离线调参
    serial_sender.py         # RA6M5 视觉误差串口下发
    vision_protocol.py       # Jetson <-> RA6M5 协议打包与解析
    vision/rect_detect.py    # 矩形靶检测
    vision/mole_detect.py    # 黑痣检测（肤色掩膜 + black-hat + 形状过滤）
    vision/reference_mole_detect.py # 无黑痣基准配准和新增暗斑检测
```

## 安装依赖

Jetson Nano 通常建议优先使用系统自带 OpenCV。如果 `python3 -c "import cv2"` 能正常运行，可以只装 numpy：

```bash
python3 -m pip install numpy
```

如果你的环境没有 OpenCV，再尝试：

```bash
python3 -m pip install -r requirements.txt
```

如果需要串口下发：

```bash
python3 -m pip install pyserial
```

## 换新摄像头后的步骤

新摄像头一般是 USB UVC，`camera_utils.py` 直接支持，但**焦距、畸变都变了**，必须重新做标定。流程：

1. 验证相机能开：

   ```bash
   python3 -m jetson_vision.camera_test --camera 0 --display 1 --flip none
   ```

2. 重新采集棋盘格标定图（15-25 张，覆盖画面四个角和中心、不同角度）：

   ```bash
   python3 -m jetson_vision.calib_capture --camera 0 --flip none --pattern 6x9 --out calib_images
   ```

3. 求解新的内参畸变（旧的 `calib_params.npz` 直接覆盖）：

   ```bash
   python3 -m jetson_vision.calib_solve --images calib_images --pattern 6x9 --square-mm 20 --out calib_params.npz
   ```

## 测试摄像头

USB/V4L2 摄像头：

```bash
python3 -m jetson_vision.camera_test --camera 0 --display 1 --flip none
```

Jetson Nano CSI 摄像头：

```bash
python3 -m jetson_vision.camera_test --camera csi --display 1 --flip none
```

无窗口只看帧率：

```bash
python3 -m jetson_vision.camera_test --camera 0 --display 0 --flip none
```

## 黑痣检测（单目方案）

### 一体化运行（推荐）

以下命令在同一窗口中完成去畸变、基准重采、配准、肤色 ROI、差分检测、形状过滤、多帧确认和坐标显示：

```bash
python3 -m jetson_vision.mole_system --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params calib_params.npz \
    --reference reference/clean_face.png

# 如需把稳定目标误差下发给 RA6M5
python3 -m jetson_vision.mole_system --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params calib_params.npz \
    --reference reference/clean_face.png \
    --serial-port /dev/ttyUSB0 --baudrate 115200
```

按键：

- `R`：相机位置改变后，重新采集无黑痣基准。按键前必须取下全部黑痣。
- `D`：贴好黑痣后进入检测。
- `P`：返回普通预览。
- `N`：切换对齐图、ROI、暗差分图和二值图。
- `S`：保存当前带检测标注的结果图。
- `Q` / Esc：退出。

按 `R` 后默认倒计时 3 秒，再采集 30 帧中值基准，便于操作者移开手。相机明显移动而未重采基准时，程序返回 `ALIGN_FAILED`，不输出坐标。

注意：重采基准只能恢复视觉检测。相机相对机械臂的位置改变后，视觉坐标到机械臂坐标的外参映射仍需重新标定。

固定摄像头和固定人脸模型时，优先使用无黑痣基准差分方案：

```bash
# 采集 30 帧中值基准图，显示模式下按 SPACE 开始
python3 -m jetson_vision.reference_capture --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params calib_params.npz --frames 30 --display 1

# 实时检测相对基准新增的黑痣，按 N 切换调试视图
python3 -m jetson_vision.reference_mole_preview --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params calib_params.npz \
    --reference reference/clean_face.png --display 1
```

该流程会先对当前画面与基准图做小幅配准。配准失败、目标不稳定或没有新增暗斑时，不输出稳定目标。

以下肤色掩膜 + black-hat 方案保留为不依赖基准图的辅助检测器。

整体流程：肤色掩膜（HSV+YCrCb）→ black-hat 提局部暗斑 → 阈值 + 形状过滤 → 按显著性排序 → 串口下发到下位机。

### 第一步：先用单图离线调参

直接在摄像头上调参很折腾。先抓一张实拍图离线调好参数：

```bash
# 单帧抓拍
python3 -m jetson_vision.snapshot --camera 0 --out face.jpg --flip none

# 或开窗按空格连拍多张
python3 -m jetson_vision.snapshot --camera 0 --out face --display 1 --flip none

# 在样本上拖滑动条调参
python3 -m jetson_vision.mole_image_test --image face.jpg
```

`mole_image_test` 滑动条说明：

- **min_area / max_area**：黑痣面积像素范围。摄像头距离不同会差很多，先按当前距离的痣实际占多少像素估个范围，再用滑动条收紧。
- **circ x100 / solid x100**：圆度和实心度，越接近 1 越像痣。眉毛细线圆度低，会被过滤。
- **aspect x10**：长宽比上限 ×10。`22` 表示长宽比 ≤ 2.2。
- **bh_ksize**：black-hat 核大小，应比痣的直径**大**一点（25-35 一般够）。
- **contrast**：痣相对周围肤色的灰度差阈值。光照亮就调高，暗就调低。
- **skin_dilate**：肤色掩膜 erode 大小，越大越严格地避开五官（也越容易把脸边缘的痣排除）。
- **skin_mask**：0 / 1 切换，调参时切到 0 看不带肤色限制的原始结果。

按键：
- `n` 在 `[overlay / skin / blackhat / binary]` 之间循环切换视图，看中间结果方便定位是哪一步把痣过滤掉了。
- `s` 把当前参数保存为 `mole_params.json`。
- `q` / Esc 退出。

### 第二步：把调好的参数应用到摄像头实时检测

```bash
# 加载离线调好的参数
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --calib-params calib_params.npz \
    --params-json mole_params.json

# 串口下发给下位机
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --calib-params calib_params.npz \
    --params-json mole_params.json \
    --serial-port /dev/ttyUSB0 --baudrate 115200

# 现场再微调（带 trackbar，按 s 保存到同一个 json）
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --params-json mole_params.json --tune
```

串口帧格式见 `vision_protocol.py` 和根目录 `视觉协议.md`。Jetson 下发视觉误差帧：

```text
[0xFF][0x05][0x03][dcx_lo][dcx_hi][dcy_lo][dcy_hi][checksum][0xFE]
```

其中 `dcx = target_x - frame_width / 2`，`dcy = target_y - frame_height / 2`，均按 `int16_t` 小端发送。RA6M5 回传状态帧为 `[0xCC][FUNC][VALUE][0xDD]`，程序会解析并打印状态。

### 视觉跟随测试

`mole_visual_follow.py` 用于单独测试“黑痣识别 + 机械臂视觉跟随”。它不会发送激光输出帧，只会在目标多帧稳定后按限速发送视觉误差帧。默认流程是：先发送 `AA 01 01 BB`，等待 RA6M5 回传 `READY`，再开始持续下发 `VISION_ERROR`。

```bash
python3 -m jetson_vision.mole_visual_follow --camera 0 \
    --width 640 --height 480 --fps 30 --flip none \
    --serial-port /dev/ttyUSB0 --baudrate 115200 \
    --laser-offset-x-px 60 --laser-offset-y-px -30 \
    --send-rate-hz 8 --deadband-px 5
```

先不接机械臂、只看识别和稳定目标：

```bash
python3 -m jetson_vision.mole_visual_follow --camera 0 \
    --width 640 --height 480 --fps 30 --flip none \
    --display 1
```

如果 RA6M5 暂时还没有回传 `READY`，可以只做低速串口下发测试：

```bash
python3 -m jetson_vision.mole_visual_follow --camera 0 \
    --width 640 --height 480 --fps 30 --flip none \
    --serial-port /dev/ttyUSB0 --baudrate 115200 \
    --wait-ready 0 --send-rate-hz 3
```

## 测试矩形检测和去畸变

未启用标定：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip none
```

启用标定参数：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip none --calib-params calib_params.npz
```

如果是 CSI 摄像头，把 `--camera 0` 换成 `--camera csi`。多路 CSI 可用 `--camera csi:1`。

## 重新采集标定图

如果上 Jetson 后实际画面分辨率、镜头或安装有变化，重新采集：

```bash
python3 -m jetson_vision.calib_capture --camera 0 --flip none --pattern 6x9 --out calib_images
python3 -m jetson_vision.calib_solve --images calib_images --pattern 6x9 --square-mm 20 --out calib_params.npz
```

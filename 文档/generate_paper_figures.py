from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


OUT_DIR = Path(__file__).resolve().parent / "论文级功能图"


COLORS = {
    "blue": "#0F4D92",
    "blue_mid": "#3775BA",
    "blue_soft": "#E7F0FA",
    "orange": "#C85A17",
    "orange_soft": "#FBE8D9",
    "green": "#2E7D4F",
    "green_soft": "#E4F4EA",
    "red": "#B64342",
    "red_soft": "#F8DFDD",
    "gray": "#5B6472",
    "gray_soft": "#F1F3F5",
    "ink": "#222222",
    "line": "#6A7280",
}


def apply_style() -> None:
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Arial",
        "DejaVu Sans",
        "sans-serif",
    ]
    plt.rcParams["svg.fonttype"] = "none"
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["font.size"] = 8
    plt.rcParams["figure.facecolor"] = "white"


def save_figure(fig: plt.Figure, name: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    base = OUT_DIR / name
    fig.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(base.with_suffix(".png"), dpi=350, bbox_inches="tight")
    plt.close(fig)


def add_panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        0.01,
        0.98,
        label,
        ha="left",
        va="top",
        fontsize=12,
        fontweight="bold",
        color=COLORS["ink"],
    )


def add_section_band(
    ax: plt.Axes,
    y: float,
    h: float,
    title: str,
    face: str,
    edge: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (0.02, y),
            0.96,
            h,
            boxstyle="round,pad=0.006,rounding_size=0.012",
            linewidth=1.0,
            edgecolor=edge,
            facecolor=face,
            zorder=0,
        )
    )
    ax.text(
        0.035,
        y + h - 0.027,
        title,
        ha="left",
        va="top",
        fontsize=8.2,
        fontweight="bold",
        color=edge,
    )


def add_box(
    ax: plt.Axes,
    x: float,
    y: float,
    w: float,
    h: float,
    text: str,
    face: str,
    edge: str,
    fontsize: float = 7.2,
    weight: str = "normal",
) -> tuple[float, float, float, float]:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.007,rounding_size=0.012",
            linewidth=1.1,
            edgecolor=edge,
            facecolor=face,
            zorder=2,
        )
    )
    ax.text(
        x + w / 2,
        y + h / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        fontweight=weight,
        color=COLORS["ink"],
        linespacing=1.25,
        zorder=3,
    )
    return x, y, w, h


def right(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, w, h = box
    return x + w, y + h / 2


def left(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, _w, h = box
    return x, y + h / 2


def top(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, w, h = box
    return x + w / 2, y + h


def bottom(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, w, _h = box
    return x + w / 2, y


def add_arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str = COLORS["line"],
    rad: float = 0.0,
    lw: float = 1.0,
    z: int = 1,
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=8,
            linewidth=lw,
            color=color,
            shrinkA=4,
            shrinkB=4,
            connectionstyle=f"arc3,rad={rad}",
            zorder=z,
        )
    )


def add_small_note(
    ax: plt.Axes,
    x: float,
    y: float,
    text: str,
    color: str = COLORS["gray"],
) -> None:
    ax.text(x, y, text, ha="center", va="center", fontsize=6.4, color=color)


def draw_control_flow() -> None:
    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    add_panel_label(ax, "a")
    ax.text(
        0.055,
        0.965,
        "软件控制流程图",
        ha="left",
        va="top",
        fontsize=13,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.055,
        0.925,
        "Jetson 视觉输入经统一协议进入 RA6M5 周期调度，最终由 S 曲线/视觉伺服和前馈+P 控制驱动执行层。",
        ha="left",
        va="top",
        fontsize=7.5,
        color=COLORS["gray"],
    )

    add_section_band(ax, 0.70, 0.18, "Jetson NX 视觉端", COLORS["blue_soft"], COLORS["blue"])
    add_section_band(ax, 0.41, 0.21, "RA6M5 控制层", COLORS["orange_soft"], COLORS["orange"])
    add_section_band(ax, 0.11, 0.22, "运动与执行层", COLORS["green_soft"], COLORS["green"])

    mono = add_box(ax, 0.06, 0.745, 0.145, 0.07, "单目黑痣识别\nECC配准 / LAB-L差分", "white", COLORS["blue"])
    stereo = add_box(ax, 0.225, 0.745, 0.13, 0.07, "双目安全距离\n距离mm + valid", "white", COLORS["blue"])
    qt = add_box(ax, 0.375, 0.745, 0.12, 0.07, "Qt界面\n三视图 / 选点", "white", COLORS["blue"])
    proto = add_box(ax, 0.525, 0.745, 0.17, 0.07, "统一协议\nA5 5A + CRC16", "white", COLORS["blue"], weight="bold")

    scheduler = add_box(ax, 0.09, 0.455, 0.18, 0.075, "vision_service_thread\n10 ms周期调度", "white", COLORS["orange"], weight="bold")
    parser = add_box(ax, 0.305, 0.455, 0.16, 0.075, "jetson_vision_process\n帧解析 / 缓存", "white", COLORS["orange"])
    capture = add_box(ax, 0.50, 0.455, 0.15, 0.075, "robot_capture_step\n三视图点位", "white", COLORS["orange"])
    target = add_box(ax, 0.685, 0.455, 0.17, 0.075, "robot_target_step\n定靶状态机", "white", COLORS["orange"], weight="bold")

    scurve = add_box(ax, 0.06, 0.18, 0.13, 0.07, "S曲线插补\n10 ms路径点", "white", COLORS["green"])
    ik = add_box(ax, 0.215, 0.18, 0.13, 0.07, "连续IK选解\n分支锁定", "white", COLORS["green"])
    servo = add_box(ax, 0.37, 0.18, 0.13, 0.07, "视觉速度伺服\nvx / vy / vz", "white", COLORS["green"], weight="bold")
    pid = add_box(ax, 0.525, 0.18, 0.13, 0.07, "前馈+P速度环\nFF_GAIN + Kp", "white", COLORS["green"], weight="bold")
    can = add_box(ax, 0.68, 0.18, 0.11, 0.07, "CAN电机\nJ1-J5", "white", COLORS["green"])
    laser = add_box(ax, 0.815, 0.18, 0.12, 0.07, "P000许可\nP801激光", "white", COLORS["green"])

    safety = add_box(
        ax,
        0.74,
        0.72,
        0.20,
        0.10,
        "安全监测\n心跳 / 距离 / 限位 / 视觉超时\n→ 关激光 + 停止伺服",
        COLORS["red_soft"],
        COLORS["red"],
        fontsize=6.8,
        weight="bold",
    )

    add_arrow(ax, right(mono), left(stereo), COLORS["blue"])
    add_arrow(ax, right(stereo), left(qt), COLORS["blue"])
    add_arrow(ax, right(qt), left(proto), COLORS["blue"])
    add_arrow(ax, (0.61, 0.745), (0.18, 0.535), COLORS["orange"], rad=0.0)
    add_arrow(ax, right(scheduler), left(parser), COLORS["orange"])
    add_arrow(ax, right(parser), left(capture), COLORS["orange"])
    add_arrow(ax, right(parser), left(target), COLORS["orange"], rad=-0.08)
    add_arrow(ax, bottom(capture), top(scurve), COLORS["green"], rad=0.0)
    add_arrow(ax, bottom(target), top(servo), COLORS["green"], rad=0.0)
    add_arrow(ax, right(scurve), left(ik), COLORS["green"])
    add_arrow(ax, right(ik), left(pid), COLORS["green"], rad=-0.08)
    add_arrow(ax, right(servo), left(pid), COLORS["green"])
    add_arrow(ax, right(pid), left(can), COLORS["green"])
    add_arrow(ax, right(pid), left(laser), COLORS["green"], rad=0.08)
    add_arrow(ax, (0.80, 0.72), top(target), COLORS["red"], rad=0.0, lw=1.2)
    add_arrow(ax, (0.88, 0.72), top(laser), COLORS["red"], rad=0.0, lw=1.2)

    save_figure(fig, "fig_software_control_flow")


def draw_target_state_machine() -> None:
    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    add_panel_label(ax, "b")
    ax.text(
        0.055,
        0.965,
        "激光定靶安全状态机",
        ha="left",
        va="top",
        fontsize=13,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.055,
        0.925,
        "激光输出只在视觉稳定、P000按住、安全距离有效且状态机启用时发生；任一异常优先进入关断路径。",
        ha="left",
        va="top",
        fontsize=7.5,
        color=COLORS["gray"],
    )

    states = [
        ("INIT\n初始", 0.045),
        ("PRE_POSITION\n预定位", 0.175),
        ("WAIT_DETECT\n等待视觉", 0.335),
        ("ALIGN\n视觉对准", 0.505),
        ("CONFIRM\n二次确认", 0.66),
        ("OUTPUT\n激光输出", 0.815),
    ]
    boxes = []
    for label, x in states:
        fc = COLORS["blue_soft"] if "OUTPUT" not in label else COLORS["green_soft"]
        ec = COLORS["blue"] if "OUTPUT" not in label else COLORS["green"]
        boxes.append(add_box(ax, x, 0.55, 0.12, 0.085, label, fc, ec, fontsize=6.8, weight="bold"))

    done = add_box(ax, 0.815, 0.35, 0.12, 0.07, "DONE\n完成/关闭", COLORS["gray_soft"], COLORS["gray"], fontsize=6.8)
    recover = add_box(ax, 0.50, 0.25, 0.16, 0.08, "RECOVER\n安全恢复", COLORS["red_soft"], COLORS["red"], fontsize=7.0, weight="bold")
    off = add_box(
        ax,
        0.68,
        0.14,
        0.23,
        0.085,
        "统一关断动作\n关P801激光 / 停止视觉伺服 / 回传ERROR",
        COLORS["red_soft"],
        COLORS["red"],
        fontsize=6.7,
        weight="bold",
    )

    for a, b in zip(boxes[:-1], boxes[1:]):
        add_arrow(ax, right(a), left(b), COLORS["blue"])
    add_arrow(ax, bottom(boxes[-1]), top(done), COLORS["green"])

    add_small_note(ax, 0.585, 0.69, "连续3帧误差≤5px\n→ CC 02 01", COLORS["green"])
    add_small_note(ax, 0.755, 0.69, "P000按住 + 距离≥110mm\n+ 视觉有效", COLORS["green"])

    guard_items = [
        ("心跳超时", 0.07, 0.77),
        ("安全距离<100mm", 0.225, 0.77),
        ("视觉超时500ms", 0.40, 0.77),
        ("松开P000", 0.57, 0.77),
        ("限位/急停", 0.72, 0.77),
    ]
    guard_boxes = [
        add_box(ax, x, y, 0.125, 0.055, text, COLORS["red_soft"], COLORS["red"], fontsize=6.5, weight="bold")
        for text, x, y in guard_items
    ]

    bus_y = 0.715
    ax.plot([0.13, 0.79], [bus_y, bus_y], color=COLORS["red"], linewidth=1.0, alpha=0.85)
    ax.text(0.82, bus_y, "异常安全总线", ha="left", va="center", fontsize=6.4, color=COLORS["red"], fontweight="bold")
    for gb in guard_boxes:
        add_arrow(ax, bottom(gb), (bottom(gb)[0], bus_y), COLORS["red"], rad=0.0, lw=0.75, z=0)
    add_arrow(ax, (0.79, bus_y), top(off), COLORS["red"], rad=0.18, lw=1.2, z=0)

    add_arrow(ax, bottom(boxes[3]), top(recover), COLORS["red"], rad=-0.18)
    add_arrow(ax, bottom(boxes[4]), top(recover), COLORS["red"], rad=-0.05)
    add_arrow(ax, bottom(boxes[5]), top(recover), COLORS["red"], rad=0.12)
    add_arrow(ax, right(recover), left(done), COLORS["gray"], rad=-0.12)
    add_arrow(ax, bottom(recover), left(off), COLORS["red"], rad=0.1)

    led = add_box(
        ax,
        0.075,
        0.27,
        0.28,
        0.10,
        "指示与许可\nCONFIRM：P003亮，提示可按P000\nOUTPUT：P003/P503亮，P801输出",
        COLORS["green_soft"],
        COLORS["green"],
        fontsize=6.7,
    )
    add_arrow(ax, bottom(boxes[4]), top(led), COLORS["green"], rad=0.1)
    add_arrow(ax, bottom(boxes[5]), right(led), COLORS["green"], rad=0.2)

    ax.add_patch(Rectangle((0.035, 0.49), 0.91, 0.19, fill=False, edgecolor="#CDD3DA", linewidth=0.9))
    ax.text(0.045, 0.485, "主状态链", ha="left", va="top", fontsize=6.6, color=COLORS["gray"])
    ax.text(0.045, 0.125, "安全中断优先级高于输出状态；激光只允许在 OUTPUT 内开启。", ha="left", va="center", fontsize=7.0, color=COLORS["gray"])

    save_figure(fig, "fig_target_safety_state_machine")


def main() -> None:
    apply_style()
    draw_control_flow()
    draw_target_state_machine()
    print(f"saved to: {OUT_DIR}")


if __name__ == "__main__":
    main()

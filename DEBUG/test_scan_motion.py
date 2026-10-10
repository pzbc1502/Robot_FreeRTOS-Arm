"""Compile actual motion C functions against a PC-only simulated CAN plant.

No serial/debugger access. This verifies software, not the physical arc/path.
Run: py -3 DEBUG/test_scan_motion.py [--gcc path/to/gcc.exe]
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from test_visual_servo_static import effective_c


ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    code = effective_c(source)
    match = re.search(r"(?m)^\s*(?:static\s+)?(?:bool|int|float|void|uint32_t|"
                      r"robot_scan_result_t|const capture_step_t \*)\s*" + name +
                      r"\s*\([^;{}]*\)\s*\{", code)
    assert match, name
    start = match.start()
    depth = 1
    end = match.end()
    while depth:
        depth += (code[end] == "{") - (code[end] == "}")
        end += 1
    return source[start:end]


def declaration(source, pattern):
    match = re.search(pattern, source, re.S)
    assert match, pattern
    return match.group(0)


def build_source(hardware_limits_enabled=None):
    robot = (ROOT / "APP/robot.c").read_text(encoding="utf-8")
    header = (ROOT / "APP/robot.h").read_text(encoding="utf-8")
    capture = (ROOT / "APP/robot_capture.c").read_text(encoding="utf-8")
    jetson = (ROOT / "Middle/jetson_vision.h").read_text(encoding="utf-8")
    macros = "\n".join(re.findall(r"(?m)^#define (?:ROBOT_|JETSON_CAPTURE_ACTION_)\w+[^\n]*",
                                 header + robot + jetson))
    if hardware_limits_enabled is not None:
        macros = re.sub(r"(?m)^#define ROBOT_JOINT_LIMIT_SWITCHES_ENABLED[^\n]*",
                        f"#define ROBOT_JOINT_LIMIT_SWITCHES_ENABLED ({int(hardware_limits_enabled)}U)", macros)
    enums = "\n".join(declaration(header, r"typedef enum\s*\{[^}]*\}\s*" + name + r";")
                      for name in ("robot_scan_segment_t", "robot_scan_result_t"))
    structs = declaration(header, r"enum motor_dir\s*\{[^}]*\};")
    structs += declaration(header, r"enum\s*\{\s*ROBOT_JOINT_1[^}]*\};")
    structs += declaration(header, r"enum dir\s*\{[^}]*\};")
    structs += declaration(header, r"struct joint\s*\{[^}]*\};")
    structs += declaration(header, r"struct position\s*\{[^}]*\};")
    table = declaration(robot, r"static struct joint g_joints_init\[[^;]*?\};")
    recipes = declaration(capture, r"typedef enum\s*\{[^}]*\}\s*capture_step_type_t;")
    recipes += declaration(capture, r"typedef struct\s*\{[^}]*\}\s*capture_step_t;")
    for name in ("left", "front", "right", "home"):
        recipes += declaration(capture, r"static const capture_step_t s_" + name + r"_steps\[\][^;]*;")
    functions = "\n".join(function(robot, name) for name in (
        "robot_motion_abort_is_requested", "robot_motion_abort",
        "robot_angle_normalize", "robot_angle_diff", "robot_angle_map",
        "robot_joint_is_full_turn", "robot_joint_compare_angle", "robot_joint_compare_error",
        "robot_joint_rotate_to", "robot_joint_hard_reset", "robot_view_arc_smoothstep", "robot_auto_final_confirm",
        "robot_pid_one_period_limited", "robot_pid_one_period", "robot_scan_apply_view",
        "robot_scan_feedback_matches", "robot_scan_segment_move", "robot_scan_result_get",
        "robot_send_scan_segment_event", "robot_scan_verify_pose"))
    capture_functions = function(capture, "capture_steps_for") + function(capture, "robot_capture_view_angles_get")
    checked_stop = function(robot, "robot_joint_stop_all_checked")
    gains = declaration(robot, r"static const float ROBOT_JOINT_KP\[[^;]*;")
    gains += declaration(robot, r"static const float ROBOT_JOINT_FF_GAIN\[[^;]*;")
    return (PREAMBLE + macros + "\n" + enums + structs + table + recipes + GLOBALS +
            gains + checked_stop + capture_functions + functions + CHECKS)


PREAMBLE = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
typedef int external_irq_instance_t;
typedef int fsp_err_t;
static const int JOINT_LIMIT_1, JOINT_LIMIT_2, JOINT_LIMIT_3,
                 JOINT_LIMIT_4, JOINT_LIMIT_5, JOINT_LIMIT_6;
static char last_scan_guard[256];
static void mock_log(const char *format, ...) {
    if (strstr(format,"[SCAN_GUARD]")) {
        va_list args; va_start(args,format);
        vsnprintf(last_scan_guard,sizeof(last_scan_guard),format,args); va_end(args);
    }
}
#define LOG(...) mock_log(__VA_ARGS__)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
#define pdFAIL 0
#define ROBOT_SCAN_SEGMENT_EVENT 13
#define ROBOT_JOINT_RESULT_RUNNING 1
#define ROBOT_RESET_RESULT_RUNNING 1
#define ROBOT_AUTO_RESULT_ABORTED 4
#define ROBOT_JOINT_RESULT_ABORTED 4
#define ROBOT_RESET_RESULT_ABORTED 4
#define ROBOT_VISUAL_SERVO_FAULT_FEEDBACK 1
typedef int BaseType_t;
"""

GLOBALS = r"""
struct robot_event { int type; uint8_t joint_id; uint32_t generation; float param[6]; };
static struct { struct joint joints[6]; uint32_t status; void *event_queue;
                struct position cur_pos; } g_robot;
static bool ROBOT_TARGET_ENABLED, g_robot_motion_abort_requested, missing_feedback,
            frozen_plant, queue_fail, capture_active, limit_triggered, pin_fail, stop_fail;
static int g_robot_auto_result, g_robot_joint_result, g_robot_reset_result;
static struct { bool stop_requested; } g_visual_servo;
static uint32_t g_robot_motion_generation = 1, now_ms, abort_at, feedback_fail_at, read_delay_ms;
static robot_scan_result_t g_robot_scan_result;
static bool g_hard_reset_done;
static float s_scan_front_target[6], commanded[5], peak_speed;
static unsigned pending, motion_batches, laser_off_calls, pos_calls, pos_steps;
static unsigned pin_read_calls, motor_enable_calls, hard_reset_calls, home_write_calls;
static uint8_t pos_direction;
static struct robot_event queued_event;
static bool robot_motion_abort_is_requested(void);
static void robot_joint_stop_all(uint8_t n) { (void)n; memset(commanded,0,sizeof(commanded)); }
bool robot_motion_abort(void);
static bool robot_joint_stop_all_checked(uint8_t n);
static bool robot_capture_is_active(void) { return capture_active; }
static bool robot_is_auto_busy(void) { return ROBOT_STATUS_IS(g_robot.status,ROBOT_STATUS_AUTO_BUSY); }
static bool robot_is_visual_servo_active(void) { return ROBOT_STATUS_IS(g_robot.status,ROBOT_STATUS_VISUAL_SERVO_ACTIVE); }
static bool robot_motion_abort_latched(void) { return g_robot_motion_abort_requested; }
static int robot_update_all_angles(uint8_t n,uint32_t *missing,uint32_t *elapsed) {
    (void)n; (void)elapsed; now_ms+=read_delay_ms;
    if(feedback_fail_at && now_ms>=feedback_fail_at) missing_feedback=true;
    *missing=missing_feedback?1:0; return missing_feedback?1:0;
}
static uint32_t HAL_GetTick(void) { return now_ms; }
static uint32_t xTaskGetTickCount(void) { return now_ms; }
static void vTaskDelay(uint32_t ms) {
    for(int j=0;j<5;j++) if(!frozen_plant) g_robot.joints[j].current_angle+=commanded[j]*ms/1000.0f;
    now_ms+=ms; if(abort_at && now_ms>=abort_at) robot_motion_abort();
}
static bool robot_joint_velocity_batch_nowait(const float *v,uint8_t n,uint8_t acc) {
    (void)acc; motion_batches++;
    for(int j=0;j<n;j++) { commanded[j]=v[j]; peak_speed=fmaxf(peak_speed,fabsf(v[j])); }
    return true;
}
static bool robot_joint_velocity_zero_all(uint8_t n) { (void)n; robot_joint_stop_all(5); return true; }
static void robot_joint_velocity_nowait(uint32_t j,float v,uint8_t a) { (void)a; commanded[j]=v; }
static void robot_visual_servo_fault_set(int f) { (void)f; }
static void BSP_Laser_Off(void) { laser_off_calls++; }
static void BSP_CAN_PrintInfo(const char *tag) {
    assert(strcmp(tag,"scan_stop")==0 && g_robot_motion_abort_requested);
    for(unsigned j=0;j<5;j++) assert(commanded[j]==0);
}
static void Emm_V5_En_Control(uint8_t addr,bool enable,bool sync) {
    (void)addr;(void)enable;(void)sync;motor_enable_calls++;
}
static bool robot_joint_reset(uint8_t joint) { (void)joint;hard_reset_calls++;return false; }
static void robot_set_home_pose_valid(void) { home_write_calls++; }
static int robot_mqtt_joints_sync(void) { return 0; }
typedef int bsp_io_level_t;
#define BSP_IO_LEVEL_LOW 0
#define BSP_IO_LEVEL_HIGH 1
#define FSP_SUCCESS 0
static int g_ioport_ctrl;
static const int g_joint_limit_pins[6]={0,1,2,3,4,5};
static int R_IOPORT_PinRead(void *ctrl,int pin,int *level) {
    (void)ctrl;(void)pin;pin_read_calls++;*level=limit_triggered?1:0;return pin_fail?1:0;
}
static bool BSP_CAN_Lock(unsigned ms) { (void)ms;return true; }
static void BSP_CAN_Unlock(void) {}
static void BSP_CAN_ClearStopFlags(uint8_t n) { (void)n; }
static bool BSP_CAN_WaitStopAll(uint8_t n,unsigned ms) { (void)n;(void)ms;return !stop_fail; }
static void Emm_V5_Stop_Now(uint8_t addr,bool sync) { (void)sync;commanded[addr-1]=0; }
static unsigned uxQueueMessagesWaiting(void *q) { (void)q; return pending; }
static int xQueueSendToBack(void *q,const struct robot_event *event,unsigned timeout) {
    (void)q; (void)timeout; if(queue_fail) return pdFAIL;
    queued_event=*event; pending=1; return pdPASS;
}
static void Emm_V5_Pos_Control(uint8_t addr,uint8_t dir,uint16_t v,uint8_t a,
                              uint32_t steps,bool absolute,bool sync) {
    (void)addr;(void)v;(void)a;(void)absolute;(void)sync;
    pos_calls++;pos_steps=steps;pos_direction=dir;
}
static bool robot_pid_one_period_limited(float *,float *,float *,int,bool,float,float);
"""

CHECKS = r"""
static void reset_fixture(void) {
    memset(&g_robot,0,sizeof(g_robot)); memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));
    g_robot.joints[0].current_angle=70; g_robot.joints[1].current_angle=120;
    g_robot.joints[2].current_angle=310; g_robot.joints[3].current_angle=330;
    g_robot.joints[4].current_angle=86; g_robot.event_queue=&queued_event;
    g_robot.cur_pos=(struct position){0,-130,-15};
    ROBOT_STATUS_SET(g_robot.status,ROBOT_STATUS_POSE_VALID);
    ROBOT_TARGET_ENABLED=g_robot_motion_abort_requested=missing_feedback=frozen_plant=queue_fail=capture_active=false;
    g_robot_joint_result=g_robot_reset_result=0; g_robot_scan_result=ROBOT_SCAN_RESULT_NONE;
    limit_triggered=pin_fail=stop_fail=false;
    pending=now_ms=abort_at=feedback_fail_at=read_delay_ms=motion_batches=laser_off_calls=pos_calls=0; peak_speed=0;
    memset(commanded,0,sizeof(commanded));
    pin_read_calls=motor_enable_calls=hard_reset_calls=home_write_calls=0;
    g_hard_reset_done=false;last_scan_guard[0]='\0';
}
static void execute_queued(void) {
    assert(pending==1); pending=0; robot_scan_segment_move((robot_scan_segment_t)queued_event.joint_id);
}
int main(void) {
    reset_fixture();
    float err,cur;
    const float raw[]={0,360,359.9997f,345.54f,-14.46f};
    const float ref[]={360,0,0,-14.46f,345.54f};
    for(unsigned i=0;i<5;i++) {
        assert(robot_joint_compare_error(3,raw[i],ref[i],&err,&cur)==0 && err<0.001f);
    }
    assert(robot_joint_compare_error(3,3,363,&err,&cur)==0 && err==0);
    assert(robot_joint_compare_error(3,NAN,0,&err,&cur)!=0);
    assert(robot_joint_compare_error(3,0,100,&err,&cur)!=0);
    g_robot.joints[3].current_angle=360;
    assert(robot_joint_rotate_to(3,DIR_POSITIVE,3,10,200,true)==0);
    assert(pos_calls==1 && pos_steps<1400 && pos_direction==g_joints_init[3].postive_direction);
    assert(robot_view_arc_smoothstep(0)==0 && robot_view_arc_smoothstep(1)==1);
    float previous=0;
    for(int i=0;i<=1200;i++) { float s=robot_view_arc_smoothstep(i/1200.0f); assert(s>=previous-1e-6f && s<=1.000001f); previous=s; }

    reset_fixture();
    assert(robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT)!=pdPASS);
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS);
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)!=pdPASS);
    execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FRONT_READY && pending==0);
    assert(robot_scan_verify_pose(2));
    pending=1;assert(!robot_scan_verify_pose(2));pending=0;
    g_robot_joint_result=ROBOT_JOINT_RESULT_RUNNING;assert(!robot_scan_verify_pose(2));g_robot_joint_result=0;
    missing_feedback=true;assert(!robot_scan_verify_pose(2));missing_feedback=false;
    assert(fabsf(g_robot.joints[0].current_angle-90)<3 && fabsf(g_robot.joints[3].current_angle)<3);
    assert(peak_speed<=ROBOT_SCAN_MAX_SPEED_DEG_S && laser_off_calls>=2);
    for(int j=0;j<5;j++) assert(commanded[j]==0);
    queue_fail=true; assert(robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT)!=pdPASS);
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FRONT_READY);
    queue_fail=false; assert(robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT)==pdPASS);
    execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_RIGHT_READY);
    assert(fabsf(g_robot.joints[0].current_angle-115)<3 && fabsf(g_robot.joints[3].current_angle-35)<3);
    assert(robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT)!=pdPASS);
    assert(!ROBOT_STATUS_IS(g_robot.status,ROBOT_STATUS_POSE_VALID));

    reset_fixture(); assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS);
    execute_queued(); g_robot.joints[1].current_angle+=10;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT)==pdPASS); unsigned before=motion_batches;
    execute_queued(); assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && motion_batches==before);

    reset_fixture(); missing_feedback=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && motion_batches==0);
    reset_fixture(); feedback_fail_at=100;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && now_ms<=110);
    for(int j=0;j<5;j++) assert(commanded[j]==0);
    reset_fixture(); limit_triggered=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    if(ROBOT_JOINT_LIMIT_SWITCHES_ENABLED) {
        assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && motion_batches==0 && pin_read_calls>0);
        assert(strstr(last_scan_guard,"hardware limit"));
    } else {
        assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FRONT_READY && pin_read_calls==0);
    }
    reset_fixture(); pin_fail=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    if(ROBOT_JOINT_LIMIT_SWITCHES_ENABLED) {
        assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && motion_batches==0 && pin_read_calls>0);
    } else {
        assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FRONT_READY && pin_read_calls==0);
    }
    reset_fixture();g_robot.joints[4].current_angle=100;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS);execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && motion_batches==0);
    reset_fixture(); stop_fail=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && g_robot_motion_abort_requested);
    reset_fixture(); read_delay_ms=30;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && now_ms<17000);
    reset_fixture(); frozen_plant=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_FAILED && g_robot_motion_abort_requested);
    assert(strstr(last_scan_guard,"angle/tracking"));
    reset_fixture(); abort_at=100;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)==pdPASS); execute_queued();
    assert(robot_scan_result_get()==ROBOT_SCAN_RESULT_ABORTED && now_ms<200);
    reset_fixture(); ROBOT_TARGET_ENABLED=true;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)!=pdPASS);
    reset_fixture(); g_robot_joint_result=ROBOT_JOINT_RESULT_RUNNING;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)!=pdPASS);
    reset_fixture(); pending=1;
    assert(robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT)!=pdPASS);
    reset_fixture();memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));
    assert(robot_scan_verify_pose(0));
    g_robot.joints[0].current_angle+=5;assert(!robot_scan_verify_pose(0));
    if(!ROBOT_JOINT_LIMIT_SWITCHES_ENABLED) {
        reset_fixture();assert(!robot_joint_hard_reset());
        assert(g_hard_reset_done && motor_enable_calls==0 && hard_reset_calls==0 && home_write_calls==0);
    }
    printf("scan motion/pose verification actual-C checks passed: hardware_limits=%u (no hardware)\n",
           (unsigned)ROBOT_JOINT_LIMIT_SWITCHES_ENABLED);
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default=shutil.which("gcc") or
                        r"E:\STM32\STM32Tool\CLion\CLion 2025.2.3\bin\mingw\bin\gcc.exe")
    args = parser.parse_args()
    env = os.environ.copy()
    env["PATH"] = str(Path(args.gcc).parent) + os.pathsep + env.get("PATH", "")
    robot = (ROOT / "APP/robot.c").read_text(encoding="utf-8")
    soft_reset = function(robot, "robot_joint_soft_reset")
    assert "if (ROBOT_JOINT_LIMIT_SWITCHES_ENABLED)" in soft_reset
    limit_isr = function(robot, "robot_joint_limit_happend")
    assert "!ROBOT_JOINT_LIMIT_SWITCHES_ENABLED" in limit_isr
    init = function(robot, "robot_init")
    assert init.index("#if ROBOT_JOINT_LIMIT_SWITCHES_ENABLED") < init.index("JOINT_LIMIT_1.p_api->open")
    with tempfile.TemporaryDirectory(prefix="scan_motion_") as directory:
        for mode in (None, True):
            exe = Path(directory) / f"check_{mode}.exe"
            subprocess.run([args.gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                            "-x", "c", "-", "-o", str(exe), "-lm"],
                           input=build_source(mode), text=True, encoding="utf-8", check=True, env=env)
            subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()

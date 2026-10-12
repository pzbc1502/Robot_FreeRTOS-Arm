"""Compile the actual batch angle reader and CAN wait against a simulated clock.

No serial, debugger, motor or firmware-download access.
Run: py -3 DEBUG/test_can_angle_feedback.py [--gcc path/to/gcc.exe]
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from test_scan_motion import ROOT, declaration, function


def build_source():
    robot = (ROOT / "APP/robot.c").read_text(encoding="utf-8")
    header = (ROOT / "APP/robot.h").read_text(encoding="utf-8")
    can = (ROOT / "USER_BSP/bsp_can.c").read_text(encoding="utf-8")
    macros = "\n".join(re.findall(
        r"(?m)^#define ROBOT_(?:MAX_JOINT_NUM|CAN_BATCH_FEEDBACK_\w+)[^\n]*", header))
    structs = declaration(header, r"enum motor_dir\s*\{[^}]*\};")
    structs += declaration(header, r"enum dir\s*\{[^}]*\};")
    structs += declaration(header, r"struct joint\s*\{[^}]*\};")
    table = declaration(robot, r"static struct joint g_joints_init\[[^;]*?\};")
    functions = "\n".join(function(can, name) for name in (
        "BSP_CAN_ClearMotorFlags", "BSP_CAN_WaitAllMotors"))
    functions += "\n" + "\n".join(function(robot, name) for name in (
        "robot_angle_normalize", "robot_update_current_angle_from_data", "robot_update_all_angles"))
    return PREAMBLE + macros + "\n" + structs + table + FIXTURE + functions + CHECKS


PREAMBLE = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t TickType_t;
typedef int external_irq_instance_t;
static const int JOINT_LIMIT_1, JOINT_LIMIT_2, JOINT_LIMIT_3,
                 JOINT_LIMIT_4, JOINT_LIMIT_5, JOINT_LIMIT_6;
#define pdMS_TO_TICKS(ms) (ms)
#define taskSCHEDULER_NOT_STARTED 0
#define S_CPOS 0x36u
#define LOG(...) ((void)0)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
"""

FIXTURE = r"""
static struct { struct joint joints[ROBOT_MAX_JOINT_NUM]; } g_robot;
static struct {
    uint8_t motor_rx_flag[6], motor_rx_dlc[6], motor_rx_buf[6][8];
} g_can_context;
static uint32_t now_ms, batch_start, reply_after[6], reply_counts[6];
static unsigned request_count, lock_count, unlock_count, drain_count;
static bool locked, lock_fails, issued[6], invalid_payload;
static uint8_t tx_fail_addr;
static uint32_t HAL_GetTick(void) { return now_ms; }
static TickType_t xTaskGetTickCount(void) { return now_ms; }
static int xTaskGetSchedulerState(void) { return 1; }
static bool BSP_CAN_Lock(uint32_t timeout_ms) {
    assert(timeout_ms==200u && !locked); lock_count++;
    if(lock_fails) return false;
    locked=true; return true;
}
static void BSP_CAN_Unlock(void) { assert(locked); locked=false; unlock_count++; }
static void BSP_CAN_DrainRxSem(void) { assert(locked); drain_count++; }
static bool Emm_V5_Read_Sys_Params(uint8_t addr,uint8_t param) {
    assert(locked && addr>=1 && addr<=6 && param==S_CPOS);
    assert(!issued[addr-1]); request_count++;
    if(addr==tx_fail_addr) return false;
    issued[addr-1]=true; return true;
}
static void clock_step(void) {
    assert(locked); now_ms++;
    for(unsigned j=0;j<6;j++) {
        if(!issued[j] || g_can_context.motor_rx_flag[j] || reply_after[j]==UINT32_MAX ||
           (uint32_t)(now_ms-batch_start)<reply_after[j]) continue;
        uint8_t *rx=g_can_context.motor_rx_buf[j];
        uint32_t counts=reply_counts[j];
        rx[0]=invalid_payload?0x35u:0x36u; rx[1]=0;
        rx[2]=(uint8_t)(counts>>24); rx[3]=(uint8_t)(counts>>16);
        rx[4]=(uint8_t)(counts>>8); rx[5]=(uint8_t)counts; rx[6]=0x6Bu;
        g_can_context.motor_rx_dlc[j]=7; g_can_context.motor_rx_flag[j]=1;
    }
}
#define taskYIELD() clock_step()
#define __NOP() clock_step()
static void reset_fixture(void) {
    now_ms=0; batch_start=0; locked=false; lock_fails=false; invalid_payload=false; tx_fail_addr=0;
    request_count=0; lock_count=0; unlock_count=0; drain_count=0;
    memset(issued,0,sizeof(issued)); memset(&g_can_context,0,sizeof(g_can_context));
    memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));
    for(unsigned j=0;j<6;j++) { reply_after[j]=2; reply_counts[j]=65536u; }
}
static void next_batch(void) {
    assert(!locked); batch_start=now_ms;
    request_count=0; lock_count=0; unlock_count=0; drain_count=0;
    memset(issued,0,sizeof(issued));
}
"""

CHECKS = r"""
static void assert_valid_angles(unsigned count) {
    for(unsigned j=0;j<count;j++) {
        float sign=g_joints_init[j].postive_direction==MOTOR_DIR_CCW?-1.0f:1.0f;
        float expected=robot_angle_normalize(g_joints_init[j].current_angle+
            sign*(float)reply_counts[j]*360.0f/65536.0f/g_joints_init[j].reduction_ratio);
        assert(fabsf(g_robot.joints[j].current_angle-expected)<0.0001f);
    }
}
static void assert_transaction(unsigned count) {
    assert(request_count==count && lock_count==1 && unlock_count==1 && !locked);
}
int main(void) {
    uint32_t missing=0xFFFFFFFFu, elapsed=0xFFFFFFFFu;

    reset_fixture();
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==2);
    assert_valid_angles(5); assert_transaction(5);

    reset_fixture(); reply_after[3]=7;
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==7);
    assert_valid_angles(5); assert_transaction(5);

    reset_fixture(); reply_after[3]=8;
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==8);
    assert_valid_angles(5); assert_transaction(5);

    reset_fixture(); reply_after[1]=8; reply_after[3]=10;
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==10);
    assert_valid_angles(5); assert_transaction(5);

    reset_fixture(); reply_after[3]=15;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x08 && elapsed<=12);
    assert_transaction(5);

    reset_fixture(); reply_after[3]=UINT32_MAX;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x08 && elapsed<=12);
    assert_transaction(5);

    reset_fixture(); for(unsigned j=0;j<5;j++) reply_after[j]=UINT32_MAX;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x1F && elapsed==7);
    assert_transaction(5);

    reset_fixture(); invalid_payload=true;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x1F && elapsed==2);
    assert_transaction(5);

    reset_fixture();
    memset(g_can_context.motor_rx_flag,1,sizeof(g_can_context.motor_rx_flag));
    memset(g_can_context.motor_rx_dlc,7,sizeof(g_can_context.motor_rx_dlc));
    for(unsigned j=0;j<5;j++) reply_after[j]=UINT32_MAX;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x1F && elapsed==7);
    assert_transaction(5);

    reset_fixture(); lock_fails=true;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x1F && request_count==0 && unlock_count==0);

    reset_fixture();
    assert(robot_update_all_angles(0,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==0 && request_count==0 && lock_count==0);

    reset_fixture();
    assert(robot_update_all_angles(7,NULL,NULL)==0);
    assert_valid_angles(6); assert_transaction(6);

    reset_fixture(); now_ms=UINT32_MAX-4u; batch_start=now_ms; reply_after[3]=8;
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==8); assert_transaction(5);

    reset_fixture();
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    next_batch(); reply_after[3]=8;
    for(unsigned j=0;j<5;j++) reply_counts[j]=131072u;
    assert(robot_update_all_angles(5,&missing,&elapsed)==0);
    assert(missing==0 && elapsed==8); assert_valid_angles(5); assert_transaction(5);
    next_batch(); reply_after[3]=UINT32_MAX;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x08 && elapsed<=12); assert_transaction(5);

    reset_fixture(); tx_fail_addr=4;
    assert(robot_update_all_angles(5,&missing,&elapsed)!=0);
    assert(missing==0x1F && elapsed==0 && request_count==4);
    assert(unlock_count==1 && !locked && !issued[3] && !issued[4]);

    puts("15 actual-C CAN batch feedback scenarios passed (no hardware)");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default=shutil.which("gcc") or
                        r"E:\STM32\STM32Tool\CLion\CLion 2025.2.3\bin\mingw\bin\gcc.exe")
    args = parser.parse_args()
    env = os.environ.copy()
    env["PATH"] = str(Path(args.gcc).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="can_angle_feedback_") as directory:
        exe = Path(directory) / "check.exe"
        subprocess.run([args.gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-x", "c", "-", "-o", str(exe), "-lm"],
                       input=build_source(), text=True, encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()

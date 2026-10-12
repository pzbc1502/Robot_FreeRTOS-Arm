"""Exercise actual CAN/Emm/robot velocity transactions with PC-only stubs.

No serial, debugger, CAN adapter or firmware-download access.
Run: py -3 -B DEBUG/test_can_velocity_ack.py [--gcc path/to/gcc.exe]
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import test_can_error_diagnostics as diagnostics
from test_scan_motion import ROOT, declaration, function


def build_source():
    can = (ROOT / "USER_BSP/bsp_can.c").read_text(encoding="utf-8")
    robot = (ROOT / "APP/robot.c").read_text(encoding="utf-8")
    header = (ROOT / "APP/robot.h").read_text(encoding="utf-8")
    emm = (ROOT / "USER_BSP/Emm_V5.c").read_text(encoding="utf-8")
    emm_header = (ROOT / "USER_BSP/Emm_V5.h").read_text(encoding="utf-8")
    types = "typedef int external_irq_instance_t;\n" + "\n".join(
        declaration(header, pattern) for pattern in (
            r"enum motor_dir\s*\{[^}]*\};", r"enum dir\s*\{[^}]*\};",
            r"struct joint\s*\{[^}]*\};"))
    types += declaration(emm_header, r"typedef enum\s*\{.*?\}\s*SysParams_t;")
    macros = "\n".join(re.findall(
        r"(?m)^#define ROBOT_(?:MAX_JOINT_NUM|ARM_JOINT_NUM|JOINT_DEFAULT_ACCELERATION|CAN_VELOCITY_ACK_WAIT_MS)[^\n]*",
        header))
    base = diagnostics.build_source().removesuffix(diagnostics.CHECKS)
    base = base.replace("#define ROBOT_MAX_JOINT_NUM 6", macros)
    base = base.replace("static CAN_Context_t g_can_context;", types + "\nstatic CAN_Context_t g_can_context;")
    base = base.replace("static struct { struct { uint32_t status; } joints[6]; } g_robot;",
                        "static struct { struct joint joints[6]; } g_robot;")
    base = base.replace(function(diagnostics.FIXTURE, "R_CANFD_Write"), FIXTURE)
    functions = "\n".join(function(can, name) for name in (
        "can_SendCmd", "BSP_CAN_ClearVelocityFlags", "BSP_CAN_ExpectVelocityReply",
        "BSP_CAN_WaitVelocityAll"))
    functions += "\n" + "\n".join(function(emm, name) for name in (
        "Emm_V5_Vel_Control", "Emm_V5_Read_Sys_Params"))
    functions += "\n" + "\n".join(function(robot, name) for name in (
        "robot_joint_velocity_send_locked", "robot_joint_velocity_batch_checked",
        "robot_joint_velocity_zero_all"))
    return base + functions + CHECKS


FIXTURE = r"""
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
#define taskSCHEDULER_NOT_STARTED 0
#define ROBOT_STATUS_CLEAR(status, bit) ((status) &= ~(bit))
#define ROBOT_STATUS_LIMIT_ENABLE 2u
static bool locked, lock_fails, abort_requested, immediate_ack;
static bool response_due[6], drop_reply[6];
static uint32_t issue_ms[6], reply_delay[6], sent_velocity[6], sent_angle[6];
static uint8_t reply_status[6], malformed, fail_addr, abort_on_addr;
static unsigned unlock_count, overlap_count;
static uint32_t last_write_id;
static uint8_t last_write_data[8], last_write_dlc;
static bool robot_motion_abort_is_requested(void) { return abort_requested; }
static bool BSP_CAN_Lock(uint32_t ms) {
    assert(ms==20 && !locked); if(lock_fails) return false; locked=true; return true;
}
static void BSP_CAN_Unlock(void) { assert(locked); locked=false; unlock_count++; }
static TickType_t xTaskGetTickCount(void) { return now_ms; }
static int xTaskGetSchedulerState(void) { return 1; }
static void receive_velocity(unsigned j) {
    can_callback_args_t args={.event=CAN_EVENT_RX_COMPLETE};
    args.frame.id=(j+1u)<<8; args.frame.id_mode=CAN_ID_MODE_EXTENDED;
    args.frame.type=CAN_FRAME_TYPE_DATA; args.frame.data_length_code=3;
    args.frame.data[0]=0xF6; args.frame.data[1]=reply_status[j]; args.frame.data[2]=0x6B;
    if(j==3) {
        if(malformed==1) args.frame.data[2]=0;
        if(malformed==2) { args.frame.data_length_code=4; args.frame.data[3]=0x6B; }
        if(malformed==3) args.frame.data[0]=0xFE;
        if(malformed==4) args.frame.id++;
        if(malformed==5) args.frame.id_mode=CAN_ID_MODE_STANDARD;
        if(malformed==6) args.frame.type=CAN_FRAME_TYPE_REMOTE;
        if(malformed==7) args.frame.id=0x600;
        if(malformed==8) { args.frame.data[0]=0; args.frame.data[1]=0xEE; }
    }
    uint32_t previous=ipsr; ipsr=16; canfd0_callback(&args); ipsr=previous;
}
static void clock_step(void) {
    now_ms++;
    for(unsigned j=0;j<6;j++) if(response_due[j] &&
        (uint32_t)(now_ms-issue_ms[j])>=reply_delay[j]) {
        response_due[j]=false;
        if(!drop_reply[j]) receive_velocity(j);
    }
}
#define taskYIELD() clock_step()
static int R_CANFD_Write(void *ctrl, uint32_t buffer, can_frame_t *frame) {
    assert(ctrl==&g_canfd0_ctrl && buffer==0 && frame->options==0 && frame->data_length_code<=8);
    last_write_id=frame->id; last_write_dlc=frame->data_length_code;
    memcpy(last_write_data,frame->data,frame->data_length_code);
    unsigned addr=frame->id>>8; assert(addr>=1 && addr<=6);
    unsigned j=addr-1;
    if(frame->data[0]==0xF6) {
        sent_velocity[j]++;
        if(addr==fail_addr) return 1;
        issue_ms[j]=now_ms; response_due[j]=true;
        if(immediate_ack && !drop_reply[j]) { response_due[j]=false; receive_velocity(j); }
    } else if(frame->data[0]==0x36) {
        sent_angle[j]++;
        if(addr==fail_addr) return 1;
        if(response_due[j]) overlap_count++;
    }
    emit(CAN_EVENT_TX_COMPLETE,0);
    if(addr==abort_on_addr) abort_requested=true;
    return FSP_SUCCESS;
}
"""

CHECKS = r"""
static const float velocity[5]={1,0,0,-1,0.5f};
static void reset_fixture(void) {
    memset(&g_can_context,0,sizeof(g_can_context)); memset(&g_robot,0,sizeof(g_robot));
    s_can_errors=(BSP_CAN_ErrorRecord_t){0}; hardware_info=(can_info_t){0};
    memset(response_due,0,sizeof(response_due)); memset(drop_reply,0,sizeof(drop_reply));
    memset(sent_velocity,0,sizeof(sent_velocity)); memset(sent_angle,0,sizeof(sent_angle));
    memset(issue_ms,0,sizeof(issue_ms));
    for(unsigned j=0;j<6;j++) {
        g_robot.joints[j].reduction_ratio=51; g_robot.joints[j].postive_direction=MOTOR_DIR_CW;
        reply_delay[j]=1; reply_status[j]=2;
    }
    primask=ipsr=log_count=info_count=now_ms=unlock_count=overlap_count=0;
    info_result=write_result=FSP_SUCCESS;
    locked=lock_fails=abort_requested=immediate_ack=false;
    malformed=fail_addr=abort_on_addr=0; last_log[0]='\0';
}
static void assert_closed(void) {
    assert(!locked && primask==0 && g_can_context.velocity_reply_pending==0);
}
static void read_after_success(void) {
    for(unsigned j=0;j<5;j++) assert(Emm_V5_Read_Sys_Params(j+1,S_CPOS));
    assert(overlap_count==0);
}
int main(void) {
    reset_fixture(); immediate_ack=true;
    assert(robot_joint_velocity_batch_checked(velocity,5,200));
    assert(now_ms==0 && unlock_count==1); assert_closed(); read_after_success();

    reset_fixture(); reply_delay[0]=4; reply_delay[3]=3;
    assert(robot_joint_velocity_batch_checked(velocity,5,200));
    assert(now_ms==4); assert_closed(); read_after_success();

    reset_fixture(); drop_reply[3]=true;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200));
    assert(now_ms==ROBOT_CAN_VELOCITY_ACK_WAIT_MS);
    assert(strstr(last_log,"missing=0x08 rejected=0x00")); assert_closed();
    assert(strstr(last_log,"tx_ms=0 wait_ms=4"));
    for(unsigned j=0;j<5;j++) assert(sent_angle[j]==0 && sent_velocity[j]==1);

    reset_fixture(); reply_delay[3]=8;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200)); assert_closed();
    while(now_ms<8) clock_step(); assert(g_can_context.motor_velocity_flag[3]==0);
    reply_delay[3]=1;
    assert(robot_joint_velocity_batch_checked(velocity,5,200)); assert(now_ms==9);

    reset_fixture(); reply_status[3]=0xE2;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200));
    assert(strstr(last_log,"J4 status=0xE2")); assert_closed();

    for(unsigned mode=1;mode<=7;mode++) {
        reset_fixture(); malformed=(uint8_t)mode;
        assert(!robot_joint_velocity_batch_checked(velocity,5,200));
        assert(strstr(last_log,"missing=0x08")); assert_closed();
    }
    reset_fixture(); malformed=8;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200));
    assert(strstr(last_log,"J4 status=0xEE")); assert_closed();

    reset_fixture(); receive_velocity(3);
    assert(g_can_context.motor_velocity_flag[3]==0);
    BSP_CAN_ExpectVelocityReply(4); reply_status[3]=0xE2; receive_velocity(3);
    reply_status[3]=2; receive_velocity(3);
    uint32_t missing=0,rejected=0;
    assert(!BSP_CAN_WaitVelocityAll(5,0,&missing,&rejected)); assert(rejected==0x08);

    reset_fixture();
    for(unsigned j=0;j<5;j++) { assert(BSP_CAN_ExpectVelocityReply(j+1)); receive_velocity(j); }
    receive_velocity(5);
    assert(BSP_CAN_WaitVelocityAll(5,0,&missing,&rejected));
    assert(missing==0 && rejected==0 && g_can_context.motor_velocity_flag[5]==0);

    reset_fixture();
    memset((void *)g_can_context.motor_velocity_flag,1,sizeof(g_can_context.motor_velocity_flag));
    memset((void *)g_can_context.motor_velocity_status,2,sizeof(g_can_context.motor_velocity_status));
    drop_reply[3]=true;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200)); assert_closed();

    reset_fixture(); fail_addr=4;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200));
    assert(now_ms==0 && sent_velocity[3]==1 && sent_velocity[4]==0); assert_closed();
    assert(!robot_joint_velocity_zero_all(5));
    assert(sent_velocity[4]==1 && now_ms==0); assert_closed();

    reset_fixture(); abort_on_addr=2;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200));
    assert(now_ms==0 && sent_velocity[2]==0); assert_closed();
    assert(robot_joint_velocity_zero_all(5));
    for(unsigned j=0;j<5;j++) assert(sent_velocity[j]>=1 && g_robot.joints[j].velocity==0);
    assert(now_ms==0); assert_closed();

    reset_fixture(); lock_fails=true;
    assert(!robot_joint_velocity_batch_checked(velocity,5,200)); assert(unlock_count==0);
    for(unsigned j=0;j<5;j++) assert(sent_velocity[j]==0);
    assert(!robot_joint_velocity_zero_all(5));

    reset_fixture(); now_ms=UINT32_MAX-1u; reply_delay[3]=3;
    assert(robot_joint_velocity_batch_checked(velocity,5,200)); assert(now_ms==1); assert_closed();

    reset_fixture();
    assert(!robot_joint_velocity_batch_checked(NULL,5,200));
    assert(robot_joint_velocity_batch_checked(velocity,0,200));
    assert(!BSP_CAN_ExpectVelocityReply(0) && !BSP_CAN_ExpectVelocityReply(7));
    primask=1; BSP_CAN_ClearVelocityFlags(); assert(primask==1);
    assert(BSP_CAN_ExpectVelocityReply(4) && primask==1); primask=0;
    BSP_CAN_ClearVelocityFlags();
    assert(BSP_CAN_WaitVelocityAll(0,0,NULL,NULL));

    reset_fixture();
    assert(Emm_V5_Vel_Control(4,1,0x1234,200,false));
    assert(last_write_id==0x400 && last_write_dlc==7);
    const uint8_t expected[]={0xF6,1,0x12,0x34,200,0,0x6B};
    assert(memcmp(expected,last_write_data,7)==0);
    assert(Emm_V5_Read_Sys_Params(4,S_CPOS)); assert(overlap_count==1);
    assert(last_write_dlc==2 && last_write_data[0]==0x36 && last_write_data[1]==0x6B);
    assert(!Emm_V5_Read_Sys_Params(4,(SysParams_t)99));
    fail_addr=4;
    assert(!Emm_V5_Read_Sys_Params(4,S_CPOS));
    assert(!can_SendCmd(NULL,3));
    uint8_t short_cmd[]={4}; assert(!can_SendCmd(short_cmd,1));

    puts("actual-C velocity ACK/phase/failure checks passed (no hardware)");
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
    with tempfile.TemporaryDirectory(prefix="can_velocity_ack_") as directory:
        exe = Path(directory) / "check.exe"
        subprocess.run([args.gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                        "-include", "math.h", "-x", "c", "-", "-o", str(exe), "-lm"],
                       input=build_source(), text=True, encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()

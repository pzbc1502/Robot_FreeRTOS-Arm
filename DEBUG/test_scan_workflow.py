"""Compile the actual workflow C with PC-only motion/IO stubs. No hardware."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GCC = shutil.which("gcc") or r"E:\STM32\STM32Tool\CLion\CLion 2025.2.3\bin\mingw\bin\gcc.exe"


def stripped(path):
    return re.sub(r'^\s*#include[^\n]*', '', (ROOT / path).read_text(encoding="utf-8"), flags=re.M)


def build_source(checks=None):
    robot = (ROOT / "APP/robot.h").read_text(encoding="utf-8")
    enums = "\n".join(re.search(r'typedef enum\s*\{[^}]*\}\s*' + name + ';', robot).group()
                      for name in ("robot_auto_result_t", "robot_reset_result_t",
                                   "robot_scan_segment_t", "robot_scan_result_t"))
    return (PREAMBLE + enums + "\n" + stripped("Middle/jetson_vision.h") + stripped("APP/robot_workflow.h") +
            stripped("APP/robot_capture.h") + stripped("APP/robot_target.h") + MOCKS +
            stripped("APP/robot_workflow.c") + (CHECKS if checks is None else checks))


PREAMBLE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#define ROBOT_MAX_JOINT_NUM 6
#define ROBOT_ARM_JOINT_NUM 5
#define ROBOT_STATUS_POSE_VALID 1u
#define ROBOT_STATUS_LIMIT_HAPPENED 2u
#define ROBOT_STATUS_IS(s,f) (((s)&(f))!=0)
#define LOG(...) ((void)0)
#define pdPASS 1
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
struct position { float x,y,z; };
struct { unsigned status; struct {unsigned status;} joints[6]; struct position cur_pos; } g_robot;
static bool aborted, busy, capture_active, verify_ok, queue_ok, laser;
static int segment_count, capture_count, target_starts, laser_on_count, reset_count;
static bool stop_ack_ok=true, stop_feedback_ok=true;
static bool receive_stop_hold;
static unsigned abort_calls, feedback_calls;
static uint32_t mock_now, feedback_delay;
static float stop_angles[5];
static uint8_t requested_action, requested_point;
static int robot_send_auto_event(const struct position *p) { (void)p; return 1; }
static int robot_send_reset_event(bool hard) { (void)hard; reset_count++; aborted=false; return 1; }
static bool robot_is_auto_busy(void) { return busy; }
static bool robot_motion_abort_latched(void) { return aborted; }
static bool robot_motion_abort(void) { aborted=true; busy=false; abort_calls++; return stop_ack_ok; }
static bool robot_motion_stop_feedback(float *angles,uint32_t *sample_ms) {
    feedback_calls++; *sample_ms=mock_now+feedback_delay;
    memcpy(angles,stop_angles,sizeof(stop_angles)); return stop_feedback_ok && aborted;
}
static void BSP_Laser_Off(void) { laser=false; }
static void BSP_Laser_On(void) { laser=true; laser_on_count++; }
static void BSP_TargetReadyLed_Set(bool v) { (void)v; }
static void BSP_TargetOutputLed_Set(bool v) { (void)v; }
'''

MOCKS = r'''
void jetson_set_workflow_stop_hold(bool hold) { receive_stop_hold=hold; }
static robot_scan_result_t scan_result;
static robot_capture_result_t capture_result;
static robot_reset_result_t reset_result;
static robot_auto_result_t auto_result;
static robot_scan_result_t robot_scan_result_get(void) { return scan_result; }
static int robot_send_scan_segment_event(robot_scan_segment_t segment) {
    if(!queue_ok) return 0;
    (void)segment; segment_count++; scan_result=ROBOT_SCAN_RESULT_RUNNING; return 1;
}
static bool robot_scan_verify_pose(uint8_t id) { (void)id; return verify_ok && !busy && !aborted; }
static robot_reset_result_t robot_reset_result_consume(void) {
    robot_reset_result_t r=reset_result; reset_result=ROBOT_RESET_RESULT_NONE; return r;
}
static robot_auto_result_t robot_auto_result_consume(void) {
    robot_auto_result_t r=auto_result; auto_result=ROBOT_AUTO_RESULT_NONE; return r;
}
void robot_capture_init(void) { capture_active=false; capture_result=ROBOT_CAPTURE_RESULT_NONE; }
bool robot_capture_request(uint8_t action,uint8_t id) {
    if(!queue_ok || capture_active || busy || aborted) return false;
    requested_action=action; requested_point=id; capture_count++;
    capture_active=true; capture_result=ROBOT_CAPTURE_RESULT_RUNNING; return true;
}
void robot_capture_step(uint32_t t) { (void)t; }
void robot_capture_cancel(void) { capture_active=false; capture_result=ROBOT_CAPTURE_RESULT_NONE; }
bool robot_capture_is_active(void) { return capture_active; }
robot_capture_result_t robot_capture_result_consume(uint8_t *a,uint8_t *id) {
    *a=requested_action; *id=requested_point;
    robot_capture_result_t r=capture_result;
    if(r!=ROBOT_CAPTURE_RESULT_RUNNING) capture_result=ROBOT_CAPTURE_RESULT_NONE;
    return r;
}
void robot_target_init(void) {}
void robot_target_stop_hold(void) {}
bool robot_target_start_at_current_pose(void) { target_starts++; return true; }
bool robot_target_output_requested(void) { return true; }
robot_target_event_t robot_target_event_consume(void) { return ROBOT_TARGET_EVENT_NONE; }
void robot_target_step(const target_obs_t *p) { (void)p; }
typedef struct { uint8_t seq,event,state,flags,action,result,error; uint32_t id; } Report;
static Report reports[512];
static int report_count;
typedef struct {uint8_t seq,event,value,error;} StatusReport;
static StatusReport status_reports[1024];
static unsigned status_count;
bool jetson_send_scan_status(uint8_t seq,uint8_t event,uint8_t state,uint8_t flags,
    uint8_t action,uint8_t result,uint8_t error,uint32_t id) {
    assert(report_count<512);
    reports[report_count++]=(Report){seq,event,state,flags,action,result,error,id}; return true;
}
bool jetson_send_status(uint8_t seq,uint8_t event,uint8_t value,uint8_t error) {
    assert(status_count<1024);
    status_reports[status_count++]=(StatusReport){seq,event,value,error};return true;
}
bool jetson_send_error(uint8_t seq,uint8_t error) { (void)seq;(void)error;return true; }
'''

CHECKS = r'''
static robot_workflow_obs_t obs;
static void step(void) { mock_now=obs.now_ms; robot_workflow_step(&obs); assert(!laser && laser_on_count==0); obs.now_ms+=10; }
static void fixture(void) {
    aborted=busy=capture_active=laser=false; verify_ok=queue_ok=true;
    stop_ack_ok=stop_feedback_ok=true; abort_calls=feedback_calls=status_count=0;
    receive_stop_hold=false;
    feedback_delay=0; memset(stop_angles,0,sizeof(stop_angles));
    segment_count=capture_count=target_starts=laser_on_count=reset_count=report_count=0;
    scan_result=ROBOT_SCAN_RESULT_NONE; reset_result=ROBOT_RESET_RESULT_NONE;
    auto_result=ROBOT_AUTO_RESULT_NONE;
    memset(&g_robot,0,sizeof(g_robot)); g_robot.status=ROBOT_STATUS_POSE_VALID;
    memset(&obs,0,sizeof(obs)); obs.heartbeat_alive=true; obs.fire_button=true; obs.now_ms=100;
    robot_workflow_init();
    // Enter SAFE_READY through the real old measurement workflow, not by writing its state.
    obs.has_workflow_control=true; obs.workflow_action=1; obs.workflow_seq=9; step();
    obs.has_workflow_control=false; reset_result=ROBOT_RESET_RESULT_OK; step();
    auto_result=ROBOT_AUTO_RESULT_OK; step(); assert(s_workflow.state==FLOW_WAIT_SAFE_DISTANCE);
    obs.has_distance=true; obs.distance_valid=true; obs.distance_mm=400;
    for(int i=1;i<=3;i++){obs.distance_seq=(uint8_t)i;step();}
    obs.has_distance=false; assert(s_workflow.state==FLOW_SAFE_READY);
}
static void control(uint8_t a,uint8_t v,uint32_t id,uint8_t seq) {
    obs.has_scan_control=true; obs.scan_control=(jetson_scan_control_t){a,v,seq,id};
    step(); obs.has_scan_control=false;
}
static void complete_capture(void) { capture_active=false; capture_result=ROBOT_CAPTURE_RESULT_OK;step(); }
static void start(void) { control(1,0,100,20); assert(s_scan.state==1 && capture_count==1); }
static void front(void) {
    start(); complete_capture(); assert(s_scan.state==2 && segment_count==1);
    scan_result=ROBOT_SCAN_RESULT_FRONT_READY;step();assert(s_scan.state==3);
}
static void right(void) {
    front();control(2,1,100,21);assert(s_scan.state==4 && segment_count==2);
    scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;step();assert(s_scan.state==5 && capture_count==2);
}
static void final(void) {
    right();complete_capture();assert(s_scan.state==10 && capture_count==3 && requested_point==2);
    complete_capture();assert(s_scan.state==11 && s_workflow.state==FLOW_SCAN_READY);
}
static Report last(void){assert(report_count);return reports[report_count-1];}
int main(void) {
    fixture();final(); assert(s_scan.flags==7 && last().event==8 && last().seq==20);
    int n=segment_count, c=capture_count;
    obs.now_ms+=5000; control(1,0,100,20);control(2,1,100,21); assert(segment_count==n && capture_count==c);
    control(0,0,100,22);assert(last().event==2 && last().state==11 && last().flags==7);
    control(0,0,99,23);assert(last().error==0x10 && last().flags==0 && last().id==99);
    control(2,0,100,21);assert(last().error==0x0D);
    obs.has_target_control=true;obs.target_value=1;obs.target_seq=50;step();obs.has_target_control=false;
    obs.has_capture_control=true;obs.capture_action=3;obs.capture_point_id=2;obs.capture_seq=51;step();obs.has_capture_control=false;
    assert(target_starts==0 && capture_count==c && s_workflow.state==FLOW_SCAN_READY);
    verify_ok=false;control(0,0,100,24);assert(s_workflow.state==FLOW_FAULT_HOLD && last().flags==0);
    fixture();front();control(2,0,100,21);assert(s_scan.state==9 && last().error==0x11 && segment_count==1);
    fixture();front();obs.now_ms=s_scan.deadline;step();assert(s_scan.error==0x12 && segment_count==1);
    control(2,1,100,21);assert(segment_count==1);
    fixture();front();control(2,1,101,21);assert(last().error==0x10 && segment_count==1);
    control(2,1,100,20);assert(last().error==0x0D && segment_count==1);
    fixture();start();control(2,1,100,21);assert(last().error==0x0C && segment_count==0);
    fixture();front();queue_ok=false;control(2,1,100,21);assert(s_scan.state==9);
    for(int phase=0;phase<6;phase++) {
        fixture();
        if(phase==0) start();
        if(phase==1){start();complete_capture();}
        if(phase==2)front();
        if(phase==3){front();control(2,1,100,21);}
        if(phase==4)right();
        if(phase==5){right();complete_capture();}
        n=segment_count;c=capture_count;
        obs.heartbeat_alive=false;step();assert(s_scan.state==9 && s_scan.flags==0);
        obs.heartbeat_alive=true;step();assert(n==segment_count && c==capture_count);
    }
    fixture();front();obs.has_workflow_control=true;obs.workflow_action=3;obs.workflow_seq=60;step();
    obs.has_workflow_control=false;assert(s_scan.state==8);scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;step();
    control(2,1,100,21);assert(segment_count==1 && capture_count==1);
    fixture();right();verify_ok=false;complete_capture();assert(s_scan.state==9 && capture_count==2);
    fixture();right();complete_capture();verify_ok=false;complete_capture();assert(s_scan.state==9);
    fixture();start();obs.now_ms=s_scan.deadline;step();assert(s_scan.error==0x13);
    fixture();front();verify_ok=false;obs.now_ms=s_scan.next_verify;step();assert(s_scan.error==4);
    fixture();final();verify_ok=false;obs.now_ms=s_scan.next_verify;step();assert(s_scan.flags==0);
    fixture();right();complete_capture();obs.now_ms=s_scan.deadline;step();assert(s_scan.error==0x13);
    fixture();start();capture_active=false;capture_result=ROBOT_CAPTURE_RESULT_FAILED;step();assert(s_scan.state==9);
    fixture();front();obs.limit_triggered=true;step();assert(s_scan.error==8);
    fixture();final();obs.has_workflow_control=true;obs.workflow_action=2;obs.workflow_seq=62;step();
    obs.has_workflow_control=false;assert(s_scan.state==12 && s_scan.flags==0);
    control(0,0,100,23);assert(last().state==12 && last().flags==0);
    fixture();control(0,0,0,23);assert(last().state==0 && capture_count==0);
    fixture();front();control(1,0,101,24);assert(last().error==0x0C && capture_count==1);
    puts("actual-C scan workflow: normal chain, replay, faults, timeouts, legacy gate passed (no hardware)");
}
'''


def main():
    env = os.environ.copy()
    env["PATH"] = str(Path(GCC).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="scan_workflow_") as d:
        exe = Path(d) / "check.exe"
        subprocess.run([GCC, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                        "-x", "c", "-", "-o", str(exe)], input=build_source(), text=True,
                       encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()

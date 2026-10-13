"""Exercise actual MCU stop-confirmation C with offline CAN/clock substitutes."""
import os
from pathlib import Path
import subprocess
import tempfile

import test_scan_workflow as workflow
from test_scan_motion import function


WORKFLOW_CHECKS = workflow.CHECKS.split("int main(void)")[0] + r'''
static unsigned successes(uint8_t seq) {
    unsigned n=0;
    for(unsigned i=0;i<status_count;i++) {
        StatusReport r=status_reports[i];
        if(r.seq==seq && r.event==RA6_TO_JETSON_WORKFLOW &&
           r.value==JETSON_WORKFLOW_ABORTED_HOLD && r.error==0) n++;
    }
    return n;
}
static bool reported(uint8_t seq,uint8_t event,uint8_t value,uint8_t error) {
    for(unsigned i=0;i<status_count;i++) {
        StatusReport r=status_reports[i];
        if(r.seq==seq && r.event==event && r.value==value && r.error==error) return true;
    }
    return false;
}
static void abort_command(uint8_t seq) {
    obs.has_workflow_control=true; obs.workflow_action=JETSON_WORKFLOW_ACTION_ABORT_HOLD;
    obs.workflow_seq=seq; step(); obs.has_workflow_control=false;
}
static void at(uint32_t t) {obs.now_ms=t;step();}
static void sample(void) {at(s_stop.next_sample);}
static void deny_start(uint8_t seq,uint8_t error) {
    int n=reset_count;
    obs.has_workflow_control=true;obs.workflow_action=1;obs.workflow_seq=seq;step();
    obs.has_workflow_control=false;
    assert(reset_count==n && reported(seq,RA6_TO_JETSON_COMMAND_ACK,0,error));
}
int main(void) {
    unsigned cases=0;
    fixture();scanning();abort_command(60);
    assert(reported(60,RA6_TO_JETSON_COMMAND_ACK,1,0) && !successes(60));
    assert(s_stop.pending && aborted && s_scan.state==8 && s_scan.flags==0);
    assert(receive_stop_hold);
    int homes=reset_count, segments=segment_count, captures=capture_count;
    sample();assert(!successes(60));sample();assert(successes(60)==1);
    assert(!s_stop.pending && !s_stop.restart_locked);
    assert(!receive_stop_hold);
    assert(reset_count==homes && segment_count==segments && capture_count==captures);cases++;

    fixture();stop_ack_ok=false;abort_command(60);
    assert(reported(60,RA6_TO_JETSON_COMMAND_ACK,1,0));
    assert(reported(60,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,6));
    assert(!successes(60) && s_stop.restart_locked && aborted && !feedback_calls);
    assert(receive_stop_hold);
    deny_start(61,6);cases++;

    fixture();stop_feedback_ok=false;abort_command(60);
    deny_start(61,JETSON_ERROR_BUSY);
    uint32_t deadline=s_stop.deadline;
    abort_command(60);assert(s_stop.deadline==deadline && abort_calls==1);
    at(deadline);assert(!s_stop.pending && s_stop.restart_locked && !successes(60));
    deny_start(62,6);cases++;

    fixture();abort_command(60);sample();stop_feedback_ok=false;sample();
    assert(s_stop.stable_count==0);stop_feedback_ok=true;sample();sample();
    assert(!successes(60));sample();assert(successes(60)==1);cases++;

    fixture();abort_command(60);stop_angles[1]=NAN;sample();
    assert(!successes(60) && s_stop.stable_count==0);
    stop_angles[1]=0;sample();sample();sample();assert(successes(60)==1);cases++;

    fixture();abort_command(60);
    stop_angles[0]=0.06f;sample();stop_angles[0]=0.12f;sample();
    assert(!successes(60) && s_stop.stable_count==1);
    sample();sample();assert(successes(60)==1);cases++;

    fixture();stop_angles[3]=359.98f;abort_command(60);
    stop_angles[3]=0.02f;sample();stop_angles[3]=0.00f;sample();
    assert(successes(60)==1);cases++;

    fixture();abort_command(60);uint32_t first=s_stop.anchor_ms;
    at(first+99);assert(s_stop.stable_count==1);
    at(first+100);assert(!successes(60));at(first+199);assert(!successes(60));
    at(first+200);assert(successes(60)==1);cases++;

    fixture();obs.now_ms=UINT32_MAX-150;abort_command(60);
    sample();assert(!successes(60));sample();assert(successes(60)==1);cases++;

    fixture();obs.now_ms=UINT32_MAX-150;stop_feedback_ok=false;abort_command(60);
    at(s_stop.deadline);assert(s_stop.restart_locked && !successes(60));cases++;

    fixture();abort_command(60);sample();feedback_delay=ROBOT_WORKFLOW_STOP_TIMEOUT_MS;
    sample();assert(!successes(60) && s_stop.restart_locked);cases++;

    fixture();abort_command(60);aborted=false;sample();sample();
    at(s_stop.deadline);assert(!successes(60) && s_stop.restart_locked);cases++;

    fixture();stop_feedback_ok=false;abort_command(60);deadline=s_stop.deadline;
    abort_command(61);assert(s_stop.deadline==deadline && abort_calls==1);
    stop_feedback_ok=true;sample();sample();sample();
    assert(successes(60)==1 && successes(61)==1);
    unsigned before=abort_calls;abort_command(60);abort_command(61);
    assert(abort_calls==before && successes(60)==2 && successes(61)==2);cases++;

    fixture();stop_feedback_ok=false;abort_command(60);deadline=s_stop.deadline;
    for(uint8_t seq=61;seq<68;seq++) abort_command(seq);
    assert(abort_calls==1 && s_stop.deadline==deadline);
    abort_command(68);assert(reported(68,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_BUSY));
    assert(s_stop.deadline==deadline && abort_calls==1);
    stop_feedback_ok=true;sample();sample();sample();
    for(uint8_t seq=60;seq<68;seq++) assert(successes(seq)==1);
    assert(!successes(68));cases++;

    fixture();stop_feedback_ok=false;abort_command(60);
    command_cache_entry_t *entry=command_find(JETSON_MSG_WORKFLOW_CTRL,60);
    entry->received_ms=obs.now_ms-WORKFLOW_COMMAND_CACHE_TTL_MS-1;
    abort_command(60);assert(abort_calls==1 && command_stop_pending(entry));cases++;

    fixture();abort_command(60);obs.limit_triggered=true;step();
    assert(!s_stop.pending && s_stop.restart_locked && !successes(60));
    assert(reported(60,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,8));
    obs.limit_triggered=false;step();deny_start(61,6);cases++;

    fixture();scanning();obs.heartbeat_alive=false;step();obs.heartbeat_alive=true;
    deny_start(61,6);abort_command(62);sample();sample();assert(successes(62)==1);
    obs.has_workflow_control=true;obs.workflow_action=1;obs.workflow_seq=63;step();
    assert(s_workflow.state==FLOW_HOMING && reported(63,RA6_TO_JETSON_COMMAND_ACK,1,0));cases++;

    fixture();scanning();stop_ack_ok=false;abort_command(60);
    assert(s_scan.state==9 && s_scan.flags==0 && s_scan.error==6);
    scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;control(0,0,100,70);
    assert(s_scan.state==9 && !successes(60));cases++;

    fixture();stop_ack_ok=false;abort_command(60);stop_ack_ok=true;
    abort_command(60);assert(abort_calls==1 && !successes(60));
    abort_command(61);sample();sample();assert(successes(61)==1 && !successes(60));cases++;

    fixture();abort_command(60);obs.has_workflow_control=true;
    obs.workflow_action=1;obs.workflow_seq=60;step();obs.has_workflow_control=false;
    assert(reported(60,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_SEQ_CONFLICT));
    sample();sample();assert(successes(60)==1);cases++;
    fixture();scanning();obs.heartbeat_alive=false;abort_command(60);
    assert(s_workflow.state==FLOW_ABORT_HOLD && s_stop.pending && receive_stop_hold);
    assert(reported(60,RA6_TO_JETSON_COMMAND_ACK,1,0) && abort_calls==1);
    sample();sample();assert(successes(60)==1 && !receive_stop_hold);
    deny_start(61,JETSON_ERROR_HEARTBEAT_TIMEOUT);cases++;

    for(unsigned stage=0;stage<8;stage++) {
        fixture();
        switch(stage) {
            case 0: s_workflow.state=FLOW_HOMING; break;
            case 1: s_workflow.state=FLOW_MEASURE_POSITION; break;
            case 2: start();break;
            case 3: start();complete_prepare();break;
            case 4: scanning();break;
            case 5: scanning();obs.now_ms+=15000;break;
            case 6: right();break;
            case 7: right();complete_capture();break;
        }
        obs.heartbeat_alive=false;abort_command(60);
        assert(s_stop.pending && receive_stop_hold && !successes(60));
        int old_captures=capture_count,old_segments=segment_count,old_resets=reset_count;
        scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;
        reset_result=ROBOT_RESET_RESULT_OK;auto_result=ROBOT_AUTO_RESULT_OK;
        sample();sample();
        assert(successes(60)==1 && !receive_stop_hold && s_workflow.state==FLOW_ABORT_HOLD);
        assert(capture_count==old_captures && segment_count==old_segments && reset_count==old_resets);
        cases++;
    }

    fixture();scanning();obs.limit_triggered=true;abort_command(60);
    assert(!successes(60) && receive_stop_hold);
    assert(reported(60,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_SAFETY));cases++;
    printf("actual-C stop workflow: %u offline scenarios passed (no hardware)\n",cases);
}
'''


LOW_LEVEL_STUBS = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#define ROBOT_ARM_JOINT_NUM 5
#define ROBOT_STATUS_AUTO_BUSY 1
#define ROBOT_STATUS_POSE_VALID 2
#define ROBOT_STATUS_POSE_DEGRADED 4
#define ROBOT_STATUS_CLEAR(s,f) ((s)&=~(f))
#define ROBOT_STATUS_SET(s,f) ((s)|=(f))
#define ROBOT_AUTO_RESULT_ABORTED 4
#define ROBOT_JOINT_RESULT_ABORTED 4
#define ROBOT_RESET_RESULT_ABORTED 4
#define ROBOT_SCAN_RESULT_RUNNING 1
#define ROBOT_SCAN_RESULT_FRONT_READY 2
#define ROBOT_SCAN_RESULT_LEFT_READY 6
#define ROBOT_SCAN_RESULT_ABORTED 5
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static struct {unsigned status;void *event_queue;struct {float current_angle;} joints[5];} g_robot;
static struct {bool stop_requested;} g_visual_servo;
static bool g_robot_motion_abort_requested, g_robot_control_event_active, stop_ack_ok;
static unsigned g_robot_auto_result,g_robot_joint_result,g_robot_reset_result,g_robot_scan_result;
static uint32_t g_robot_motion_generation=1,now_ms;
static unsigned pending,reads,read_mode;
static bool robot_joint_stop_all_checked(uint8_t n) {assert(n==5);return stop_ack_ok;}
static unsigned uxQueueMessagesWaiting(void *q) {(void)q;return pending;}
static uint32_t HAL_GetTick(void) {return now_ms;}
static float robot_angle_normalize(float a) {a=fmodf(a,360);return a<0?a+360:a;}
static int robot_update_all_angles(uint8_t n,uint32_t *missing,uint32_t *elapsed) {
    assert(n==5 && !elapsed);reads++;now_ms+=6;*missing=read_mode==1?1:0;
    if(read_mode==2) return 1;
    if(read_mode==3) g_robot_motion_generation++;
    if(read_mode==4) g_robot_motion_abort_requested=false;
    if(read_mode==5) g_robot_control_event_active=true;
    if(read_mode==6) pending=1;
    return 0;
}
'''

LOW_LEVEL_CHECKS = r'''
static void fixture(void) {
    g_robot.event_queue=&g_robot;g_robot.status=3;
    g_robot_motion_abort_requested=g_robot_control_event_active=false;
    stop_ack_ok=true;pending=reads=read_mode=0;now_ms=100;
    for(unsigned i=0;i<5;i++)g_robot.joints[i].current_angle=(float)i;
}
int main(void) {
    float angles[5];uint32_t sampled=0;unsigned cases=0;
    fixture();stop_ack_ok=false;assert(!robot_motion_abort());
    assert(g_robot_motion_abort_requested && g_visual_servo.stop_requested);
    assert((g_robot.status&4) && !(g_robot.status&3));cases++;
    fixture();assert(robot_motion_abort());uint32_t gen=g_robot_motion_generation;
    assert(robot_motion_abort() && g_robot_motion_generation==gen);cases++;
    assert(robot_motion_stop_feedback(angles,&sampled) && sampled==106 && reads==1);cases++;
    fixture();assert(!robot_motion_stop_feedback(angles,&sampled) && !reads);cases++;
    fixture();assert(robot_motion_abort());g_robot_control_event_active=true;
    assert(!robot_motion_stop_feedback(angles,&sampled) && !reads);cases++;
    fixture();assert(robot_motion_abort());pending=1;
    assert(!robot_motion_stop_feedback(angles,&sampled) && !reads);cases++;
    for(unsigned mode=1;mode<=6;mode++) {
        fixture();assert(robot_motion_abort());read_mode=mode;
        assert(!robot_motion_stop_feedback(angles,&sampled) && reads==1);cases++;
    }
    fixture();assert(robot_motion_abort());g_robot.joints[4].current_angle=NAN;
    assert(!robot_motion_stop_feedback(angles,&sampled));cases++;
    fixture();assert(robot_motion_abort());g_robot.joints[3].current_angle=-0.02f;
    g_robot.joints[4].current_angle=360;
    assert(robot_motion_stop_feedback(angles,&sampled));
    assert(fabsf(angles[3]-359.98f)<0.001f && angles[4]==0);cases++;
    assert(!robot_motion_stop_feedback(NULL,&sampled));
    assert(!robot_motion_stop_feedback(angles,NULL));g_robot.event_queue=NULL;
    assert(!robot_motion_stop_feedback(angles,&sampled));cases++;
    printf("actual-C stop feedback/latch: %u offline scenarios passed (no hardware)\n",cases);
}
'''


def run_c(source, label):
    env = os.environ.copy()
    env["PATH"] = str(Path(workflow.GCC).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="stop_confirmation_") as directory:
        exe = Path(directory) / (label + ".exe")
        subprocess.run([workflow.GCC, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-function", "-x", "c", "-", "-o", str(exe), "-lm"],
                       input=source, text=True, encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


def main():
    robot = (workflow.ROOT / "APP/robot.c").read_text(encoding="utf-8")
    dispatcher = function(robot, "robot_control_task")
    assert "g_robot_control_event_active = true;" in dispatcher
    assert "event_done:\n        g_robot_control_event_active = false;" in dispatcher
    assert "continue;" not in dispatcher
    run_c(LOW_LEVEL_STUBS + function(robot, "robot_motion_abort") +
          function(robot, "robot_motion_stop_feedback") + LOW_LEVEL_CHECKS, "feedback")
    run_c(workflow.build_source(WORKFLOW_CHECKS), "workflow")


if __name__ == "__main__":
    main()

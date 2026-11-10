"""Compile the actual debug parser, event provenance and workflow gate. No IO."""
import re

import test_scan_workflow as workflow
from test_scan_motion import function
from test_stop_confirmation import run_c


PREAMBLE = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <assert.h>
#define pdPASS 1
#define pdFAIL 0
#define ROBOT_MAX_JOINT_NUM 6
#define ROBOT_STATUS_RMODE_ENABLE 1
#define ROBOT_STATUS_SET(s,f) ((s)|=(f))
#define ROBOT_STATUS_CLEAR(s,f) ((s)&=~(f))
#define ROBOT_STATUS_IS(s,f) ((s)&(f))
#define ROBOT_REMOTE_MAX_VELOCITY 20
#define ROBOT_REMOTE_MAX_RPM 10
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define LOG(...) do { if(0) printf(__VA_ARGS__); } while(0)
struct position {float x,y,z;};
struct robot_cmd {int type;char cmd[160];};
struct robot_event {int type;uint32_t debug_epoch,generation;};
struct {unsigned status;void *cmd_service_handle;} g_robot;
struct {float vx,vy,vz,rx,ry;} g_remote_control;
enum {ROBOT_JOINT_ABS_ROTATE=2,ROBOT_AUTO_EVENT,ROBOT_JOINTS_SYNC_EVENT,ROBOT_LIMIT_SWITCH_EVENT,ROBOT_READ_ALL_EVENT};
typedef int gripper_result_t;
typedef int gripper_fruit_t;
enum {GRIPPER_OK=0,GRIPPER_FRUIT_MAX=3};
typedef struct {unsigned current_limit_ma;} gripper_grasp_cfg_t;
static unsigned calls, stops;
static uint32_t epoch=1,g_robot_motion_generation=1;
static bool busy;
static void *current_task;
static uint32_t robot_workflow_debug_epoch(void){return epoch;}
static void *xTaskGetCurrentTaskHandle(void){return current_task;}
static bool robot_is_auto_busy(void){return busy;}
static bool robot_capture_is_active(void){return false;}
static bool robot_is_visual_servo_active(void){return false;}
static void vTaskDelay(unsigned n){(void)n;}
static int robot_send_remote_event(void){calls++;return 1;}
static int robot_send_rel_rotate_event(uint8_t j,float a){(void)j;(void)a;calls++;return 1;}
static int robot_send_abs_rotate_event(uint8_t j,float a){(void)j;(void)a;calls++;return 1;}
static int robot_send_auto_event(struct position *p){(void)p;calls++;return 1;}
static int robot_send_time_func_event(float t,float r){(void)t;(void)r;calls++;return 1;}
static int robot_send_view_arc_event(float t){(void)t;calls++;return 1;}
static int robot_send_reset_event(bool h){(void)h;calls++;return 1;}
static int robot_send_read_all_event(void){return 1;}
static bool robot_motion_abort(void){stops++;return true;}
static void robot_target_disable_request(void){stops++;}
static void BSP_Laser_Off(void){stops++;}
static bool gripper_get_preset(int t,gripper_grasp_cfg_t *c){(void)t;(void)c;return true;}
static int gripper_stop(int j){(void)j;stops++;return 0;}
static int gripper_open(void){calls++;return 0;}
static int gripper_read_phase_current_ma(int j,uint16_t *v){(void)j;*v=0;return 0;}
static int gripper_grasp_force(gripper_grasp_cfg_t *c,uint16_t *v){(void)c;*v=0;calls++;return 0;}
'''

CHECKS = r'''
static void send(const char *text){struct robot_cmd c={0};snprintf(c.cmd,sizeof(c.cmd),"%s",text);robot_uart1_handle(&c);}
int main(void){
    const char *invalid[]={"abs_rotate","abs_rotate 0","abs_rotate 0 nope","abs_rotate 0 nan",
      "abs_rotate -1 0","abs_rotate 1.5 0","abs_rotate 6 0","auto 1 2","auto 1 2 3 4",
      "auto 1 2 3junk","auto 1e100 0 0","gripper_grasp 256","gripper_grasp 1.5","soft_reset 1"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);i++){unsigned n=calls;send(invalid[i]);assert(calls==n);}
    send("abs_rotate 0 70");send("auto 0 -130 -15");assert(calls==2);
    busy=true;send("soft_reset");assert(calls==2);busy=false;
    epoch=0;
    const char *motion[]={"abs_rotate 0 70","auto 0 -130 -15","soft_reset","hard_reset","zero","gripper_open"};
    for(unsigned i=0;i<sizeof(motion)/sizeof(*motion);i++)send(motion[i]);
    assert(calls==2);send("motion_abort");send("laser_off");assert(stops==2);
    struct robot_cmd mqtt={0};snprintf(mqtt.cmd,sizeof(mqtt.cmd),"+MQTTSUBRECV:0,\"arm/change\",30,[MCU][%d][0 70 0 0 0 0]",ROBOT_JOINT_ABS_ROTATE);
    robot_mqtt_handle(&mqtt);assert(calls==2);
    epoch=3;robot_mqtt_handle(&mqtt);assert(calls==3);
    g_robot.cmd_service_handle=(void*)1;current_task=(void*)1;
    struct robot_event e={.type=ROBOT_AUTO_EVENT};robot_event_stamp(&e);
    assert(e.debug_epoch==3 && !robot_event_is_stale(&e));
    epoch=0;assert(robot_event_is_stale(&e));epoch=4;assert(robot_event_is_stale(&e));
    current_task=(void*)2;e.debug_epoch=0;robot_event_stamp(&e);
    assert(e.debug_epoch==0 && !robot_event_is_stale(&e));
    g_robot_motion_generation++;assert(robot_event_is_stale(&e));
    puts("debug parser, MQTT admission and queued-event provenance passed (no hardware)");
}
'''


def main():
    robot = (workflow.ROOT / 'APP/robot.c').read_text(encoding='utf-8')
    source = PREAMBLE + workflow.stripped('Middle/robot_cmd.h')
    source += workflow.stripped('Middle/robot_cmd.c')
    source += function(robot, 'robot_event_stamp') + function(robot, 'robot_event_is_stale')
    run_c(source + CHECKS, 'debug_guards')
    run_c(workflow.build_source(checks=r'''
int main(void) {
    assert(robot_workflow_debug_epoch()==0);
    robot_workflow_init();uint32_t old=robot_workflow_debug_epoch();assert(old);
    for(unsigned state=1;state<=FLOW_SCAN_READY;state++) {
        workflow_enter((workflow_state_t)state,0);assert(robot_workflow_debug_epoch()==0);
    }
    workflow_enter(FLOW_IDLE,0);assert(robot_workflow_debug_epoch()!=old);
    puts("all workflow states gate debug motion; old epoch remains invalid (no hardware)");
}
'''), 'workflow_debug_epoch')


if __name__ == '__main__':
    main()

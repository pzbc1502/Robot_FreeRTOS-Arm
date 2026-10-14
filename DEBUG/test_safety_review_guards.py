"""Actual-C safety review regressions. All UART, clocks and motors are fake."""
import argparse
import subprocess

import test_ik_visual_guards as visual
import test_scan_motion as motion
import test_scan_target_handoff as target
import test_scan_uart as uart
import test_scan_workflow as workflow
import test_can_error_diagnostics as can
from test_stop_confirmation import run_c


CHECK = r'''
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"review guard failed: %s\n",#x);return 1;} } while(0)
'''


def stop_source():
    source = visual.visual_source().removesuffix(visual.VISUAL_CHECKS)
    source = "static unsigned stop_failure_mode,fault_visible_during_abort;\n" + source
    zero = motion.function(source, "robot_joint_velocity_zero_all")
    source = source.replace(zero, zero.replace("(void)n;", "(void)n;if(now_ms>=5000 && (stop_failure_mode&1))return false;"))
    lock = motion.function(source, "BSP_CAN_Lock")
    source = source.replace(lock, lock.replace("return true;", "return now_ms<5000 || !(stop_failure_mode&2);"))
    lock = motion.function(source, "BSP_CAN_Lock")
    source = source.replace(lock, lock.replace("(void)ms;", "(void)ms;if(g_robot_motion_abort_requested && now_ms>=5000)fault_visible_during_abort=(g_visual_servo.fault!=0);"))
    wait = motion.function(source, "BSP_CAN_WaitStopAll")
    source = source.replace(wait, wait.replace("return !stop_fail;", "return !stop_fail && (now_ms<5000 || !(stop_failure_mode&4));"))
    return source + visual.VISUAL_CHECKS.split("int main(void)")[0] + CHECK + r'''
int main(void) {
    for(unsigned mode=1;mode<8;mode++) {
        fixture();stop_failure_mode=mode;fault_visible_during_abort=0;int result=robot_visual_servo_run();
        CHECK(result!=0 && g_visual_servo.fault!=ROBOT_VISUAL_SERVO_FAULT_NONE);
        CHECK(g_robot_motion_abort_requested && !ROBOT_STATUS_IS(g_robot.status,ROBOT_STATUS_POSE_VALID));
        CHECK(fault_visible_during_abort);
    }
    fixture();stop_failure_mode=0;CHECK(robot_visual_servo_run()==0);
    puts("review visual exit: 7 stop failures plus normal exit passed (no hardware)");
}
'''


def relative_source():
    return motion.build_source().removesuffix(motion.CHECKS) + motion.CHECKS.split("int main(void)")[0] + CHECK + r'''
int main(void) {
    reset_fixture();g_robot.joints[3].current_angle=85;
    CHECK(robot_joint_rotate_to(3,DIR_POSITIVE,10,1,1,false)!=0);
    CHECK(pos_calls==0 && g_robot.joints[3].current_angle==85);
    CHECK(robot_joint_rotate_to(3,DIR_POSITIVE,360,1,1,false)!=0 && pos_calls==0);
    CHECK(robot_joint_rotate_to(3,DIR_POSITIVE,5,1,1,false)==0 && pos_calls==1);
    reset_fixture();g_robot.joints[3].current_angle=345;
    CHECK(robot_joint_rotate_to(3,DIR_POSITIVE,3,1,1,false)==0);
    CHECK(fabsf(g_robot.joints[3].current_angle+12)<0.001f);
    reset_fixture();g_robot.joints[3].current_angle=270;
    CHECK(robot_joint_rotate_to(3,DIR_NEGATIVE,3,1,1,false)!=0 && pos_calls==0);
    reset_fixture();CHECK(robot_joint_rotate_to(5,DIR_POSITIVE,1e30f,1,1,false)!=0 && pos_calls==0);
    for(unsigned j=1;j<=4;j++) {
        reset_fixture();float lo=g_robot.joints[j].min_angle,hi=g_robot.joints[j].max_angle;
        g_robot.joints[j].current_angle=(lo+hi)/2;
        CHECK(robot_joint_rotate_to(j,DIR_POSITIVE,(hi-lo)/2+1,1,1,false)!=0 && pos_calls==0);
        CHECK(robot_joint_rotate_to(j,DIR_POSITIVE,(hi-lo)/2,1,1,false)==0 && pos_calls==1);
        reset_fixture();g_robot.joints[j].current_angle=lo;
        CHECK(robot_joint_rotate_to(j,DIR_NEGATIVE,1,1,1,false)!=0 && pos_calls==0);
    }
    puts("review relative rotation: limit, signed mapping and numeric rejection passed (no hardware)");
}
'''


def target_source(fail_start=True):
    source = workflow.build_source(checks="")
    for name in ("robot_target_init", "robot_target_stop_hold", "robot_target_start_at_current_pose",
                 "robot_target_output_requested", "robot_target_event_consume", "robot_target_step"):
        source = source.replace(motion.function(source, name), "\n")
    io = target.TARGET_IO
    start = motion.function(io, "robot_visual_servo_start")
    if fail_start:
        io = io.replace(start, "static int robot_visual_servo_start(void) {servo_starts++;return -1;}")
    source += io + "\n#define robot_target_start_at_current_pose target_start_real\n"
    source += workflow.stripped("APP/robot_target.c") + "\n#undef robot_target_start_at_current_pose\n"
    source += "bool robot_target_start_at_current_pose(void) {target_starts++;return target_start_ok && target_start_real();}\n"
    source += target.CHECKS.split("int main(void)")[0] + CHECK
    if not fail_start:
        return source + r'''
int main(void) {
    handoff();obs.now_ms=1000;obs.vision_received_ms=501;
    mock_now=obs.now_ms;robot_workflow_step(&obs);
    CHECK(s_target.last_vision_ms==501 && s_target.state==ROBOT_TARGET_STATE_ALIGN);
    obs.now_ms=1002;obs.has_vision=false;mock_now=obs.now_ms;robot_workflow_step(&obs);
    CHECK(!laser && s_target.state==ROBOT_TARGET_STATE_WAIT_DETECT);
    CHECK(s_target.last_vision_ms==501);
    handoff();servo_fault=1;aborted=true;tick();
    CHECK(s_workflow.state==FLOW_FAULT_HOLD && reported(s_workflow.workflow_seq,
          RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,JETSON_ERROR_MOTION_FAILED));
    puts("review target receive age and visual-fault classification passed (no hardware)");
}
'''
    return source + r'''
int main(void) {
    handoff();tick();
    CHECK(s_workflow.state==FLOW_FAULT_HOLD && s_workflow.fault_latched && !laser);
    CHECK(reported(50,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,JETSON_ERROR_MOTION_FAILED) || aborted);
    CHECK(!reported(50,RA6_TO_JETSON_VISION_STATE,1,0));
    s_target.event=ROBOT_TARGET_EVENT_READY;target_set_event(ROBOT_TARGET_EVENT_FAULT);
    target_set_event(ROBOT_TARGET_EVENT_VISION_RECOVERED);
    CHECK(robot_target_event_consume()==ROBOT_TARGET_EVENT_FAULT);
    puts("review target events: FAULT overrides recovery/READY (no hardware)");
}
'''


def uart_source():
    vectors = {
        "on": uart.build_unified_frame(2, 61, b"\x01"),
        "off": uart.build_unified_frame(2, 60, b"\x00"),
        "heartbeat": uart.build_unified_frame(1, 20, bytes(4)),
        "vision": uart.build_unified_frame(3, 21, bytes(4) + b"\x01"),
        "depth": uart.build_unified_frame(5, 22, b"\x90\x01\x01"),
        "invalid_on": uart.build_unified_frame(2, 62, b"\x02"),
        "off_two": uart.build_unified_frame(2, 63, b"\x00"),
    }
    arrays = "\n".join("static const uint8_t " + name + "[]={" +
                       ",".join(map(str, data)) + "};" for name, data in vectors.items())
    return (uart.PREAMBLE + workflow.stripped("Middle/crc16.h") +
            workflow.stripped("Middle/jetson_vision.h") + workflow.stripped("Middle/crc16.c") +
            workflow.stripped("Middle/jetson_vision.c") + arrays + CHECK + r'''
static void enqueue(const uint8_t *p,unsigned n) {
    for(unsigned i=0;i<n;i++)jetson_notify_rx_char_from_isr(p[i]);
}
int main(void) {
    uint8_t value,seq;int16_t x,y;bool valid;uint32_t received;uint16_t distance;unsigned cases=0;
    jetson_vision_init();enqueue(off,sizeof(off));enqueue(on,sizeof(on));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==60);
    CHECK(!jetson_get_target_control_value(&value,&seq));
    CHECK(tx[3]==0x81 && tx[4]==61 && tx[6]==0x21 && tx[7]==0 && tx[8]==JETSON_ERROR_BUSY);
    cases++;
    jetson_vision_init();enqueue(on,sizeof(on));enqueue(off,sizeof(off));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==60);
    CHECK(tx[4]==61 && tx[8]==JETSON_ERROR_BUSY);
    cases++;
    jetson_vision_init();enqueue(off,sizeof(off));enqueue(off_two,sizeof(off_two));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==60);
    CHECK(tx[4]==63 && tx[8]==JETSON_ERROR_BUSY);cases++;
    enqueue(off_two,sizeof(off_two));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==63);cases++;
    jetson_vision_init();enqueue(off,sizeof(off));enqueue(off,sizeof(off));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==60);
    CHECK(!jetson_get_target_control_value(&value,&seq));cases++;
    jetson_vision_init();enqueue(off,sizeof(off));enqueue(invalid_on,sizeof(invalid_on));jetson_vision_process();
    CHECK(jetson_get_target_control_value(&value,&seq) && value==0 && seq==60);
    CHECK(tx[4]==62 && tx[8]==JETSON_ERROR_INVALID_PARAM);cases++;
    jetson_vision_init();now_ms=100;enqueue(heartbeat,sizeof(heartbeat));enqueue(vision,sizeof(vision));
    now_ms=2000;jetson_vision_process();
    CHECK(!jetson_is_link_alive(now_ms));
    CHECK(jetson_get_vision_error_ex(&x,&y,&valid,&seq) && !valid);
    cases++;
    jetson_vision_init();now_ms=100;enqueue(vision,sizeof(vision));jetson_vision_process();
    now_ms=601;CHECK(jetson_get_vision_error_timed(&x,&y,&valid,&seq,&received) && !valid && received==100);cases++;
    jetson_vision_init();now_ms=100;enqueue(vision,sizeof(vision));
    now_ms=600;jetson_vision_process();CHECK(jetson_get_vision_error_ex(&x,&y,&valid,&seq) && valid);cases++;
    jetson_vision_init();now_ms=100;enqueue(depth,sizeof(depth));now_ms=601;jetson_vision_process();
    CHECK(jetson_get_safe_distance_ex(&distance,&valid,&seq) && !valid && distance==400);cases++;
    jetson_vision_init();now_ms=100;enqueue(heartbeat,4);jetson_vision_process();
    now_ms=2000;enqueue(heartbeat+4,sizeof(heartbeat)-4);jetson_vision_process();
    CHECK(!jetson_is_link_alive(now_ms));cases++;
    jetson_vision_init();now_ms=100;enqueue(heartbeat,sizeof(heartbeat));jetson_vision_process();
    CHECK(jetson_is_link_alive(700) && !jetson_is_link_alive(701));cases++;
    jetson_vision_init();now_ms=UINT32_MAX-100;enqueue(heartbeat,sizeof(heartbeat));enqueue(vision,sizeof(vision));
    now_ms=49;jetson_vision_process();CHECK(jetson_is_link_alive(now_ms));
    CHECK(jetson_get_vision_error_timed(&x,&y,&valid,&seq,&received) && valid && received==UINT32_MAX-100);
    CHECK(!jetson_is_link_alive(600));cases++;
    jetson_vision_init();now_ms=100;enqueue(vision,sizeof(vision));jetson_vision_process();
    CHECK(jetson_get_vision_error_ex(&x,&y,&valid,&seq) && valid);
    now_ms=600;enqueue(vision,sizeof(vision));jetson_vision_process();
    CHECK(!jetson_get_vision_error_ex(&x,&y,&valid,&seq) && s_latest_vision_received_ms==100);cases++;
    jetson_vision_init();uint8_t corrupt[sizeof(heartbeat)];memcpy(corrupt,heartbeat,sizeof(corrupt));
    corrupt[sizeof(corrupt)-2]=0xA5;corrupt[sizeof(corrupt)-1]=0x5A;
    now_ms=100;enqueue(corrupt,sizeof(corrupt)-1);jetson_vision_process();
    now_ms=200;enqueue(corrupt+sizeof(corrupt)-1,1);enqueue(vision+2,sizeof(vision)-2);jetson_vision_process();
    CHECK(jetson_get_vision_error_timed(&x,&y,&valid,&seq,&received) && valid && received==100);cases++;
    printf("review UART: %u STOP/freshness/wrap scenarios passed (no hardware)\n",cases);
}
''')


def stop_ack_source():
    source = can.build_source().removesuffix(can.CHECKS)
    code = workflow.stripped("USER_BSP/bsp_can.c")
    source += r'''
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define taskSCHEDULER_NOT_STARTED 0
static uint32_t xTaskGetTickCount(void) {return now_ms;}
static int xTaskGetSchedulerState(void) {return 1;}
#define taskYIELD() (++now_ms)
#undef __NOP
#define __NOP() (++now_ms)
#define __enable_irq() __set_PRIMASK(0)
static void BSP_CAN_DrainRxSem(void) {}
'''
    source += motion.function(code, "BSP_CAN_ClearStopFlags") + motion.function(code, "BSP_CAN_WaitStopAll")
    source += can.CHECKS.split("int main(void)")[0] + CHECK + r'''
int main(void) {
    unsigned cases=0;
    reset_fixture();const uint8_t ok[]={0xFE,0x02,0x6B},bad[]={0xFE,0xEE,0x6B};
    receive(0x400,ok,3);CHECK(g_can_context.motor_stop_flag[3]==1);cases++;
    receive(0x400,bad,3);receive(0x400,ok,3);
    CHECK(g_can_context.motor_stop_flag[3]==2 && !BSP_CAN_WaitStopAll(5,10) && !primask);cases++;
    BSP_CAN_ClearStopFlags(5);CHECK(!g_can_context.motor_stop_flag[3]);
    for(unsigned j=1;j<=5;j++)receive(j<<8,ok,3);
    CHECK(BSP_CAN_WaitStopAll(5,10));cases++;
    BSP_CAN_ClearStopFlags(5);receive(0x401,ok,3);CHECK(!g_can_context.motor_stop_flag[3]);cases++;
    const uint8_t short_frame[]={0xFE,0x6B};receive(0x400,short_frame,2);
    CHECK(!g_can_context.motor_stop_flag[3]);cases++;
    can_callback_args_t args={.event=CAN_EVENT_RX_COMPLETE};args.frame.id=0x400;
    args.frame.data_length_code=3;memcpy(args.frame.data,ok,3);
    args.frame.id_mode=CAN_ID_MODE_STANDARD;args.frame.type=CAN_FRAME_TYPE_DATA;
    canfd0_callback(&args);CHECK(!g_can_context.motor_stop_flag[3]);cases++;
    args.frame.id_mode=CAN_ID_MODE_EXTENDED;args.frame.type=CAN_FRAME_TYPE_REMOTE;
    canfd0_callback(&args);CHECK(!g_can_context.motor_stop_flag[3]);cases++;
    printf("review CAN stop ACK: %u canonical/rejection scenarios passed (no hardware)\n",cases);
}
'''
    return source


def log_source():
    code = workflow.stripped("USER_BSP/bsp_uart.c")
    source = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
typedef void *QueueHandle_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define taskSCHEDULER_NOT_STARTED 0
#define pdMS_TO_TICKS(x) (x)
#define portYIELD_FROM_ISR(x) ((void)(x))
#define FSP_SUCCESS 0
#define UART1_RX_BUFF_SIZE 128
#define CMD_TYPE_UART1 0
#define UART_EVENT_RX_CHAR 0
#define UART_EVENT_TX_COMPLETE 1
typedef struct {int event;unsigned data;} uart_callback_args_t;
static char uart1_rx_buff[130];static unsigned uart1_rx_pos;
static bool g_uart1_tx_complete=true;
static uint8_t s_uart1_tx_buffer[256];
static int robot_uart1_ctrl;
static SemaphoreHandle_t g_log_mutex=(void*)1,g_uart1_tx_sem=(void*)2;
static unsigned writes,send_count,max_timeout;static bool full,write_fail;
static const uint8_t *dma_data;
static int xTaskGetSchedulerState(void) {return 1;}
static void robot_cmd_send_from_isr(const char *p,int type) {(void)p;(void)type;}
static int xSemaphoreTake(void *s,unsigned ms) {(void)s;(void)ms;return 1;}
static void xSemaphoreGive(void *s) {(void)s;}
static void xSemaphoreGiveFromISR(void *s,int *wake) {(void)s;*wake=0;}
static int R_SCI_UART_Write(void *s,const uint8_t *p,unsigned n) {
    (void)s;(void)n;writes++;dma_data=p;return write_fail?1:0;
}
static void BSP_USB_CDC_WriteBestEffort(const char *p,unsigned n) {(void)p;(void)n;}
'''
    source += motion.declaration(code, r"typedef struct\s*\{[^}]*\}\s*uart_log_line_t;")
    source += r'''
static QueueHandle_t s_log_queue=(void*)3;
static TaskHandle_t s_log_task=(void*)4;
static uint32_t s_log_dropped;
static uart_log_line_t queued;
static int xQueueSendToBack(void *q,const void *p,unsigned ms) {
    (void)q;send_count++;max_timeout=ms;memcpy(&queued,p,sizeof(queued));return !full;
}
'''
    for name in ("robot_uart1_callback", "safe_printf", "safe_printf_nonblocking", "BSP_UART_LogDropped"):
        source += motion.function(code, name)
    return source + CHECK + r'''
int main(void) {
    safe_printf_nonblocking("value=%d",7);
    CHECK(send_count==1 && max_timeout==0 && writes==0 && strcmp(queued.text,"value=7")==0);
    full=true;safe_printf_nonblocking("full");CHECK(BSP_UART_LogDropped()==1 && writes==0);
    s_log_task=NULL;safe_printf_nonblocking("not initialized");CHECK(BSP_UART_LogDropped()==2);
    safe_printf("first");CHECK(writes==1 && dma_data==s_uart1_tx_buffer && !g_uart1_tx_complete);
    safe_printf("second");CHECK(writes==1 && strcmp((const char*)dma_data,"first")==0);
    uart_callback_args_t args={.event=UART_EVENT_TX_COMPLETE};robot_uart1_callback(&args);
    safe_printf("second");CHECK(writes==2 && memcmp(dma_data,"second",6)==0);
    robot_uart1_callback(&args);write_fail=true;safe_printf("failed");CHECK(g_uart1_tx_complete);
    puts("review log queue: zero-wait/full/uninitialized and TX-buffer lifetime passed (no hardware)");
}
'''


if __name__ == "__main__":
    cases = {"stop": stop_source, "relative": relative_source, "target": target_source,
             "uart": uart_source, "age": lambda: target_source(False),
             "stop_ack": stop_ack_source, "log": log_source}
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=cases)
    args = parser.parse_args()
    failed = []
    for name in ([args.case] if args.case else cases):
        try:
            run_c(cases[name](), "review_" + name)
        except subprocess.CalledProcessError:
            failed.append(name)
    if failed:
        raise SystemExit("Review regressions failed: " + ", ".join(failed))

"""Actual-C P000 reset tests with fake time, GPIO observations, CAN and UART."""
import test_global_soft_reset as reset
import test_scan_uart as uart
import test_scan_workflow as workflow
from test_scan_motion import function
from test_stop_confirmation import run_c


HELPERS = reset.HELPERS.replace(
    "sample();sample();",
    "unsigned init_calls=target_init_calls;sample();sample();",
).replace("target_init_calls==2", "target_init_calls==init_calls+1")


CHECKS = r'''
#include <stdlib.h>
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
''' + HELPERS + r'''
static void setup(void) {
    fixture();status_count=abort_calls=0;reset_count=0;
    obs.reset_button_valid=true;obs.reset_button=false;
}
static void ticks(unsigned count) { for(unsigned i=0;i<count;i++)step(); }
static void arm(void) {
    obs.reset_button=false;ticks(5);assert(s_reset_key.armed);
}
static void before_trigger(void) {
    obs.reset_button=true;ticks(200);assert(!s_global_reset.active && !abort_calls);
}
static void trigger(void) {
    arm();before_trigger();step();
    assert(s_global_reset.active && s_global_reset.local && s_global_reset.seq==0);
    assert(s_stop.pending && receive_stop_hold && abort_calls==1 && !reset_count && !laser);
    for(unsigned i=0;i<status_count;i++)assert(status_reports[i].event!=RA6_TO_JETSON_COMMAND_ACK);
}
static void complete(void) {
    stopped();assert(reset_count==1);
    assert(reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));
    done(0);assert(!command_find(JETSON_MSG_WORKFLOW_CTRL,0));
}
int main(void) {
    unsigned cases=0;
    setup();obs.reset_button=true;ticks(450);
    assert(!s_global_reset.active && !abort_calls && !reset_count);cases++;
    obs.reset_button=false;ticks(4);obs.reset_button=true;ticks(250);
    assert(!s_global_reset.active && !abort_calls);cases++;

    setup();trigger();complete();ticks(400);
    assert(reset_count==1 && abort_calls==1 && s_workflow.state==FLOW_IDLE);cases++;
    arm();obs.reset_button=true;ticks(201);
    assert(s_global_reset.active && abort_calls==2);stopped();done(0);
    assert(reset_count==2 && !command_find(JETSON_MSG_WORKFLOW_CTRL,0));cases++;

    setup();arm();obs.reset_button=true;ticks(100);obs.reset_button=false;step();
    obs.reset_button=true;ticks(200);assert(!abort_calls);step();assert(abort_calls==1);cases++;
    setup();arm();obs.reset_button=true;ticks(100);obs.reset_button_valid=false;step();
    obs.reset_button_valid=true;ticks(250);assert(!abort_calls && !s_reset_key.armed);
    trigger();complete();cases++;
    setup();arm();obs.reset_button=true;ticks(100);
    obs.now_ms+=ROBOT_WORKFLOW_RESET_KEY_SAMPLE_GAP_MS+1;ticks(250);
    assert(!abort_calls && !s_reset_key.armed);trigger();complete();cases++;

    setup();obs.now_ms=UINT32_MAX-1000u;trigger();complete();cases++;

    for(unsigned linked=0;linked<2;linked++)
    for(unsigned state=FLOW_IDLE;state<=FLOW_SCAN_READY;state++) {
        setup();arm();before_trigger();s_workflow.state=(workflow_state_t)state;
        obs.heartbeat_alive=linked!=0;
        capture_active=busy=laser=true;s_workflow.laser_active=s_workflow.target_enabled=true;
        capture_result=ROBOT_CAPTURE_RESULT_RUNNING;
        s_scan.id=s_scan.high_water=100;s_scan.start_seq=20;s_scan.state=JETSON_SCAN_SCANNING;
        step();assert(s_global_reset.active && !laser && !capture_active && !s_scan.flags);
        assert(!reset_count && s_stop.pending);complete();cases++;
    }

    setup();arm();before_trigger();obs.has_workflow_control=true;
    obs.workflow_action=JETSON_WORKFLOW_ACTION_START_MEASURE;obs.workflow_seq=90;step();
    obs.has_workflow_control=false;
    assert(s_global_reset.active && reported(90,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_BUSY));
    complete();cases++;
    setup();arm();before_trigger();command(3,90);
    assert(!s_global_reset.active && s_stop.pending && abort_calls==1 && !reset_count);
    sample();sample();ticks(250);assert(!s_global_reset.active && !reset_count);cases++;
    setup();arm();before_trigger();command(4,90);
    assert(s_global_reset.active && !s_global_reset.local && s_global_reset.seq==90);
    stopped();done(90);assert(!reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));cases++;

    setup();arm();before_trigger();s_stop.pending=true;s_stop.restart_locked=true;
    s_stop.deadline=obs.now_ms+5000;stop_feedback_ok=false;step();
    assert(!s_global_reset.active && !abort_calls && !reset_count);
    s_stop.pending=false;s_stop.restart_locked=false;ticks(250);assert(!abort_calls);cases++;
    setup();s_workflow.state=FLOW_IDLE;obs.heartbeat_alive=false;trigger();complete();ticks(400);
    assert(!s_global_reset.local && reset_count==1 && abort_calls==1);
    command(1,91);assert(reset_count==1 && reported(91,RA6_TO_JETSON_COMMAND_ACK,0,7));
    command(4,92);assert(!s_global_reset.active && reported(92,RA6_TO_JETSON_COMMAND_ACK,0,7));
    obs.heartbeat_alive=true;command(1,93);
    assert(s_workflow.state==FLOW_HOMING && reset_count==2);cases++;

    for(unsigned safety=0;safety<2;safety++) {
        setup();arm();before_trigger();
        obs.heartbeat_alive=false;
        obs.estop_active=safety==0;obs.limit_triggered=safety==1;step();
        assert(!s_global_reset.active && !reset_count);
        obs.estop_active=obs.limit_triggered=false;ticks(250);
        assert(!s_global_reset.active && !reset_count);cases++;
    }
    for(unsigned stage=0;stage<2;stage++)
    for(unsigned safety=0;safety<2;safety++) {
        setup();trigger();obs.heartbeat_alive=false;if(stage)stopped();int homes=reset_count;
        obs.estop_active=safety==0;obs.limit_triggered=safety==1;step();
        assert(!s_global_reset.active && !s_global_reset.local && reset_count==homes);
        assert(s_workflow.state==FLOW_FAULT_HOLD && receive_stop_hold && s_stop.restart_locked);
        assert(reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,8));cases++;
    }

    for(unsigned linked=0;linked<2;linked++) {
        setup();arm();before_trigger();obs.heartbeat_alive=linked!=0;stop_ack_ok=false;step();
        assert(!s_global_reset.active && !s_global_reset.local && s_stop.restart_locked && !reset_count);
        assert(reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,6));cases++;
        setup();trigger();obs.heartbeat_alive=linked!=0;
        stop_feedback_ok=false;obs.now_ms=s_stop.deadline;step();
        assert(!s_global_reset.active && !s_global_reset.local && !reset_count && s_stop.restart_locked);
        assert(!reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));cases++;
    }

    for(unsigned stage=0;stage<2;stage++) {
        setup();trigger();if(stage)stopped();
        obs.heartbeat_alive=false;step();
        assert(s_global_reset.active && s_global_reset.local && receive_stop_hold);
        if(!stage)stopped();
        obs.heartbeat_alive=true;step();obs.heartbeat_alive=false;step();done(0);
        assert(!s_global_reset.local && reset_count==1);
        assert(!reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,7));cases++;
    }
    for(unsigned linked=0;linked<2;linked++)
    for(unsigned failure=0;failure<4;failure++) {
        setup();trigger();obs.heartbeat_alive=linked!=0;stopped();
        if(failure==0)reset_result=ROBOT_RESET_RESULT_FAILED;
        if(failure==1)reset_result=ROBOT_RESET_RESULT_ABORTED;
        if(failure==2)obs.now_ms=s_workflow.state_deadline_ms;
        if(failure==3){reset_result=ROBOT_RESET_RESULT_OK;verify_ok=false;}
        step();assert(!s_global_reset.active && !s_global_reset.local && receive_stop_hold && s_stop.restart_locked);
        assert(!reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));cases++;
    }
    for(unsigned linked=0;linked<2;linked++)
    for(unsigned stage=0;stage<2;stage++) {
        setup();trigger();if(stage)stopped();int homes=reset_count;
        obs.heartbeat_alive=linked!=0;
        command(3,90);assert(!s_global_reset.active && !s_global_reset.local);sample();sample();
        assert(reset_count==homes && reported(90,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));
        assert(!reported(0,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));cases++;
    }
    const workflow_state_t normal_states[]={FLOW_SCAN_ACTIVE,FLOW_TARGET_ACTIVE,FLOW_RETURN_HOME};
    for(unsigned i=0;i<sizeof(normal_states)/sizeof(normal_states[0]);i++) {
        setup();s_workflow.state=FLOW_IDLE;obs.heartbeat_alive=false;trigger();complete();
        s_workflow.state=normal_states[i];step();
        assert(s_workflow.state==FLOW_FAULT_HOLD && !s_global_reset.local && reset_count==1);
        assert(receive_stop_hold && s_stop.restart_locked);cases++;
    }
    setup();arm();obs.reset_button=true;ticks(100);robot_workflow_step(NULL);
    ticks(250);assert(!abort_calls && !s_reset_key.armed);cases++;
    printf("actual-C P000 local reset: %u offline scenarios passed\n",cases);
}
'''

INTEGRATION = (
    "static bool key_pressed, key_valid;\n" + reset.INTEGRATION.split("int main(void)")[0]
).replace(
    "received.heartbeat_alive=jetson_is_link_alive(now_ms);",
    "received.heartbeat_alive=jetson_is_link_alive(now_ms);"
    "received.reset_button=key_pressed;received.reset_button_valid=key_valid;",
) + r'''
static void hold_key(bool linked) {
    key_pressed=false;key_valid=true;
    if(linked)enqueue(heartbeat,sizeof(heartbeat));
    for(unsigned i=0;i<5;i++)pump();
    key_pressed=true;
    for(unsigned i=0;i<200;i++) {
        if(linked && i%20==0)enqueue(heartbeat,sizeof(heartbeat));pump();
    }
    assert(s_reset_key.armed && !s_global_reset.active);
}
int main(void) {
    fixture();scanning();jetson_vision_init();now_ms=obs.now_ms;hold_key(true);
    int homes=reset_count,captures=capture_count,segments=segment_count;
    enqueue(heartbeat,sizeof(heartbeat));enqueue(start_frame,sizeof(start_frame));
    enqueue(target,sizeof(target));enqueue(capture,sizeof(capture));pump();
    assert(s_global_reset.active && s_global_reset.local && s_stop.pending && s_workflow_stop_hold);
    assert(reset_count==homes && !target_starts);pump();reset_stopped();
    assert(reset_count==homes+1 && capture_count==captures && segment_count==segments);
    enqueue(fresh_start,4);pump();reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(s_workflow.state==FLOW_IDLE && !s_global_reset.active && !s_workflow_stop_hold);
    assert(tx[4]==0 && tx[6]==0x20 && tx[7]==5 && tx[8]==0);
    enqueue(fresh_start+4,sizeof(fresh_start)-4);pump();
    assert(s_workflow.state==FLOW_IDLE && reset_count==homes+1);
    enqueue(fresh_start,sizeof(fresh_start));pump();
    assert(s_workflow.state==FLOW_HOMING && reset_count==homes+2);

    fixture();scanning();jetson_vision_init();now_ms=obs.now_ms;hold_key(true);
    pump();reset_stopped();homes=reset_count;
    enqueue(abort_frame,sizeof(abort_frame));pump();
    now_ms=s_stop.next_sample;pump();now_ms=s_stop.next_sample;pump();
    reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(!s_global_reset.active && s_workflow.state==FLOW_ABORT_HOLD && reset_count==homes);
    assert(tx[4]==60 && tx[6]==0x20 && tx[7]==6 && tx[8]==0);

    fixture();s_workflow.state=FLOW_IDLE;jetson_vision_init();now_ms=obs.now_ms;hold_key(false);
    assert(!jetson_is_link_alive(now_ms));homes=reset_count;
    enqueue(start_frame,sizeof(start_frame));enqueue(target,sizeof(target));pump();
    assert(s_global_reset.active && s_global_reset.local && s_stop.pending && s_workflow_stop_hold);
    assert(reset_count==homes && !target_starts);reset_stopped();
    reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(s_workflow.state==FLOW_IDLE && !s_global_reset.active && !s_global_reset.local);
    assert(!s_workflow_stop_hold && !jetson_is_link_alive(now_ms) && reset_count==homes+1);
    assert(tx[4]==0 && tx[6]==0x20 && tx[7]==5 && tx[8]==0);
    enqueue(fresh_start,sizeof(fresh_start));pump();
    assert(s_workflow.state==FLOW_IDLE && reset_count==homes+1 && tx[4]==22 && tx[8]==7);
    enqueue(reset_frame,sizeof(reset_frame));pump();
    assert(!s_global_reset.active && reset_count==homes+1 && tx[4]==70 && tx[8]==7);
    enqueue(heartbeat,sizeof(heartbeat));enqueue(start_frame,sizeof(start_frame));pump();
    assert(s_workflow.state==FLOW_IDLE && reset_count==homes+1);

    fixture();s_workflow.state=FLOW_IDLE;jetson_vision_init();now_ms=obs.now_ms;hold_key(false);
    pump();reset_stopped();homes=reset_count;
    enqueue(abort_frame,sizeof(abort_frame));pump();
    now_ms=s_stop.next_sample;pump();now_ms=s_stop.next_sample;pump();
    reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(!s_global_reset.active && !s_global_reset.local && s_workflow.state==FLOW_ABORT_HOLD);
    assert(reset_count==homes && !jetson_is_link_alive(now_ms));
    assert(tx[4]==60 && tx[6]==0x20 && tx[7]==6 && tx[8]==0);
    puts("actual-C P000 reset + UART: 4 integrated online/offline scenarios passed");
}
'''


def main() -> None:
    source = workflow.build_source(CHECKS)
    source = source.replace(function(source, "robot_send_reset_event"),
                            "static int robot_send_reset_event(bool hard);")
    source = source.replace("static robot_auto_result_t auto_result;", r'''
static robot_auto_result_t auto_result;
static bool reset_send_ok=true;
static unsigned target_init_calls;
static int robot_send_reset_event(bool hard) {
    assert(!hard);reset_count++;
    if(!reset_send_ok)return 0;
    aborted=false;reset_result=ROBOT_RESET_RESULT_RUNNING;return pdPASS;
}
''')
    source = source.replace("void robot_target_init(void) {}",
                            "void robot_target_init(void) {target_init_calls++;}")
    source = source.replace(function(source, "robot_capture_cancel"),
                            "void robot_capture_cancel(void) {capture_active=false;capture_result=ROBOT_CAPTURE_RESULT_ABORTED;}")
    run_c(source, "local_reset_key")
    mocks = workflow.MOCKS
    for name in ("jetson_set_workflow_stop_hold", "jetson_send_status",
                 "jetson_send_scan_status", "jetson_send_error"):
        mocks = mocks.replace(function(mocks, name), "")
    mocks = mocks.replace("static StatusReport status_reports[1024];", "")
    integrated = (uart.PREAMBLE + workflow.build_source("").replace(workflow.MOCKS, mocks) +
                  uart.stripped("Middle/crc16.h") + uart.stripped("Middle/crc16.c") +
                  uart.stripped("Middle/jetson_vision.c") + reset.declarations(INTEGRATION) + INTEGRATION)
    run_c(integrated, "local_reset_key_uart")


if __name__ == "__main__":
    main()

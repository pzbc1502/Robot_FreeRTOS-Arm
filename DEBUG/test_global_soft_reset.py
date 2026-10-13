"""Actual-C global reset/UART checks. No serial port or hardware access."""
import re
import sys

import test_scan_workflow as workflow
import test_scan_uart as uart
import test_workflow_uart as workflow_uart
from test_scan_motion import function
from test_stop_confirmation import run_c


HELPERS = workflow.CHECKS.split("int main(void)")[0].replace(
    "static void fixture(void)", "static void base_fixture(void)"
) + r'''
static void fixture(void) { reset_send_ok=true;target_init_calls=0;base_fixture(); }
static bool reported(uint8_t seq,uint8_t event,uint8_t value,uint8_t error) {
    for(unsigned i=0;i<status_count;i++) {
        StatusReport r=status_reports[i];
        if(r.seq==seq && r.event==event && r.value==value && r.error==error) return true;
    }
    return false;
}
static void command(uint8_t action,uint8_t seq) {
    obs.has_workflow_control=true;obs.workflow_action=action;obs.workflow_seq=seq;
    step();obs.has_workflow_control=false;
}
static void sample(void) {obs.now_ms=s_stop.next_sample;step();}
static void stopped(void) {
    sample();sample();
    assert(s_global_reset.active && !s_stop.pending && !s_stop.restart_locked);
    assert(s_workflow.state==FLOW_RETURN_HOME && receive_stop_hold && !laser);
    assert(capture_result==ROBOT_CAPTURE_RESULT_NONE && target_init_calls==2);
}
static void done(uint8_t seq) {
    reset_result=ROBOT_RESET_RESULT_OK;step();
    assert(s_workflow.state==FLOW_IDLE && !s_global_reset.active && !receive_stop_hold);
    assert(!s_workflow.distance_safe_latched && !s_workflow.selected_view_id);
    assert(!s_workflow.capture_done_mask && !s_workflow.target_enabled && !laser);
    assert(s_scan.id==0 && s_scan.state==0 && s_scan.flags==0);
    assert(reported(seq,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));
}
'''

CHECKS = HELPERS + r'''
int main(void) {
    unsigned cases=0;
    for(unsigned state=FLOW_IDLE;state<=FLOW_SCAN_READY;state++) {
        fixture();s_workflow.state=(workflow_state_t)state;
        s_scan.id=s_scan.high_water=100;s_scan.state=JETSON_SCAN_FRONT_OBSERVE_HOLD;
        s_scan.flags=6;s_scan.start_seq=20;
        capture_active=busy=laser=true;capture_result=ROBOT_CAPTURE_RESULT_RUNNING;
        s_workflow.laser_active=s_workflow.target_enabled=true;
        int homes=reset_count,segments=segment_count,captures=capture_count;
        command(4,70);
        assert(reported(70,RA6_TO_JETSON_COMMAND_ACK,1,0));
        assert(s_global_reset.active && s_stop.pending && receive_stop_hold && !laser);
        assert(!capture_active && !s_workflow.distance_safe_latched && !s_scan.flags);
        assert(reset_count==homes && capture_result==ROBOT_CAPTURE_RESULT_ABORTED);
        stopped();assert(reset_count==homes+1);done(70);
        assert(segment_count==segments && capture_count==captures && !target_starts);
        assert(s_scan.high_water==100);control(0,0,100,71);
        assert(last().error==JETSON_ERROR_SCAN_ID_MISMATCH && !last().flags);cases++;
    }

    fixture();command(4,70);uint32_t deadline=s_stop.deadline;
    int homes=reset_count;command(4,70);
    assert(abort_calls==1 && reset_count==homes && s_stop.deadline==deadline);
    command(4,71);assert(reported(71,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_BUSY));
    assert(s_global_reset.seq==70 && s_stop.deadline==deadline);stopped();
    deadline=s_workflow.state_deadline_ms;obs.now_ms+=4000;command(4,70);
    assert(s_workflow.state_deadline_ms==deadline && reset_count==homes+1);
    for(uint8_t seq=80;seq<92;seq++)command(2,seq);
    assert(command_find(JETSON_MSG_WORKFLOW_CTRL,70));done(70);
    obs.now_ms+=5000;command(4,70);assert(reset_count==homes+1 && !s_global_reset.active);
    for(uint8_t seq=92;seq<104;seq++)command(2,seq);
    command(1,70);assert(reported(70,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_SEQ_CONFLICT));
    assert(reset_count==homes+1);cases++;

    fixture();homes=reset_count;stop_ack_ok=false;command(4,70);
    assert(!s_global_reset.active && s_stop.restart_locked && receive_stop_hold);
    assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,6));
    stop_ack_ok=true;command(4,70);assert(abort_calls==1 && reset_count==homes);
    command(4,71);stopped();done(71);cases++;

    fixture();homes=reset_count;stop_feedback_ok=false;command(4,70);
    obs.now_ms=s_stop.deadline;step();
    assert(!s_global_reset.active && reset_count==homes && receive_stop_hold);
    assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,6));cases++;

    fixture();homes=reset_count;command(4,70);stop_angles[3]=NAN;sample();sample();
    assert(s_stop.pending && reset_count==homes);
    obs.now_ms=s_stop.deadline;step();assert(!s_global_reset.active && receive_stop_hold);cases++;

    fixture();homes=reset_count;command(4,70);reset_result=ROBOT_RESET_RESULT_OK;
    sample();assert(s_stop.pending && reset_count==homes);sample();
    assert(reset_count==homes+1 && s_global_reset.active && s_workflow.state==FLOW_RETURN_HOME);
    assert(reset_result!=ROBOT_RESET_RESULT_OK);done(70);cases++;

    fixture();homes=reset_count;command(4,70);command(3,71);
    assert(!s_global_reset.active && s_stop.pending && reset_count==homes);
    assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,0x0F));
    sample();sample();
    assert(reported(71,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));
    assert(s_workflow.state==FLOW_ABORT_HOLD && reset_count==homes);cases++;

    fixture();command(4,70);stopped();homes=reset_count;command(3,71);
    sample();sample();reset_result=ROBOT_RESET_RESULT_OK;step();
    assert(!s_global_reset.active && s_workflow.state==FLOW_ABORT_HOLD && reset_count==homes);
    assert(!reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));cases++;

    fixture();command(3,69);homes=reset_count;command(4,70);
    assert(reported(70,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_BUSY));
    assert(!s_global_reset.active && reset_count==homes);sample();sample();cases++;

    fixture();s_workflow.state=FLOW_IDLE;obs.heartbeat_alive=false;homes=reset_count;command(4,70);
    assert(reported(70,RA6_TO_JETSON_COMMAND_ACK,0,7) && reset_count==homes);cases++;
    for(unsigned stage=0;stage<2;stage++) {
        fixture();command(4,70);if(stage)stopped();homes=reset_count;
        obs.heartbeat_alive=false;step();
        assert(!s_global_reset.active && receive_stop_hold && reset_count==homes);
        assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,7));cases++;
    }
    fixture();homes=reset_count;obs.estop_active=true;command(4,70);
    assert(reported(70,RA6_TO_JETSON_COMMAND_ACK,0,8) && reset_count==homes);cases++;
    for(unsigned stage=0;stage<2;stage++) {
        fixture();command(4,70);if(stage)stopped();homes=reset_count;
        obs.limit_triggered=true;step();
        assert(!s_global_reset.active && receive_stop_hold && reset_count==homes);
        assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,8));cases++;
    }

    fixture();command(4,70);reset_send_ok=false;sample();sample();
    assert(!s_global_reset.active && receive_stop_hold && s_workflow.state==FLOW_FAULT_HOLD);
    assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,6));cases++;
    for(unsigned mode=0;mode<4;mode++) {
        fixture();command(4,70);stopped();
        if(mode==0)reset_result=ROBOT_RESET_RESULT_FAILED;
        if(mode==1)reset_result=ROBOT_RESET_RESULT_ABORTED;
        if(mode==2)obs.now_ms=s_workflow.state_deadline_ms;
        if(mode==3){reset_result=ROBOT_RESET_RESULT_OK;verify_ok=false;}
        step();assert(!s_global_reset.active && receive_stop_hold && s_workflow.state==FLOW_FAULT_HOLD);
        assert(reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,
                        mode==1?0x0F:(mode==3?4:0x0B)));
        reset_result=ROBOT_RESET_RESULT_OK;step();
        assert(!reported(70,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));cases++;
    }
    fixture();
    for(uint8_t seq=70;seq<77;seq++) {
        command(4,seq);sample();sample();done(seq);
    }
    homes=reset_count;command(4,77);
    assert(reported(77,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_BUSY));
    assert(!s_global_reset.active && reset_count==homes);
    command(3,78);sample();sample();
    assert(reported(78,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));cases++;
    printf("actual-C global soft reset: %u offline workflow scenarios passed\n",cases);
}
'''

UART_CHECKS = r'''
static void feed(const uint8_t *data,unsigned n) {
    for(unsigned i=0;i<n;i++)jetson_notify_rx_char_from_isr(data[i]);
    jetson_vision_process();
}
int main(void) {
    uint8_t action,seq;unsigned cases=0;
    for(unsigned split=0;split<=sizeof(reset_frame);split++) {
        jetson_vision_init();feed(reset_frame,split);feed(reset_frame+split,sizeof(reset_frame)-split);
        assert(jetson_get_workflow_control(&action,&seq) && action==4 && seq==70);
        assert(!jetson_get_workflow_control(&action,&seq));cases++;
    }
    jetson_vision_init();for(unsigned i=0;i<8;i++)handle_workflow_control(1,(uint8_t)(80+i));
    handle_workflow_control(4,70);assert(s_new_global_reset);
    assert(jetson_get_workflow_control(&action,&seq) && action==4 && seq==70);
    jetson_set_workflow_stop_hold(true);
    assert(!jetson_get_workflow_control(&action,&seq) && !s_workflow_count);cases++;
    jetson_vision_init();handle_workflow_control(4,70);handle_workflow_control(4,70);
    handle_workflow_control(4,71);assert(tx[4]==71 && tx[7]==0 && tx[8]==5);
    handle_workflow_control(3,60);
    assert(!s_new_global_reset && jetson_get_workflow_control(&action,&seq) && action==3);
    assert(!jetson_get_workflow_control(&action,&seq));cases++;
    jetson_vision_init();handle_workflow_control(3,60);handle_workflow_control(4,70);
    assert(!s_new_global_reset && tx[4]==70 && tx[8]==5);cases++;
    jetson_vision_init();uint8_t corrupt[sizeof(reset_frame)];
    memcpy(corrupt,reset_frame,sizeof(corrupt));corrupt[sizeof(corrupt)-1]^=1;
    feed(corrupt,sizeof(corrupt));feed(zero_seq,sizeof(zero_seq));feed(bad_len,sizeof(bad_len));
    assert(!s_new_global_reset && !jetson_get_workflow_control(&action,&seq));cases++;
    printf("actual-C global reset UART: %u offline scenarios passed\n",cases);
}
'''

INTEGRATION = workflow_uart.INTEGRATION_CHECKS.split("int main(void)")[0] + r'''
static void reset_stopped(void) {
    now_ms=s_stop.next_sample;pump();now_ms=s_stop.next_sample;pump();
    assert(!s_stop.pending && s_global_reset.active && s_workflow_stop_hold);
    assert(s_workflow.state==FLOW_RETURN_HOME);
}
int main(void) {
    fixture();scanning();jetson_vision_init();now_ms=obs.now_ms;
    enqueue(heartbeat,sizeof(heartbeat));enqueue(target,sizeof(target));
    enqueue(vision,sizeof(vision));enqueue(depth,sizeof(depth));
    enqueue(reset_frame,sizeof(reset_frame));enqueue(start_frame,sizeof(start_frame));
    enqueue(capture,sizeof(capture));enqueue(scan_start,sizeof(scan_start));
    int homes=reset_count,captures=capture_count,segments=segment_count;
    pump();assert(s_global_reset.active && s_stop.pending && s_workflow_stop_hold);
    assert(!target_starts && reset_count==homes);pump();reset_stopped();
    assert(reset_count==homes+1 && capture_result==ROBOT_CAPTURE_RESULT_NONE);
    enqueue(fresh_start,4);pump();reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(s_workflow.state==FLOW_IDLE && !s_global_reset.active && !s_workflow_stop_hold);
    assert(tx[4]==70 && tx[6]==0x20 && tx[7]==5 && tx[8]==0);
    enqueue(fresh_start+4,sizeof(fresh_start)-4);pump();
    assert(s_workflow.state==FLOW_IDLE && reset_count==homes+1);
    assert(capture_count==captures && segment_count==segments && !target_starts);
    enqueue(reset_frame,sizeof(reset_frame));pump();assert(reset_count==homes+1);
    enqueue(fresh_start,sizeof(fresh_start));pump();
    assert(s_workflow.state==FLOW_HOMING && reset_count==homes+2);

    fixture();scanning();jetson_vision_init();now_ms=obs.now_ms;
    enqueue(heartbeat,sizeof(heartbeat));enqueue(reset_frame,sizeof(reset_frame));pump();reset_stopped();
    homes=reset_count;enqueue(abort_frame,sizeof(abort_frame));pump();
    now_ms=s_stop.next_sample;pump();now_ms=s_stop.next_sample;pump();
    reset_result=ROBOT_RESET_RESULT_OK;pump();
    assert(!s_global_reset.active && s_workflow.state==FLOW_ABORT_HOLD && reset_count==homes);
    assert(!s_workflow_stop_hold && !laser);
    fixture();scanning();jetson_vision_init();now_ms=obs.now_ms;
    enqueue(heartbeat,sizeof(heartbeat));enqueue(reset_frame,sizeof(reset_frame));pump();reset_stopped();
    reset_result=ROBOT_RESET_RESULT_OK;pump();homes=reset_count;
    enqueue(fresh_start,sizeof(fresh_start));enqueue(reset_frame,sizeof(reset_frame));pump();
    assert(s_workflow.state==FLOW_IDLE && reset_count==homes && s_workflow_count==1);
    assert(!s_workflow_queue[s_workflow_read].cancelled);
    pump();assert(s_workflow.state==FLOW_HOMING && reset_count==homes+1);
    puts("actual-C global reset UART + workflow: 3 integrated scenarios passed");
}
'''


def declarations(checks):
    vectors = {
        "reset_frame": uart.build_unified_frame(6, 70, b"\x04"),
        "zero_seq": uart.build_unified_frame(6, 0, b"\x04"),
        "bad_len": uart.build_unified_frame(6, 70, b"\x04\x00"),
        "heartbeat": uart.build_unified_frame(1, 1, bytes(4)),
        "start_frame": uart.build_unified_frame(6, 20, b"\x01"),
        "fresh_start": uart.build_unified_frame(6, 22, b"\x01"),
        "abort_frame": uart.build_unified_frame(6, 60, b"\x03"),
        "target": uart.build_unified_frame(2, 32, b"\x01"),
        "vision": uart.build_unified_frame(3, 35, bytes(4) + b"\x01"),
        "depth": uart.build_unified_frame(5, 34, b"\x90\x01\x01"),
        "capture": uart.build_unified_frame(4, 31, b"\x01\x01"),
        "scan_start": uart.build_scan_control(1, 0, 100, 30),
    }
    return "\n".join(
        "static const uint8_t " + name + "[]={" + ",".join(map(str, data)) + "};"
        for name, data in vectors.items() if re.search(r"\b" + name + r"\b", checks)
    )


def main():
    sys.path.insert(0, str(workflow.ROOT / "UpperComputer"))
    import ra6m5_protocol as protocol
    assert protocol.build_unified_workflow_control_frame(protocol.WORKFLOW_GLOBAL_SOFT_RESET, 70) == \
        bytes.fromhex("A5 5A 01 06 46 01 04 F8 CE")
    document = (workflow.ROOT / "doc" /
                "Jetson\u5168\u5c40\u8f6f\u4ef6\u590d\u4f4d\u63a5\u53e3\u5bf9\u63a5_20261013.md").read_text(encoding="utf-8")
    for payload, expected in (
        (b"\x21\x01\x00", "A5 5A 01 81 46 03 21 01 00 CD C0"),
        (b"\x20\x05\x00", "A5 5A 01 81 46 03 20 05 00 9E C0"),
        (b"\x20\x07\x06", "A5 5A 01 81 46 03 20 07 06 1F A2"),
        (b"\x20\x07\x0F", "A5 5A 01 81 46 03 20 07 0F DF A4"),
    ):
        assert protocol.build_unified_frame(0x81, 70, payload) == bytes.fromhex(expected)
        assert expected in document
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
    run_c(source, "global_reset_workflow")
    receive = (uart.PREAMBLE + uart.stripped("Middle/crc16.h") +
               uart.stripped("Middle/jetson_vision.h") + uart.stripped("Middle/crc16.c") +
               uart.stripped("Middle/jetson_vision.c"))
    run_c(receive + declarations(UART_CHECKS) + UART_CHECKS, "global_reset_uart")
    mocks = workflow.MOCKS
    for name in ("jetson_set_workflow_stop_hold", "jetson_send_status",
                 "jetson_send_scan_status", "jetson_send_error"):
        mocks = mocks.replace(function(mocks, name), "")
    mocks = mocks.replace("static StatusReport status_reports[1024];", "")
    mocks = mocks.replace(function(mocks, "robot_capture_cancel"),
        "void robot_capture_cancel(void) {capture_active=false;capture_result=ROBOT_CAPTURE_RESULT_ABORTED;}")
    integrated = (uart.PREAMBLE + workflow.build_source("").replace(workflow.MOCKS, mocks) +
                  uart.stripped("Middle/crc16.h") + uart.stripped("Middle/crc16.c") +
                  uart.stripped("Middle/jetson_vision.c") + declarations(INTEGRATION) + INTEGRATION)
    run_c(integrated, "global_reset_integrated")


if __name__ == "__main__":
    main()

"""Test the real workflow RX queues with offline UART/clock substitutes."""
import re
import test_scan_uart as uart
import test_scan_workflow as workflow
from test_scan_motion import function
from test_stop_confirmation import run_c


CHECKS = r'''
static void enqueue(const uint8_t *data,unsigned n) {
    for(unsigned i=0;i<n;i++) jetson_notify_rx_char_from_isr(data[i]);
}
static void feed(const uint8_t *data,unsigned n) {
    enqueue(data,n);jetson_vision_process();
}
static void ack(uint8_t seq,uint8_t error) {
    assert(tx_len==11 && tx[3]==0x81 && tx[4]==seq && tx[5]==3);
    assert(tx[6]==0x21 && tx[7]==0 && tx[8]==error);
}
static void take_abort(uint8_t expected) {
    uint8_t action,seq;
    assert(jetson_get_workflow_control(&action,&seq) && action==3 && seq==expected);
}
int main(void) {
    uint8_t action,seq,value,point;bool valid;uint16_t distance;int16_t x,y;
    jetson_scan_control_t scan;unsigned cases=0;
    for(unsigned split=0;split<=sizeof(abort_frame);split++) {
        jetson_vision_init();
        feed(abort_frame,split);feed(abort_frame+split,sizeof(abort_frame)-split);
        take_abort(60);assert(!jetson_get_workflow_control(&action,&seq));cases++;
    }

    jetson_vision_init();enqueue(start_frame,sizeof(start_frame));enqueue(home_frame,sizeof(home_frame));
    jetson_vision_process();
    assert(jetson_get_workflow_control(&action,&seq) && action==1 && seq==20);
    assert(jetson_get_workflow_control(&action,&seq) && action==2 && seq==21);
    assert(!jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();enqueue(start_frame,sizeof(start_frame));enqueue(abort_frame,sizeof(abort_frame));
    enqueue(home_frame,sizeof(home_frame));enqueue(heartbeat,sizeof(heartbeat));
    jetson_vision_process();take_abort(60);jetson_set_workflow_stop_hold(true);
    assert(s_rx_read_index!=s_rx_write_index);
    jetson_vision_process();assert(!jetson_get_workflow_control(&action,&seq));ack(20,5);
    assert(jetson_is_link_alive(now_ms));
    jetson_set_workflow_stop_hold(false);feed(start_frame,sizeof(start_frame));
    assert(jetson_get_workflow_control(&action,&seq) && action==1);cases++;

    jetson_vision_init();enqueue(abort_frame,sizeof(abort_frame));enqueue(start_frame,sizeof(start_frame));
    enqueue(home_frame,sizeof(home_frame));jetson_vision_process();take_abort(60);
    jetson_set_workflow_stop_hold(true);jetson_set_workflow_stop_hold(false);
    jetson_vision_process();assert(!jetson_get_workflow_control(&action,&seq));ack(21,5);
    assert(!s_release_drain_active);feed(start_frame,sizeof(start_frame));
    assert(jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();for(unsigned i=0;i<9;i++)feed(start_frame,sizeof(start_frame));ack(20,5);
    assert(s_workflow_count==8);feed(abort_frame,sizeof(abort_frame));take_abort(60);
    assert(!jetson_get_workflow_control(&action,&seq) && !s_workflow_count);cases++;

    jetson_vision_init();handle_workflow_control(3,60);handle_workflow_control(3,60);
    handle_workflow_control(3,61);assert(s_abort_count==2);take_abort(60);take_abort(61);
    assert(!jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();for(uint8_t n=60;n<69;n++)handle_workflow_control(3,n);
    ack(68,5);for(uint8_t n=60;n<68;n++)take_abort(n);
    assert(!jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();feed(abort_frame,sizeof(abort_frame));take_abort(60);
    jetson_set_workflow_stop_hold(true);
    feed(start_frame,sizeof(start_frame));ack(20,5);feed(home_frame,sizeof(home_frame));ack(21,5);
    feed(scan_start,sizeof(scan_start));assert(tx[3]==0x82 && tx[4]==30 && tx[11]==5);
    feed(capture,sizeof(capture));ack(31,5);feed(target,sizeof(target));ack(32,5);
    feed(query,sizeof(query));assert(jetson_get_scan_control(&scan) && scan.action==0);
    feed(target_off,sizeof(target_off));assert(jetson_get_target_control_value(&value,&seq) && value==0);
    feed(heartbeat,sizeof(heartbeat));assert(jetson_is_link_alive(now_ms));
    feed(abort_two,sizeof(abort_two));take_abort(61);
    assert(!jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();feed(scan_start,sizeof(scan_start));feed(query,sizeof(query));
    feed(capture,sizeof(capture));feed(target,sizeof(target));
    feed(depth,sizeof(depth));feed(vision,sizeof(vision));
    feed(abort_frame,sizeof(abort_frame));take_abort(60);
    jetson_set_workflow_stop_hold(true);jetson_set_workflow_stop_hold(false);
    assert(!jetson_get_capture_control_ex(&action,&point,&seq));ack(31,5);
    assert(!jetson_get_target_control_value(&value,&seq));ack(32,5);
    assert(jetson_get_scan_control(&scan) && scan.action==0);
    assert(!jetson_get_scan_control(&scan));
    assert(!jetson_get_safe_distance_ex(&distance,&valid,&seq));
    assert(!jetson_get_vision_error_ex(&x,&y,&valid,&seq));cases++;

    jetson_vision_init();feed(start_frame,sizeof(start_frame));
    uint8_t corrupt[sizeof(abort_frame)];memcpy(corrupt,abort_frame,sizeof(corrupt));
    corrupt[sizeof(corrupt)-1]^=1;feed(corrupt,sizeof(corrupt));
    feed(zero_seq,sizeof(zero_seq));feed(bad_len,sizeof(bad_len));feed(bad_action,sizeof(bad_action));
    ack(66,3);assert(!s_abort_count && !s_workflow_stop_hold);
    assert(jetson_get_workflow_control(&action,&seq) && action==1);cases++;

    jetson_vision_init();feed(abort_frame,sizeof(abort_frame));take_abort(60);
    jetson_set_workflow_stop_hold(true);feed(start_frame,4);
    jetson_set_workflow_stop_hold(false);feed(start_frame+4,sizeof(start_frame)-4);
    ack(20,5);assert(!jetson_get_workflow_control(&action,&seq));
    assert(!s_release_drain_active);feed(start_frame,sizeof(start_frame));
    assert(jetson_get_workflow_control(&action,&seq));cases++;

    jetson_vision_init();for(unsigned i=0;i<400;i++)jetson_notify_rx_char_from_isr(0);
    jetson_vision_process();assert(s_rx_read_index==JETSON_RX_PROCESS_BUDGET);
    jetson_vision_process();assert(s_rx_read_index==s_rx_write_index);cases++;

    jetson_vision_init();feed(abort_frame,sizeof(abort_frame));take_abort(60);
    jetson_set_workflow_stop_hold(true);jetson_vision_init();feed(start_frame,sizeof(start_frame));
    assert(jetson_get_workflow_control(&action,&seq) && action==1);cases++;
    printf("actual-C workflow UART: %u offline scenarios passed (no hardware)\n",cases);
}
'''


INTEGRATION_CHECKS = workflow.CHECKS.split("int main(void)")[0] + r'''
static void enqueue(const uint8_t *data,unsigned n) {
    for(unsigned i=0;i<n;i++)jetson_notify_rx_char_from_isr(data[i]);
}
static void pump(void) {
    robot_workflow_obs_t received={0};
    jetson_vision_process();received.now_ms=now_ms;mock_now=now_ms;
    received.has_workflow_control=jetson_get_workflow_control(&received.workflow_action,&received.workflow_seq);
    if(!received.has_workflow_control || received.workflow_action!=JETSON_WORKFLOW_ACTION_ABORT_HOLD) {
        received.has_scan_control=jetson_get_scan_control(&received.scan_control);
        received.has_capture_control=jetson_get_capture_control_ex(&received.capture_action,
            &received.capture_point_id,&received.capture_seq);
        received.has_target_control=jetson_get_target_control_value(&received.target_value,&received.target_seq);
        received.has_vision=jetson_get_vision_error_ex(&received.dcx,&received.dcy,
            &received.vision_valid,&received.vision_seq);
        received.has_distance=jetson_get_safe_distance_ex(&received.distance_mm,
            &received.distance_valid,&received.distance_seq);
    }
    received.heartbeat_alive=jetson_is_link_alive(now_ms);
    robot_workflow_step(&received);now_ms+=10;
}
static void confirm(void) {
    now_ms=s_stop.next_sample;pump();now_ms=s_stop.next_sample;pump();
    assert(!s_stop.pending && !s_stop.restart_locked && !s_workflow_stop_hold);
    assert(s_workflow.state==FLOW_ABORT_HOLD && !laser);
}
int main(void) {
    fixture();front();jetson_vision_init();now_ms=obs.now_ms;
    enqueue(start_frame,sizeof(start_frame));enqueue(abort_frame,sizeof(abort_frame));
    enqueue(home_frame,sizeof(home_frame));enqueue(capture,sizeof(capture));
    enqueue(target,sizeof(target));enqueue(scan_start,sizeof(scan_start));
    enqueue(heartbeat,sizeof(heartbeat));
    int homes=reset_count,captures=capture_count,segments=segment_count;
    pump();assert(s_stop.pending && abort_calls==1 && s_workflow_stop_hold);
    pump();confirm();pump();
    assert(reset_count==homes && capture_count==captures && segment_count==segments && !target_starts);
    assert(!s_workflow_count && !s_abort_count && !s_scan_count);
    enqueue(fresh_start,sizeof(fresh_start));pump();
    assert(s_workflow.state==FLOW_HOMING && reset_count==homes+1);

    fixture();front();jetson_vision_init();now_ms=obs.now_ms;stop_ack_ok=false;
    enqueue(abort_frame,sizeof(abort_frame));enqueue(start_frame,sizeof(start_frame));
    enqueue(heartbeat,sizeof(heartbeat));homes=reset_count;pump();pump();
    assert(s_stop.restart_locked && s_workflow_stop_hold && reset_count==homes);
    stop_ack_ok=true;enqueue(abort_two,sizeof(abort_two));pump();confirm();
    enqueue(fresh_start,sizeof(fresh_start));pump();
    assert(s_workflow.state==FLOW_HOMING && reset_count==homes+1);

    fixture();front();jetson_vision_init();now_ms=obs.now_ms;
    enqueue(abort_frame,sizeof(abort_frame));pump();
    assert(s_stop.pending && abort_calls==1 && !jetson_is_link_alive(now_ms));
    confirm();assert(!laser);
    puts("actual-C UART + workflow: 3 integrated stop/recovery scenarios passed (no hardware)");
}
'''


def main():
    vectors = {
        "start_frame": uart.build_unified_frame(6, 20, b"\x01"),
        "home_frame": uart.build_unified_frame(6, 21, b"\x02"),
        "abort_frame": uart.build_unified_frame(6, 60, b"\x03"),
        "abort_two": uart.build_unified_frame(6, 61, b"\x03"),
        "heartbeat": uart.build_unified_frame(1, 1, bytes(4)),
        "scan_start": uart.build_scan_control(1, 0, 100, 30),
        "query": uart.build_scan_control(0, 0, 0, 40),
        "capture": uart.build_unified_frame(4, 31, b"\x01\x01"),
        "target": uart.build_unified_frame(2, 32, b"\x01"),
        "target_off": uart.build_unified_frame(2, 33, b"\x00"),
        "depth": uart.build_unified_frame(5, 34, b"\x90\x01\x01"),
        "vision": uart.build_unified_frame(3, 35, bytes(4) + b"\x01"),
        "zero_seq": uart.build_unified_frame(6, 0, b"\x03"),
        "bad_len": uart.build_unified_frame(6, 65, b"\x03\x00"),
        "bad_action": uart.build_unified_frame(6, 66, b"\x04"),
        "fresh_start": uart.build_unified_frame(6, 22, b"\x01"),
    }
    def declarations(checks):
        return "\n".join(
            "static const uint8_t " + name + "[]={" + ",".join(map(str, data)) + "};"
            for name, data in vectors.items() if re.search(r"\b" + name + r"\b", checks)
        )
    source = (uart.PREAMBLE + uart.stripped("Middle/crc16.h") +
              uart.stripped("Middle/jetson_vision.h") + uart.stripped("Middle/crc16.c") +
              uart.stripped("Middle/jetson_vision.c") + declarations(CHECKS) + CHECKS)
    run_c(source, "workflow_uart")
    mocks = workflow.MOCKS
    for name in ("jetson_set_workflow_stop_hold", "jetson_send_status",
                 "jetson_send_scan_status", "jetson_send_error"):
        mocks = mocks.replace(function(mocks, name), "")
    mocks = mocks.replace("static StatusReport status_reports[1024];", "")
    integrated = (uart.PREAMBLE + workflow.build_source("")
                  .replace(workflow.MOCKS, mocks) + uart.stripped("Middle/crc16.h") +
                  uart.stripped("Middle/crc16.c") + uart.stripped("Middle/jetson_vision.c") +
                  declarations(INTEGRATION_CHECKS) + INTEGRATION_CHECKS)
    run_c(integrated, "workflow_uart_integrated")


if __name__ == "__main__":
    main()

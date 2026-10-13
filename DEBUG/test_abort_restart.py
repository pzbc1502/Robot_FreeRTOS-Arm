"""Actual workflow + capture lifecycle C; motion/clock are offline substitutes."""
import test_scan_workflow as workflow
from test_scan_motion import declaration, function
from test_stop_confirmation import WORKFLOW_CHECKS, run_c


CHECKS = WORKFLOW_CHECKS.split("int main(void)")[0].replace(
    "capture_active=false; capture_result=ROBOT_CAPTURE_RESULT_OK;step();",
    "capture_set_terminal_result(ROBOT_CAPTURE_RESULT_OK);step();"
) + r'''
static void command(uint8_t action,uint8_t seq) {
    obs.has_workflow_control=true;obs.workflow_action=action;obs.workflow_seq=seq;
    step();obs.has_workflow_control=false;
}
static void capture_phase(unsigned phase) {
    fixture();right();
    if(phase) complete_capture();
    assert(robot_capture_is_active());
}
static void stopped_capture(void) {
    abort_command(60);sample();sample();
    assert(successes(60)==1 && !s_stop.pending && !s_stop.restart_locked);
    assert(s_workflow.state==FLOW_ABORT_HOLD && s_result==ROBOT_CAPTURE_RESULT_ABORTED);
}
static void measured_again(void) {
    reset_result=ROBOT_RESET_RESULT_OK;step();
    assert(s_workflow.state==FLOW_MEASURE_POSITION);
    auto_result=ROBOT_AUTO_RESULT_OK;step();
    assert(s_workflow.state==FLOW_WAIT_SAFE_DISTANCE && !s_workflow.distance_safe_latched);
    obs.has_distance=obs.distance_valid=true;obs.distance_mm=400;
    obs.distance_seq=4;step();step();
    assert(s_workflow.state==FLOW_WAIT_SAFE_DISTANCE && !s_workflow.distance_safe_latched);
    obs.distance_seq=5;step();assert(!s_workflow.distance_safe_latched);
    obs.distance_seq=6;step();obs.has_distance=false;
    assert(s_workflow.state==FLOW_SAFE_READY && s_workflow.distance_safe_latched);
}
int main(void) {
    unsigned cases=0;
    for(unsigned phase=0;phase<2;phase++) {
        capture_phase(phase);int homes=reset_count;
        abort_command(60);
        assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED && !robot_capture_is_active());
        deny_start(61,JETSON_ERROR_BUSY);
        assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED);
        sample();sample();assert(successes(60)==1 && reset_count==homes);
        command(1,62);
        if(s_workflow.state!=FLOW_HOMING) {
            fprintf(stderr,"ABORT -> START regression: phase=%u state=%s; stale capture result faults new round\n",
                    phase,workflow_state_name(s_workflow.state));
            return 1;
        }
        assert(reported(62,RA6_TO_JETSON_COMMAND_ACK,1,0) && reset_count==homes+1);
        assert(s_result==ROBOT_CAPTURE_RESULT_NONE && !robot_capture_is_active());
        assert(!s_capture.base_pose_ready && !s_global_reset.active && !receive_stop_hold);
        assert(!s_workflow.selected_view_id && !s_workflow.target_enabled && !s_scan.flags);
        command(1,62);assert(reset_count==homes+1 && s_workflow.state==FLOW_HOMING);
        measured_again();cases++;

        int segments=segment_count;
        control(1,0,101,80);
        assert(s_scan.id==101 && s_scan.state==JETSON_SCAN_PREPARING && segment_count==segments+1);
        scan_result=ROBOT_SCAN_RESULT_LEFT_READY;step();
        scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;step();
        complete_capture();complete_capture();
        assert(s_workflow.state==FLOW_SCAN_READY && s_scan.flags==6);cases++;
    }

    capture_phase(0);stop_ack_ok=false;abort_command(60);
    deny_start(61,JETSON_ERROR_MOTION_FAILED);
    assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED && s_stop.restart_locked);cases++;

    capture_phase(1);stop_feedback_ok=false;abort_command(60);at(s_stop.deadline);
    deny_start(61,JETSON_ERROR_MOTION_FAILED);
    assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED && s_stop.restart_locked);cases++;

    capture_phase(0);stopped_capture();obs.heartbeat_alive=false;
    deny_start(61,JETSON_ERROR_HEARTBEAT_TIMEOUT);
    assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED);
    obs.heartbeat_alive=true;command(1,62);assert(s_workflow.state==FLOW_HOMING);cases++;

    capture_phase(0);stopped_capture();obs.estop_active=true;
    deny_start(61,JETSON_ERROR_SAFETY);
    assert(s_result==ROBOT_CAPTURE_RESULT_ABORTED && s_stop.restart_locked);cases++;

    capture_phase(0);deny_start(61,JETSON_ERROR_INVALID_STATE);
    assert(robot_capture_is_active() && s_result==ROBOT_CAPTURE_RESULT_RUNNING);cases++;

    capture_phase(0);stopped_capture();command(1,61);
    reset_result=ROBOT_RESET_RESULT_FAILED;step();
    assert(s_workflow.state==FLOW_FAULT_HOLD);
    assert(reported(61,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_FAULT_HOLD,JETSON_ERROR_SOFT_RESET_FAILED));cases++;

    capture_phase(0);stopped_capture();command(1,61);measured_again();
    control(1,0,101,80);scan_result=ROBOT_SCAN_RESULT_LEFT_READY;step();
    scan_result=ROBOT_SCAN_RESULT_RIGHT_READY;step();
    capture_fail(ROBOT_CAPTURE_RESULT_FAILED);step();
    assert(s_workflow.state==FLOW_FAULT_HOLD && s_scan.error==JETSON_ERROR_MOTION_ABORTED);cases++;
    printf("actual-C ABORT -> START recovery: %u offline scenarios passed (no global reset or hardware)\n",cases);
}
'''


def build_source():
    capture = (workflow.ROOT / "APP/robot_capture.c").read_text(encoding="utf-8")
    source = workflow.build_source(CHECKS).replace("static robot_capture_result_t capture_result;", "")
    context = "\n".join(declaration(capture, r"typedef (?:enum|struct)\s*\{[^}]*\}\s*" + name + r";")
                        for name in ("capture_state_t", "capture_ctx_t"))
    context += declaration(capture, r"static capture_ctx_t s_capture;.*?static uint8_t s_result_point_id[^;]*;")
    context += "\nstatic void robot_visual_servo_stop(void) {}\n"
    context += function(capture, "capture_set_terminal_result") + function(capture, "capture_fail")
    source = source.replace(function(source, "robot_capture_init"), context + function(capture, "robot_capture_init"))
    for name in ("robot_capture_cancel", "robot_capture_is_active", "robot_capture_result_consume"):
        source = source.replace(function(source, name), function(capture, name))
    request = function(source, "robot_capture_request")
    request_real_context = request.replace("capture_active ||", "(s_capture.state!=CAPTURE_IDLE) ||")
    request_real_context = request_real_context.replace(
        "capture_active=true; capture_result=ROBOT_CAPTURE_RESULT_RUNNING; return true;",
        "s_capture.state=CAPTURE_START_STEP;s_capture.action=action;s_capture.point_id=id;"
        "s_capture.base_pose_ready=true;s_result=ROBOT_CAPTURE_RESULT_RUNNING;return true;"
    )
    assert request_real_context != request and "s_capture.action=action;" in request_real_context
    return source.replace(request, request_real_context)


if __name__ == "__main__":
    run_c(build_source(), "abort_restart")

"""Exercise actual workflow and target C together; all motion/IO is simulated."""
import os
from pathlib import Path
import subprocess
import tempfile

import test_scan_workflow as workflow
from test_scan_motion import function


TARGET_IO = r'''
static bool servo_active;
static unsigned servo_starts;
static float servo_x, servo_y, servo_z;
#define ROBOT_VISUAL_SERVO_FAULT_NONE 0u
static uint32_t HAL_GetTick(void) { return mock_now; }
static bool robot_is_visual_servo_active(void) { return servo_active; }
static int robot_visual_servo_start(void) { servo_active=true;servo_starts++;return pdPASS; }
static void robot_visual_servo_stop(void) { servo_active=false;servo_x=servo_y=servo_z=0; }
static void robot_visual_servo_set_velocity(float x,float y,float z) { servo_x=x;servo_y=y;servo_z=z; }
static void robot_visual_servo_fault_clear(void) { servo_fault=0; }
'''

CHECKS = workflow.CHECKS.split("int main(void)")[0] + r'''
static bool reported(uint8_t seq,uint8_t event,uint8_t value,uint8_t error) {
    for(unsigned i=0;i<status_count;i++) {
        StatusReport r=status_reports[i];
        if(r.seq==seq && r.event==event && r.value==value && r.error==error) return true;
    }
    return false;
}
static void target(uint8_t value,uint8_t seq) {
    obs.has_target_control=true;obs.target_value=value;obs.target_seq=seq;
    step();obs.has_target_control=false;
}
static void tick(void) {
    if(obs.has_vision)obs.vision_received_ms=obs.now_ms;
    mock_now=obs.now_ms;robot_workflow_step(&obs);obs.now_ms+=10;
}
static void handoff(void) {
    fixture();final();servo_starts=0;
    assert(s_workflow.selected_view_id==2 && s_workflow.capture_done_mask==0);
    unsigned reads=pose_verify_calls;
    int captures=capture_count,segments=segment_count,homes=reset_count;
    obs.has_vision=obs.vision_valid=true;obs.dcx=obs.dcy=0;
    target(1,50);
    assert(reported(50,RA6_TO_JETSON_COMMAND_ACK,1,0));
    assert(reported(50,RA6_TO_JETSON_TARGET_CTRL,1,0));
    assert(reported(50,RA6_TO_JETSON_READY,1,0));
    assert(s_workflow.state==FLOW_TARGET_ACTIVE && target_starts==1);
    assert(s_scan.state==JETSON_SCAN_CHAIN_COMPLETED && s_scan.flags==0);
    assert(pose_verify_calls==reads+1 && !servo_starts && !servo_active);
    assert(robot_target_state_get()==ROBOT_TARGET_STATE_WAIT_DETECT && !s_target.has_vision);
    assert(!robot_target_output_requested());
    assert(capture_count==captures && segment_count==segments && reset_count==homes);
}
static void deny(uint8_t error) {
    static uint8_t next_seq=70;
    uint8_t seq=++next_seq;
    int before=target_starts;
    target(1,seq);
    assert(target_starts==before && !ROBOT_TARGET_ENABLED);
    assert(reported(seq,RA6_TO_JETSON_COMMAND_ACK,0,error));
}
static void output_fixture(void) {
    handoff();obs.fire_button=false;
    for(unsigned i=0;i<8;i++)tick();
    assert(s_target.alignment_confirmed && !laser);
    obs.fire_button=true;tick();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    assert(laser && laser_on_count==1);
}
static void realign_held(void) {
    obs.has_vision=obs.vision_valid=true;obs.dcx=obs.dcy=0;
    for(unsigned i=0;i<8;i++)tick();
}
static void rearm_and_fire(void) {
    uint32_t release_ms=obs.now_ms;
    obs.fire_button=false;tick();
    obs.now_ms=release_ms+TARGET_FIRE_KEY_DEBOUNCE_MS-1;tick();
    assert(!s_target.fire_release_seen && !laser);
    obs.now_ms=release_ms+TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    assert(s_target.fire_release_seen && !laser);
    uint32_t press_ms=obs.now_ms;
    obs.fire_button=true;tick();
    obs.now_ms=press_ms+TARGET_FIRE_KEY_DEBOUNCE_MS-1;tick();assert(!laser);
    obs.now_ms=press_ms+TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    assert(laser && robot_target_output_requested() && laser_on_count==2);
}
int main(void) {
    unsigned cases=0;
    handoff();cases++;
    obs.has_vision=false;
    target(1,50);assert(target_starts==1);cases++;
    target(1,51);assert(target_starts==1);
    assert(reported(51,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_INVALID_STATE));cases++;

    verify_ok=false;obs.now_ms+=400;control(0,0,100,24);
    assert(last().state==JETSON_SCAN_CHAIN_COMPLETED && last().flags==0);
    assert(s_workflow.state==FLOW_TARGET_ACTIVE);cases++;
    target(0,52);assert(s_workflow.state==FLOW_TARGET_HOLD && !ROBOT_TARGET_ENABLED);
    target(1,53);assert(s_workflow.state==FLOW_TARGET_ACTIVE && target_starts==2);
    assert(robot_target_state_get()==ROBOT_TARGET_STATE_WAIT_DETECT);cases++;

    handoff();obs.has_vision=false;
    obs.has_workflow_control=true;obs.workflow_action=2;obs.workflow_seq=60;step();
    obs.has_workflow_control=false;reset_result=ROBOT_RESET_RESULT_OK;step();
    assert(s_workflow.state==FLOW_IDLE && !s_workflow.distance_safe_latched);
    assert(reported(60,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_RETURN_HOME_DONE,0));
    target(1,50);assert(s_workflow.state==FLOW_IDLE && !ROBOT_TARGET_ENABLED && target_starts==1);
    deny(JETSON_ERROR_INVALID_STATE);cases++;

    for(unsigned stage=0;stage<5;stage++) {
        fixture();
        if(stage==1) start();
        if(stage==2) scanning();
        if(stage==3) right();
        if(stage==4){right();complete_capture();}
        deny(JETSON_ERROR_INVALID_STATE);cases++;
    }
    for(unsigned bad=0;bad<7;bad++) {
        fixture();final();
        if(bad==0)s_scan.id=0;
        if(bad==1)s_scan.flags=3;
        if(bad==2)s_scan.state=JETSON_SCAN_RETURNING_FRONT;
        if(bad==3)s_scan.error=JETSON_ERROR_MOTION_FAILED;
        if(bad==4)s_workflow.selected_view_id=3;
        if(bad==5)s_workflow.selected_view_id=0;
        if(bad==6)s_scan.flags=7; // Old middle-confirmation mode cannot qualify.
        deny(bad==5 ? JETSON_ERROR_TARGET_GATE_DENIED : JETSON_ERROR_INVALID_STATE);cases++;
    }
    fixture();final();verify_ok=false;deny(JETSON_ERROR_POSE_INVALID);
    assert(s_workflow.state==FLOW_FAULT_HOLD && s_scan.flags==0);
    verify_ok=true;deny(JETSON_ERROR_INVALID_STATE);cases++;
    fixture();final();g_robot.status=0;deny(JETSON_ERROR_POSE_INVALID);cases++;
    fixture();final();s_workflow.distance_safe_latched=false;deny(JETSON_ERROR_TARGET_GATE_DENIED);cases++;
    fixture();final();busy=true;deny(JETSON_ERROR_BUSY);cases++;
    fixture();final();capture_active=true;deny(JETSON_ERROR_BUSY);cases++;
    fixture();final();obs.heartbeat_alive=false;deny(JETSON_ERROR_HEARTBEAT_TIMEOUT);cases++;
    fixture();final();obs.estop_active=true;deny(JETSON_ERROR_SAFETY);cases++;
    fixture();final();obs.limit_triggered=true;deny(JETSON_ERROR_SAFETY);cases++;
    fixture();final();aborted=true;deny(JETSON_ERROR_MOTION_ABORTED);cases++;
    fixture();final();s_workflow.fault_latched=true;deny(JETSON_ERROR_MOTION_ABORTED);cases++;
    fixture();final();target_start_ok=false;target(1,50);
    assert(s_scan.state==JETSON_SCAN_FRONT_OBSERVE_HOLD && s_scan.flags==6);
    assert(reported(50,RA6_TO_JETSON_COMMAND_ACK,0,JETSON_ERROR_MOTION_FAILED));
    target_start_ok=true;target(1,51);assert(s_workflow.state==FLOW_TARGET_ACTIVE);cases++;

    handoff();obs.has_vision=false;
    obs.has_workflow_control=true;obs.workflow_action=3;obs.workflow_seq=60;step();
    obs.has_workflow_control=false;obs.now_ms=s_stop.next_sample;step();
    obs.now_ms=s_stop.next_sample;step();
    assert(reported(60,RA6_TO_JETSON_WORKFLOW,JETSON_WORKFLOW_ABORTED_HOLD,0));
    assert(!s_workflow.distance_safe_latched && s_workflow.selected_view_id==0);
    deny(JETSON_ERROR_INVALID_STATE);cases++;

    // A held P000 and stale start-cycle vision cannot request output.
    handoff();
    for(unsigned i=0;i<8;i++)step();
    assert(s_target.alignment_confirmed && !robot_target_output_requested() && !laser);
    obs.fire_button=false;step();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;step();
    obs.fire_button=true;step();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    assert(laser && laser_on_count==1 && !servo_active);cases++;
    obs.vision_valid=false;tick();assert(!laser && !robot_target_output_requested());cases++;

    handoff();obs.fire_button=false;
    for(unsigned i=0;i<8;i++)step();
    obs.fire_button=true;step();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    assert(laser);obs.has_vision=false;obs.now_ms+=TARGET_VISION_VALID_MS+1;tick();
    assert(!laser && !robot_target_output_requested());cases++;
    handoff();obs.has_vision=false;obs.now_ms+=TARGET_VISION_VALID_MS+1;step();
    assert(robot_target_state_get()==ROBOT_TARGET_STATE_WAIT_DETECT && !laser);cases++;

    // A recovered observation must not reuse the interrupted output's held key.
    output_fixture();ROBOT_TARGET_FIRE_ENABLE=true;
    obs.vision_valid=false;tick();assert(!laser);
    realign_held();
    if(laser || laser_on_count!=1) {
        fprintf(stderr,"held P000 reused after vision recovery: output_count=%u\n",laser_on_count);
        return 1;
    }
    assert(s_target.alignment_confirmed && !s_target.fire_release_seen && !ROBOT_TARGET_FIRE_ENABLE);
    cases++;rearm_and_fire();cases++;

    output_fixture();obs.has_vision=false;obs.now_ms+=TARGET_VISION_VALID_MS+1;tick();
    assert(!laser && !s_target.fire_release_seen);
    realign_held();assert(!laser && laser_on_count==1);cases++;
    rearm_and_fire();cases++;

    output_fixture();obs.dcy=10;tick();assert(!laser && !s_target.fire_release_seen);
    realign_held();assert(!laser && laser_on_count==1);cases++;
    rearm_and_fire();cases++;

    // Also revoke a press made before CONFIRM was interrupted.
    for(unsigned loss=0;loss<3;loss++) {
        handoff();obs.fire_button=false;
        for(unsigned i=0;i<8;i++)tick();
        obs.fire_button=true;tick();
        assert(s_target.alignment_confirmed && !laser);
        if(loss==0)obs.vision_valid=false;
        if(loss==1){obs.has_vision=false;obs.now_ms+=TARGET_VISION_VALID_MS+1;}
        if(loss==2)obs.dcx=10;
        tick();realign_held();
        assert(s_target.alignment_confirmed && !laser && !s_target.fire_release_seen);cases++;
    }

    output_fixture();ROBOT_TARGET_FIRE_ENABLE=true;servo_fault=1;tick();
    assert(!laser && !s_target.fire_release_seen && !ROBOT_TARGET_FIRE_ENABLE);
    assert(s_workflow.state==FLOW_FAULT_HOLD);servo_fault=0;cases++;

    output_fixture();uint32_t output_ms=s_target.enter_ms;
    for(unsigned elapsed=100;elapsed<TARGET_OUTPUT_MAX_MS;elapsed+=100) {
        obs.now_ms=output_ms+elapsed;tick();assert(laser);
    }
    obs.now_ms=output_ms+TARGET_OUTPUT_MAX_MS;tick();
    assert(!laser && s_workflow.state==FLOW_TARGET_HOLD && !s_target.fire_release_seen);
    for(unsigned i=0;i<8;i++)tick();assert(!laser && laser_on_count==1);cases++;
    obs.has_target_control=true;obs.target_value=1;obs.target_seq=54;tick();
    obs.has_target_control=false;
    realign_held();assert(!laser && laser_on_count==1);cases++;
    rearm_and_fire();cases++;

    // The physical gate closes on raw release; the substate must close too.
    output_fixture();obs.fire_button=false;tick();
    assert(!laser && s_workflow.state==FLOW_TARGET_HOLD && !s_target.fire_release_seen);
    obs.fire_button=true;realign_held();assert(!laser && laser_on_count==1);cases++;

    handoff();ROBOT_TARGET_FIRE_ENABLE=true;target(0,52);target(1,53);
    realign_held();assert(!laser && !ROBOT_TARGET_FIRE_ENABLE);cases++;

    // A brief release just before confirmation cannot preserve a debounced press.
    handoff();obs.fire_button=false;obs.dcx=10;
    for(unsigned i=0;i<8;i++)tick();
    obs.fire_button=true;tick();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    obs.dcx=0;for(unsigned i=0;i<4;i++)tick();
    assert(robot_target_state_get()==ROBOT_TARGET_STATE_CONFIRM && !laser);
    obs.fire_button=false;tick();
    assert(s_target.alignment_confirmed && !robot_target_output_requested());
    uint32_t new_press_ms=obs.now_ms;obs.fire_button=true;tick();assert(!laser);
    obs.now_ms=new_press_ms+TARGET_FIRE_KEY_DEBOUNCE_MS-1;tick();assert(!laser);
    obs.now_ms=new_press_ms+TARGET_FIRE_KEY_DEBOUNCE_MS;tick();assert(laser);cases++;

    output_fixture();obs.vision_valid=false;tick();
    obs.fire_button=false;tick();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    obs.fire_button=true;tick();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
    realign_held();assert(!laser && laser_on_count==1 && !s_target.fire_release_seen);cases++;

    fixture();final();ROBOT_TARGET_FIRE_ENABLE=true;
    obs.has_vision=obs.vision_valid=true;obs.dcx=obs.dcy=0;
    target(1,50);realign_held();
    assert(!laser && !ROBOT_TARGET_FIRE_ENABLE && !s_target.fire_release_seen);cases++;

    // Exercise the configured pixel boundary without relaxing the P000 gate.
    const int16_t edge=(int16_t)TARGET_ALIGN_TOL_PX;
    for(unsigned boundary=0;boundary<4;boundary++) {
        handoff();obs.fire_button=false;
        int16_t sign=(boundary&1u)?-1:1;
        if(boundary<2)obs.dcx=sign*edge;else obs.dcy=sign*edge;
        for(unsigned i=0;i<8;i++)tick();
        assert(s_target.alignment_confirmed && !laser);cases++;
        obs.fire_button=true;tick();obs.now_ms+=TARGET_FIRE_KEY_DEBOUNCE_MS;tick();
        assert(laser && laser_on_count==1);
        if(boundary<2)obs.dcx=sign*(edge+1);else obs.dcy=sign*(edge+1);
        tick();assert(!laser && !s_target.fire_release_seen &&
                      s_target.state==ROBOT_TARGET_STATE_ALIGN);cases++;
    }
    printf("actual-C workflow + target handoff: %u offline scenarios passed (no hardware)\n",cases);
}
'''


def main():
    source = workflow.build_source(checks="")
    for name in ("robot_target_init", "robot_target_stop_hold", "robot_target_start_at_current_pose",
                 "robot_target_output_requested", "robot_target_event_consume", "robot_target_step"):
        source = source.replace(function(source, name), "\n")
    source += TARGET_IO + "\n#define robot_target_start_at_current_pose target_start_real\n"
    source += workflow.stripped("APP/robot_target.c")
    source += "\n#undef robot_target_start_at_current_pose\n"
    source += "bool robot_target_start_at_current_pose(void) { target_starts++; return target_start_ok && target_start_real(); }\n"
    source += CHECKS
    env = os.environ.copy()
    env["PATH"] = str(Path(workflow.GCC).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="scan_target_handoff_") as directory:
        exe = Path(directory) / "check.exe"
        subprocess.run([workflow.GCC, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                        "-x", "c", "-", "-o", str(exe)], input=source, text=True,
                       encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()

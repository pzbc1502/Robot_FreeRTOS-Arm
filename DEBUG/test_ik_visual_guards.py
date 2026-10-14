"""Actual-C IK and visual-control regressions. No serial/debugger/hardware."""
import re

import test_scan_motion as motion
import test_scan_workflow as workflow
from test_stop_confirmation import run_c


KIN_PREAMBLE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#define M_PI 3.14159265358979323846
#define ROBOT_MAX_JOINT_NUM 6
#define LOG(...) do {if(0)printf(__VA_ARGS__);}while(0)
struct position {float x,y,z;};
struct {struct {float min_angle,max_angle;} joints[6];} g_robot;
'''

KIN_CHECKS = r'''
static void limits(void) {
    const float bounds[6][2]={{0,360},{88,180},{-92,90},{-90,90},{0,92},{0,360}};
    for(unsigned j=0;j<6;j++) {
        g_robot.joints[j].min_angle=bounds[j][0];g_robot.joints[j].max_angle=bounds[j][1];
    }
}
static bool matrix_close(const float a[4][4],const float b[4][4]) {
    for(unsigned r=0;r<4;r++) for(unsigned c=0;c<4;c++)
        if(!isfinite(b[r][c]) || fabsf(a[r][c]-b[r][c])>0.003f) return false;
    return true;
}
int main(void) {
    limits();float T[4][4],q[6]={90,120,-50,0,83,0};
    struct position p={0,-128.39f,-14.99f};
    robot_kinematics_cal_T(T_0_6_reset,T,&p);robot_kinematics_joint_angle_update(q);
    if(robot_kinematics_inverse((float *)T,q,0)!=0)return 1;
    q[3]=0;q[4]=83;q[5]=0;robot_kinematics_forward(q,T);
    robot_kinematics_joint_angle_update(q);robot_kinematics_reset_branch_lock();
    unsigned steps=0;
    for(unsigned i=0;i<=1000;i++) {
        float C[4][4];memcpy(C,T,sizeof(C));C[0][3]+=i*.02f;C[2][3]-=i*.02f;
        float previous=q[0];
        if(robot_kinematics_inverse((float *)C,q,0)!=0) {
            fprintf(stderr,"J1 sign regression: displacement=%.2f previous=%.6f candidate=%.6f mask=0x%X\n",
                    i*.02f,previous,g_robot_kinematics.result[0][0],g_robot_kinematics.result_invalid_mask);
            return 1;
        }
        assert(fabsf(q[0]-previous)<0.1f);
        float solved[4][4];assert(robot_kinematics_forward(q,solved)==0);
        for(unsigned r=0;r<3;r++) {
            assert(fabsf(C[r][3]-solved[r][3])<0.003f);
            assert(fabsf(C[r][2]-solved[r][2])<0.0001f);
        }
        robot_kinematics_joint_angle_update(q);steps++;
    }
    unsigned poses=0;
    const float yaw[]={0,0.01f,37,89.99f,90,135,179.99f,180,225,269.99f,270,315,359.99f};
    const float wrist[]={-30,0,35};
    for(unsigned y=0;y<sizeof(yaw)/sizeof(yaw[0]);y++)
        for(unsigned w=0;w<sizeof(wrist)/sizeof(wrist[0]);w++) {
            float measured[6]={yaw[y],120,-50,wrist[w],83,0};
            float desired[4][4],solved[4][4],result[6];
            assert(robot_kinematics_forward(measured,desired)==0);
            robot_kinematics_joint_angle_update(measured);robot_kinematics_reset_branch_lock();
            assert(robot_kinematics_inverse((float *)desired,result,0)==0);
            assert(robot_kinematics_forward(result,solved)==0 && matrix_close(desired,solved));
            poses++;
        }
    unsigned base_checks=0;
    for(unsigned branch=0;branch<2;branch++) for(unsigned y=0;y<sizeof(yaw)/sizeof(yaw[0]);y++) {
        float t2=(branch?120:-60)*(float)M_PI/180,t3=-50*(float)M_PI/180;
        float radial=a2*cosf(t2)+a3*cosf(t2-t3)+d4*sinf(t2-t3);
        float expected=yaw[y]*(float)M_PI/180,angle;
        g_robot_kinematics.T[0][3]=radial*cosf(expected);
        g_robot_kinematics.T[1][3]=radial*sinf(expected);
        assert(__robot_kinematics_calc_theta1(t2,t3,&angle));
        assert(fabsf(remainderf(angle-expected,2*(float)M_PI))<0.00001f);base_checks++;
    }
    unsigned auto_steps=0;
    float home[6]={90,90,-90,0,90,0};robot_kinematics_joint_angle_update(home);
    robot_kinematics_reset_branch_lock();
    for(unsigned i=0;i<=300;i++) {
        struct position at={0,-130*(float)i/300,-15*(float)i/300};
        float desired[4][4],solved[4][4],result[6];
        robot_kinematics_cal_T(T_0_6_reset,desired,&at);
        assert(robot_kinematics_inverse((float *)desired,result,0)==0);
        assert(robot_kinematics_forward(result,solved)==0 && matrix_close(desired,solved));
        robot_kinematics_joint_angle_update(result);auto_steps++;
    }
    float impossible[4][4];memcpy(impossible,T,sizeof(T));impossible[0][3]=2000;
    assert(robot_kinematics_inverse((float *)impossible,q,0)!=0);
    memcpy(impossible,T,sizeof(T));impossible[1][3]=NAN;
    assert(robot_kinematics_inverse((float *)impossible,q,0)!=0);
    float angle=0;
    g_robot_kinematics.T[0][3]=g_robot_kinematics.T[1][3]=0;
    assert(!__robot_kinematics_calc_theta1(120*(float)M_PI/180,-50*(float)M_PI/180,&angle));
    g_robot_kinematics.T[0][3]=10;g_robot_kinematics.T[1][3]=10;
    assert(!__robot_kinematics_calc_theta1(120*(float)M_PI/180,-50*(float)M_PI/180,&angle));
    printf("actual-C IK: %u visual steps, %u quadrant/FK poses, %u signed-radius checks, %u AUTO-line samples, 4 rejections passed (no hardware)\n",
           steps,poses,base_checks,auto_steps);
}
'''


def kinematics_source():
    robot = (workflow.ROOT / "APP/robot.c").read_text(encoding="utf-8")
    header = (workflow.ROOT / "APP/robot.h").read_text(encoding="utf-8")
    source = KIN_PREAMBLE
    source += re.search(r"^#define ROBOT_ERROR_RANGE[^\n]*", header, re.M).group() + "\n"
    for pattern, text in (
        (r"enum\s*\{\s*ROBOT_JOINT_1[^}]*\};", header),
        (r"const float D_H\[6\]\[4\][^;]*;", robot),
        (r"const float T_0_6_reset\[4\]\[4\][^;]*;", robot),
    ):
        source += motion.declaration(text, pattern) + "\n"
    return source + workflow.stripped("Middle/robot_kinematics.h") + "\n" + workflow.stripped("Middle/robot_kinematics.c")


VISUAL_CHECKS = r'''
static void fixture(void) {
    memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));
    float q[6]={90,119.337f,-50.964f,0,83,0};
    for(unsigned j=0;j<6;j++)g_robot.joints[j].current_angle=q[j];
    g_robot.status=0;
    ROBOT_STATUS_SET(g_robot.status,ROBOT_STATUS_VISUAL_SERVO_ACTIVE);
    ROBOT_STATUS_SET(g_robot.status,ROBOT_STATUS_AUTO_BUSY);
    ROBOT_STATUS_SET(g_robot.status,ROBOT_STATUS_POSE_VALID);
    memset(&g_visual_servo,0,sizeof(g_visual_servo));
    g_visual_servo.vx=1;g_visual_servo.vz=-1;
    now_ms=motion_batches=ik_calls=0;peak_speed=0;
    read_delay_ms=abort_at=feedback_fail_at=0;missing_feedback=false;
    velocity_transaction_fail=g_robot_motion_abort_requested=false;
    visual_zero_fail=false;diagnostic_calls=0;
    visual_keep_fresh=true;visual_stop_at=5000;visual_bad_tracking=false;ik_mode=0;
    memset(commanded,0,sizeof(commanded));
}
static void stopped(void) {
    assert(!robot_is_visual_servo_active() && !robot_is_auto_busy());
    for(unsigned j=0;j<5;j++)assert(commanded[j]==0);
    assert(peak_speed<=ROBOT_VISUAL_SERVO_MAX_JOINT_SPEED_DEG_S);
}
static bool target_check(float q[6],float previous[6],bool first) {
    float T[4][4];assert(robot_kinematics_forward(q,T)==0);
    return robot_visual_servo_target_valid(T,q,previous,first);
}
int main(void) {
    unsigned cases=0;
    fixture();assert(robot_visual_servo_run()==0);stopped();
    assert(ik_calls>200 && motion_batches>200 && g_robot.cur_pos.x>4.5f && !diagnostic_calls);cases++;

    fixture();ik_mode=1;assert(robot_visual_servo_run()!=0);stopped();
    assert(ik_calls==4 && !motion_batches && diagnostic_calls==1 &&
           g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_IK);cases++;
    for(unsigned mode=2;mode<=6;mode++) {
        fixture();ik_mode=mode;assert(robot_visual_servo_run()!=0);stopped();
        assert(ik_calls==2 && !motion_batches && g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_IK);cases++;
    }
    for(unsigned mode=1;mode<=2;mode++) {
        fixture();ik_mode=mode;visual_zero_fail=true;
        assert(robot_visual_servo_run()!=0);stopped();
        assert(!diagnostic_calls && !motion_batches &&
               g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_FEEDBACK);cases++;
    }
    fixture();missing_feedback=true;assert(robot_visual_servo_run()!=0);stopped();
    assert(!motion_batches && g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_FEEDBACK);cases++;
    fixture();feedback_fail_at=100;assert(robot_visual_servo_run()!=0);stopped();
    assert(now_ms<300 && g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_FEEDBACK);cases++;
    fixture();velocity_transaction_fail=true;assert(robot_visual_servo_run()!=0);stopped();
    assert(!motion_batches && g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_FEEDBACK);cases++;
    fixture();visual_bad_tracking=true;assert(robot_visual_servo_run()!=0);stopped();
    assert(now_ms<300 && g_visual_servo.fault==ROBOT_VISUAL_SERVO_FAULT_FEEDBACK);cases++;
    fixture();visual_keep_fresh=false;visual_stop_at=1000;
    assert(robot_visual_servo_run()==0);stopped();assert(g_robot.cur_pos.x<0.5f);cases++;
    fixture();abort_at=100;assert(robot_visual_servo_run()==0);stopped();
    assert(g_robot_motion_abort_requested && now_ms<200);cases++;

    float previous[6]={90,120,-50,0,83,0},q[6],T[4][4];
    fixture();memcpy(q,previous,sizeof(q));assert(target_check(q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));q[0]+=.1f;assert(target_check(q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));q[0]+=.3f;assert(!target_check(q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));q[0]+=3;assert(target_check(q,previous,true));cases++;
    memcpy(q,previous,sizeof(q));q[0]+=3.3f;assert(!target_check(q,previous,true));cases++;
    previous[0]=359.99f;memcpy(q,previous,sizeof(q));q[0]=.01f;
    assert(target_check(q,previous,false));previous[0]=90;cases++;
    previous[3]=-30;memcpy(q,previous,sizeof(q));q[3]=330;
    assert(target_check(q,previous,false) && q[3]==-30);previous[3]=0;cases++;
    memcpy(q,previous,sizeof(q));q[3]=120;assert(!target_check(q,previous,true));cases++;
    memcpy(q,previous,sizeof(q));assert(robot_kinematics_forward(q,T)==0);q[1]=NAN;
    assert(!robot_visual_servo_target_valid(T,q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));assert(robot_kinematics_forward(q,T)==0);T[0][3]+=1;
    assert(!robot_visual_servo_target_valid(T,q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));assert(robot_kinematics_forward(q,T)==0);T[1][2]+=.1f;
    assert(!robot_visual_servo_target_valid(T,q,previous,false));cases++;
    memcpy(q,previous,sizeof(q));q[5]=180;assert(target_check(q,previous,false) && q[5]==0);cases++;

    fixture();memcpy(q,previous,sizeof(q));float ff[6]={100,100,100,100,100,0};
    for(unsigned j=0;j<5;j++)g_robot.joints[j].current_angle=previous[j];
    assert(robot_pid_one_period_limited(q,ff,NULL,5,true,
        ROBOT_VISUAL_SERVO_MAX_JOINT_SPEED_DEG_S,ROBOT_VISUAL_SERVO_MAX_TRACK_ERROR_DEG));
    assert(peak_speed==ROBOT_VISUAL_SERVO_MAX_JOINT_SPEED_DEG_S);cases++;
    printf("actual-C visual runner/guards/PID: %u offline scenarios passed (no hardware)\n",cases);
}
'''


def visual_source():
    robot = (workflow.ROOT / "APP/robot.c").read_text(encoding="utf-8")
    source = motion.build_source().removesuffix(motion.CHECKS)
    source = "#include <float.h>\n#define M_PI 3.14159265358979323846\n" + source
    target = (workflow.ROOT / "APP/robot_target.h").read_text(encoding="utf-8")
    source = re.search(r"^#define TARGET_VS_CMD_TIMEOUT_MS[^\n]*", target, re.M).group() + "\n" + source
    source = source.replace("struct position cur_pos; } g_robot;", "struct position cur_pos; float T[4][4]; } g_robot;")
    source = source.replace("static unsigned velocity_fault_count;", "")
    source = source.replace("static char last_scan_guard[256];",
                            "static char last_scan_guard[256];\nstatic unsigned diagnostic_calls;")
    source = source.replace('if (strstr(format,"[SCAN_GUARD]")) {',
                            'if(strstr(format,"[VS_IK]"))diagnostic_calls++;\n'
                            '    if (strstr(format,"[SCAN_GUARD]")) {')
    source = source.replace("static struct { bool stop_requested; } g_visual_servo;", r'''
typedef int robot_visual_servo_fault_t;
#define ROBOT_VISUAL_SERVO_FAULT_NONE 0
#define ROBOT_VISUAL_SERVO_FAULT_FK 2
#define ROBOT_VISUAL_SERVO_FAULT_IK 3
static struct {float vx,vy,vz;uint32_t last_update_ms;bool stop_requested;int fault;} g_visual_servo;
static bool visual_keep_fresh,visual_bad_tracking,visual_zero_fail;
static uint32_t visual_stop_at;
static unsigned ik_mode,ik_calls;
''')
    stub = motion.function(source, "robot_visual_servo_fault_set")
    source = source.replace(stub, "\nstatic void robot_visual_servo_fault_set(robot_visual_servo_fault_t fault);\n")
    zero = motion.function(source, "robot_joint_velocity_zero_all")
    source = source.replace(zero, zero.replace("return true;", "return !visual_zero_fail;"))
    source = source.replace("now_ms+=ms; if(abort_at", r'''
    now_ms+=ms;
    if(visual_keep_fresh)g_visual_servo.last_update_ms=now_ms;
    if(visual_stop_at && now_ms>=visual_stop_at)g_visual_servo.stop_requested=true;
    if(abort_at''')
    source = source.replace("if(feedback_fail_at && now_ms>=feedback_fail_at)",
                            "if(visual_bad_tracking && now_ms>=100)g_robot.joints[1].current_angle=150;\n"
                            "    if(feedback_fail_at && now_ms>=feedback_fail_at)")
    source += motion.declaration(robot, r"const float D_H\[6\]\[4\][^;]*;") + "\n"
    source += motion.declaration(robot, r"const float T_0_6_reset\[4\]\[4\][^;]*;") + "\n"
    source += workflow.stripped("Middle/robot_kinematics.h") + "\n"
    kinematics = workflow.stripped("Middle/robot_kinematics.c")
    source += kinematics.replace("int robot_kinematics_inverse(", "int robot_kinematics_inverse_real(")
    source += r'''
int robot_kinematics_inverse(float *T,volatile float *result,int show) {
    int ret=robot_kinematics_inverse_real(T,result,show);ik_calls++;
    if(ik_calls>=2) {
        if(ik_mode==1)return -1;
        if(ik_mode==2)result[0]+=20;
        if(ik_mode==3)result[3]=120;
        if(ik_mode==4)result[1]+=.1f;
        if(ik_mode==5)result[2]=NAN;
        if(ik_mode==6)result[4]+=1;
    }
    return ret;
}
'''
    for name in ("robot_visual_servo_fault_set", "robot_visual_servo_fault_get",
                 "robot_visual_servo_sync_actual_pose", "robot_visual_servo_wait_period",
                 "robot_visual_servo_target_valid", "robot_visual_servo_run"):
        source += motion.function(robot, name)
    return source + VISUAL_CHECKS


if __name__ == "__main__":
    run_c(kinematics_source() + KIN_CHECKS, "ik")
    run_c(visual_source(), "visual_guards")

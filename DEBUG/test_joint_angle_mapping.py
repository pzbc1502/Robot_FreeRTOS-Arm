"""Actual-C angle mapping and logged FK guard regression. No hardware."""
import test_ik_visual_guards as visual
import test_scan_motion as motion
from test_stop_confirmation import run_c


def build_source():
    robot = (motion.ROOT / "APP/robot.c").read_text(encoding="utf-8")
    source = visual.visual_source().removesuffix(visual.VISUAL_CHECKS)
    source += motion.function(robot, "robot_angle_map_soft_reset")
    return source + CHECKS


CHECKS = r'''
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"angle mapping failed: %s\n",#x);return 1; } } while (0)
int main(void) {
    const float current_j5_max=g_joints_init[4].max_angle;
    /* Reproduce the historical failure under its original 92-degree contract. */
    g_joints_init[4].max_angle=92;
    memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));
    float T[4][4]={
        {-1.89426737e-6f,-1,-7.33790648e-6f,-.0535883904f},
        {.0473493934f,7.23998437e-6f,-.998878419f,-176.015533f},
        {.998878419f,-2.23958796e-6f,.0473493934f,-28.3036976f},
        {0,0,0,1}
    };
    float previous[6]={89.9825516f,128.966705f,-50.2207031f,.0170111693f,91.9013519f,0};
    float q[6],F[4][4],mapped;
    robot_kinematics_joint_angle_update(previous);
    robot_kinematics_reset_branch_lock();
    CHECK(robot_kinematics_inverse_real((float *)T,q,false)==0);
    float original_j5=q[4];
    CHECK(robot_visual_servo_target_valid(T,q,previous,false));
    CHECK(q[4]==original_j5 && q[5]==0);
    CHECK(robot_kinematics_forward(q,F)==0);
    CHECK(fabsf(F[2][2]-T[2][2])<0.001f);
    printf("logged J5=%.9g retained; FK Z-axis residual=%.9g\n",q[4],F[2][2]-T[2][2]);

    float continued[4][4];memcpy(continued,T,sizeof(continued));
    continued[2][3]-=.3f;
    CHECK(robot_kinematics_inverse_real((float *)continued,q,false)==0 && q[4]<=92);
    memcpy(continued,T,sizeof(continued));continued[2][3]-=.4f;
    CHECK(robot_kinematics_inverse_real((float *)continued,q,false)!=0);
    CHECK(g_geometry_invalid_mask==0 &&
          (g_robot_kinematics.result_invalid_mask & 1u)!=0 &&
          g_robot_kinematics.result[0][4]+360>92);

    g_joints_init[4].max_angle=current_j5_max;
    memcpy(g_robot.joints,g_joints_init,sizeof(g_joints_init));

    unsigned bounds=0;
    for (unsigned j=1;j<5;j++) {
        float lo=g_joints_init[j].min_angle,hi=g_joints_init[j].max_angle;
        const float inside[]={lo,lo+.05f,(lo+hi)/2,hi-.05f,hi};
        for (unsigned i=0;i<5;i++) {
            CHECK(robot_angle_map(inside[i],lo,hi,&mapped)==0 && mapped==inside[i]);
            CHECK(robot_joint_compare_angle(j,inside[i],&mapped)==0 && mapped==inside[i]);
            bounds++;
        }
        CHECK(robot_angle_map(lo-ROBOT_ERROR_RANGE*.5f,lo,hi,&mapped)==0 && mapped==lo);bounds++;
        CHECK(robot_angle_map(hi+ROBOT_ERROR_RANGE*.5f,lo,hi,&mapped)==0 && mapped==hi);bounds++;
        CHECK(robot_angle_map(lo-ROBOT_ERROR_RANGE*2,lo,hi,&mapped)!=0);bounds++;
        CHECK(robot_angle_map(hi+ROBOT_ERROR_RANGE*2,lo,hi,&mapped)!=0);bounds++;
        CHECK(robot_angle_map(lo-.05f,lo,hi,&mapped)!=0);bounds++;
        CHECK(robot_angle_map(hi+.05f,lo,hi,&mapped)!=0);bounds++;
        CHECK(robot_angle_map(inside[3]+360,lo,hi,&mapped)==0 &&
              fabsf(mapped-inside[3])<=ROBOT_ERROR_RANGE);bounds++;
        CHECK(robot_angle_map(inside[1]-360,lo,hi,&mapped)==0 &&
              fabsf(mapped-inside[1])<=ROBOT_ERROR_RANGE);bounds++;
    }
    CHECK(robot_joint_compare_angle(3,345,&mapped)==0 && mapped==-15);
    CHECK(robot_joint_compare_angle(3,270,&mapped)==0 && mapped==-90);
    CHECK(robot_joint_compare_angle(0,360,&mapped)==0 && mapped==0);

    mapped=-123;
    CHECK(robot_angle_map(NAN,0,92,&mapped)!=0 && mapped==-123);
    CHECK(robot_angle_map(INFINITY,0,92,&mapped)!=0);
    CHECK(robot_angle_map(-INFINITY,0,92,&mapped)!=0);
    CHECK(robot_angle_map(90,NAN,92,&mapped)!=0);
    CHECK(robot_angle_map(90,0,INFINITY,&mapped)!=0);
    CHECK(robot_angle_map(90,92,0,&mapped)!=0);
    CHECK(robot_angle_map(90,0,92,NULL)!=0);

    CHECK(robot_angle_map(92.5f,0,92,&mapped)!=0);
    CHECK(robot_angle_map_soft_reset(92.5f,0,92,&mapped)==0 && mapped==92);
    CHECK(robot_angle_map_soft_reset(-.5f,0,92,&mapped)==0 && mapped==0);
    CHECK(robot_angle_map_soft_reset(93.1f,0,92,&mapped)!=0);
    CHECK(robot_angle_map_soft_reset(91.95f,0,92,&mapped)==0 && mapped==91.95f);

    unsigned poses=0;
    for (unsigned i=0;i<=200;i++) {
        memcpy(q,previous,sizeof(q));q[4]=current_j5_max-.2f+i*.001f;q[5]=0;
        CHECK(robot_kinematics_forward(q,T)==0);
        float anchor[6];memcpy(anchor,q,sizeof(anchor));
        float expected_j5=q[4];
        CHECK(robot_visual_servo_target_valid(T,q,anchor,false));
        CHECK(q[4]==expected_j5);poses++;
    }
    memcpy(q,previous,sizeof(q));q[4]=current_j5_max+.01f;
    CHECK(robot_kinematics_forward(q,T)==0);
    CHECK(!robot_visual_servo_target_valid(T,q,previous,false));
    memcpy(q,previous,sizeof(q));
    CHECK(robot_kinematics_forward(q,T)==0);T[2][2]+=.003f;
    CHECK(!robot_visual_servo_target_valid(T,q,previous,false));
    printf("actual-C mapping: historical 92-degree log, current J5 max=%.2f, %u boundary scenarios, %u near-limit FK poses, "
           "J4 wrap, numeric rejection, soft-reset allowance and guard rejection passed (no hardware)\n",current_j5_max,bounds,poses);
}
'''


if __name__ == "__main__":
    run_c(build_source(), "joint_angle_mapping")

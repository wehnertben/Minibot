#include "chassis_task.h"
#include "robot.h"
#include "remote.h"
#include <motor.h>
#include <dji_motor.h>
#include <math.h>
#include <stdlib.h>


typedef struct robotSpeeds{
    float xVel,yVel,w;
} robotSpeeds_t;

typedef struct constants{
    float theta,radius,dist,maxSpeed;
} constants_t;

typedef struct motorSpeeds{
    float vel1, vel2, vel3, vel4;
} motorSpeeds_t;

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

constants_t chassisConstants = {PI/4 ,0.2 ,0.3,10};
motorSpeeds_t ChassisSpeeds = {0.0,0.0,0.0,0.0};
robotSpeeds_t TargetSpeeds = {0.0,0.0,0.0};

void Chassis_Task_Init()
{
    // Init chassis hardware
    DJI_Motor_Handle_t *motor1;
    DJI_Motor_Handle_t *motor2;
    DJI_Motor_Handle_t *motor3;
    DJI_Motor_Handle_t *motor4;
    
    int motorID = 1;

    Motor_Config_t chassis_motor_config = {
        .can_bus = 1,
        .speed_controller_id = motorID,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid=
        {
            .kp = 500.0f,
            .kd = 20.0f,
            .kf = 100.0f,
            .output_limit = M2006_MAX_CURRENT,
        }
    };

    motor1 = DJI_Motor_Init(&chassis_motor_config,M2006);
    motorID+=1;
    motor2 = DJI_Motor_Init(&chassis_motor_config,M2006);
    motorID+=1;
    motor3 = DJI_Motor_Init(&chassis_motor_config,M2006);
    motorID+=1;
    motor4 = DJI_Motor_Init(&chassis_motor_config,M2006);

    
}

void Chassis_Ctrl_Loop()
{
    Process_Remote_Input();
    TargetSpeeds.xVel = g_robot_state.chassis.x_speed; 
    TargetSpeeds.yVel =g_robot_state.chassis.y_speed; 
    TargetSpeeds.w =g_robot_state.chassis.omega;
    
    mapping(TargetSpeeds,chassisConstants,&ChassisSpeeds);
    desaturation(&ChassisSpeeds, chassisConstants.maxSpeed);

}



void mapping(robotSpeeds_t speeds, constants_t constants, motorSpeeds_t* mSpeeds){
    (*mSpeeds).vel1 = -1*sin(constants.theta)*speeds.xVel + cos(constants.theta)*speeds.yVel + constants.dist*speeds.w;
    (*mSpeeds).vel2 = -1*cos(constants.theta)*speeds.xVel - sin(constants.theta)*speeds.yVel + constants.dist*speeds.w;
    (*mSpeeds).vel3 = sin(constants.theta)*speeds.xVel - cos(constants.theta)*speeds.yVel + constants.dist*speeds.w;
    (*mSpeeds).vel4 = cos(constants.theta)*speeds.xVel + sin(constants.theta)*speeds.yVel + constants.dist*speeds.w;
}

void desaturation(motorSpeeds_t* mSpeeds, float speedMax){
    float bigSpeed = max((*mSpeeds).vel1, (*mSpeeds).vel2, (*mSpeeds).vel3, (*mSpeeds).vel4);

    if (bigSpeed > speedMax){
        float ratio = speedMax/bigSpeed;

        (*mSpeeds).vel1 *= ratio;
        (*mSpeeds).vel2 *= ratio;
        (*mSpeeds).vel3 *= ratio;
        (*mSpeeds).vel4 *= ratio;
    }
}

float max(float a, float b, float c, float d){
    float m = fmaxf(a,b);
    m = fmaxf(m,c);
    m = fmaxf(m,d);
    return m;
    
}


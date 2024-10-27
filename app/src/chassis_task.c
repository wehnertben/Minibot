#include "chassis_task.h"
#include "robot.h"
#include "remote.h"
#include <motor.h>
#include <dji_motor.h>
#include <math.h>
#include <stdlib.h>




extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad,ratio,bigSpeed;
//                           theta,radius,dist,maxSpeed
constants_t chassisConstants = {PI/4 ,0.035 ,0.26,100};
motorSpeeds_t ChassisSpeeds = {0.0,0.0,0.0,0.0};
robotSpeeds_t TargetSpeeds = {0.0,0.0,0.0};

DJI_Motor_Handle_t *motor1;
DJI_Motor_Handle_t *motor2;
DJI_Motor_Handle_t *motor3;
DJI_Motor_Handle_t *motor4;

void Chassis_Task_Init()
{
    // Init chassis hardware
    
    
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
    chassis_motor_config.speed_controller_id = 1;
    motor1 = DJI_Motor_Init(&chassis_motor_config,M2006);
    chassis_motor_config.speed_controller_id = 2;
    motor2 = DJI_Motor_Init(&chassis_motor_config,M2006);
    chassis_motor_config.speed_controller_id = 3;
    motor3 = DJI_Motor_Init(&chassis_motor_config,M2006);
    chassis_motor_config.speed_controller_id = 4;
    motor4 = DJI_Motor_Init(&chassis_motor_config,M2006);

    
}

void Chassis_Ctrl_Loop()
{
    Process_Remote_Input();
    TargetSpeeds.xVel = g_robot_state.chassis.x_speed; 
    TargetSpeeds.yVel =g_robot_state.chassis.y_speed; 
    TargetSpeeds.w = g_robot_state.chassis.omega;
    
    mapping(TargetSpeeds,chassisConstants,&ChassisSpeeds);
    desaturation(&ChassisSpeeds, chassisConstants.maxSpeed);

    DJI_Motor_Set_Velocity(motor1,ChassisSpeeds.vel1);
    DJI_Motor_Set_Velocity(motor2,ChassisSpeeds.vel2);
    DJI_Motor_Set_Velocity(motor3,ChassisSpeeds.vel3);
    DJI_Motor_Set_Velocity(motor4,ChassisSpeeds.vel4);

}



void mapping(robotSpeeds_t speeds, constants_t constants, motorSpeeds_t* mSpeeds){
    (*mSpeeds).vel1 = (-1*sin(constants.theta)*speeds.xVel + cos(constants.theta)*speeds.yVel + constants.dist*speeds.w)/constants.radius;
    (*mSpeeds).vel2 = (-1*cos(constants.theta)*speeds.xVel - sin(constants.theta)*speeds.yVel + constants.dist*speeds.w)/constants.radius;
    (*mSpeeds).vel3 = (sin(constants.theta)*speeds.xVel - cos(constants.theta)*speeds.yVel + constants.dist*speeds.w)/constants.radius;
    (*mSpeeds).vel4 = (cos(constants.theta)*speeds.xVel + sin(constants.theta)*speeds.yVel + constants.dist*speeds.w)/constants.radius;
}


void desaturation(motorSpeeds_t* mSpeeds, float speedMax){
    bigSpeed = max((*mSpeeds).vel1, (*mSpeeds).vel2, (*mSpeeds).vel3, (*mSpeeds).vel4);

    if (bigSpeed > speedMax){
        ratio = speedMax/bigSpeed;

        (*mSpeeds).vel1 *= ratio;
        (*mSpeeds).vel2 *= ratio;
        (*mSpeeds).vel3 *= ratio;
        (*mSpeeds).vel4 *= ratio;
    }
}

float max(float a, float b, float c, float d){
    float a1= abs(a);
    float b1= abs(b);
    float c1= abs(c);
    float d1= abs(d);

    float m = fmaxf(a1,b1);
    m = fmaxf(m,c1);
    m = fmaxf(m,d1);
    return m;
    
}


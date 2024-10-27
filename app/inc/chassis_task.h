#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

typedef struct robotSpeeds{
    float xVel,yVel,w;
} robotSpeeds_t;

typedef struct constants{
    float theta,radius,dist,maxSpeed;
} constants_t;

typedef struct motorSpeeds{
    float vel1, vel2, vel3, vel4;
} motorSpeeds_t;

void mapping(robotSpeeds_t speeds, constants_t constants, motorSpeeds_t* mSpeeds);
void desaturation(motorSpeeds_t* mSpeeds, float speedMax);
float max(float a, float b, float c, float d);




#endif // CHASSIS_TASK_H

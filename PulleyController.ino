#include <AccelStepper.h>

#define BAUD_RATE 115200
#define FREEZE_COMMAND 'F'
#define HOME_COMMAND 'H'
#define COORDS_COMMAND 'C'

bool waiting_on_x = false;
bool waiting_on_y = false;

int home_coord_x = 0;
int home_coord_y = 0;
int goal_coord_x = home_coord_x;
int goal_coord_y = home_coord_y;
int last_goal_x = 0;
int last_goal_y = 0;

bool freeze_motor = false;

AccelStepper motor1 = AccelStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper motor2 = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper motor3 = AccelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
AccelStepper motor4 = AccelStepper(AccelStepper::FULL4WIRE, 14, 15, 16, 17);

const float MOTOR_MAX_SPEED = 100.0;
const float MOTOR_ACCELERATION = 100.0;

void setup()
{
    Serial.init(BAUD_RATE);

    motor1.setMaxSpeed(MOTOR_MAX_SPEED);
    motor2.setMaxSpeed(MOTOR_MAX_SPEED);
    motor3.setMaxSpeed(MOTOR_MAX_SPEED);
    motor4.setMaxSpeed(MOTOR_MAX_SPEED);

    motor1.setAcceleration(MOTOR_ACCELERATION);
    motor2.setAcceleration(MOTOR_ACCELERATION);
    motor3.setAcceleration(MOTOR_ACCELERATION);
    motor4.setAcceleration(MOTOR_ACCELERATION);

    updateCoords(home_coord_x, home_coord_y);
}

void loop()
{
    //handle serial
    while (Serial.available())
    {
        if (waiting_on_x)
        {
            String s_in = Serial.readString();
            goal_coord_x = atoi(s_in);
            waiting_on_y = true;
        }
        else if (waiting_on_y)
        {
            String s_in = Serial.readString();
            goal_coord_y = atoi(s_in);
            updateCoords(goal_coord_x, goal_coord_y);
        }
        else
        {
            char c_in = Serial.read();
            switch (c_in)
            {
            case FREEZE_COMMAND:
                freeze_motor = !freeze_motor;
                break;
            case HOME_COMMAND:
                updateCoords(home_coord_x, home_coord_y);
                break;
            case COORDS_COMMAND:
                delay(10); //let buffer fill
                waiting_on_x = true;
                break;
            }
        }
    }

    //handle motors
    if (!freeze_motor)
    {
        motor1.run();
        motor2.run();
        motor3.run();
        motor4.run();
    }
}

void updateCoords(int x, int y)
{
    float d11 = sqrt((x - m1x) * (x - m1x) + (y - m1y) * (y - m1y));
    float d21 = sqrt((x - m2x) * (x - m2x) + (y - m2y) * (y - m2y));
    float d31 = sqrt((x - m3x) * (x - m3x) + (y - m3y) * (y - m3y));
    float d41 = sqrt((x - m4x) * (x - m4x) + (y - m4y) * (y - m4y));

    float d10 = sqrt((last_goal_x - m1x) * (last_goal_x - m1x) + (last_goal_y - m1y) * (last_goal_y - m1y));
    float d20 = sqrt((last_goal_x - m2x) * (last_goal_x - m2x) + (last_goal_y - m2y) * (last_goal_y - m2y));
    float d30 = sqrt((last_goal_x - m3x) * (last_goal_x - m3x) + (last_goal_y - m3y) * (last_goal_y - m3y));
    float d40 = sqrt((last_goal_x - m4x) * (last_goal_x - m4x) + (last_goal_y - m4y) * (last_goal_y - m4y));

    float delt1 = abs(d11 - d10);
    float delt2 = abs(d21 - d20);
    float delt3 = abs(d31 - d30);
    float delt4 = abs(d41 - d40);

    float deltmax = delt1;
    if (delt2 > deltmax)
        deltmax = delt2;
    if (delt3 > deltmax)
        deltmax = delt3;
    if (delt4 > deltmax)
        deltmax = delt4;

    float v1 = MOTOR_MAX_SPEED * delt1 / deltmax;
    float v2 = MOTOR_MAX_SPEED * delt2 / deltmax;
    float v3 = MOTOR_MAX_SPEED * delt3 / deltmax;
    float v4 = MOTOR_MAX_SPEED * delt4 / deltmax;

    motor1.setMaxSpeed(v1);
    motor2.setMaxSpeed(v2);
    motor3.setMaxSpeed(v3);
    motor4.setMaxSpeed(v4);

    motor1.moveTo(d11);
    motor2.moveTo(d11);
    motor3.moveTo(d11);
    motor4.moveTo(d11);
}
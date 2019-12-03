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

bool freeze_motor = false;

AccelStepper motor1 = AccelStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper motor2 = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper motor3 = AccelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
AccelStepper motor4 = AccelStepper(AccelStepper::FULL4WIRE, 14, 15, 16, 17);

const double MOTOR_MAX_SPEED = 300.0;
const double MOTOR_ACCELERATION = 100.0;

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
}
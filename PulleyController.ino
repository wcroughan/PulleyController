
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

void setup()
{
    Serial.init(BAUD_RATE);
}

void loop()
{
    //handle serial
    while (Serial.available())
    {
        char c_in = Serial.read();
        if (waiting_on_x)
        {
            //TODO parse coord
            waiting_on_y = true;
        }
        else if (waiting_on_y)
        {
            waiting_on_x = true;
        }
        else
        {
            switch (c_in)
            {
            case FREEZE_COMMAND:
                freeze_motor = !freeze_motor;
                break;
            case HOME_COMMAND:
                goal_coord_x = home_coord_x;
                goal_coord_y = home_coord_y;
                break;
            case COORDS_COMMAND:
                waiting_on_x = true;
                break;
            }
        }
    }

    //handle motor
}
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)


def main_loop():
    # check for update coordinates from arduino
    input = ser.readline()
    print(input)

    # calculate what the motor commands should be

    # send commands
    ser.write(b'hi\n')


loop_n = 1000

if __name__ == "__main__":
    iter = 0
    while iter < loop_n:
        iter += 1
        main_loop()

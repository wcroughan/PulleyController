import serial
import numpy as np
from TrodesInterface import SGClient

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
CMD_READY_FOR_POS = 8


class MotorSerialCommunicator:
    EXPECT_CONFIRM_MSG = True

    def parse_input(self, input):
        # print(input)
        if len(input) != 9 or (input[0] != 0):
            print("Malformed input from arduino!")
            return False, 0, 0
        return True, int.from_bytes(input[1:5], "little", signed=True), \
            int.from_bytes(input[5:9], "little", signed=True)

    def encode_motor_command(self, cmd):
        a = [0]
        return bytes(a)


class MotorController:
    MSTATE_FLOAT = 0
    MSTATE_BRAKE = 1
    MSTATE_FORWARD = 2
    MSTATE_REVERSE = 3

    MIN_X_GOAL = 0
    MIN_Y_GOAL = 0
    MAX_X_GOAL = 100
    MAX_Y_GOAL = 100

    MAX_VEL_DIFF_COEFF = 1
    VEL_RAMP_SPEED = 1
    POSITION_TOLERANCE = 10
    BRAKE_TRIGGER_VEL = 1

    def __init__(self):
        self._xpos = 0
        self._ypos = 0
        self._xvel = 0
        self._yvel = 0
        self._xgoal = 0
        self._ygoal = 0

        self._state_m1 = self.MSTATE_FLOAT
        self._state_m2 = self.MSTATE_FLOAT

    def getMaxVelForDiff(self, diff):
        return min(255, abs(diff * self.MAX_VEL_DIFF_COEFF))

    def get_motor_command(self):
        # output should be one of the following for each motor:
        #   float command, brake command
        #   move command with a velocity
        ydiff = self._ygoal - self._ypos
        if abs(ydiff) < self.POSITION_TOLERANCE:
            next_yvel = 0
            if abs(self._yvel) > self.BRAKE_TRIGGER_VEL:
                next_ystate = self.MSTATE_BRAKE
            else:
                next_ystate = self.MSTATE_FLOAT
        else:
            max_yvel = self.getMaxVelForDiff(ydiff)
            next_yvel = self._yvel + self.VEL_RAMP_SPEED * np.sign(ydiff)
            if abs(next_yvel) > max_yvel:
                next_yvel = max_yvel * np.sign(next_yvel)

            if next_yvel < 0:
                next_ystate = self.MSTATE_REVERSE
            else:
                next_ystate = self.MSTATE_FORWARD
            next_yvel = abs(next_yvel)

        # xdiff = self._xgoal - self._xpos
        next_xstate = self.MSTATE_FLOAT
        next_xvel = 0
        return next_xstate, next_xvel, next_ystate, next_yvel

    def set_goal(self, xgoal, ygoal):
        self._xgoal = xgoal
        self._ygoal = ygoal
        if self._xgoal < self.MIN_X_GOAL:
            self._xgoal = self.MIN_X_GOAL
            print("Clipping xgoal of {} to min: {}".format(
                xgoal, self.MIN_X_GOAL))
        elif self._xgoal > self.MAX_X_GOAL:
            self._xgoal = self.MAX_X_GOAL
            print("Clipping xgoal of {} to max: {}".format(
                xgoal, self.MAX_X_GOAL))
        if self._ygoal < self.MIN_Y_GOAL:
            self._ygoal = self.MIN_Y_GOAL
            print("Clipping ygoal of {} to min: {}".format(
                ygoal, self.MIN_Y_GOAL))
        elif self._ygoal > self.MAX_Y_GOAL:
            self._ygoal = self.MAX_Y_GOAL
            print("Clipping ygoal of {} to max: {}".format(
                ygoal, self.MAX_Y_GOAL))

    def update_position(self, xpos, ypos):
        self._xpos = xpos
        self._ypos = ypos


if __name__ == "__main__":
    print("hello!")
    # connect to trodes
    tclient = SGClient("pulleycon")
    tclient.subscribeToPosition()

    # connect to arduino
    com = MotorSerialCommunicator()
    ctrl = MotorController()

    # ser.write(bytes([CMD_READY_FOR_POS]))

    loop_n = 10
    iter = 0
    while iter < loop_n:
        iter += 1
        print(iter)

        # get goal location from Trodes
        ts, xgoal, ygoal = tclient.getPosition()
        if ts == 0:
            print("Blank input from Trodes")
        else:
            ctrl.set_goal(xgoal, ygoal)
            print("trodes sent this pos:", xgoal, ygoal)

        # check for update coordinates from arduino
        input = ser.read(9)
        goodform, xpos, ypos = com.parse_input(input)
        print("got position {}, {}\n".format(xpos, ypos))

        # calculate what the motor commands should be
        if goodform:
            ctrl.update_position(xpos, ypos)
            cmd = ctrl.get_motor_command()

            # send commands
            output = com.encode_motor_command(cmd)
            # ser.write(output)
            ser.write(bytes([iter % 4]))

            if com.EXPECT_CONFIRM_MSG:
                print("waiting for message")
                msgb = ser.readline()
                print(msgb)
                msg = msgb.decode("ascii")
                print(msg)
                if "forward" in msg or "reverse" in msg:
                    msg2 = ser.readline().decode("ascii")
                    print(msg2)

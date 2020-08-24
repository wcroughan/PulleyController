import serial
import signal
import numpy as np
from time import sleep
from TrodesInterface import SGClient

VERBOSITY = 0
X_COORD_IS_MOTOR_1 = False

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

CMD_CALIBRATE_HOME = 10
CMD_GO_HOME = 11
CMD_QUERY_RANGES = 12
CMD_RESET_EDGE_DETECTOR = 9
CMD_READY_FOR_POS = 8
CMD_F1 = 0
CMD_F2 = 1
CMD_B1 = 2
CMD_B2 = 3
CMD_G1 = 4
CMD_G2 = 5
CMD_R1 = 6
CMD_R2 = 7

MSTATE_FLOAT = 0
MSTATE_BRAKE = 1
MSTATE_FORWARD = 2
MSTATE_REVERSE = 3

TRACKING_MOVE_COMMAND_FREQ = 15
END_ON_BLANK = True

motor_range_x = 0
motor_range_y = 0
motor_range_calibrated = False


class MotorController:
    MIN_X_GOAL = -3000
    MIN_Y_GOAL = -3000
    MAX_X_GOAL = 3000
    MAX_Y_GOAL = 3000

    MAX_VEL_DIFF_COEFF = 0.2
    VEL_RAMP_SPEED = 5
    POSITION_TOLERANCE = 20
    BRAKE_TRIGGER_VEL = 1000
    MINIMUM_SPEED = 60
    MAXIMUM_SPEED = 120

    def __init__(self):
        self._xpos = 0
        self._ypos = 0
        self._xvel = 0
        self._yvel = 0
        self._xgoal = 0
        self._ygoal = 0

        self._state_m1 = MSTATE_FLOAT
        self._state_m2 = MSTATE_FLOAT

    def getMaxVelForDiff(self, diff):
        return int(min(255, abs(diff * self.MAX_VEL_DIFF_COEFF)))

    def get_motor_command(self):
        # output should be one of the following for each motor:
        #   float command, brake command
        #   move command with a velocity
        ydiff = self._ygoal - self._ypos
        if abs(ydiff) < self.POSITION_TOLERANCE or (np.sign(ydiff) != np.sign(self._yvel) and np.sign(self._yvel) != 0):
            next_yvel = 0
            if abs(self._yvel) > self.BRAKE_TRIGGER_VEL:
                next_ystate = MSTATE_BRAKE
            else:
                next_ystate = MSTATE_FLOAT
        else:
            max_yvel = self.getMaxVelForDiff(ydiff)
            next_yvel = self._yvel + self.VEL_RAMP_SPEED * np.sign(ydiff)
            if abs(next_yvel) > max_yvel:
                next_yvel = max_yvel * np.sign(next_yvel)
            if abs(next_yvel) < self.MINIMUM_SPEED:
                next_yvel = self.MINIMUM_SPEED * np.sign(next_yvel)
            if abs(next_yvel) > self.MAXIMUM_SPEED:
                next_yvel = self.MAXIMUM_SPEED * np.sign(next_yvel)
            self._yvel = next_yvel

            if next_yvel < 0:
                next_ystate = MSTATE_FORWARD
            else:
                next_ystate = MSTATE_REVERSE
            next_yvel = abs(next_yvel)
            if VERBOSITY >= 2:
                print("maxyvel = {}, ydiff = {}, yvel = {}".format(
                    max_yvel, ydiff, next_yvel))

        xdiff = self._xgoal - self._xpos
        if abs(xdiff) < self.POSITION_TOLERANCE:
            next_xvel = 0
            if abs(self._xvel) > self.BRAKE_TRIGGER_VEL:
                next_xstate = MSTATE_BRAKE
            else:
                next_xstate = MSTATE_FLOAT
        else:
            max_xvel = self.getMaxVelForDiff(xdiff)
            next_xvel = self._xvel + self.VEL_RAMP_SPEED * np.sign(xdiff)
            if abs(next_xvel) > max_xvel:
                next_xvel = max_xvel * np.sign(next_xvel)
            if abs(next_xvel) < self.MINIMUM_SPEED:
                next_xvel = self.MINIMUM_SPEED * np.sign(next_xvel)
            if abs(next_xvel) > self.MAXIMUM_SPEED:
                next_xvel = self.MAXIMUM_SPEED * np.sign(next_xvel)
            self._xvel = next_xvel

            if next_xvel < 0:
                next_xstate = MSTATE_FORWARD
            else:
                next_xstate = MSTATE_REVERSE
            next_xvel = abs(next_xvel)
            if VERBOSITY >= 2:
                print("maxxvel = {}, xdiff = {}, xvel = {}".format(
                    max_xvel, xdiff, next_xvel))

        if VERBOSITY >= 2:
            print(next_xstate, next_xvel, next_ystate, next_yvel)
        return next_xstate, next_xvel, next_ystate, next_yvel

    def motor_is_stopped(self):
        self._xvel = 0
        self._yvel = 0

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


def mv_done(signum, frame):
    print("move done")
    raise TimeoutError("end of timeout")


# Returns 1 when user requests to cancel within duration, returns 2 o/w
def allow_cancel_prompt(duration):
    signal.setitimer(signal.ITIMER_REAL, duration)
    try:
        input("press enter to cancel...")
        signal.alarm(0)
        return 1
    except Exception:
        return 2


def timed_move(duration, motor_num):
    if motor_num == 1:
        # CMD_END_MV = CMD_B1
        CMD_END_MV = CMD_F1
    else:
        # CMD_END_MV = CMD_B2
        CMD_END_MV = CMD_F2
    cancel_method = allow_cancel_prompt(duration)
    ser.write(bytes([CMD_END_MV]))
    if cancel_method == 1:
        print("sent end move command by user request")
    elif cancel_method == 2:
        print("sent end move command by timeout")
    else:
        print("send end move command, unrecognized return code")
    recv_confirm()


signal.signal(signal.SIGALRM, mv_done)


class MotorSerialCommunicator:
    EXPECT_CONFIRM_MSG = True

    def parse_input(self, inp):
        # print(inp)
        if len(inp) != 9 or (inp[0] != 0):
            print("Malformed input from arduino!")
            print(inp)
            return False, 0, 0
        return True, int.from_bytes(inp[1:5], "little", signed=True), \
            int.from_bytes(inp[5:9], "little", signed=True)

    def encode_manual_motor_command(self, cmd, motor_num, ignore_edge_detect=False):
        if motor_num == 1:
            CMD_F = CMD_F1
            CMD_R = CMD_R1
            CMD_B = CMD_B1
            CMD_G = CMD_G1
        else:
            CMD_F = CMD_F2
            CMD_R = CMD_R2
            CMD_B = CMD_B2
            CMD_G = CMD_G2

        if cmd[0] == "f":
            a = [CMD_F]
        elif cmd[0] == "b":
            a = [CMD_B]
        elif cmd[0] == "g":
            if ignore_edge_detect:
                a = [CMD_G, cmd[1], 1]
            else:
                a = [CMD_G, cmd[1], 0]
        elif cmd[0] == "r":
            if ignore_edge_detect:
                a = [CMD_R, cmd[1], 1]
            else:
                a = [CMD_R, cmd[1], 0]

        return bytes(a)

    def encode_motor_command(self, cmd, ignore_edge_detect=False, ignore_edge_detect_x=False, ignore_edge_detect_y=False):
        if X_COORD_IS_MOTOR_1:
            next_m1_state, next_m1_vel, next_m2_state, next_m2_vel = cmd
            ignore_edge_detect_m1 = ignore_edge_detect_x
            ignore_edge_detect_m2 = ignore_edge_detect_y
        else:
            next_m2_state, next_m2_vel, next_m1_state, next_m1_vel = cmd
            ignore_edge_detect_m1 = ignore_edge_detect_y
            ignore_edge_detect_m2 = ignore_edge_detect_x

        if next_m1_state == MSTATE_BRAKE:
            m1_cmd = [CMD_B1]
        elif next_m1_state == MSTATE_FLOAT:
            m1_cmd = [CMD_F1]
        elif next_m1_state == MSTATE_FORWARD:
            if ignore_edge_detect or ignore_edge_detect_m1:
                m1_cmd = [CMD_G1, next_m1_vel, 1]
            else:
                m1_cmd = [CMD_G1, next_m1_vel, 0]
        elif next_m1_state == MSTATE_REVERSE:
            if ignore_edge_detect or ignore_edge_detect_m1:
                m1_cmd = [CMD_R1, next_m1_vel, 1]
            else:
                m1_cmd = [CMD_R1, next_m1_vel, 0]

        if next_m2_state == MSTATE_BRAKE:
            m2_cmd = [CMD_B2]
        elif next_m2_state == MSTATE_FLOAT:
            m2_cmd = [CMD_F2]
        elif next_m2_state == MSTATE_FORWARD:
            if ignore_edge_detect or ignore_edge_detect_m2:
                m2_cmd = [CMD_G2, next_m2_vel, 1]
            else:
                m2_cmd = [CMD_G2, next_m2_vel, 0]
        elif next_m2_state == MSTATE_REVERSE:
            if ignore_edge_detect or ignore_edge_detect_m2:
                m2_cmd = [CMD_R2, next_m2_vel, 1]
            else:
                m2_cmd = [CMD_R2, next_m2_vel, 0]

        return [bytes(m1_cmd), bytes(m2_cmd)]


motorcoms = MotorSerialCommunicator()
mcon = MotorController()


def recv_confirm(return_messages=False):
    ret_msgs = []
    if motorcoms.EXPECT_CONFIRM_MSG:
        print("[arduino]: ", end="")
        msgb = ser.readline()
        # print(msgb)
        msg = msgb.decode("ascii")
        if msg[-1] == "\n":
            msg = msg[0:-1]
        elif msg[-2:] == "\n\r" or msg[-2:] == "\r\n":
            msg = msg[0:-2]
        print(msg)
        if return_messages:
            ret_msgs.append(msg)
        if "forward" in msg or "backward" in msg or "unknown" in msg:
            num_followups = 1
        elif "Resetting" in msg:
            num_followups = 4
        elif "Ranges:" in msg:
            num_followups = 2
        else:
            num_followups = 0

        for _ in range(num_followups):
            msg2 = ser.readline().decode("ascii")
            if msg2[-1] == "\n":
                msg2 = msg2[0:-1]
            elif msg2[-2:] == "\n\r" or msg2[-2:] == "\r\n":
                msg2 = msg2[0:-2]
            print("[\t", msg2)
            if return_messages:
                ret_msgs.append(msg2)

        if "Cannot move" in msg:
            raise Exception("unimplemented, need to say from arduino which wall was hit")
            if return_messages:
                return [1]
            return 1

    if return_messages:
        return ret_msgs
    return 0


def getPosition():
    ser.write(bytes([CMD_READY_FOR_POS]))
    recv_confirm()
    inp = ser.read(9)
    return motorcoms.parse_input(inp)


TVID_XMIN = 270
TVID_XMAX = 1050
TVID_YMIN = 100
TVID_YMAX = 850

T2M_XM = 0
T2M_XB = 0
T2M_YM = 0
T2M_YB = 0


def initialize_t2m_constants():
    global T2M_XM, T2M_XB, T2M_YM, T2M_YB
    T2M_XM = motor_range_x / (TVID_XMAX - TVID_XMIN)
    T2M_XB = -TVID_XMIN * T2M_XM
    T2M_YM = motor_range_y / (TVID_YMAX - TVID_YMIN)
    T2M_YB = -TVID_YMIN * T2M_YM


def transform_to_motor_coords(xgoal, ygoal):
    return (xgoal * T2M_XM + T2M_XB, ygoal * T2M_YM + T2M_YB)


def run_tracking_function(tclient):
    is_tracking = True
    TRACKING_CANCEL_PROMPT_DURATION = 1.0 / float(TRACKING_MOVE_COMMAND_FREQ)

    initialize_t2m_constants()

    override_goal_x = False
    override_goal_y = False
    ignore_edge_detect_x = False
    ignore_edge_detect_y = False

    Y_RECOVER_GOAL_1 = 10
    Y_RECOVER_GOAL_2 = motor_range_y - 10
    X_RECOVER_GOAL_1 = 10
    X_RECOVER_GOAL_2 = motor_range_x - 10
    manual_goal_x = 0
    manual_goal_y = 0

    while is_tracking:
        # get motor position from arduino
        goodform, xpos, ypos = getPosition()
        if not goodform:
            print("Couldn't parse motor position from arduino, ENDING TRACKING")
            is_tracking = False
            break

        # get goal (rat) position from trodes
        ts, xgoal, ygoal = tclient.getPosition()
        if ts == 0:
            print("Blank input from Trodes")
            if END_ON_BLANK:
                is_tracking = False
                break
        else:
            xgoal_m, ygoal_m = transform_to_motor_coords(xgoal, ygoal)
            if override_goal_x:
                xgoal_m = manual_goal_x
            if override_goal_y:
                ygoal_m = manual_goal_y
            if VERBOSITY >= 1:
                print("({}, {}) ==> ({}, {})".format(xgoal, ygoal, xgoal_m, ygoal_m))

        # set motor goal and position
        mcon.update_position(xpos, ypos)
        mcon.set_goal(xgoal_m, ygoal_m)

        # get motor command
        cmd = mcon.get_motor_command()
        output = motorcoms.encode_motor_command(
            cmd, ignore_edge_detect_x=ignore_edge_detect_x, ignore_edge_detect_y=ignore_edge_detect_y)
        if cmd[1] == 0 and cmd[3] == 0:
            mcon.motor_is_stopped()

        # send motor command
        move_recv = 0
        for op in output:
            ser.write(op)
            move_recv = recv_confirm()

            # check if reached end zone. If so, set goal to back outside motor zone, next loop instead of rat location, set ignore flag to true
            if move_recv > 0:
                print("Edge {} detected by arduino".format(move_recv))
                Exception("depending on the edge, back out")
                override_goal_y = True
                ignore_edge_detect_y = True
                manual_goal_y = Y_RECOVER_GOAL_1

        if override_goal_y and ypos > Y_RECOVER_GOAL_1 and ypos < Y_RECOVER_GOAL_2:
            override_goal_y = False
            ignore_edge_detect_y = False

        if override_goal_x and xpos > X_RECOVER_GOAL_1 and xpos < X_RECOVER_GOAL_2:
            override_goal_x = False
            ignore_edge_detect_x = False

        # pause (roughly camera rate or slower) and allow for cancelling input
        cancel_ret_val = allow_cancel_prompt(TRACKING_CANCEL_PROMPT_DURATION)
        if cancel_ret_val == 1:
            # user wants to cancel
            ser.write(bytes([CMD_F1]))
            recv_confirm()
            ser.write(bytes([CMD_F2]))
            recv_confirm()
            mcon.motor_is_stopped()
            print("Cancelling tracking")
            is_tracking = False


running = True
first_iter = True
while running:
    if not first_iter:
        goodform, xpos, ypos = getPosition()
        print("got position {}, {}\n".format(xpos, ypos))
    else:
        first_iter = False

    new_inpcom = input("input command:")
    if new_inpcom != ".":
        inpcom = new_inpcom
        repeat_last = False
    else:
        repeat_last = True

    ignore_edge_detect = False

    if len(inpcom) == 0:
        continue
    elif len(inpcom) == 1:
        if not (inpcom in ["q", "h", "p", "l", "m", "a", "c", "t"]):
            print("need motor number")
            continue
        com = inpcom[0]
    elif len(inpcom) == 2:
        try:
            com = inpcom[0]
            motor_num = int(inpcom[1])
        except:
            print("couldn't parse command")
            continue
    elif len(inpcom) == 3:
        if inpcom[0:2] == "gg" or inpcom[0:2] == "rr":
            ignore_edge_detect = True
        else:
            print("invalid command")
            continue

        try:
            com = inpcom[0]
            motor_num = int(inpcom[2])
        except:
            print("couldn't parse command")
            continue
    else:
        print("invalid command")
        continue

    if com == "q":
        running = False
        break
    elif com == "g":
        # go forward
        if not repeat_last:
            delay = float(input("delay before move (secs):"))
            dur = float(input("duration (secs):"))
            speed = int(input("speed (0-255):"))
        cmd = (com, speed)
        output = motorcoms.encode_manual_motor_command(
            cmd, motor_num, ignore_edge_detect=ignore_edge_detect)
        sleep(delay)
        ser.write(output)
        recv_confirm()
        timed_move(dur, motor_num)
    elif com == "r":
        # reverse
        if not repeat_last:
            delay = float(input("delay before move (secs):"))
            dur = float(input("duration (secs):"))
            speed = int(input("speed (0-255):"))
        cmd = (com, speed)
        output = motorcoms.encode_manual_motor_command(
            cmd, motor_num, ignore_edge_detect=ignore_edge_detect)
        sleep(delay)
        ser.write(output)
        recv_confirm()
        timed_move(dur, motor_num)
    elif com == "b":
        # brakes
        cmd = (com, )
        output = motorcoms.encode_manual_motor_command(
            cmd, motor_num, ignore_edge_detect=ignore_edge_detect)
        ser.write(output)
        recv_confirm()
    elif com == "f":
        # float
        cmd = (com, )
        output = motorcoms.encode_manual_motor_command(
            cmd, motor_num, ignore_edge_detect=ignore_edge_detect)
        ser.write(output)
        recv_confirm()
    elif com == "h":
        print("g - go forward\nr - go in reverse\nb - brake\nf - float\np - get position\nm - move to\nt - trodes tracking\nh - help\nq - quit")
        continue
    elif com == "p":
        pass
    elif com == "m":
        goodform, xpos, ypos = getPosition()
        if not goodform:
            print("Couldn't get position")
            continue
        if not repeat_last:
            ygoal = int(
                input("current position: {}\nGoal location:".format(ypos)))
        mcon.set_goal(0, ygoal)

        is_moving = True
        while is_moving:
            mcon.update_position(0, ypos)
            cmd = mcon.get_motor_command()
            output = motorcoms.encode_motor_command(
                cmd, ignore_edge_detect=ignore_edge_detect)
            if cmd[1] == 0 and cmd[3] == 0:
                is_moving = False
            move_recv = 0
            for op in output:
                ser.write(op)
                move_recv = recv_confirm()
                if move_recv == 1:
                    print("Edge detected by arduino")
                    break
            if move_recv == 1:
                break

            goodform, xpos, ypos = getPosition()
            print("{}, {}".format(xpos, ypos))
            if not goodform:
                print("bad position input")
                # CMD_END_MV1 = CMD_B1
                # CMD_END_MV2 = CMD_B2
                CMD_END_MV1 = CMD_F1
                CMD_END_MV2 = CMD_F2
                # ser.write(CMD_END_MV1)
                ser.write(CMD_END_MV2)
                # recv_confirm()
                recv_confirm()
                break

        mcon.motor_is_stopped()

    elif com == "l":
        ser.write(bytes([CMD_RESET_EDGE_DETECTOR]))
        recv_confirm()
    elif com == "c":
        ser.write(bytes([CMD_CALIBRATE_HOME]))
        recv_confirm()
    elif com == "a":
        ser.write(bytes([CMD_QUERY_RANGES]))
        calib_msgs = recv_confirm(return_messages=True)
        motor_range_x = int(calib_msgs[1])
        motor_range_y = int(calib_msgs[2])
        motor_range_calibrated = True

    elif com == "t":
        if not motor_range_calibrated:
            print("Please calibrate the motor first, then run query ranges (a)")
            continue
        # t for track rat, or trodes input, or time to follow the rat
        if tclient is None:
            # first time setup
            tclient = SGClient("pulleycon")
            tclient.subscribeToPosition()

        if tclient is None:
            print("Not connected to Trodes")
            continue

        run_tracking_function(tclient)

    else:
        print("unknown command {}".format(com))
        continue

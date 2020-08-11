import serial
from spikegadgets import trodesnetwork as tn
import logging

BAUDRATE = 115200
DEFAULT_PORT = '/dev/ttyACM0'
FREEZE_COMMAND = b'F'
HOME_COMMAND = b'H'
COORDS_COMMAND = b'C'


class CoordinateSender(serial.Serial):
    """
    class to communicate with arduino to controller pulleys
    """

    def __init__(self, port=DEFAULT_PORT, baud=BAUDRATE):
        self._is_enabled = False
        serial.Serial.__init__(self, port, baud, timeout=0,
                               xonxoff=False, rtscts=False, dsrdtr=False)

    def getStatus(self):
        return self._is_enabled

    def enable(self):
        """
        Enable the serial port (remove pin values from defaults)
        """
        self._is_enabled = True

    def disable(self):
        """
        Disable serial port (allow pin values to be changed by outside input).
        """
        self._is_enabled = False

    def freezeMotor(self):
        if self._is_enabled:
            self.write(FREEZE_COMMAND)
        else:
            print("Serial port not enabled")

    def returnToHome(self):
        if self._is_enabled:
            self.write(HOME_COMMAND)
        else:
            print("Serial port not enabled")

    def sendCoordinates(self, xcoord, ycoord):
        if self._is_enabled:
            self.write(COORDS_COMMAND)
            self.write(xcoord)
            self.write(ycoord)
        else:
            print("Serial port not enabled")


class SGClient(tn.AbstractModuleClient):
    """
    Extension of SpikeGadgets client for communicating with Trodes which is
    either recording data in real time or playing back a pre-recorded session.

    William Croughan
    2019/02/11
    """

    timestamps = []

    def __init__(self, name, connection="tcp://127.0.0.1", port=49152):
        """
        Pass on some descriptors to the parent class
        """

        # Call the parent class constructor
        tn.AbstractModuleClient.__init__(self, name, connection, port)
        if (self.initialize() != 0):
            self.recv_quit()
            self.closeConnections()
            error_message = "Could not connect to Trodes!"
            logging.info(MODULE_IDENTIFIER + error_message)
            raise Exception(error_message)
        else:
            logging.info(MODULE_IDENTIFIER + "Initialized connection to Trodes.")


def main():
    # connect to motors
    motor = CoordinateSender()
    motor.enable()
    motor.returnToHome()

    # connect to trodes
    sg_client = SGClient("PulleyController")
    position_consumer = sg_client.subscribeHighFreqData("PositionData", "CameraModule")
    if (position_consumer is None):
        # Failed to open connection to camera module
        logging.warning("Failed to open Camera Module")
        raise Exception("Error: Could not connect to camera, aborting.")
    position_consumer.initialize()

    while True:
        n_available_frames = position_consumer.available(0)
        if n_available_frames == 0:
            down_time += 0.02
            time.sleep(0.02)
            if down_time > 1.0:
                down_time = 0.0
                print(MODULE_IDENTIFIER + "Warning: Not receiving position data.")
        else:
            down_time = 0.0

            position_consumer.readData(data_field)
            px = data_field['position_x']
            py = data_field['position_y']

            motor.sendCoordinates(px, py)


if __name__ == "__main__":
    main()

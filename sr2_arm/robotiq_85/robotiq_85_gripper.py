# Copyright (c) 2022 Walking Machine
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

__author__ = "Walking Machine"
__copyright__ = "Copyright (C) 2022 Walking Machine"
__license__ = "MIT"
__version__ = "1.0"

import serial

from sr2_arm.robotiq_85.robotiq_source_files.gripper_io import GripperIO
from sr2_arm.robotiq_85.robotiq_source_files.modbus_crc import verify_modbus_rtu_crc
from sr2_arm.robotiq_85.robotiq_gripper_interface import RobotiqGripperInterface


class Robotiq85Gripper(RobotiqGripperInterface):

    def __init__(self, num_grippers: int = 1, comport: str = '/dev/ttyUSB0', baud: int = 115200):
        super().__init__()
        try:
            self.ser = serial.Serial(comport, baud, timeout=0.2)
        except OSError:
            self._init_success = False
            return

        self._gripper = []
        self._num_grippers = num_grippers
        for i in range(self._num_grippers):
            self._gripper.append(GripperIO(i))
        self._init_success = True
        self._shutdown_driver = False

    def shutdown(self):
        self._shutdown_driver = True
        self.ser.close()

    def process_act_cmd(self, dev=0) -> bool:
        if (dev >= self._num_grippers) or self._shutdown_driver:
            return False
        try:
            self.ser.write(self._gripper[dev].act_cmd_bytes)
            rsp = self.ser.read(8)
            rsp = [x for x in rsp]
            if len(rsp) != 8:
                return False
            return verify_modbus_rtu_crc(rsp)
        except OSError:
            return False

    def process_stat_cmd(self, dev=0) -> bool:
        try:
            self.ser.write(self._gripper[dev].stat_cmd_bytes)
            rsp = self.ser.read(21)
            rsp = [x for x in rsp]
            if len(rsp) != 21:
                return False
            return self._gripper[dev].parse_rsp(rsp)
        except OSError:
            return False

    def activate_gripper(self, dev=0) -> bool:
        if dev >= self._num_grippers:
            return False
        self._gripper[dev].activate_gripper()
        return True

    def deactivate_gripper(self, dev=0) -> bool:
        if dev >= self._num_grippers:
            return False
        self._gripper[dev].deactivate_gripper()
        return True

    def activate_emergency_release(self, dev=0, open_gripper=True):
        if dev >= self._num_grippers:
            return
        self._gripper[dev].activate_emergency_release(open_gripper)

    def deactivate_emergency_release(self, dev=0):
        if dev >= self._num_grippers:
            return
        self._gripper[dev].deactivate_emergency_release()

    def goto(self, dev=0, pos=0.0, vel=1.0, force=1.0):
        if dev >= self._num_grippers:
            return
        self._gripper[dev].goto(pos, vel, force)

    def stop(self, dev=0):
        if dev >= self._num_grippers:
            return
        self._gripper[dev].stop()

    def is_ready(self, dev=0):
        if dev >= self._num_grippers:
            return False
        return self._gripper[dev].is_ready()

    def is_reset(self, dev=0):
        if dev >= self._num_grippers:
            return False
        return self._gripper[dev].is_reset()

    def is_moving(self, dev=0):
        if dev >= self._num_grippers:
            return False
        return self._gripper[dev].is_moving()

    def object_detected(self, dev=0):
        if dev >= self._num_grippers:
            return False
        return self._gripper[dev].object_detected()

    def get_fault_status(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        return self._gripper[dev].get_fault_status()

    def get_pos(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        return self._gripper[dev].get_pos()

    def get_req_pos(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        return self._gripper[dev].get_req_pos()

    def get_current(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        return self._gripper[dev].get_current()

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

from abc import ABC, abstractmethod
import os

class RobotiqGripperInterface(ABC):

    def __init__(self):
        if os.geteuid() != 0:
            exit("You need to be ROOT to run this...")
        self._init_success = True

    @property
    def init_success(self) -> bool:
        return self._init_success

    @abstractmethod
    def shutdown(self) -> bool:
        """
        Close all active connections.

        :return: True if successful, else False
        """
        pass

    @abstractmethod
    def process_act_cmd(self, dev: int = 0) -> bool:
        """
        Send act command to gripper.

        :param dev: Index of device, default 0
        :return:   True if response is valid else False
        """
        pass

    @abstractmethod
    def process_stat_cmd(self, dev: int = 0) -> bool:
        """
        Send stat command to gripper.

        :param dev: Index of the device, defaults to 0
        :return: True if communication is a success, else False
        """
        pass

    @abstractmethod
    def activate_gripper(self, dev: int = 0) -> bool:
        """
        Activate the gripper.

        :param dev: Index of the device, defaults to 0
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def deactivate_gripper(self, dev: int = 0) -> bool:
        """
        Deactivate Gripper.

        :param dev: Index of device, defaults to 0
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def activate_emergency_release(self, dev: int = 0, gripper_open: bool = True) -> bool:
        """
        Emergency release, open gripper and lock.

        :param dev: Index of device, defaults to 0
        :param gripper_open: Emergency state is open, defaults to True
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def deactivate_emergency_release(self, dev: int = 0) -> bool:
        """
        Deactivate emergency release.

        :param dev: Index of device, defaults to 0
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def goto(self, dev: int = 0, pos: float = 0.0, vel: float = 0.1, force: float = 5.0) -> bool:
        """
        Send position to Gripper.

        :param dev: Index of device, defaults to 0
        :param pos: Wanted position, defaults to 0.0
        :param vel: Speed of movement, defaults to 0.1
        :param force: Force of movement, defaults to 5.0
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def stop(self, dev: int = 0) -> bool:
        """
        Stop last command.

        :param dev: Index of device, defaults to 0
        :return: True if successful else False
        """
        pass

    @abstractmethod
    def is_ready(self, dev: int = 0) -> bool:
        """
        If the last status of the gripper is ready.

        :param dev: Index of device, defaults to 0
        :return: True if ready else False
        """
        pass

    @abstractmethod
    def is_reset(self, dev: int = 0) -> bool:
        """
        Determine if the gripper is in release mode (see faults for reason).

        :param dev: Index of device, defaults to 0
        :return: True if in reset mode else False
        """
        pass

    @abstractmethod
    def is_moving(self, dev: int = 0) -> bool:
        """
        Determine if the gripper is moving.

        :param dev: Index of device, defaults to 0
        :return: True of moving else False
        """
        pass

    @abstractmethod
    def object_detected(self, dev: int = 0) -> bool:
        """
        Determine of there was an object detected when closing.

        :param dev: Index of device, defaults to 0
        :return: True of object detected else false
        """
        pass

    @abstractmethod
    def get_fault_status(self, dev: int = 0) -> int:
        """
        If in reset, this will return the fault status.

        :param dev: Index of device, defaults to 0
        :return: 0 if no fault else a fault number
        """
        pass

    @abstractmethod
    def get_pos(self, dev: int = 0) -> float:
        """
        Get the current position of the gripper.

        :param dev: Index of device, defaults to 0
        :return: The position of the gripper
        """
        pass

    @abstractmethod
    def get_req_pos(self, dev: int = 0) -> float:
        """
        Get the last requested position sent to the gripper.

        :param dev: Index of device, defaults to 0
        :return: Requested position of gripper
        """
        pass

    @abstractmethod
    def get_current(self, dev: int = 0) -> float:
        """
        Get the current sent to the gripper.

        :param dev: Index of device, defaults to 0
        :return: Current sent to the gripper
        """
        pass

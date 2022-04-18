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

__author__ = "WalkingMachine"
__copyright__ = "Copyright (C) 2022 Walking Machine"
__license__ = "MIT"
__version__ = "1.0"


import os
from dataclasses import dataclass
from threading import Thread, Lock
from time import sleep
from functools import partial

from sr2_arm.robotiq_85.robotiq_gripper_interface import RobotiqGripperInterface


class RobotiqTestGripper(RobotiqGripperInterface):
    """
    This is a test class that replicates a connection to the gripper.

    :param num_grippers: Number of grippers connected, defaults to 1
    """

    _POS_MIN: float = 0.0
    _POS_MAX: float = 0.085
    _VEL_MIN: float = 0.013
    _VEL_MAX: float = 0.1
    _FORCE_MIN: float = 5.0
    _FORCE_MAX: float = 220.0

    def __init__(self, num_grippers: int = 1, *args) -> None:
        """Initialize the Test Gripper."""
        super().__init__()
        if os.geteuid() != 0:
            exit("You need to be ROOT to run this...")

        self._grippers = []
        self._local_grippers = []
        self._num_grippers = num_grippers
        self._running_threads = {}
        for i in range(self._num_grippers):
            self._grippers.append(TestGripperIO(index=i))
            self._local_grippers.append(TestGripperIO(index=i))
            self._running_threads[i] = []

        self._init_success = True
        self._is_connected = True
        self._emergency_release = False
        self._mutex = Lock()

    def shutdown(self) -> bool:
        self._is_connected = False
        return True

    def process_act_cmd(self, dev: int = 0) -> bool:
        return self._is_connected

    def process_stat_cmd(self, dev: int = 0) -> bool:
        with self._mutex:
            if self._is_connected:
                local_device = self._local_grippers[dev]
                device = self._grippers[dev]
                device.emergency_release = local_device.emergency_release
                device.is_active = local_device.is_active
                device.req_pos = local_device.req_pos
                device.is_moving = local_device.is_moving
                device.vel = local_device.vel
                device.force = local_device.force

                # check for dead threads
                for i in self._running_threads[dev]:
                    if not i.is_alive():
                        self._running_threads[dev].remove(i)

                # TODO Check if emergency release works even if the gripper is not active (Nimai)
                if device.emergency_release and device.pos != device.req_pos:
                    t = Thread(target=partial(
                        self._move_gripper_worker, device), daemon=True)
                    t.start()
                    self._running_threads[dev].append(t)
                elif device.is_active and not device.emergency_release:
                    # if we need to move
                    if device.pos != device.req_pos and not device.is_moving:
                        t = Thread(target=partial(
                            self._move_gripper_worker, device), daemon=True)
                        t.start()
                        self._running_threads[dev].append(t)

                local_device.pos = device.pos
                local_device.vel = device.vel
                local_device.force = device.force
                local_device.is_moving = device.is_moving

                return True
            return False

    def activate_gripper(self, dev: int = 0) -> bool:
        with self._mutex:
            self._local_grippers[dev].is_active = True
            return True

    def deactivate_gripper(self, dev: int = 0) -> bool:
        with self._mutex:
            self._local_grippers[dev].is_active = False
            return True

    def activate_emergency_release(self, dev: int = 0, gripper_open: bool = True) -> bool:
        with self._mutex:
            self._local_grippers[dev].req_pos = self._POS_MAX if gripper_open else self._POS_MIN
            self._local_grippers[dev].emergency_release = True
            return True

    def deactivate_emergency_release(self, dev: int = 0) -> bool:
        with self._mutex:
            self._local_grippers[dev].emergency_release = False
            return True

    def goto(self, dev: int = 0, pos: float = 0.0, vel: float = 0.1, force: float = 5.0) -> True:
        with self._mutex:
            self._local_grippers[dev].req_pos = pos
            self._local_grippers[dev].vel = vel
            self._local_grippers[dev].force = force
            return True

    def stop(self, dev: int = 0) -> True:
        with self._mutex:
            self._local_grippers[dev].is_moving = False
            return True

    def is_ready(self, dev: int = 0) -> bool:
        return self._grippers[dev].is_active

    def is_reset(self, dev: int = 0) -> bool:
        return self._grippers[dev].emergency_release

    def is_moving(self, dev: int = 0) -> bool:
        return self._grippers[dev].is_moving

    def object_detected(self, dev: int = 0) -> bool:
        """"""
        return False

    def get_fault_status(self, dev: int = 0) -> int:
        return 0

    def get_pos(self, dev: int = 0) -> float:
        return self._grippers[dev].pos

    def get_req_pos(self, dev: int = 0) -> float:
        return self._grippers[dev].req_pos

    def get_current(self, dev: int = 0) -> float:
        return 17.5

    @staticmethod
    def _move_gripper_worker(gripper: "TestGripperIO"):
        gripper.is_moving = True
        while gripper.req_pos != gripper.pos and gripper.is_moving:
            if gripper.req_pos > gripper.pos:
                gripper.pos += 0.005
            else:
                gripper.pos -= 0.005
            sleep(0.01)
        gripper.is_moving = False
        gripper.vel = 0.0
        gripper.force = 0.0


@dataclass
class TestGripperIO:
    index: int
    pos: float = 0.0
    req_pos: float = 0.0
    is_active: bool = True
    emergency_release: bool = False
    is_moving: bool = False
    object_detected: bool = False
    fault_status: int = 0
    vel: float = 0.0
    force: float = 0.0

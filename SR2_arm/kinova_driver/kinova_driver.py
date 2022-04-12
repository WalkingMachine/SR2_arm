from ctypes import *
import logging
import os
from typing import List

from kinovaCommInterface import KinovaDevice

try:
    from kinovaUsbInterface import InitAPI, CloseAPI, GetAngluarForce, GetAngularPosition, GetDevices, SendAdvanceTrajectory, GetSensorsInfo
    from kinovaCommInterface import MAX_KINOVA_DEVICE, ERROR_NO_DEVICE_FOUND
    from kinova_types import AngularPosition, TrajectoryPoint, SensorsInfo, UserPosition, POSITION_TYPE
except Exception as e:
    print("Shared libraries not found or error")


class KinovaDriver:
    _logger = logging.getLogger("KinovaDriver")

    _MAX_SPEED = 40.0

    def __init__(self) -> None:
        if os.geteuid() != 0:
            self._logger.error("You need to have root privileges to run this script.\nPlease try again, this time using 'sudo'.")
            return

        self._initialized = False

        self._logger.info("Kinova Arm initialized.")

        # Mapping for joints 0 - 5
        self._joint_index = 0

    def __del__(self) -> None:
        CloseAPI()
    
    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def joint_index(self) -> int:
        return self._joint_index

    def _check_device(self) -> int:
        devices = (KinovaDevice * MAX_KINOVA_DEVICE)()
        result = c_int()

        GetDevices(devices=devices, result=result)
        res = result.value
        if res != 1:
            self._logger.warning(f"Error Getting devices: {result}")
            return res

        if devices[0].DeviceType != -1:
            self._logger.info(f"Device found!")
            return res
        
        self._logger.error(f"Device not found")
        return ERROR_NO_DEVICE_FOUND

    def _build_trajectory(self, speed: float) -> TrajectoryPoint:
        trajectory_point = TrajectoryPoint(Position=UserPosition(Type=POSITION_TYPE.ANGULAR_VELOCITY.value))
        if self._joint_index == 0:
            trajectory_point.Position.Actuators.Actuator1 = speed
        else:
            trajectory_point.Position.Actuators.Actuator1 = 0.0

        if self._joint_index == 1:
            trajectory_point.Position.Actuators.Actuator2 = speed
        else:
            trajectory_point.Position.Actuators.Actuator2 = 0.0

        if self._joint_index == 2:
            trajectory_point.Position.Actuators.Actuator3 = speed
        else:
            trajectory_point.Position.Actuators.Actuator3 = 0.0

        if self._joint_index == 3:
            trajectory_point.Position.Actuators.Actuator4 = speed
        else:
            trajectory_point.Position.Actuators.Actuator4 = 0.0

        if self._joint_index == 4:
            trajectory_point.Position.Actuators.Actuator5 = speed
        else:
            trajectory_point.Position.Actuators.Actuator5 = 0.0

        return trajectory_point

    def initialize(self) -> int:
        res = InitAPI()
        if res == 1:
            self._initialized = True
            return res * self._check_device()
        else:
            self._logger.warning("Initialization Failed")
            return res

    def joint_up(self) -> None:
        self._joint_index += 1
        if self.joint_index > 4:
            self._joint_index = 0
        return

    def joint_down(self) -> None:
        self._joint_index -= 1
        if self._joint_index < 0:
            self._joint_index = 4
        return

    def set_joint(self, joint) -> bool:
        if 0 <= joint < 5:
            self._joint_index = joint
            return True
        return False

    def get_forces(self) -> List[float]:
        force_list = AngularPosition()
        if GetAngluarForce(force_list) != 1:
            self._logger.warning("Failed to get Forces")
            return [0.0]
        values = []
        values.append(force_list.Actuators.Actuator1)
        values.append(force_list.Actuators.Actuator2)
        values.append(force_list.Actuators.Actuator3)
        values.append(force_list.Actuators.Actuator4)
        values.append(force_list.Actuators.Actuator5)
        return values

    def get_positions(self) -> List[float]:
        pos_list = AngularPosition()
        if GetAngularPosition(pos_list) != 1:
            self._logger.warning("Failed to get Position")
            return [0.0]
        values = []
        values.append(pos_list.Actuators.Actuator1)
        values.append(pos_list.Actuators.Actuator2)
        values.append(pos_list.Actuators.Actuator3)
        values.append(pos_list.Actuators.Actuator4)
        values.append(pos_list.Actuators.Actuator5)
        return values

    def get_current(self) -> float:
        response = SensorsInfo()
        if GetSensorsInfo(response) == 1:
            return response.Current
        else:
            self._logger.warning("Could not get Current")
            return -1.0

    def get_voltage(self) -> float:
        response = SensorsInfo()
        if GetSensorsInfo(response) == 1:
            return response.Voltage
        else:
            self._logger.warning("Could not get Voltage")
            return -1.0

    def get_actuator_temps(self) -> List[float]:
        response = SensorsInfo()
        if GetSensorsInfo(response) == 1:
            return [
                response.ActuatorTemp1,
                response.ActuatorTemp2,
                response.ActuatorTemp3,
                response.ActuatorTemp4,
                response.ActuatorTemp5,
            ]
        else:
            self._logger.warning("Could not get Actuator Temperatures")
            return [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]

    def send_cmd_val(self, speed: float) -> bool:
        if speed > self._MAX_SPEED:
            speed = self._MAX_SPEED
        elif speed < -self._MAX_SPEED:
            speed = -self._MAX_SPEED
        
        trajectory_point = self._build_trajectory(speed)

        return SendAdvanceTrajectory(trajectory_point) == 1

    def send_estop(self) -> bool:
        trajectory_point = TrajectoryPoint(Position=UserPosition(Type=POSITION_TYPE.ANGULAR_VELOCITY.value))
        return SendAdvanceTrajectory(trajectory_point) == 1

    def shutdown(self) -> bool:
        if CloseAPI() == 1:
            self._initialized = False
            return True
        return False
        
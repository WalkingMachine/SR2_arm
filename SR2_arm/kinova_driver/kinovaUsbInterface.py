from ctypes import POINTER, CDLL, byref, c_int
from typing import List

from kinovaCommInterface import MAX_KINOVA_DEVICE, KinovaDevice
from kinova_types import AngularInfo, AngularPosition, CartesianPosition, SensorsInfo, TrajectoryFIFO, TrajectoryPoint

_usb_lib = CDLL("./Kinova.API.USBCommandLayerUbuntu.so")

# region Error Codes
"""ERROR CODES"""
ERROR_INIT_API = 2001  # Error while initializing the API
ERROR_LOAD_COMM_DLL = 2002  # Error while loading the communication layer

# Those 3 codes are mostly for internal use
JACO_NACK_FIRST = 2003
JACO_COMM_FAILED = 2004
JACO_NACK_NORMAL = 2005

# Unable to initialize the communication layer.
ERROR_INIT_COMM_METHOD = 2006

# Unable to load the Close() function from the communication layer.
ERROR_CLOSE_METHOD = 2007

# Unable to load the GetDeviceCount() function from the communication layer.
ERROR_GET_DEVICE_COUNT_METHOD = 2008

# Unable to load the SendPacket() function from the communication layer.
ERROR_SEND_PACKET_METHOD = 2009

# Unable to load the SetActiveDevice() function from the communication layer.
ERROR_SET_ACTIVE_DEVICE_METHOD = 2010

# Unable to load the GetDeviceList() function from the communication layer.
ERROR_GET_DEVICES_LIST_METHOD = 2011

# Unable to initialized the system semaphore.
ERROR_SEMAPHORE_FAILED = 2012

# Unable to load the ScanForNewDevice() function from the communication layer.
ERROR_SCAN_FOR_NEW_DEVICE = 2013

# Unable to load the GetActiveDevice function from the communication layer.
ERROR_GET_ACTIVE_DEVICE_METHOD = 2014

# A function's parameter is not valid.
ERROR_INVALID_PARAM = 2100

# The API is not initialized.
ERROR_API_NOT_INITIALIZED = 2101

# Unable to load the InitDataStructure() function from the communication layer.
ERROR_INIT_DATA_STRUCTURES_METHOD = 2102
# endregion

# This represents the size of an array containing Cartesian values.
CARTESIAN_SIZE = 6

# This represents the max actuator count in our context.
MAX_ACTUATORS = 6

# This represents the max actuator count in our context.
MAX_INVENTORY = 15

# This represents the size of the array returned by the function GetCodeVersion.
CODE_VERSION_COUNT = 37

# This represents the size of the array returned by the function GetAPIVersion.
API_VERSION_COUNT = 3

# This represents the size of the array returned by the function GetPositionCurrentActuators.
POSITION_CURRENT_COUNT = 12

# This represents the size of the array returned by the function GetSpasmFilterValues and sent to SetSpasmFilterValues.
SPASM_FILTER_COUNT = 1

# Version of the API 5.01.04
COMMAND_LAYER_VERSION = 50105

COMMAND_SIZE = 70

OPTIMAL_Z_PARAM_SIZE = 16

GRAVITY_VECTOR_SIZE = 3

GRAVITY_PARAM_SIZE = 42

GRAVITY_PAYLOAD_SIZE = 4

_usb_lib.GetDevices.argtypes = [KinovaDevice * MAX_KINOVA_DEVICE, POINTER(c_int)]
_usb_lib.GetDevices.restype = c_int


def GetDevices(devices: List[KinovaDevice], result: c_int) -> int:
    return _usb_lib.GetDevices(devices, byref(result))


_usb_lib.SetActiveDevice.argtypes = [KinovaDevice]
_usb_lib.SetActiveDevice.restype = c_int


def SetActiveDevice(device: KinovaDevice) -> int:
    return _usb_lib.SetActiveDevice(device)


_usb_lib.RefresDevicesList.restype = c_int


def RefresDevicesList() -> int:
    return _usb_lib.RefresDevicesList()


_usb_lib.InitAPI.restype = c_int


def InitAPI() -> int:
    return _usb_lib.InitAPI()


_usb_lib.CloseAPI.restype = c_int


def CloseAPI() -> int:
    return _usb_lib.CloseAPI()


_usb_lib.GetCodeVersion.argtypes = [c_int * CODE_VERSION_COUNT]
_usb_lib.GetCodeVersion.restype = c_int


def GetCodeVersion(Response: c_int * CODE_VERSION_COUNT) -> int:
    return _usb_lib.GetCodeVersion(Response)


_usb_lib.GetAPIVersion.argtypes = [c_int * API_VERSION_COUNT]
_usb_lib.GetAPIVersion.restype = c_int


def GetAPIVersion(Response: c_int * API_VERSION_COUNT) -> int:
    return _usb_lib.GetAPIVersion(Response)


_usb_lib.GetCartesianPosition.argtypes = [CartesianPosition]
_usb_lib.GetCartesianPosition.restype = c_int


def GetCartesianPosition(Response: CartesianPosition) -> int:
    return _usb_lib.GetCartesianPosition(byref(Response))


_usb_lib.GetAngularPosition.argtypes = [POINTER(AngularPosition)]
_usb_lib.GetAngularPosition.restype = c_int


def GetAngularPosition(Response: AngularPosition) -> int:
    return _usb_lib.GetAngularPosition(byref(Response))


_usb_lib.GetCartesianForce.argtypes = [POINTER(CartesianPosition)]
_usb_lib.GetCartesianForce.restype = c_int

def GetCartesianForce(Response: CartesianPosition) -> int:
    return _usb_lib.GetCartesianForce(byref(Response))


_usb_lib.GetAngularForce.argtypes = [POINTER(AngularPosition)]
_usb_lib.GetAngularForce.restype = c_int


def GetAngluarForce(Response: AngularPosition):
    return _usb_lib.GetAngularForce(byref(Response))


_usb_lib.GetAngularCurrent.argtypes = [POINTER(AngularPosition)]
_usb_lib.GetAngularCurrent.restype = c_int


def GetAngularCurrent(Response: AngularPosition):
    return _usb_lib.GetAngularCurrent(byref(Response))


_usb_lib.GetActualTrajectoryInfo.argtypes = [POINTER(TrajectoryPoint)]
_usb_lib.GetActualTrajectoryInfo.restype = c_int


def GetActualTrajectoryInfo(Response: TrajectoryPoint):
    return _usb_lib.GetActualTrajectoryInfo(byref(Response))


_usb_lib.GetGlobalTrajectoryInfo.argtypes = [POINTER(TrajectoryFIFO)]
_usb_lib.GetGlobalTrajectoryInfo.restype = c_int


def GetGlobalTrajectoryInfo(Response: TrajectoryFIFO):
    return _usb_lib.GetGlobalTrajectoryInfo(byref(Response))


_usb_lib.GetSensorsInfo.argtypes = [POINTER(SensorsInfo)]
_usb_lib.GetSensorsInfo.restype = c_int


def GetSensorsInfo(Response: SensorsInfo):
    return _usb_lib.GetSensorsInfo(byref(Response))


_usb_lib.StartControlAPI.restype = c_int


def StartControlAPI() -> c_int:
    return _usb_lib.StartControlAPI()


_usb_lib.StopControlAPI.restype = c_int


def StopControlAPI() -> int:
    return _usb_lib.StopControlAPI()


_usb_lib.SendAdvanceTrajectory.argtypes = [TrajectoryPoint]
_usb_lib.SendAdvanceTrajectory.restype = c_int


def SendAdvanceTrajectory(trajectory: TrajectoryPoint) -> int:
    return _usb_lib.SendAdvanceTrajectory(trajectory)

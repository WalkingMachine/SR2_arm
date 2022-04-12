from ctypes import Array, Structure, Union, byref, c_float, c_int, c_char, c_short, c_ubyte, c_uint, CDLL, POINTER

# region Error codes
""" ERROR CODES"""
NO_ERROR_KINOVA = 1

# We know that an error has occured but we don't know where it comes from.
UNKNOWN_ERROR = 666

# Unable to load the USB library.
ERROR_LOAD_USB_LIBRARY = 1001

# Unable to access the Open method from the USB library.
ERROR_OPEN_METHOD = 1002

# Unable to access the Write method from the USB library.
ERROR_WRITE_METHOD = 1003

# Unable to access the Read method from the USB library.
ERROR_READ_METHOD = 1004

# Unable to access the Read Int method from the USB library.
ERROR_READ_INT_METHOD = 1005

# Unable to access the Free Library method from the USB library.
ERROR_FREE_LIBRARY = 1006

# There is a problem with the USB connection between the device and the computer.
ERROR_JACO_CONNECTION = 1007

# Unable to claim the USB interface.
ERROR_CLAIM_INTERFACE = 1008

# Unknown type of device.
ERROR_UNKNOWN_DEVICE = 1009

# The functionality you are trying to use has not been initialized.
ERROR_NOT_INITIALIZED = 1010

# The USB library cannot find the device.
ERROR_LIBUSB_NO_DEVICE = 1011

# The USB Library is bussy and could not perform the action.
ERROR_LIBUSB_BUSY = 1012

# The functionality you are trying to perform is not supported by the version installed.
ERROR_LIBUSB_NOT_SUPPORTED = 1013

# Unknown error while sending a packet.
ERROR_SENDPACKET_UNKNOWN = 1014

# Cannot find the requested device.
ERROR_NO_DEVICE_FOUND = 1015

# The operation was not entirely completed :)
ERROR_OPERATION_INCOMPLETED = 1016

# Handle used is not valid.
ERROR_RS485_INVALID_HANDLE = 1017

# An overlapped I/O operation is in progress but has not completed.
ERROR_RS485_IO_PENDING = 1018

# Not enough memory to complete the opreation.
ERROR_RS485_NOT_ENOUGH_MEMORY = 1019

# The operation has timed out.
ERROR_RS485_TIMEOUT = 1020

# You are trying to call a USB function that is not available in the current context.
ERROR_FUNCTION_NOT_ACCESSIBLE = 1021

""" END OF ERROR CODES"""
# endregion

# region RS-485 Command Id List
""" RS-485 COMMAND ID LIST """
# All RS-485 commands refer to the RS-485 communication protocol documentation.

# This message report an error.
RS485_MSG_REPORT_ERROR = 0x30

# This message clear the error flag.
RS485_MSG_CLEAR_FAULT_FLAG = 0x33

# This message represents a NACK.
RS485_MSG_NACK = 0x3E

# This message represents a ACK.
RS485_MSG_ACK = 0x3F

# This message type is sent to initialise de device.
RS485_MSG_SET_ADDRESS = 0x00

# This message type is used to read position without sending a new command.
RS485_MSG_GET_ACTUALPOSITION = 0x01

# This message type sends back the position with Current, Speed and Torque.
RS485_MSG_SEND_ACTUALPOSITION = 0x02

# Start control of the motor.
RS485_MSG_STAR_ASSERV = 0x03

# Stop the control of the motor.
RS485_MSG_STOP_ASSERV = 0x04

# This message type sends the Feed through value to bypass the PID of the
# actuator. Value range is -1.0 to +1.0, and both DataLow and DataHigh must
# be same value for command to be accepted. When using this command, the
# gains Kp, Ki and Kd must be set to zero first.
RS485_MSG_FEEDTHROUGH = 0x09

# This message type sends the position command to the slave device.
RS485_MSG_GET_POSITION_COMMAND = 0x10

# This message contains the slave device actual position and current consumption.
RS485_MSG_SEND_POSITION_CURRENT = 0x11

# Send a new position command like the message 0x10 but it returns more insformation about the robot.
RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES = 0x14

# This message is an answer to the message 0x14.
RS485_MSG_SEND_ALL_VALUES_1 = 0x15

# This message is an answer to the message 0x14.
RS485_MSG_SEND_ALL_VALUES_2 = 0x16

# This message is an answer to the message 0x14.
RS485_MSG_SEND_ALL_VALUES_3 = 0x17

# This message contains the maximum and minimum position of the joint in degrees.
RS485_MSG_POSITION_MAX_MIN = 0x21

# This message contains the Kp gain of the system.
RS485_MSG_KP_GAIN = 0x24

# This message contains the Ki dans Kd gain of the system.
RS485_MSG_KI_KD_GAIN = 0x25

# This message tells the drive that its actual position is the position zero.
RS485_MSG_PROGRAM_JOINT_ZERO = 0x26

# This message requests the code version of the slave device.
RS485_MSG_GET_CODE_VERSION = 0x27

# This message contains the code version of the slave device.
RS485_MSG_SEND_CODE_VERSION = 0x28

# This message requests the slave device’s specific information.
RS485_MSG_GET_DEVICE_INFO = 0x29

# This message contains device’s specific information.
RS485_MSG_SEND_DEVICE_INFO = 0x2A

# This message get the temperature of the actuator.
RS485_MSG_GET_TEMPERATURE = 0x2E

# This message set the temperature of the actuator.
RS485_MSG_SET_TEMPERATURE = 0x2F

# This message set all the filters applied to the PID controller.
RS485_SET_PID_FILTERS = 0x40

# This message set the current position as the reference(180 degrees). WARNING: do not use if you are not familiar with the procedure.
RS485_SET_ZERO_TORQUESENSOR = 0x41

# This message set the gain applied on the torque sensor value. WARNING: do not use if you are not familiar with the procedure.
RS485_SET_GAIN_TORQUESENSOR = 0x42

# Activate or Deactivate the control with encoder.
RS485_SET_CONTROL_WITH_ENCODER = 0x43

# This message returns the encoder's statuses.
RS485_GET_ENCODER_STATUSSES = 0x44

# This message set the advanced PID parameters.
RS485_SET_PID_ADVANCED_PARAMETERS = 0x45

""" END OF RS-485 COMMAND ID LIST """
# endregion

# Time out in ms.
COMMUNICATION_TIME_OUT = 5000

# Total size of our packet in bytes.
PACKET_SIZE = 64

# Data's size of a single packet.
PACKET_DATA_SIZE = 56

# Header's size of a packet.
PACKET_HEADER_SIZE = 8

# Version of this library.
COMM_LAYER_VERSION = 10002

# Max character count in our string.
SERIAL_LENGTH = 20

# The maximum devices count that the API can control.
MAX_KINOVA_DEVICE = 20

# Size of a RS485 message in bytes.
RS485_MESSAGE_SIZE = 20

# Max Qty of RS-485 message hold by a USB packet.
RS485_MESSAGE_MAX_COUNT = 3


class Packet(Structure):
    """
    That represents a packet. As a developper you don't have to use this 
    structure
    """
    _fields_ = [
        ("IdPacket", c_short),
        ("TotalPacketCount", c_short),
        ("IdCommand", c_short),
        ("TotalDataSize", c_short),
        ("Data", c_ubyte * PACKET_DATA_SIZE)
    ]


class KinovaDevice(Structure):
    """That is a device you can communicate with via this library."""
    _fields_ = [
        # The serial number of the device. If you are communicating with more
        # than 1 device, this will be used to identify the devices.
        ("SerialNumber", c_char * SERIAL_LENGTH),
        # The model of the device.
        ("Model", c_char * SERIAL_LENGTH),
        # Those variables represents the code version - Major.Minor.Release
        ("VersionMajor", c_int),
        ("VersionMinor", c_int),
        ("VersionMRelease", c_int),
        # The type of the device.
        ("DeviceType", c_int),
        # This is a device ID used by the API. User should not use it.
        ("DeviceID", c_int)
    ]


class _Data(Union):
    _fields_ = [
        ("DataByte", c_ubyte * 16),
        ("DataFloat", c_float * 4),
        ("DataLong", c_uint * 4)
    ]


class RS485_Message(Structure):
    """This structure represents a RS-485 message"""
    _anonymous_ = [
        "u"
    ]
    _fields_ = [
        # Command ID of the message. Use from the COMMAND ID LIST above.
        ("Command", c_short),
        # Source of the message. If this is an actuator, it will be an address.
        ("SourceAddress", c_ubyte),
        # Destination of the message. Use the address of the actuator.
        ("DestinationAddress", c_ubyte),
        ("u", _Data)
    ]


_comm_lib = CDLL("Kinova.API.CommLayerUbuntu.so")

"""NORMAL USB FUNCTIONS"""

_comm_lib.InitCommunication.restype = c_int


def InitCommunication() -> c_int:
    return _comm_lib.InitCommunication()


_comm_lib.CloseCommunication.restype = c_int


def CloseCommunication() -> c_int:
    return _comm_lib.CloseCommunication()


_comm_lib.GetDeviceCount.argtypes = [POINTER(c_int)]
_comm_lib.GetDeviceCount.restype = c_int


def GetDeviceCount(result: c_int) -> c_int:
    return _comm_lib.GetDeviceCount(byref(result))


_comm_lib.SendPacket.argtypes = [Packet, Packet, c_int]
_comm_lib.SendPacket.restype = Packet


def SendPacket(packet_out: Packet, packet_in: Packet, result: c_int) -> Packet:
    return _comm_lib.SendPacket(byref(packet_out), byref(packet_in), byref(result))


_comm_lib.ScanForNewDevice.restype = c_int


def ScanForNewDevice() -> c_int:
    return _comm_lib.ScanForNewDecive()


_comm_lib.GetDevices.argtypes = [KinovaDevice * MAX_KINOVA_DEVICE, POINTER(c_int)]
_comm_lib.GetDevices.restype = c_int


def GetDevices(devices: KinovaDevice*MAX_KINOVA_DEVICE, result: c_int) -> c_int:
    return _comm_lib.GetDevices(devices, byref(result))


_comm_lib.SetActiveDevice.argtypes = [KinovaDevice]
_comm_lib.SetActiveDevice.restype = c_int


def SetActiveDevice(device: KinovaDevice):
    return _comm_lib.SetActiveDevice(device)


_comm_lib.GetActiveDevice.argtypes = [KinovaDevice]
_comm_lib.GetActiveDevice.restype = c_int


def GetActiveDevice(device: KinovaDevice):
    return _comm_lib.GetActiveDevice(byref(device))

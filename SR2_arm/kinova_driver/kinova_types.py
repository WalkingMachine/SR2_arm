from ctypes import c_int, c_uint, c_ushort, Structure, c_float
from enum import Enum, auto
from typing import Any, List

# Size of the ControlsModeMap array in the structure JoystickCommand.
JOYSTICK_BUTTON_COUNT = 16

# Max size of the advance retract trajectory that is stored in the ClientConfigurations.
NB_ADVANCE_RETRACT_POSITION = 20

# This is the size of the data array stored in a SystemError object.
ERROR_DATA_COUNT_MAX = 50

# The robot's firmware has several software layers. This describes how many layer there is in the firmware.
ERROR_LAYER_COUNT = 7

# This represents the max count of protection zones that can be stored in the robot's memory.
LEGACY_CONFIG_NB_ZONES_MAX = 10

# This is the size of the array Points in a ZoneShape.
LEGACY_CONFIG_NB_POINTS_COUNT = 8

# This is the size of the array Mapping contained in the structure ControlMappingCharts.
CONTROL_MAPPING_COUNT = 6

# This is the size of the arrays ModeControlsA and ModeControlsB contained in the structure ControlMapping.
MODE_MAP_COUNT = 6

# This is the size of the array ControlSticks contained in the structure ControlsModeMap.
STICK_EVENT_COUNT = 6

# This is the size of the array ControlButtons contained in the structure ControlsModeMap.
BUTTON_EVENT_COUNT = 26

# This is the size of all strings stored in the robot's firmware.
STRING_LENGTH = 20

# This is the max finger count in a robot. (Jaco has 3 fingers and Mico has 2 fingers)
JACO_FINGERS_COUNT = 3

# This is an error code. It means that the file you are trying to interact with does not exist
# or is corrupted. Either way, the OS does not recognise it.
ERROR_UNKNOWFILE = 5001

# This is an error code. It means that there was a memory related error. Most of the time it is because
# the system does not have enough memory.
ERROR_MEMORY = 5002

# This is an error code. It means that the function has some problem reading a file. Most of the time
# it is because the user don't have the privileges to do it.
ERROR_FILEREADING = 5003

# This represents the size of a memory page used to program the robot.
PAGE_SIZE = c_uint(2048)

# This represents the size of a page's address
ADDRESS_PAGE_SIZE = c_int(4)

# This represents the quantity of USB packet stored in a memory page.
PACKET_PER_PAGE_QTY = c_ushort(40)

# That represents the data's size of each USB packet during firmware update.
PAGEPACKET_SIZE = c_int(52)

# That represents the size of a USB packet's header.
USB_HEADER_SIZE = c_int(8)

# That represents the data's size of a normal USB packet.
USB_DATA_SIZE = c_int(56)


class POSITION_TYPE(Enum):
    """
    That represents the type of a position. If used during a trajectory, the type of position
    will change the behaviour of the robot. For example if the position type is CARTESIAN_POSITION,
    then the robot's end effector will move to that position using the inverse kinematics. But
    if the type of position is CARTESIAN_VELOCITY then the robot will use the values as velocity command.
    """

    # Used for initialisation.
    NOMOVEMENT_POSITION = c_int(0)

    # A cartesian position described by a translation X, Y, Z and an orientation ThetaX, thetaY and ThetaZ.
    CARTESIAN_POSITION = c_int(1)

    #  An angular position described by a value for each actuator.
    ANGULAR_POSITION = c_int(2)

    #  The robotic arm is in retracted mode. It may be anywhere between the HOME position and the RETRACTED position.
    RETRACTED = c_int(3)

    # The robotic arm is moving to the pre defined position #1.
    PREDEFINED1 = c_int(4)

    # The robotic arm is moving to the pre defined position #2.
    PREDEFINED2 = c_int(5)

    # The robotic arm is moving to the pre defined position #3.
    PREDEFINED3 = c_int(6)

    # A velocity vector used for velocity control.
    CARTESIAN_VELOCITY = c_int(7)

    # Used for initialisation.
    ANGULAR_VELOCITY = c_int(8)

    # The robotic arm is moving to the pre defined position #4.
    PREDEFINED4 = c_int(9)

    # The robotic arm is moving to the pre defined position #5.
    PREDEFINED5 = c_int(10)

    # Not used.
    ANY_TRAJECTORY = c_int(11)

    # The robotic arm is on time delay.
    TIME_DELAY = c_int(12)


class PORT_TYPE(Enum):
    """That represents the type of port from a peripheral."""

    # generic port.
    PERIPHERAL_PORT_ANY = c_int(0)

    # Internal CAN port.
    PERIPHERAL_PORT_CAN_INTERNAL = c_int(1)

    # External CAN port.
    PERIPHERAL_PORT_PORT_CAN_EXTERNAL = c_int(2)

    # SPI 0 port.
    PERIPHERAL_PORT_PORT_SPI_0 = c_int(3)

    # SPI 1 port.
    PERIPHERAL_PORT_PORT_SPI_1 = c_int(4)

    # USB port.
    PERIPHERAL_PORT_PORT_USB = c_int(5)

    # UART 0 port.
    PERIPHERAL_PORT_PORT_UART_0 = c_int(6)

    # UART 1 port.
    PERIPHERAL_PORT_PORT_UART_1 = c_int(7)

    # UART 2 port.
    PERIPHERAL_PORT_PORT_UART_2 = c_int(8)

    # Virtual port.
    PERIPHERAL_PORT_PORT_VIRTUAL = c_int(9)


class PERIPHERAL_TYPE(Enum):
    """That represents the type of a peripheral."""

    # Unknown type.
    PERIPHERAL_TYPE_NONE = c_int(0)

    # Abstract peripheral. internal use only.
    PERIPHERAL_TYPE_ANY = c_int(1)

    # Unknown peripheral. internal use only.
    PERIPHERAL_TYPE_UNKNOWN = c_int(2)

    # A generic rotary actuator.
    PERIPHERAL_TYPE_ACTUATOR_GENERIC = c_int(100)

    # A big 19 Newton*Meter rotary actuator.
    PERIPHERAL_TYPE_ACTUATOR_BIG_19NM = c_int(101)

    # A big 37 Newton*Meter rotary actuator.
    PERIPHERAL_TYPE_ACTUATOR_BIG_37NM = c_int(102)

    # A small 7 Newton*Meter rotary actuator.
    PERIPHERAL_TYPE_ACTUATOR_SMALL_7NM = c_int(103)

    # A generic linear actuator.
    PERIPHERAL_TYPE_LINEAR_ACTUATOR_GENERIC = c_int(200)

    # A 120 Newton(finger) linear actuator.
    PERIPHERAL_TYPE_LINEAR_ACTUATOR_120N = c_int(201)

    # A generic joystick.
    PERIPHERAL_TYPE_JOYSTICK = c_int(300)

    # A virtual joystick. This is mainly used by the API.
    PERIPHERAL_TYPE_VIRTUAL_JOYSTICK = c_int(301)

    # A kinovian joystick.
    PERIPHERAL_TYPE_KINOVA_JOYSTICK_3AXIS = c_int(302)

    # A universal interface V2.
    PERIPHERAL_TYPE_UNIVERSAL_INTERFACE_V2 = c_int(303)

    # A CAN interface on the main board.
    PERIPHERAL_TYPE_CAN_INTERFACE = c_int(400)


class HAND_MODE(Enum):
    """That indicates how the end effector will be used."""

    def _generate_next_value_(self, start: int, count: int, last_values: List[Any]) -> Any:
        return c_int(count)

    # Fingers wil not move
    HAND_NOMOVEMENT = auto()

    # Fingers will move using position control.
    POSITION_MODE = auto()

    # Fingers will move using velocity control.
    VELOCITY_MODE = auto()


class ArmLaterality(Enum):
    """That indicates if the robot will be left handed or right handed."""

    def _generate_next_value_(self, start: int, count: int, last_values: List[Any]) -> Any:
        return c_int(count)

    # Right handed
    RIGHTHAND = auto()

    # Left handed
    LEFTHAND = auto()


class TORQUECONTROL_TYPE(Enum):
    DIRECTTORQUE = c_int(0)
    IMPEDANCEANGULAR = c_int(1)
    IMPEDANCECARTESIAN = c_int(2)


class GENERALCONTROL_TYPE(Enum):
    def _generate_next_value_(self, start: int, count: int, last_values: List[Any]) -> Any:
        return c_int(count)

    POSITION = auto()

    TORQUE = auto()


class Controller(Enum):
    """This represents a type of controller. A controller is an entity that can send control commands to the robot."""

    # A three axis joystick controller.
    THREE_AXIS_JOYSTICK = c_int(0)

    # A two axis joystick controller.
    TWO_AXIS_JOYSTICK = c_int(1)

    # The kinova API.
    API = c_int(2)

    # The easy rider controller.
    EASY_RIDER = c_int(3)

    # The kinova universal interface controller.
    UNIVERSAL_INTERFACE = c_int(4)

    # An external custom interface controller.
    EXTERNAL_CUSTOMINTERFACE = c_int(5)

    # No interface.
    NONE = c_int(6)

    # An OLED display.
    OLED_DISPLAY = c_int(7)


class CONTROL_TYPE(Enum):
    """This represents a type of control. For now, there is 3 type of control, the cartesian control,
    the angular control or the force control."""

    # Cartesian control. (translation and orientation)
    CONTROL_TYPE_CARTESIAN = c_int(0)

    # Angular control. (joint by joint)
    CONTROL_TYPE_ANGULAR = c_int(1)

    # Unknown control type.
    CONTROL_TYPE_UNKNOWN = c_int(9999)


class CONTROL_MODULE(Enum):
    """That describes a control module of the robotical arm's firmware."""

    def _generate_next_value_(self, start: int, count: int, last_values: List[Any]) -> Any:
        return c_int(count)

    # No control module selected. The robotical arm cannot moves.
    CONTROL_MODULE = auto()

    # Angular velocity control mode. Values sent to the actuators are velocity. Unit: degree / second
    CONTROL_MODULE_ANGULAR_VELOCITY = auto()

    # Angular position control mode. Values sent to the actuators are position. Unit: degree
    CONTROL_MODULE_ANGULAR_POSITION = auto()

    # Cartesian velocity control mode. values sent to the end effector are velocity.
    # translation Unit: meter / second,orientation unit: RAD / second.
    CONTROL_MODULE_CARTESIAN_VELOCITY = auto()

    # Cartesian position control mode. values sent to the actuators are velocity. translation Unit: meter,
    # orientation unit: RAD.
    CONTROL_MODULE_CARTESIAN_POSITION = auto()

    # Retract control mode. This manage movement between the READY(HOME) and the RETRACTED position.
    # This can be angular or cartesian position control.
    CONTROL_MODULE_RETRACT = auto()

    # Not used for now.
    CONTROL_MODULE_TRAJECTORY = auto()

    # This manages the pre programmed position(GOTO). This is position control.
    CONTROL_MODULE_PREDEFINED = auto()

    # This manages the time delay during a trajectory.
    CONTROL_MODULE_TIMEDELAY = auto()


class RETRACT_TYPE(Enum):
    """This describes the retract type the robotical arm."""

    # The robotical arm was in a normal custom position and is going toward the READY position.
    RETRACT_TYPE_NORMAL_TO_READY = c_int(0)

    # The robotical arm is in READY position and is waiting for a command.
    RETRACT_TYPE_READY_STANDBY = c_int(1)

    # The robotical arm was in READY position and is going toward the RETRACTED position.
    RETRACT_TYPE_READY_TO_RETRACT = c_int(2)

    # The robotical arm was in RETRACT position and is waiting for a command.
    RETRACT_TYPE_RETRACT_STANDBY = c_int(3)

    # The robotical arm was in RETRACT position and is going toward the READY position.
    RETRACT_TYPE_RETRACT_TO_READY = c_int(4)

    # The robotical arm is initialized and is anywhere in the workspace but it is not retracted,
    # in READY position or between.
    RETRACT_TYPE_NORMAL_STANDBY = c_int(5)

    # The robotical arm is not initialized.
    RETRACT_TYPE_NOT_INITIALIZED = c_int(6)

    # An error has occured during the retract process.
    RETRACT_ERROR = c_int(25000)


class AngularInfo(Structure):
    """This data structure holds values in an angular(joint by joint) control context. As an example struct could
    contains position, temperature, torque, ..."""

    _fields_ = [
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the first actuator. Unit depends on the context it's been used.
        ("Actuator1", c_float),
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the second actuator. Unit depends on the context it's been used.
        ("Actuator2", c_float),
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the third actuator. Unit depends on the context it's been used.
        ("Actuator3", c_float),
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the actuator #4. Unit depends on the context it's been used.
        ("Actuator4", c_float),
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the actuator #5. Unit depends on the context it's been used.
        ("Actuator5", c_float),
        # As an example if the current control mode is angular position the unit will be degree but if the control mode
        # is angular velocity then the unit will be degree per second.
        # This is the value related to the actuator #6. Unit depends on the context it's been used.
        ("Actuator6", c_float)
    ]

    def __init__(self, Actuator1=c_float(0.0), Actuator2=c_float(0.0),
                 Actuator3=c_float(0.0), Actuator4=c_float(0.0),
                 Actuator5=c_float(0.0), Actuator6=c_float(0.0)):
        super(AngularInfo, self).__init__(Actuator1, Actuator2, Actuator3, Actuator4, Actuator5, Actuator6)


class CartesianInfo(Structure):
    """This data structure holds values in a cartesian control context."""

    _fields_ = [
        ("X", c_float),
        ("Y", c_float),
        ("Z", c_float),
        ("ThetaX", c_float),
        ("ThetaY", c_float),
        ("ThetaZ", c_float),
    ]

    def __init__(self, X=c_float(0.0), Y=c_float(0.0), Z=c_float(0.0),
                 ThetaX=c_float(0.0), ThetaY=c_float(0.0), ThetaZ=c_float(0.0)):
        super(CartesianInfo, self).__init__(X, Y, Z, ThetaX, ThetaY, ThetaZ)


class SensorsInfo(Structure):
    _fields_ = [
        ("Voltage", c_float),
        ("Current", c_float),
        ("AccelerationX", c_float),
        ("AccelerationY", c_float),
        ("AccelerationZ", c_float),
        ("ActuatorTemp1", c_float),
        ("ActuatorTemp2", c_float),
        ("ActuatorTemp3", c_float),
        ("ActuatorTemp4", c_float),
        ("ActuatorTemp5", c_float),
        ("ActuatorTemp6", c_float),
        ("FingerTemp1", c_float),
        ("FingerTemp2", c_float),
        ("FingerTemp3", c_float)
    ]

    def __init__(self, Voltage=c_float(0.0), Current=c_float(0.0),
                 AccelerationX=c_float(0.0), AccelerationY=c_float(0.0), AccelerationZ=c_float(0.0),
                 ActuatorTemp1=c_float(0.0), ActuatorTemp2=c_float(0.0), ActuatorTemp3=c_float(0.0),
                 ActuatorTemp4=c_float(0.0), ActuatorTemp5=c_float(0.0), ActuatorTemp6=c_float(0.0),
                 FingerTemp1=c_float(0.0), FingerTemp2=c_float(0.0), FingerTemp3=c_float(0.0)):
        super(SensorsInfo, self).__init__(Voltage, Current, AccelerationX, AccelerationY, AccelerationZ,
                                          ActuatorTemp1, ActuatorTemp2, ActuatorTemp3, ActuatorTemp4, ActuatorTemp5,
                                          ActuatorTemp6, FingerTemp1, FingerTemp2, FingerTemp3)


class FingerPosition(Structure):
    _fields_ = [
        ("Finger1", c_float),
        ("Finger2", c_float),
        ("Finger3", c_float),
    ]

    def __init__(self, Finger1=c_float(0.0), Finger2=c_float(0.0), Finger3=c_float(0.0)):
        super(FingerPosition, self).__init__(Finger1, Finger2, Finger3)


class CartesianPosition(Structure):
    _fields_ = [
        ("Coordinates", CartesianInfo),
        ("Fingers", FingerPosition),
    ]

    def __init__(self, Coordinates=CartesianInfo(), Fingers=FingerPosition()):
        super(CartesianPosition, self).__init__(Coordinates, Fingers)


class AngularPosition(Structure):
    _fields_ = [
        ("Actuators", AngularInfo),
        ("Fingers", FingerPosition)
    ]

    def __init__(self, Actuators=AngularInfo(), Fingers=FingerPosition()):
        super(AngularPosition, self).__init__(Actuators, Fingers)


class Limitation(Structure):
    _fields_ = [
        ("speedParameter1", c_float),
        ("speedParameter2", c_float),
        ("speedParameter3", c_float),
        ("forceParameter1", c_float),
        ("forceParameter2", c_float),
        ("forceParameter3", c_float),
        ("accelerationParameter1", c_float),
        ("accelerationParameter2", c_float),
        ("accelerationParameter3", c_float),
    ]

    def __init__(self, speedParameter1=c_float(0.0), speedParameter2=c_float(0.0), speedParameter3=c_float(0.0),
                 forceParameter1=c_float(0.0), forceParameter2=c_float(0.0), forceParameter3=c_float(0.0),
                 accelerationParameter1=c_float(0.0), accelerationParameter2=c_float(0.0),
                 accelerationParameter3=c_float(0.0)):
        super(Limitation, self).__init__(speedParameter1, speedParameter2, speedParameter3, forceParameter1,
                                         forceParameter2, forceParameter3, accelerationParameter1,
                                         accelerationParameter2, accelerationParameter3)


class UserPosition(Structure):
    _fields_ = [
        ("Type", c_int),
        ("Delay", c_float),
        ("CartesianPosition", CartesianInfo),
        ("Actuators", AngularInfo),
        ("HandMode", c_int),
        ("Fingers", FingerPosition)
    ]

    def __init__(self,
                 Type: POSITION_TYPE = POSITION_TYPE.CARTESIAN_POSITION.value,
                 Delay: c_float = c_float(0.0),
                 CartesianPosition: CartesianInfo = CartesianInfo(),
                 Actuators: AngularInfo = AngularInfo(),
                 HandMode: HAND_MODE = HAND_MODE.POSITION_MODE.value,
                 Fingers: FingerPosition = FingerPosition()):
        super(UserPosition, self).__init__(Type, Delay, CartesianPosition, Actuators, HandMode, Fingers)


class TrajectoryPoint(Structure):
    _fields_ = [
        ("Position", UserPosition),
        ("LimitationActive", c_int),
        ("SynchroType", c_int),
        ("Limitations", Limitation)
    ]

    def __init__(self, Position=UserPosition(), LimitationActive=c_int(0), SynchroType=c_int(0),
                 Limitations=Limitation()):
        super(TrajectoryPoint, self).__init__(Position, LimitationActive, SynchroType, Limitations)


class TrajectoryFIFO(Structure):
    _fields_ = [
        ("TrajectoryCount", c_uint),
        ("UsedPercentage", c_float),
        ("MaxSize", c_uint)
    ]
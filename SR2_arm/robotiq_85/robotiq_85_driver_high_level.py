from robotiq_85_gripper import Robotiq85Gripper

import numpy as np

retry_limit = 5

class Robotiq85Driver:
    def __init__(self, comport = '/dev/ttyUSB0', baud = "115200", num_gripper = 1):

        self._baud = baud
        self._comport = comport
        self._num_grippers = num_gripper

        gripper = Robotiq85Gripper(self._num_grippers, self._comport, self._num_grippers)

        count = 0
        while gripper.init_success == False:
            if count == retry_limit:
                print("Unable to open commport to %s" % self._comport)
                break
            gripper = Robotiq85Gripper()
            count +=1

    
            if not self._gripper.init_success:
                rospy.logerr("Unable to open commport to %s" % self._comport)
                return

            if (self._num_grippers == 1):
                rospy.Subscriber("/gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
                self._gripper_pub = rospy.Publisher('/gripper/stat', GripperStat, queue_size=10)
                self._gripper_joint_state_pub = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)
            elif (self._num_grippers == 2):
                rospy.Subscriber("/left_gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
                self._left_gripper_pub = rospy.Publisher('/left_gripper/stat', GripperStat, queue_size=10)
                self._left_gripper_joint_state_pub = rospy.Publisher('/left_gripper/joint_states', JointState, queue_size=10)
                rospy.Subscriber("/right_gripper/cmd", GripperCmd, self._update_right_gripper_cmd, queue_size=10)
                self._right_gripper_pub = rospy.Publisher('/right_gripper/stat', GripperStat, queue_size=10)
                self._right_gripper_joint_state_pub = rospy.Publisher('/right_gripper/joint_states', JointState, queue_size=10)
            else:
                rospy.logerr("Number of grippers not supported (needs to be 1 or 2)")
                return

            self._seq = [0] * self._num_grippers
            self._prev_js_pos = [0.0] * self._num_grippers
            self._prev_js_time = [rospy.get_time()] * self._num_grippers
            self._driver_state = 0
            self._driver_ready = False

            success = True
            for i in range(self._num_grippers):
                success &= self._gripper.process_stat_cmd(i)
                if not success:
                    bad_gripper = i
            if not success:
                rospy.logerr("Failed to contact gripper %d....ABORTING"%bad_gripper)
                return

            self._run_driver()
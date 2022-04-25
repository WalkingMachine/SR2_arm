from robotiq_85.robotiq_85_gripper import Robotiq85Gripper


gripper = Robotiq85Gripper(comport="/dev/ttyUSB1")


if gripper.init_success == False:
    print("failed, sorry !")
    exit()

print("hi it supposed to work !")

gripper.process_stat_cmd()

gripper.process_act_cmd()
gripper.process_stat_cmd()

gripper.goto(pos=0.085, vel=0.1, force=5.0)

while True:
    gripper.process_act_cmd()
    gripper.process_stat_cmd()
    if gripper.get_pos() == gripper.get_req_pos():
        break


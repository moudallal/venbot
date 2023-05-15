#!/usr/bin/python3

import os
import math
import copy
import json
import random
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
from vision.msg import LegoStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from voice_commands.msg import Command
import PyKDL

command = Command()

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "X1-Y2-Z1": {
        "home": [0.264589, -0.293903, 0.777] 
    },
    "X2-Y2-Z2": {
        "home": [0.277866, -0.724482, 0.777] 
    },
    "X1-Y3-Z2": {
        "home": [0.268053, -0.513924, 0.777]  
    },
    "X1-Y2-Z2": {
        "home": [0.429198, -0.293903, 0.777] 
    },
    "X1-Y2-Z2-CHAMFER": {
        "home": [0.592619, -0.293903, 0.777]  
    },
    "X1-Y4-Z2": {
        "home": [0.108812, -0.716057, 0.777] 
    },
    "X1-Y1-Z2": {
        "home": [0.088808, -0.295820, 0.777] 
    },
    "X1-Y2-Z2-TWINFILLET": {
        "home": [0.103547, -0.501132, 0.777] 
    },
    "X1-Y3-Z2-FILLET": {
        "home": [0.433739, -0.507130, 0.777]  
    },
    "X1-Y4-Z1": {
        "home": [0.589908, -0.501033, 0.777]  
    },
    "X2-Y2-Z2-FILLET": {
        "home": [0.442505, -0.727271, 0.777] 
    }
}

# COLORS_INFO = {
#     "RED": {
#         "center": [],
#         "upper_left_corner": [0.069017, -0.137642, 0.777],
#         "lower_right_corner": [0.312757, -0.404576, 0.777]
#     },
#     "BLUE": {
#         "center": [],
#         "upper_left_corner": [0.420563, -0.141769, 0.777],
#         "lower_right_corner": [0.659010, -0.405402, 0.777]
#     },
#     "GREEN": {
#         "center": [],
#         "upper_left_corner": [0.083016, -0.582958, 0.777],
#         "lower_right_corner": [0.308283, -0.837076, 0.777]
#     },
#     "YELLOW": {
#         "center": [],
#         "upper_left_corner": [0.426449, -0.580072, 0.777],
#         "lower_right_corner": [0.638743, -0.822116, 0.777]
#     }
# }

COLORS_INFO = {
    "red": {
        "center": [],
        "upper_left_corner": [0.048724, -0.204377, 0.777],
        "lower_right_corner": [0.230023, -0.396315, 0.777]
    },
    "blue": {
        "center": [],
        "upper_left_corner": [0.361868, -0.200604, 0.777],
        "lower_right_corner": [0.524640, -0.400440, 0.777]
    },
    "green": {
        "center": [],
        "upper_left_corner": [0.054904, -0.575555, 0.777],
        "lower_right_corner": [0.217644, -0.767094, 0.777]
    },
    "yellow": {
        "center": [],
        "upper_left_corner": [0.347410, -0.563181, 0.777],
        "lower_right_corner": [0.506025, -0.734134, 0.777]
    }
}

for model, model_info in MODELS_INFO.items():
    pass
    #MODELS_INFO[model]["home"] = model_info["home"] + np.array([0.0, 0.10, 0.0])

for model, info in MODELS_INFO.items():
    model_json_path = os.path.join(PKG_PATH, "..", "models", f"lego_{model}", "model.json")
    # make path absolute
    model_json_path = os.path.abspath(model_json_path)
    # check path exists
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = (np.max(corners[:, 0]) - np.min(corners[:, 0]))
    size_y = (np.max(corners[:, 1]) - np.min(corners[:, 1]))
    size_z = (np.max(corners[:, 2]) - np.min(corners[:, 2]))

    #print(f"{model}: {size_x:.3f} x {size_y:.3f} x {size_z:.3f}")

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        print('ds = ' + str(ds))
        print('model_pose = ' + str(model_pose.position.x) + ' ' + str(model_pose.position.y))
        print('vision_pose = ' + str(vision_model_pose.position.x) + ' ' + str(vision_model_pose.position.y))
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]


def get_legos_pos(vision=False):
    #get legos position reading vision topic
    if vision:
        legos = rospy.wait_for_message("/lego_detections", LegoStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        legos = ModelStates()

        for name, pose in zip(models.name, models.pose):
            if "X" not in name:
                continue
            name = get_model_name(name)

            legos.name.append(name)
            legos.pose.append(pose)
    return [(lego_name, lego_pose, lego_color) for lego_name, lego_pose, lego_color in zip(legos.name, legos.pose, legos.color)]


def straighten(model_pose, gazebo_model_name):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)

    model_size = MODELS_INFO[get_model_name(gazebo_model_name)]["size"]

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = get_axis_facing_camera(model_quat)
    approach_angle = get_approach_angle(model_quat, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Angle of approaching measures {approach_angle:.2f} deg")

    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)

    # Get above the object
    controller.move_to(x, y, target_quat=approach_quat)

    # Calculate target quat
    regrip_quat = DEFAULT_QUAT
    if facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):  # Side
        target_quat = DEFAULT_QUAT
        pitch_angle = -math.pi/2 + 0.2

        if abs(approach_angle) < math.pi/2:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        else:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
        target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat

        if facing_direction == (0, 1, 0):
            regrip_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) * regrip_quat

    elif facing_direction == (0, 0, -1):
        """
            Pre-positioning
        """
        controller.move_to(z=z, target_quat=approach_quat)
        close_gripper(gazebo_model_name, model_size[0])

        tmp_quat = PyQuaternion(axis=(0, 0, 1), angle=2*math.pi/6) * DEFAULT_QUAT
        controller.move_to(SAFE_X, SAFE_Y, z+0.05, target_quat=tmp_quat, z_raise=0.1)  # Move to safe position
        controller.move_to(z=z)
        open_gripper(gazebo_model_name)

        approach_quat = tmp_quat * PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)

        target_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi)  # Add a yaw rotation of 180 deg

        regrip_quat = tmp_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

    """
        Grip the model
    """
    if facing_direction == (0, 0, 1) or facing_direction == (0, 0, -1):
        closure = model_size[0]
        z = SURFACE_Z + model_size[2] / 2
    elif facing_direction == (1, 0, 0):
        closure = model_size[1]
        z = SURFACE_Z + model_size[0] / 2
    elif facing_direction == (0, 1, 0):
        closure = model_size[0]
        z = SURFACE_Z + model_size[1] / 2
    controller.move_to(z=z, target_quat=approach_quat)
    close_gripper(gazebo_model_name, closure)

    """
        Straighten model if needed
    """
    if facing_direction != (0, 0, 1):
        z = SURFACE_Z + model_size[2]/2

        controller.move_to(z=z+0.05, target_quat=target_quat, z_raise=0.1)
        controller.move(dz=-0.05)
        open_gripper(gazebo_model_name)

        # Re grip the model
        controller.move_to(z=z, target_quat=regrip_quat, z_raise=0.1)
        close_gripper(gazebo_model_name, model_size[0])


def close_gripper(gazebo_model_name, closure=0):
    set_gripper(0.81-closure*10)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(gazebo_model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)


def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi/2:
            yaw_angle = math.pi/2
        else:
            yaw_angle = -math.pi/2
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat


def get_axis_facing_camera(quat):
    axis_x = np.array([1, 0, 0])
    axis_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])
    new_axis_x = quat.rotate(axis_x)
    new_axis_y = quat.rotate(axis_y)
    new_axis_z = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return 0, 0, 1
    elif angle < np.pi / 3 * 2 * 1.2:
        if abs(new_axis_x[2]) > abs(new_axis_y[2]):
            return 1, 0, 0
        else:
            return 0, 1, 0
        #else:
        #    raise Exception(f"Invalid axis {new_axis_x}")
    else:
        return 0, 0, -1


def get_approach_angle(model_quat, facing_direction):#get gripper approach angle
    if facing_direction == (0, 0, 1):
        return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = -1  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()

def command_callback(cmd):
    # print(command.verb + ' ' + command.color)
    command.verb = cmd.verb
    command.color = cmd.color
    command.valid = cmd.valid

def check_path(x,y,z):
    # Define the UR5 kinematic chain
    L1 = 0.089159 #d1 between base and shoulder
    L2 = 0.425 #a2 between shoulder and elbow
    L3 = 0.39225 #a3 between elbow and first wrist
    L4 = 0.10915 #d4
    L5 = 0.09465 #d5
    L6 = 0.0823 #d6
    robot = PyKDL.Chain()
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame(PyKDL.Vector(0, 0, L1))))
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, 0, 0))))
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, L2, 0))))
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, L3, 0))))
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotX), PyKDL.Frame(PyKDL.Vector(0, L4, L5))))
    robot.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, 0, L6))))
    # Define the inverse kinematics solver
    ik_solver = PyKDL.ChainIkSolverPos_LMA(robot)
    # Define the joint limits for each joint
    jl = math.pi
    joint_limits = [
        (-jl,jl),  # Joint 1
        (-jl,jl),  # Joint 2
        (-jl,jl),  # Joint 3
        (-jl,jl),  # Joint 4
        (-jl,jl),  # Joint 5
        (-jl,jl)   # Joint 6
]
    # Define the target end-effector pose (position and orientation)
    pos = np.array([x, y, z])
    rot = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    target_pose = PyKDL.Frame(PyKDL.Rotation(rot[0][0], rot[0][1], rot[0][2], rot[1][0], rot[1][1], rot[1][2], rot[2][0], rot[2][1], rot[2][2]), PyKDL.Vector(pos[0], pos[1], pos[2]))
    # Define the initial joint angles for the inverse kinematics calculation
    q_init = PyKDL.JntArray(6)
    q_init[0] = -1.5245
    q_init[1] = -1.0301
    q_init[2] = -1.4913
    q_init[3] = -2.1925
    q_init[4] = 1.5708
    q_init[5] = 0.0463
    # Calculate the inverse kinematics solution
    q_out = PyKDL.JntArray(6)
    ik_solver.CartToJnt(q_init, target_pose, q_out)
    # Check if the joint angles are within the joint limits
    is_within_limits = True
    for i in range(6):
        if q_out[i] < joint_limits[i][0] or q_out[i] > joint_limits[i][1]:
            is_within_limits = False
            break
    if is_within_limits:
        print('valid path')
        return True
    else:
        print('invalid path')
        return False
  
def pick(to_pick, legos):

    lego = {}

    for model_name, model_pose, model_color in legos:
        if model_color == to_pick:
            lego_name = model_name
            lego_pose = model_pose
            lego_color = to_pick
            break
    
    open_gripper()

    # Get actual model_name at model xyz coordinates
    gazebo_model_name = get_gazebo_model_name(lego_name, lego_pose)
    lego["lego_name"] = lego_name
    lego["lego_pose"] = lego_pose
    lego["lego_color"] = lego_color
    lego["gazebo_model_name"] = gazebo_model_name

    # Straighten lego
    straighten(lego_pose, gazebo_model_name)

    return lego

def place(to_place, lego_in_op, prev, legos):
    if to_place == 'previous':
        lego_color = prev
    else:
        lego_color = to_place
    for model_name, model_pose, model_color in legos:
        if model_color == lego_color:
            lego_name = model_name
            lego_pose = model_pose

    # model_home = MODELS_INFO[lego_name]["home"]
    model_size = MODELS_INFO[lego_name]["size"]

    target_upper_left = COLORS_INFO[lego_color]["upper_left_corner"]
    target_lower_right = COLORS_INFO[lego_color]["lower_right_corner"]  

    while True:
        # x, y, z = model_home
        x = random.uniform(target_upper_left[0], target_lower_right[0])
        y = random.uniform(target_upper_left[1], target_lower_right[1])
        z = target_upper_left[2]
        z += model_size[2] / 2 + 0.004
        if check_path(x,y,z):
            break

    print(f"Moving model {model_name} to {x} {y} {z}")
    controller.move_to(x, y, target_quat=DEFAULT_QUAT * PyQuaternion(axis=[0, 0, 1], angle=math.pi / 2))
    # Lower the object and release
    controller.move_to(x, y, z)
    set_model_fixed(lego_in_op["gazebo_model_name"])
    open_gripper(lego_in_op["gazebo_model_name"])

def sort_bricks(legos_available):
    copy_legos = list(legos_available)
    while copy_legos:
        random_lego = random.choice(copy_legos)
        lego_in_operation = pick(random_lego[2], legos)
        prev = lego_in_operation["lego_color"]
        controller.move(dz=0.15)
        controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
        place(random_lego[2], lego_in_operation, prev, legos)
        controller.move(dz=0.15)
        controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
        copy_legos.pop(copy_legos.index(random_lego))

if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")
    rospy.Subscriber("/command", Command, command_callback)
    global controller
    controller = ArmController()
    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

    print("Waiting for detection of the models")
    rospy.sleep(0.5)
    legos = get_legos_pos(vision=True)
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))
    prev = ''
    lego_in_operation = {}
    picked = False
    while not rospy.is_shutdown():
        if command.verb == 'pick' and not picked:
            # moving to destination would be done when processing the "place" command
            lego_in_operation = pick(command.color, legos)
            prev = lego_in_operation["lego_color"]
            picked = True
            controller.move(dz=0.15)
            controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

        elif command.verb == 'place' and picked:
            place(command.color, lego_in_operation, prev, legos)
            controller.move(dz=0.15)
            controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
            picked = False
            for i in range(len(legos) - 1, -1, -1):
                if legos[i][0] == lego_in_operation["lego_name"]:
                    del legos[i]

        elif command.verb == 'sort':
            sort_bricks(legos)

        elif command.verb == 'stop':
            break
            # if controller.gripper_pose[0][1] > -0.3 and controller.gripper_pose[0][0] > 0:
            #     controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)

            # increment z in order to stack lego correctly
            #MODELS_INFO[model_name]["home"][2] += model_size[2] - INTERLOCKING_OFFSET  
        else:
            pass

    print("Moving to Default Position")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)
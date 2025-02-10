#!/usr/bin/env python3
import os
import rospy
import rospkg
import rosnode
import roslaunch
import roslaunch.rlutil
from roslaunch.parent import ROSLaunchParent
from node_manager.srv import nodeInfo, nodeInfoResponse, nodeInfoRequest

# ANSI 转义码
GREEN = '\033[92m'
RESET = '\033[0m'

flag = 0
node_type = ""
node_name = ""
pack_name = ""
node_state = ""
launch_name = ""
dir_node = {}
dir_launch = {}


def check_pack_exists(pack):
    ros_pack = rospkg.RosPack()
    package_path = ros_pack.get_path(pack)
    if package_path is None:
        return False
    else:
        return True


def node_manager_bc(req):
    global node_state, pack_name, node_name, node_type, launch_name, flag
    node_state = req.nodeState
    if node_state == "start":
        node_type = req.nodeType
        if node_type == "node":
            pack_name = req.packName
            node_name = req.nodeName
            if not check_pack_exists(pack_name):
                rospy.logwarn("package: %s : %s is inexistence" % pack_name)
                flag = 0
                return nodeInfoResponse(ok=False)
            flag = 1
        elif node_type == "launch":
            pack_name = req.packName
            node_name = req.nodeName
            ros_pack = rospkg.RosPack()
            package_path = ros_pack.get_path(pack_name)
            launch_name = package_path + "/launch/" + node_name + ".launch"
            if package_path is None:
                rospy.logwarn("package: %s is inexistence" % pack_name)
                flag = 0
                return nodeInfoResponse(ok=False)
            if not os.path.isfile(launch_name):
                rospy.logwarn("launch file: %s is inexistence" % launch_name)
                flag = 0
                return nodeInfoResponse(ok=False)
            flag = 1
        else:
            rospy.logwarn("%s is error(nodeType is node/launch)" % node_type)
            flag = 0
            return nodeInfoResponse(ok=False)
    elif node_state == "stop":
        node_type = req.nodeType
        if node_type == "node":
            pack_name = req.packName
            node_name = req.nodeName
            node_info = pack_name + "/" + node_name
            node = dir_node.get(node_info)
            if node is None:
                rospy.logwarn("nodeName: %s No start" % node_info)
                flag = 0
                return nodeInfoResponse(ok=False)
            flag = 1
        elif node_type == "launch":
            pack_name = req.packName
            node_name = req.nodeName
            ros_pack = rospkg.RosPack()
            package_path = ros_pack.get_path(pack_name)
            launch_name = package_path + "/launch/" + node_name + ".launch"
            node = dir_launch.get(launch_name)
            if package_path is None:
                rospy.logwarn("package: %s is inexistence" % pack_name)
                flag = 0
                return nodeInfoResponse(ok=False)
            if node is None:
                rospy.logwarn("launch node: %s No start" % launch_name)
                flag = 0
                return nodeInfoResponse(ok=False)
            flag = 1
        else:
            rospy.logwarn("%s is error(nodeType is node/launch)" % node_type)
            flag = 0
            return nodeInfoResponse(ok=False)
    else:
        rospy.logwarn("%s is error(nodeState is start/stop)" % node_state)
        flag = 0
        return nodeInfoResponse(ok=False)
    return nodeInfoResponse(ok=True)


def manager():
    global node_state, pack_name, node_name, node_type, launch_name, flag
    if flag:
        if node_state == "start":
            if node_type == "node":
                launch_api = roslaunch.scriptapi.ROSLaunch()
                launch_api.start()
                node = roslaunch.core.Node(package=pack_name, node_type=node_name)
                process = launch_api.launch(node)
                node_info = pack_name + "/" + node_name
                dir_node[node_info] = process
                rospy.loginfo(f"{GREEN}start: %s  %s{RESET}" % (pack_name, node_name))
            elif node_type == "launch":
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                launch = ROSLaunchParent(uuid, [launch_name])
                launch.start()
                dir_launch[launch_name] = launch
                rospy.loginfo(f"{GREEN}start: %s {RESET}" % launch_name)
            flag = 0

        elif node_state == "stop":
            if node_type == "node":
                node_info = pack_name + "/" + node_name
                process = dir_node[node_info]
                process.stop()
                del dir_node[node_info]
                rospy.loginfo(f"{GREEN}stop: %s  %s{RESET}" % (pack_name, node_name))
            elif node_type == "launch":
                launch = dir_launch[launch_name]
                launch.shutdown()
                del dir_launch[launch_name]
                rospy.loginfo(f"{GREEN}stop: %s {RESET}" % launch_name)
            flag = 0


if __name__ == "__main__":
    rospy.init_node("node_manager", anonymous=False)
    server = rospy.Service("/nodeManager", nodeInfo, node_manager_bc)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        manager()
        rate.sleep()

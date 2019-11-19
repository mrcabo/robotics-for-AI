#!/usr/bin/python
import rospy
import rosparam
import rospkg
import os
import sys
import time

if __name__ == "__main__":
    rospy.init_node("package_starter_node")

    command = None

    if len(sys.argv) == 2:
        command = sys.argv[1] 
        os.system(command + " &")
        exit()
    else:
        try:
            file_name = rospy.get_param("~file")
        except:
            rospy.logerr("No file given as parameter")
            exit()

    rospack = rospkg.RosPack()

    try:
        path = rospack.get_path("package_config") 
    except:
        rospy.logerr("Could not find package_config package")
        exit()

    try:
        data = rosparam.load_file(path + "/config/" + file_name)
    except:
        rospy.logerr("Could not load {0}, in path: {1}".format(file_name, path))
        exit()
    
    data = data[0][0]

    for key in data:
        command = data[key]
        if command == 'sleep':
            time.sleep(5)
        else:
            os.system(command + " &")    

    os.system("sleep 3") 

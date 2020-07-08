#!/usr/bin/env python
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def callback(config, level):
    #rospy.loginfo("Received reconf call: " + str(config))
    return config

if __name__ == '__main__':
    rospy.loginfo("Marathon Parameters - Running")
    rospy.init_node('marathon_params')

    # Create a D(ynamic)DynamicReconfigure
    player = DDynamicReconfigure("player")

    # Add variables (name, description, default value, min, max, edit_method)
    # ddynrec.add_variable("decimal", "float/double variable", 0.0, -1.0, 1.0)  
    player.add_variable("Pan_Kp",      "Pan Kp",      0.0, 0, 1)
    player.add_variable("Pan_Ki",      "Pan Ki",      0.0, 0, 1)
    player.add_variable("Pan_Kd",      "Pan Kd",      0.0, 0, 0.000010)
    player.add_variable("Tilt_Kp",     "Tilt Kp",     0.0, 0, 1)
    player.add_variable("Tilt_Ki",     "Tilt Ki",     0.0, 0, 1)
    player.add_variable("Tilt_Kd",     "Tilt Kd",     0.0, 0, 0.000010)
    player.add_variable("Pan_Step",    "Pan Step",    0.0, 0, 0.2)
    player.add_variable("Tilt_Step",   "Tilt Step",   0.0, 0, 0.5)
    player.add_variable("Tilt_Angle",  "Tilt Angle",  0.0, -2, 0)
    player.add_variable("Scan_Rate",   "Scan Rate",   0,   0, 10)
    player.add_variable("Body_Kp",     "Body Kp",     0.0, 0, 5)

    # # Yellow
    # lyellow = DDynamicReconfigure("line/yellow")
    # lyellow.add_variable("H_Max",  "H Max", 255, 0, 255)
    # lyellow.add_variable("H_Min",  "H Min",   0, 0, 255)
    # lyellow.add_variable("S_Max",  "S Max", 255, 0, 255)
    # lyellow.add_variable("S_Min",  "S Min",   0, 0, 255)
    # lyellow.add_variable("V_Max",  "V Max", 255, 0, 255)
    # lyellow.add_variable("V_Min",  "V Min",   0, 0, 255)
    # lyellow.add_variable("Erode",  "Erode",   0, 0,  10)
    # lyellow.add_variable("Dilate", "Dilate",  0, 0,  10)
    # lyellow.add_variable("Size",   "Size",    0, 0,  1000)

    # # BLue
    # lblue = DDynamicReconfigure("line/blue")
    # lblue.add_variable("H_Max",  "H Max", 255, 0, 255)
    # lblue.add_variable("H_Min",  "H Min",   0, 0, 255)
    # lblue.add_variable("S_Max",  "S Max", 255, 0, 255)
    # lblue.add_variable("S_Min",  "S Min",   0, 0, 255)
    # lblue.add_variable("V_Max",  "V Max", 255, 0, 255)
    # lblue.add_variable("V_Min",  "V Min",   0, 0, 255)
    # lblue.add_variable("Erode",  "Erode",   0, 0,  10)
    # lblue.add_variable("Dilate", "Dilate",  0, 0,  10)
    # lblue.add_variable("Size",    "Size",   0, 0,  1000)
    
    # Start the server
    player.start(callback)
    # lyellow.start(callback)
    # lblue.start(callback)

    rospy.spin()
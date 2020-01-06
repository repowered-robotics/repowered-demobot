#!/usr/bin/env python3
import rospy
import rospkg
import time
import numpy as np
import json
from uwb.msg import UwbData
from imu.msg import ImuData
from motor_ctrl.srv import SetMotor
from std_msgs.msg import String
#===============================================================================

class UwbTag:
    def __init__(self):
        self.x = np.nan
        self.y = np.nan
        self.measurements = {}
        self.anchor_persist_time = 2.0
        self.uwb_sub = rospy.Subscriber("uwb_data", UwbData, self.uwb_data_callback, queue_size=10)

    def uwb_data_callback(self, msg):
        anchor_id = msg.id
        anchor_x = msg.x
        anchor_y = msg.y
        distance = msg.distance
        if distance >= 0:
            self.measurements[anchor_id]['timestamp'] = time.time()
            self.measurements[anchor_id]['distance'] = distance
            self.measurements[anchor_id]['position'] = (anchor_x, anchor_y)
        else:
            self.measurements[anchor_id] = np.nan  # set to invalid value to be ignored

    def get_position(self):
        # if no update/valid distance from an anchor for this long, delete it
        for id in self.measurements.keys():
            if (time.time() - self.measurements[id]['timestamp']) > self.anchor_persist_time:
                del self.measurements[id]

        if len(list(self.measurements.keys())) >= 3:
            start_time = time.time()
            P = Project(mode='2D', solver='LSE_GC')

            for anchor_id in self.measurements.keys():
                P.add_anchor(anchor_id, self.measurements[anchor_id]['position'])

            t,label=P.add_target()

            for anchor_id in self.measurements.keys():
                t.add_measure(anchor_id, self.measurements[sensor]['distance'])

            P.solve()
            # Then the target location is:
            position = t.loc
            if position is not None:
                self.x = position.x
                self.y = position.y
            #print('Triangulation took %.4f seconds' % (time.time() - start_time))
        else:
            print('Not enough points to triangulate node...')
        return self.x, self.y

#===============================================================================
class NavigationNode:
    def __init__(self):
        print('Booting up navigation node...')
        rospy.init_node('nav', anonymous=True)
        self.rp = rospkg.RosPack()
        self.tag = UwbTag()
        self.rate = rospy.Rate(20) # 20Hz
        self.arena_theta = 0.0 # offset from magnetic north
        self.bearing = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_bearing = 0.0
        self.x, self.y = 0.0, 0.0
        self.imu_sub = rospy.Subscriber("imu_data", ImuData, self.imu_data_callback, queue_size=10)
        self.server_sub = rospy.Subscriber("server_data", String, self.server_data_callback, queue_size=4)

    def imu_data_callback(self, msg):
        self.mag_x = msg.mag_x
        self.mag_y = msg.mag_y
        theta = np.arctan(self.mag_y/self.mag_x)
        if self.mag_x < 0.0:
            theta += np.pi
        self.bearing = theta + self.arena_theta


    def server_data_callback(self, msg):
        print("Received " + msg.data + " from server")
        target_pos = json.JSONDecoder().decode(msg.data)
        self.target_x = float(target_pos['x'])
        self.target_y = float(target_pos['y'])
        self.x, self.y = self.tag.get_position()
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        theta = np.arctan(dy/dx)
        if dx < 0.0:
            theta += np.pi
        self.target_bearing = theta

#===============================================================================
#===============================================================================
if __name__ == '__main__':
    nav_node    = NavigationNode()
    print("starting the main loop")
    while not rospy.is_shutdown():
        try:
            print("Target: ({:.1f}m, {:.1f}m) @ {:.2f} rad".format(nav_node.target_x, nav_node.target_y, nav_node.target_bearing))
            
            nav_node.rate.sleep()
        except rospy.ROSInterruptException:
            nav_node.cleanup()

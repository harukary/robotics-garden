import numpy as np
import math

import rospy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from collections import deque

# 0: front, 1~:counter-clockiwise
class OccupancyCheck:
    def __init__(self,viz=False):
        self.viz = viz
        if self.viz:
            self.rviz_pub = rospy.Publisher('/rviz_occupancy', MarkerArray, queue_size=1)

    def execute(self,scan=None, r_min=1, r_max=6, n=8):
        incr = scan.angle_increment
        theta_a = np.pi / n
        target_a = (r_max**2-r_min**2) * theta_a
        scan_d = deque(scan.ranges)
        scan_d.rotate(int((-scan.angle_min-theta_a)/incr))
        # print('scan   ',list(scan_d)[0],list(scan_d)[:int(theta_a*2/incr)][-1])
        occupancy = []
        for i in range(n):
            scan_d.rotate(int(theta_a*2/incr))
            scan_n = list(scan_d)[:int(theta_a*2/incr)]
            # print('scan['+str(i)+']',scan_n[0],scan_n[-1])
            for j,r in enumerate(scan_n):
                if r > r_max:
                    scan_n[j] = r_max
                elif r < r_min:
                    scan_n[j] = r_min
            # if i == 0:
            #     print(target_a)
            #     print(r_max**2 * theta_a, sum([r**2*incr/2 for r in scan_n]), scan_n[0]**2*incr/2)
            occupied = r_max**2 * theta_a - sum([r**2*incr/2 for r in scan_n])
            occupancy.append((occupied/target_a))
        if self.viz:
            self.visualize(occupancy, r_min, r_max, n)
        return occupancy

    def visualize(self, occupancy, r_min, r_max, n):
        markerArray = MarkerArray()
        r = (r_min+r_max)/2
        theta = 2 * np.pi / n
        for i,data in enumerate(occupancy):
            marker = Marker()
            marker.header.frame_id = "/velodyne"
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = data
            marker.color.a = 0.5
            # if i==0:
            #     marker.color.r=1.0
            # elif i==1:
            #     marker.color.g=1.0
            # else:
            #     marker.color.b=1.0
            if data < 0.2:
                marker.color.b = 1.0
            elif data > 0.5:
                marker.color.r = 1.0
            else:
                marker.color.g = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = r*math.cos(i*(-theta))
            marker.pose.position.y = r*math.sin(i*(-theta)) 
            marker.pose.position.z = 0. 
            # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
            markerArray.markers.append(marker)
            # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        # Publish the MarkerArray
        self.rviz_pub.publish(markerArray)




scan = LaserScan()
scan.angle_increment = math.radians(1.)
scan.angle_min = - np.pi
scan.angle_max = np.pi
scan.ranges = [i for i in range(0,360)]

occupancy_check = OccupancyCheck()
occupancy = occupancy_check.execute(scan)
# print(occupancy)
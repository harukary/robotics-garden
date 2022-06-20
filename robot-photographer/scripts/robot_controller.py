import rospy
import sys
sys.path.append('.')
from RobotController import RobotController

from visualization_msgs.msg import Marker, MarkerArray

topics = {
    'twist'                 : '/diff_drive_controller/cmd_vel', #'/cmd_vel',
    'compressed_image'      : '/camera0/compressed', #'/camera/rgb/image_raw',
    'image'                 : '/photographer_image',
    'depth'                 : '/velodyne_points', # '/camera/depth/image',
    'scan'                  : '/scan',
    'pose'                  : '/odom',
    'nav_s'                 : '/move_base',
    'nav_r'                 : '/move_base/result',
    'obj'                   : '/yolov5_result',
    'obstacles'             : '/raw_obstacles',
    'face'                  : '/face_result'
}

rviz_pub = rospy.Publisher('/rviz_objects', MarkerArray, queue_size=1)

def publish_rviz(objects):
    markerArray = MarkerArray()
    for obj in objects:
        if obj['p_xy'] is not None:
            marker = Marker()
            marker.header.frame_id = "/velodyne"
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 0.5
            if obj['class'] == 0: # person
                marker.color.r = 1.0
            elif obj['class'] != 0: # other
                marker.color.b = 1.0
            else:
                marker.color.g = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = obj['p_xy'][0]
            marker.pose.position.y = obj['p_xy'][1] 
            marker.pose.position.z = 0.
            marker.lifetime = rospy.Duration(0.1)
            # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
            markerArray.markers.append(marker)
    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    # Publish the MarkerArray
    print(len(markerArray.markers))
    rviz_pub.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    robot = RobotController(topics)
    rate = rospy.Rate(10)

    print("ready")
    while not rospy.is_shutdown():
        # robot.roomba_walk()
        print('----------------------------------')
        for i,obj in enumerate(robot.objects):
            # if obj['p_rt'] is not None:
            #     print(i,obj['class'],'{:.2f}'.format(obj['p_rt'][0]),'{:.2f}'.format(obj['p_xy'][0]),'{:.2f}'.format(obj['p_xy'][1]))
            if obj['p_xy_map'] is not None:
                print(i,'{:.2f}'.format(obj['p_xy_map'][0]),'{:.2f}'.format(obj['p_xy_map'][1]))
        publish_rviz(robot.objects)
        rate.sleep()
import rospy
# import ros_numpy
import actionlib
import datetime
import cv2
import numpy as np

from collections import deque

import math

from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from obstacle_detector.msg import Obstacles

from tf.listener import TransformListener, Transformer

from std_msgs.msg import Float32MultiArray # -> SSD object topic

from occupancy_check import OccupancyCheck

PATH = '/root/Desktop/'

class RobotController:
    # pass topic names
    def __init__(self, topics):
        # rospy.init_node('robot_controller')
        self.pose = None
        self.scan = None
        self.state = 'go'
        self.objects = []
        self.faceboxes = []
        self.facelands = []
        self.scan_objs = []
        self.bridge = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)

        # camera parameter TODO: get from json or yaml or camera_info topic 
        self.focal_length = 0.3
        self.img_W = 640
        self.img_H = 480
        self.FOV_W = 80
        self.FOV_H = 60

        self.tf_listener = TransformListener()

        self.occupancy_check = OccupancyCheck(True)

        # Sub
        self.pose_sub = rospy.Subscriber(topics['pose'], PoseStamped, self.pose_callback)
        # self.image_sub = rospy.Subscriber(topics['image'],Image,self.image_callback)
        self.image_sub = rospy.Subscriber(topics['compressed_image'],CompressedImage,self.compressedimage_callback)
        # self.depth_sub = rospy.Subscriber(topics['depth'],PointCloud2,self.depth_callback)

        self.scan_sub = rospy.Subscriber(topics['scan'],LaserScan,self.scan_callback)
        # self.navres_sub = rospy.Subscriber(topics['nav_r'], MoveBaseActionResult, self.nav_callback)
        self.objects_sub = rospy.Subscriber(topics['obj'], Float32MultiArray, self.objects_callback)
        self.scan_obj_sub = rospy.Subscriber(topics['obstacles'], Obstacles, self.obstacles_callback)

        self.faces_sub = rospy.Subscriber(topics['face'], Float32MultiArray, self.faces_callback)
        # Pub
        self.twist_pub = rospy.Publisher(topics['twist'], Twist, queue_size=1)

        # Action client
        # self.client = actionlib.SimpleActionClient(topics['nav_s'], MoveBaseAction)
        # self.client.wait_for_server()

    # update pose
    def pose_callback(self, msg):
        self.pose = msg
        # print(self.pose)
    
    # update scan
    def scan_callback(self, msg):
        self.scan = msg
        occupancy_data = self.occupancy_check.execute(scan=self.scan, r_max=4)
        # for i,data in enumerate(occupancy_data):
        #     print(i,data)
    
    # # waiting for navigation result
    # def nav_callback(self, msg):
    #     if msg.status.status == 3 and self.state == 'moving':
    #         self.state = 'reached'

    # update image
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        image = cv2.resize(self.image, (int(self.image.shape[1]/2), int(self.image.shape[0]/2)))
        cv2.imshow("camera", image)
        cv2.waitKey(2)
    
    # update image
    def compressedimage_callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        # cv2.imshow('camera', self.cv_image)
        # cv2.waitKey(2)
    
    # def depth_callback(self, msg):
    #     pass
        # xyz_array = ros_numpy.point_cloud2.get_xyz_points(msg)
        # print(xyz_array)
        # for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        #     pt_x = point[0]
        #     pt_y = point[1]
        #     pt_z = point[2]
        #     print(pt_x,pt_y,pt_z)
        #     break

    # obj['bb'][0] : ymin
    # obj['bb'][1] : xmin
    # obj['bb'][2] : ymax
    # obj['bb'][3] : xmax
    # update ssd objects
    def objects_callback(self, msg):
        yolov5_result = np.array(msg.data)
        # print(yolov5_result)
        self.objects = []
        objs = yolov5_result.reshape(-1,6)
        print(len(objs))
        for bb in objs:
            obj = self.detect_object(bb)
            self.objects.append(obj)
            # print(' o'+str(i)+':',theta_o)
    
    # recognize object position
    ## TODO: test
    def detect_object(self, bb):
        obj = {'class':int(bb[0]), 'bb':list(bb[1:5]), 'conf':float(bb[5]), 'p_rt':None, 'p_xy':None, 'p_xy_map':None,'by':[]} # definition of 'object' (p_rt for r,theta)
        if obj['class'] == 0:
            r_xmin = 0.5 - obj['bb'][1]
            r_xmax = 0.5 - obj['bb'][3]
            theta_max = self.rad_in_img(r_xmin)
            theta_min = self.rad_in_img(r_xmax)
            theta_c = (theta_min+theta_max)/2
            if self.scan is not None:
                r_list = self.get_scan(math.degrees(theta_min),math.degrees(theta_max))
                clusters = self.scan_cluster(r_list)
                # for c in clusters:
                #     print('{:.2f}'.format(np.mean(c)))
                clusters.sort(key=lambda x:np.mean(x['ranges'])) # nearest cluster is the target
                r = np.mean(clusters[0]['ranges']) 
                obj['p_rt'] = (r,theta_c)
                obj['p_xy'] = (r*math.cos(theta_c),r*math.sin(theta_c))
                obj['by'].append('scan')
            try:
                self.tf_listener.waitForTransform('/velodyne', "/map", rospy.Time.now(), rospy.Duration(2.0))
                p_xy = PointStamped()
                p_xy.point = Point(obj['p_xy'][0],obj['p_xy'][1],0.)
                p_xy.header.frame_id = '/velodyne'
                # p_xy.header.stamp = rospy.Time.now()
                p_map = self.tf_listener.transformPoint("/map", p_xy)
                obj['p_xy_map'] = (p_map.point.x,p_map.point.y)
            except Exception as e:
                print e
            print('?')
        return obj
    
    # convert x in image to angle
    def rad_in_img(self, img_x):
        x = img_x*2*self.focal_length*math.tan(math.radians(self.FOV_W)/2)
        return math.atan2(x,self.focal_length)
    
    def get_scan(self, deg_min, deg_max):
        incr = self.scan.angle_increment
        a_min = self.scan.angle_min
        scan_d = deque(self.scan.ranges)
        scan_d.rotate(int((a_min-math.radians(deg_min))/incr))
        scan_list = list(scan_d)[:int(math.radians(deg_max-deg_min)/incr)]
        return scan_list
    
    # clustering scan points
    def scan_cluster(self,ranges):
        DIFF_THRESHOLD = 1.0
        ranges = [r for r in ranges if not (math.isinf(r) or math.isnan(r))]
        objs = []
        for i,r_i in enumerate(ranges):
            if i==0:
                objs.append({'angles':[i],'ranges':[r_i]})
            else:
                if abs(r_i-ranges[i-1]) < DIFF_THRESHOLD: # 
                    objs[-1]['ranges'].append(r_i)
                    objs[-1]['angles'].append(i)
                else:
                    objs.append({'angles':[i],'ranges':[r_i]})
        return objs

    # update obstacles
    def obstacles_callback(self,msg):
        self.scan_objs = []
        for c in msg.circles:
            self.scan_objs.append({
                'p_xy':(c.center.x,c.center.y),
                'p_rt':(math.sqrt(c.center.x**2+c.center.y**2),math.atan2(c.center.y,c.center.x))
            })
    
    # update face detect
    def faces_callback(self, msg):
        face_result = np.array(msg.data)
        # print(box_result)
        
        self.faceboxes = []
        self.facelands = []
        faces = face_result.reshape(-1,15)
        for face in faces:
            self.faceboxes.append({'box':list(face[0:4]), 'conf':float(face[4])})
            self.facelands.append({'land':list(face[5:15])})
        self.judge_center()
    
    # judge face
    def judge_face(self):
        counter = 0
        min_size = 0
        size = 0
        self.index = -1
        for facebox in self.faceboxes:
            xyxy_fbb = [facebox['box'][0], facebox['box'][1], facebox['box'][0]+facebox['box'][2], facebox['box'][1]+facebox['box'][3]]
            #print(xyxy_fbb)
            size = facebox['box'][2]*facebox['box'][3]
            for obj in self.objects:
                if(obj['class']==0):
                   xyxy_bb = [obj['bb'][1]*self.img_W, obj['bb'][0]*self.img_H, obj['bb'][3]*self.img_W, obj['bb'][2]*self.img_H]
                   if(xyxy_fbb[0]>=xyxy_bb[0])and(xyxy_fbb[1]>=xyxy_bb[1])and(xyxy_fbb[2]<=xyxy_bb[2])and(xyxy_fbb[3]<=xyxy_bb[3]):
                      # print(xyxy_fbb)
                      #print(xyxy_bb)
                      if(min_size<=size):
                         min_size = size
                         self.index = counter
            counter += 1
        return self.index


    # judge center
    def judge_center(self):
        th = 1 #thleth pixcel
        index = self.judge_face()
        result = -1
        #print(index)
        if ((index>=0)and(index<len(self.facelands))):
            nose_x = self.facelands[index]['land'][4]
            leye_x = self.facelands[index]['land'][0]
            reye_x = self.facelands[index]['land'][2]
            ceye_x = (leye_x + reye_x)/2
            #print("nose:{}".format(nose_x))
            #print("leye:{}".format(leye_x)) 
            #print("reye:{}".format(reye_x)) 
            #print("ceye:{}".format(ceye_x))
            #print("check:{}".format(self.faceboxes[index]['box']))
            diff = nose_x-ceye_x
            if(diff>th):
               print("face is right!")
               result = 1
            elif(-th<=diff<=th):
               print("face is center!")
               result = 2
            elif(diff<-th):
               print("face is left")
               result = 3
        return result
    
    # put 0 to twist
    def stop(self):
        msg = Twist() # 0
        self.twist_pub.publish(msg)

    # rotation in vel
    def rotate(self, vel=0.3):
        msg = Twist()
        msg.angular.z = vel
        self.twist_pub.publish(msg)

    # go forward or go back in vel
    def translate(self, vel=0.2):
        msg = Twist()
        msg.linear.x = vel
        self.twist_pub.publish(msg)
        
    def go_and_rotate(self, linear_vel=0.2, angular_vel=0.3):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.twist_pub.publish(msg)

    def roomba_walk(self):
        # print(self.scan)
        if self.scan is None:
            return
        forward_left = self.get_scan(20,40)
        forward_right = self.get_scan(-40.,-20.)
        left = np.mean([r for r in forward_left if not (math.isinf(r) or math.isnan(r))])
        right = np.mean([r for r in forward_right if not (math.isinf(r) or math.isnan(r))])
        # print 'range:', left, right

        if left > 2 and right > 2:
            self.translate(0.2)
            print('go')
        elif left <= 2:
            self.rotate(-0.3)
            print('right')
        elif right <= 2:
            self.rotate(0.3)
            print('left')
    
    # another patrolling algorithm
    # def explore(self):
    #     print(self.state)
    #     if self.state == 'go':
    #         something = []            
    #         for obj in self.scan_objs:
    #             if obj['p_rt'][0] < 5.:
    #                 something.append((obj['p_rt'][0],obj))
    #         if something:
    #             print(something)
    #             something.sort(key=lambda tup: tup[0])
    #             target = something[0][1]
    #             if target['p_rt'] is not None:
    #                 if abs(target['p_rt'][1]) < math.degrees(5.):
    #                     self.state = 'approach'
    #                 elif target['p_rt'][1] < 0.:
    #                     self.rotate(0.2)
    #                 elif target['p_rt'][1] > 0.:
    #                     self.rotate(-0.2)
    #         else:
    #             self.translate(0.2)
    #     elif self.state == 'approach':
    #         self.translate(0.2)
    #         if self.objects:
    #             self.state = 'register'         
    #     elif self.state == 'register':
    #         self.stop()
    #         self.state = 'rotate'
    #         self.rotation_angle = math.radians(45.)
    #         return self.objects
    #     elif self.state == 'rotate':
    #         self.rotate(0.3)
    #         self.rotation_angle -= 0.3*0.01
    #         if self.rotation_angle < 0.:
    #             self.state = 'go'
    #     return None

    def detect_target(self, target=0):#person:0
        CONFIDENSE_THRESHOLD = 0.5
        DISTANCE_THRESHOLD = 30.
        targets = []
        for obj in self.objects:
            if obj['class'] == target:
                if obj['p_rt'] is not None and obj['conf'] > CONFIDENSE_THRESHOLD and obj['p_rt'][0] < DISTANCE_THRESHOLD:
                    targets.append(obj)
        return targets
    

    # obj['bb'][0] : ymin
    # obj['bb'][1] : xmin
    # obj['bb'][2] : ymax
    # obj['bb'][3] : xmax
    def approach_object(self, target=0):
        res = 'approaching'
        if self.scan is not None:
            RANGE = 30
            RANGE2 = 60
            # print 'len:', len(self.scan.ranges)
            incr = self.scan.angle_increment
            a_min = self.scan.angle_min
            forward_left = []
            forward_right = []
            left_max = 0
            right_max = 0
            for i,s in enumerate(self.scan.ranges):
                degree = math.degrees(i*incr+a_min)
                if s == np.inf:
                    pass
                elif RANGE-5 < degree < RANGE+5:
                    forward_left.append(s)
                elif -RANGE-5 < degree < -RANGE+5:
                    forward_right.append(s)
                if 0 < degree < RANGE2 and s <= 2:
                    left_max = max(left_max, abs(degree))
                elif -RANGE2 < degree < 0 and s <= 2:
                    right_max = max(right_max, abs(degree))

            left = np.mean(forward_left)
            right = np.mean(forward_right)
            # print 'range:', left, right

            if left > 2 and right >2:
                #self.translate(0.2)
                print('go')
            elif left <= 2 or right <= 2:
                if left_max > right_max:
                    self.rotate(-0.3)
                    print('right')
                else:
                    self.rotate(0.3)
                    print('left')
                return res

        lost = True
        ymin = 10000
        xmin = 10000
        ymax = 0
        xmax = 0
        for obj in self.objects:
            if obj['class'] == target and obj['conf'] > 0.5:
                lost = False                
                if obj['bb'][0] < ymin: ymin = obj['bb'][0]
                if obj['bb'][1] < xmin: xmin = obj['bb'][1]
                if obj['bb'][2] > ymax: ymax = obj['bb'][2]
                if obj['bb'][3] > xmax: xmax = obj['bb'][3]

        if lost:
            res = 'lost'
        else:
            center = ((xmin+xmax)/2,(ymin+ymax)/2)
            height = ymax - ymin
            if 0.45 < center[0] < 0.55:
                if 0.5 < height < 0.9 and ymax > 0.5:
                    self.stop()
                    res = 'reached'
                elif height <= 0.5:
                    self.translate(vel=0.3)
                elif height >= 0.9:
                    self.translate(vel=-0.3)
            elif center[0] < 0.5:
                self.go_and_rotate(linear_vel=0.3, angular_vel=0.2)
            else:
                self.go_and_rotate(linear_vel=0.3, angular_vel=-0.2)
        return res
        
    # # send a navigation goal
    # def send_goal(self, goal):
    #     goal_msg = self.goal_pose(goal)
    #     self.client.send_goal(goal_msg)
    #     self.state = "moving"

    # # convert pose to navigation goal
    # def goal_pose(self, pose):
    #     goal_pose = MoveBaseGoal()
    #     goal_pose.target_pose.header.frame_id = 'map'
    #     goal_pose.target_pose.pose.position.x = pose[0][0]
    #     goal_pose.target_pose.pose.position.y = pose[0][1]
    #     goal_pose.target_pose.pose.orientation.x = pose[1][0]
    #     goal_pose.target_pose.pose.orientation.y = pose[1][1]
    #     goal_pose.target_pose.pose.orientation.z = pose[1][2]
    #     goal_pose.target_pose.pose.orientation.w = pose[1][3]
    #     return goal_pose
    
    # get robot state
    def get_state(self):
        return self.state
    
    # shoot a photo
    # obj['bb'][0] : ymin
    # obj['bb'][1] : xmin
    # obj['bb'][2] : ymax
    # obj['bb'][3] : xmax
    # def shoot(self,target=0):
    #     ymin = 10000
    #     xmin = 10000
    #     ymax = 0
    #     xmax = 0
    #     targets = []
    #     index = self.judge_face()
    #     for obj in self.objects:
    #         if obj['class'] == target:
    #             targets.append(obj)
    #             if obj['bb'][0] < ymin: ymin = obj['bb'][0]
    #             if obj['bb'][1] < xmin: xmin = obj['bb'][1]
    #             if obj['bb'][2] > ymax: ymax = obj['bb'][2]
    #             if obj['bb'][3] > xmax: xmax = obj['bb'][3]
    #     center = ((xmin+xmax)/2,(ymin+ymax)/2)
    #     # height = ymax - ymin
    #     good_pic = False
    #     if (0.45 < center[0] < 0.55) and 0.1 < ymin and ymax < 0.9:
    #         good_pic = True
    #         now = datetime.datetime.now()
    #         cv2.imwrite(PATH+'photo_' + now.strftime('%Y%m%d%H%M%S' + '.jpg'), self.cv_image)
    #         if(self.index>=0):
    #            x1 = self.faceboxes[self.index]['box'][0]-10
    #            y1 = self.faceboxes[self.index]['box'][1]-10
    #            x2 = self.faceboxes[self.index]['box'][0]+self.faceboxes[self.index]['box'][2]+10
    #            y2 = self.faceboxes[self.index]['box'][1]+self.faceboxes[self.index]['box'][3]+10
    #            if(x1 <= 0):
    #               x1 = 0
    #            elif(x1 >=self.img_W):
    #               x1 = self.img_W
    #            if(y1 <= 0):
    #               y1 = 0
    #            elif(y1 >=self.img_H):
    #               y1 = self.img_H    
    #            if(x2 <= 0):
    #               x2 = 0
    #            elif(x2 >=self.img_W):
    #               x2 = self.img_W
    #            if(y2 <= 0):
    #               y2 = 0
    #            elif(y2 >=self.img_H):
    #               y2 = self.img_H  
    #            #print(self.cv_image.shape)
    #            cv2.imwrite(PATH+'photo_' + now.strftime('%Y%m%d%H%M%S' +'_face'+'.jpg'), self.cv_image[int(y1):int(y2),int(x1):int(x2),:])
    #     else:
    #         self.translate(-0.2)
    #     return good_pic, targets
    
    def shoot(self,target=0):
        ymin = 10000
        xmin = 10000
        ymax = 0
        xmax = 0
        targets = []
        distances = []
        for obj in self.objects:
            if obj['class'] == target and obj['conf'] > 0.5 and obj['bb'][2] - obj['bb'][0] > 0.2:
                targets.append(obj)
                distances.append(obj['p_rt'][0])
                if obj['bb'][0] < ymin: ymin = obj['bb'][0]
                if obj['bb'][1] < xmin: xmin = obj['bb'][1]
                if obj['bb'][2] > ymax: ymax = obj['bb'][2]
                if obj['bb'][3] > xmax: xmax = obj['bb'][3]
        center = ((xmin+xmax)/2,(ymin+ymax)/2)
        height = ymax - ymin

        if distances:
            distance = sum(distances) / len(distances)
        else:
            distance = None
        #print('distance: ', distance)

        """target_face = {'idx': -1, 'box': [0, 0, 0, 0], 'conf': 0}
        for i, facebox in enumerate(self.faceboxes):
            if facebox['conf'] > target_face['conf']:
                facebox['idx'] = i
                target_face = facebox
        if target_face['idx'] != -1:
            target_land = self.facelands[target_face['idx']]
            face_center_x = (target_face['box'][0] + target_face['box'][2] / 2)"""
            #print("target_land['land'][0]: ", target_land['land'][0])
            #print("target_land['land'][2]: ", target_land['land'][2])
            #print("face_center_x: ", face_center_x)
            #print((target_land['land'][0] < face_center_x) ^ (target_land['land'][6] < face_center_x))
        judge = self.judge_center()
        #print(judge)

        if self.scan is not None:
            RANGE = 30
            RANGE2 = 45
            # print 'len:', len(self.scan.ranges)
            incr = self.scan.angle_increment
            a_min = self.scan.angle_min
            forward_left = []
            forward_right = []
            left_max = 0
            right_max = 0

            #print(min(min(self.scan.ranges[:5]), min(self.scan.ranges[-5:])))
            if min(min(self.scan.ranges[:5]), min(self.scan.ranges[-5:])) < 1.5:
                back = False
            else:
                back = True

            for i,s in enumerate(self.scan.ranges):
                degree = math.degrees(i*incr+a_min)
                if s == np.inf:
                    pass
                elif RANGE-5 < degree < RANGE+5:
                    forward_left.append(s)
                elif -RANGE-5 < degree < -RANGE+5:
                    forward_right.append(s)
                if 0 < degree < RANGE2 and s <= 2:
                    left_max = max(left_max, abs(degree))
                elif -RANGE2 < degree < 0 and s <= 2:
                    right_max = max(right_max, abs(degree))

            left = np.mean(forward_left)
            right = np.mean(forward_right)
            # print 'range:', left, right

            if left > 2 and right >2:
                #self.translate(0.2)
                print('go')
            elif left <= 2 or right <= 2:
                if left_max > right_max:
                    self.rotate(-0.3)
                    print('right')
                else:
                    self.rotate(0.3)
                    print('left')
                return False, targets

        print(self.shoot_state)
        if self.shoot_state is None:
            #print('center: ', center[0])
            if center[0] > 1:
                return False, targets
            elif center[0] < 0.4:
                print('turn left')
                self.rotate(0.3)
                return False, targets
            elif center[0] > 0.6:
                print('turn right')
                self.rotate(-0.3)
                return False, targets
            #elif height <= 0.5:
             #   self.translate(0.3)
              #  return False, targets"""
            elif (ymin < 0.1 or ymax > 0.9) and back:
                print('back')
                self.translate(-0.2)
                return False, targets
            #elif target_face['idx'] != -1:
            elif judge > 0:
                #if not (target_land['land'][0] <= face_center_x) ^ (target_land['land'][2] < face_center_x):
                if judge != 2:
                    self.stop()
                    self.shoot_state = 'rotate1'
                    self.shoot_count = 0
                    self.target_distance = distance
                    #if target_land['land'][0] < face_center_x:
                    if judge == 3:
                        self.shoot_direction = 1
                        #print('aleft')
                    else:
                        self.shoot_direction = -1
                        #print('aright')
                    return False, targets
                else:
                    now = datetime.datetime.now()
                    cv2.imwrite(PATH+'photo_' + now.strftime('%Y%m%d%H%M%S' + '.jpg'), self.cv_image)
                    if(self.index>=0):
                        x1 = self.faceboxes[self.index]['box'][0]-10
                        y1 = self.faceboxes[self.index]['box'][1]-10
                        x2 = self.faceboxes[self.index]['box'][0]+self.faceboxes[self.index]['box'][2]+10
                        y2 = self.faceboxes[self.index]['box'][1]+self.faceboxes[self.index]['box'][3]+10
                        if(x1 <= 0):
                            x1 = 0
                        elif(x1 >=self.img_W):
                            x1 = self.img_W
                        if(y1 <= 0):
                            y1 = 0
                        elif(y1 >=self.img_H):
                            y1 = self.img_H    
                        if(x2 <= 0):
                            x2 = 0
                        elif(x2 >=self.img_W):
                            x2 = self.img_W
                        if(y2 <= 0):
                            y2 = 0
                        elif(y2 >=self.img_H):
                            y2 = self.img_H  
                        #print(self.cv_image.shape)
                        cv2.imwrite(PATH+'photo_' + now.strftime('%Y%m%d%H%M%S' +'_face'+'.jpg'), self.cv_image[int(y1):int(y2),int(x1):int(x2),:])
                    return True, targets
            else:
                self.stop()
                self.shoot_state = 'rotate1'
                self.shoot_count = 0
                self.target_distance = distance
                self.shoot_direction = 1
                print('left')
                return False, targets
        elif self.shoot_state == 'rotate1':
            if self.shoot_count < round(130 * 78.75 / 360): # 67.5
                self.rotate(self.shoot_direction * 2 * np.pi / 10)
            else:
                self.stop()
                self.shoot_state = 'straight'
                self.shoot_count = 0
            return False, targets
        elif self.shoot_state == 'straight':
            goal = 4 * 0.39 #0.7655
            if self.shoot_count < round(goal / 0.2 * 10):
                self.translate(0.2)
            else:
                self.stop()
                self.shoot_state = 'rotate2'
                self.shoot_count = 0
            return False, targets
        elif self.shoot_state == 'rotate2':
            if self.shoot_count < round(130 * (180 - 78.75) / 360):
                self.rotate(self.shoot_direction * 2 * np.pi / 10 * -1)
            else:
                self.stop()
                self.shoot_state = None
                self.shoot_count = 0
            return False, targets
        else:
            return False, targets
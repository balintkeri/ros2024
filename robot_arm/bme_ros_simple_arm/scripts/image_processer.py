#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32 # Message type used in the node
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import io, json, os



def camera_to_coordinate(coordinates):
     x = (121-coordinates[1])/150+0.5
     y = (161-coordinates[0])/145
     return [round(x,2),round(y,2)]

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.process_image = False

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        
        # Subscribe to the trigger topic
        self.trigger_sub = rospy.Subscriber('/publisher_topic', Int32, self.trigger_callback)

        self.trigger_sub = rospy.Subscriber('/publisher_topic', Int32, self.trigger_callback)

    def image_callback(self, msg):
        if self.process_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                rospy.loginfo("Displaying an image!")
                

                I = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                #A robotkar kitakarni
                black_color = (0, 0, 0)
                thickness = -1
                cv2.rectangle(I, (130, 160), (190, 240), black_color, thickness)
                cv2.rectangle(I, (130, 160), (190, 240), black_color, thickness)
                cv2.rectangle(I, (0, 0),     (320, 30), black_color, thickness)
                cv2.rectangle(I, (0, 30), (50, 240), black_color, thickness)
                cv2.rectangle(I, (270, 30), (320, 240), black_color, thickness)
                cv2.rectangle(I, (80, 35),     (120, 65), black_color, thickness)

                I_green = I.copy()
                I_green[:, :, 0] = 0
                I_green[:, :, 2] = 0
                I_green_gray = cv2.cvtColor(I_green, cv2.COLOR_BGR2GRAY)
                I_red = I.copy()
                I_red[:, :, 1] = 0
                I_red[:, :, 2] = 0
                I_red_gray = cv2.cvtColor(I_red, cv2.COLOR_BGR2GRAY)
                I_blue = I.copy()
                I_blue[:, :, 0] = 0
                I_blue[:, :, 1] = 0
                I_blue_gray = cv2.cvtColor(I_blue, cv2.COLOR_BGR2GRAY)

                kernel_size = (2, 2)
                kernel = np.ones(kernel_size, np.uint8)

                

                ret,I_blue_gray_thresh = cv2.threshold(I_blue_gray,40,255,cv2.THRESH_BINARY)
                ret,I_red_gray_thresh = cv2.threshold(I_red_gray,25,255,cv2.THRESH_BINARY)
                ret,I_green_gray_thresh = cv2.threshold(I_green_gray,140,255,cv2.THRESH_BINARY)

                # Perform dilation
                I_blue_gray_thresh = cv2.erode( cv2.dilate(I_blue_gray_thresh, kernel, iterations=2) , kernel, iterations=2)
                I_green_gray_thresh = cv2.erode( cv2.dilate(I_green_gray_thresh, kernel, iterations=2) , kernel, iterations=2)
                I_red_gray_thresh = cv2.erode( cv2.dilate(I_red_gray_thresh, kernel, iterations=2) , kernel, iterations=2)
            

                
                I_kocka_blue = I.copy()
                blue_contours, _ = cv2.findContours(I_blue_gray_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(I_kocka_blue, blue_contours, -1, (255, 0, 0), 1)

                I_kocka_green = I.copy()
                green_contours, _ = cv2.findContours(I_green_gray_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(I_kocka_green, green_contours, -1, (0, 0, 255), 1)

                I_kocka_red = I.copy()
                red_contours, _ = cv2.findContours(I_red_gray_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(I_kocka_red, red_contours, -1, (0, 255, 0), 1)

                contours = {
                    'red': red_contours,
                    'green' : green_contours,
                    'blue': blue_contours
                }

                cubes = {
                    'red': [],
                    'green' : [],
                    'blue': []
                }

                for color in cubes:
                    points = []
                    for i, contour in enumerate(contours[color]):
                        X, Y = [], []
                        for point in contour:
                            x, y = point[0]
                            X.append(x)
                            Y.append(y)
                        points.append( camera_to_coordinate(
                            [
                                int(sum(X) / len(X)),
                                int(sum(Y) / len(Y))]
                            ))
                    cubes[color] = points
                self.process_image = False
                rospy.loginfo(cubes)
                json_path = os.path.dirname(os.path.abspath(__file__)) + '/position.json'
                with io.open(json_path, 'w', encoding='utf-8') as f:
                    f.write(json.dumps(cubes, ensure_ascii=False))

            except CvBridgeError as e:
                rospy.logerr(e)


    def trigger_callback(self, msg):
        rospy.loginfo("Capture trigger received!")
        self.process_image = True if msg.data%2 == 0 else False
       


rospy.init_node('image_subscriber', anonymous=True)
image_subscriber = ImageSubscriber()
rospy.loginfo("Image subscriber node started. Waiting for trigger...")
rospy.spin()




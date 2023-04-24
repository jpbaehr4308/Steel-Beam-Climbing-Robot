#MIT License
#
#Copyright (c) 2023 jpbaehr4308
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

#!/usr/bin/env python3
import jetson.inference
import jetson.utils
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int32


cam = jetson.utils.videoSource("/dev/video0")
disp = jetson.utils.videoOutput("display://0")
net = jetson.inference.detectNet(argv=['--model=/home/jb/catkin_ws/src/jetson-inference/python/training/detection/ssd/models/beam-v2/ssd-mobilenet.onnx','--labels=/home/jb/catkin_ws/src/jetson-inference/python/training/detection/ssd/models/beam-v2/labels.txt', '--input-blob=input_0','--output-cvg=scores', '--output-bbox=boxes'], threshold=0.75)
net2 = jetson.inference.detectNet(argv=['--model=/home/jb/catkin_ws/src/jetson-inference/python/training/detection/ssd/models/bell/ssd-mobilenet.onnx','--labels=/home/jb/catkin_ws/src/jetson-inference/python/training/detection/ssd/models/bell/labels.txt', '--input-blob=input_0','--output-cvg=scores', '--output-bbox=boxes'], threshold=0.75)

#cap=cv2.VideoCapture("csi://0")
detect = 0
approach = 0
def outputCallback(data):
        global detect
        detect = data.data
def outputCallback2(data2):
        global detect
        approach = data2.data

while not rospy.is_shutdown():
    #while disp.IsStreaming():
    pub = rospy.Publisher('data',Int8,queue_size=10)
    sub = rospy.Subscriber('detect', Int32, outputCallback)
    sub2 = rospy.Subscriber('approach', Int32, outputCallback2)
    rospy.init_node('jetson', anonymous=True)

    rate=rospy.Rate(8)
    if detect == 0 and approach == 0:
        img=cam.Capture()
        detections = net.Detect(img)
        disp.Render(img)
        disp.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        flag = 0

        #Find object
        for det in detections:
            center_x = int(det.Left + det.Right) // 2
            center_y = int(det.Top + det.Bottom) // 2
            #jetson.utils.cudaDrawCircle(img, (center_x, center_y,), 5,(0,255,0,200))
            flag = 1

        if(flag == 1):
            if center_x < img.width // 2:
                #print("Move Left")  
                #print("Detect: %s", detect)
                rospy.loginfo(0)
                pub.publish(0)
                rate.sleep()
            if abs(center_x - img.width // 2) <= 35:
                #print("Center")  
                #print("Detect: %s", detect)
                rospy.loginfo(1)
                pub.publish(1)
                rate.sleep()
            if center_x > img.width // 2:
                #print("Move Right")  
                #print("Detect: %s", detect)
                rospy.loginfo(2)
                pub.publish(2)
                rate.sleep()

        if detect == 0 and approach == 1:
            img=cam.Capture()
            detections = net.Detect(img)
            disp.Render(img)
            disp.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
            flag = 0
            print("Approach: %s", approach)
        #Find object
        for det in detections:
            center_x = int(det.Left + det.Right) // 2
            center_y = int(det.Top + det.Bottom) // 2
            #jetson.utils.cudaDrawCircle(img, (center_x, center_y,), 5,(0,255,0,200))
            flag = 1

        if(flag == 1):
            if center_x < img.width // 2:
                #print("Move Left")  
                #print("Detect: %s", detect)
                rospy.loginfo(0)
                pub.publish(0)
                rate.sleep()
            if abs(center_x - img.width // 2) <= 10:
                #print("Center")  
                #print("Detect: %s", detect)
                rospy.loginfo(1)
                pub.publish(1)
                rate.sleep()
            if center_x > img.width // 2:
                #print("Move Right")  
                #print("Detect: %s", detect)
                rospy.loginfo(2)
                pub.publish(2)
                rate.sleep()
            
        if(flag == 0):
            #print("No object")  #No object
            #print("Detect: %s", detect)
            rospy.loginfo(2)
            pub.publish(2)
            rate.sleep()

    if detect == 1:
        img2=cam.Capture()
        detections2 = net2.Detect(img2)
        disp.Render(img)
        disp.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        flag = 0

        #Find object
        for det2 in detections2:
            center_x = int(det2.Left + det2.Right) // 2
            center_y = int(det2.Top + det2.Bottom) // 2
            #jetson.utils.cudaDrawCircle(img2, (center_x, center_y,), 5,(0,255,0,200))
            flag = 1

        if(flag == 1):
            if center_x < img2.width // 2:
                #print("Move Left")  
                #print("Detect: %s", detect)
                rospy.loginfo(5)
                pub.publish(5)
                rate.sleep()
            if center_x > img2.width // 2:
                #print("Move Right")  
                #print("Detect: %s", detect)
                rospy.loginfo(4)
                pub.publish(4)
                rate.sleep()
            if abs(center_x - img2.width // 2) <= 5:
                #print("Center")  
                #print("Detect: %s", detect)
                rospy.loginfo(6)
                pub.publish(6)
                rate.sleep()
        if(flag == 0):
            #print("No object")  #No object
            #print("Detect: %s", detect)
            rospy.loginfo(2)
            pub.publish(2)
            rate.sleep()
            

    

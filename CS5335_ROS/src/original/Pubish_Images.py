import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    
    
    rospy.init_node("videoTracker")
    br = CvBridge()
    with np.load("50_images.npz") as data:
	 im = Image()
         images = data["images"]
         img = images[1]
      
         #im.image.data = img
         #im.image.header
         #im.image.height = img.shape(1)
         #im.image.width = img.shape(2)
         #im.image.encoding = 
    image_published = rospy.Publisher('/webcam/image_raw', Image, queue_size = 1)
    i=0
    while (i<1000000):
         image_published.publish(br.cv2_to_imgmsg(img,encoding='bgr8'))
         i=i+1

if __name__=='__main__':
    main()

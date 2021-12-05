#!/usr/bin/env python
# simple script to publish multiple images from a list of file 
import rospy
import rospkg
import time
import cv2
import sensor_msgs.msg

def callback(self):
    """ Convert a image to a ROS compatible message
        (sensor_msgs.Image).
    """
    global publishers, image_paths

    for i in range(len(image_paths)):
        img = cv2.imread(image_paths[i], cv2.IMREAD_UNCHANGED)


        rosimage = sensor_msgs.msg.Image()
        if img.dtype.itemsize == 2:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr16'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra16'
            else:
                rosimage.encoding = 'mono16'
        if img.dtype.itemsize == 1:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr8'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra8'
            else:
                rosimage.encoding = 'mono8'
    #    print "Encoding: ", rosimage.encoding

        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0]
        rosimage.data = img.tostring()
        rosimage.header.stamp = rospy.Time.now()
        rosimage.header.frame_id = 'map'
        
        publishers[i].publish(rosimage)



#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publishers, image_paths
    rospack = rospkg.RosPack()
    rospy.init_node('image_publisher')

    # if not rospy.has_param("image_list"):
    #     raise AssertionError("No image_list parameter found on parameter server")
    image_paths = rospy.get_param("image_list")
    print("[ImagePublisher]: Loading image from : ", image_paths)
    topic_names = rospy.get_param("topic_list")
    # image_paths = [rospack.get_path('traversability_pipeline') + '/data/edited/elevation.png',
                    #  rospack.get_path('traversability_pipeline') + '/data/edited/occupancy.png']
    # topic_names = ["image/elevation", "image/occupancy"]

    publishers = []

    for topic_name in topic_names:
        publishers.append(rospy.Publisher(topic_name, sensor_msgs.msg.Image, queue_size=10))

    rospy.Timer(rospy.Duration(2), callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass

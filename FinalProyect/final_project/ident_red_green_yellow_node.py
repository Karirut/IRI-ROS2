import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import traceback
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Int32

class final_project(Node):
    def __init__(self):
        super().__init__('ident_red_green_yellow_node')
        self.video_source_subscription = self.create_subscription(Image, 'video_source/raw',self.videoSource_callback, rclpy.qos.qos_profile_sensor_data)

        self.red_density_pub = self.create_publisher(Float32, 'red_density', rclpy.qos.qos_profile_sensor_data)

        self.green_density_pub = self.create_publisher(Float32, 'green_density', rclpy.qos.qos_profile_sensor_data)

        self.yellow_density_pub = self.create_publisher(Float32, 'yellow_density', rclpy.qos.qos_profile_sensor_data)

        #self.msk = self.create_publisher(Image, 'msk', rclpy.qos.qos_profile_sensor_data)

        self.bridge = CvBridge()

        self.img = Image()

        #self.img_msg_color = Image()

        self.red_density = Float32()
        self.green_density = Float32()
        self.yellow_density = Float32()

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New node succesfully initializated')


    def videoSource_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def timer_callback(self):
        try:
            self.get_logger().info('Imagen recibida :D')

            img_flipped = cv2.flip(self.img, -1)
            img_received = img_flipped[0:720, 0:1280]

            hsv_img = cv2.cvtColor(img_received, cv2.COLOR_RGB2HSV)

            #ROJO
            #HUE CHANNEL ROJO
            red_lower_mask = hsv_img[:,:,0] >= 150
            red_higher_mask = hsv_img[:,:,0] <= 255 #50

            #TRANSPARENCY CHANNEL RED
            red_lower_saturation_mask = hsv_img[:,:,1] >= 125

            red_mask_union = red_lower_mask*red_higher_mask*red_lower_saturation_mask

            #RED MASK
            red_red = img_received[:,:,0]*red_mask_union
            green_red = img_received[:,:,1]*red_mask_union
            blue_red = img_received[:,:,2]*red_mask_union

            red_masked = np.dstack((red_red,green_red,blue_red))


            #VERDE
            #HUE CHANNEL VERDE
            green_lower_mask = hsv_img[:,:,0] > 40
            green_higher_mask = hsv_img[:,:,0] < 95

            #TRANSPARENCY CHANNEL GREEN
            green_lower_saturation_mask = hsv_img[:,:,1] > 100

            green_mask_union = ((green_lower_mask*green_higher_mask))*green_lower_saturation_mask

            #GREEN MASK
            red_green = img_received[:,:,0]*green_mask_union
            green_green = img_received[:,:,1]*green_mask_union
            blue_green = img_received[:,:,2]*green_mask_union

            green_masked = np.dstack((red_green,green_green,blue_green))


            #AMARILLO
            #HUE CHANNEL AMARILLO
            yellow_lower_mask = hsv_img[:,:,0] >= 15
            yellow_higher_mask = hsv_img[:,:,0] <= 35 #50

            #TRANSPARENCY CHANNEL AMARILLO
            yellow_lower_saturation_mask = hsv_img[:,:,1] >= 65

            yellow_mask_union = yellow_lower_mask*yellow_higher_mask*yellow_lower_saturation_mask

            #YELLOW MASK
            red_yellow = img_received[:,:,0]*yellow_mask_union
            green_yellow = img_received[:,:,1]*yellow_mask_union
            blue_yellow = img_received[:,:,2]*yellow_mask_union

            yellow_masked = np.dstack((red_yellow,green_yellow,blue_yellow))

            #CLOSING
            kernel = np.ones((7, 7), np.uint8)

            #RED CLOSING
            red_masked = cv2.cvtColor(red_masked, cv2.COLOR_RGB2GRAY)

            red_dilatetion = cv2.dilate(red_masked, kernel, iterations = 2)
            red_masked = cv2.erode(red_dilatetion, kernel, iterations = 1)
            red_masked = cv2.dilate(red_masked, kernel, iterations = 1)

            #GREEN CLOSING
            green_masked = cv2.cvtColor(green_masked, cv2.COLOR_RGB2GRAY)

            green_dilatetion = cv2.dilate(green_masked, kernel, iterations = 2)
            green_masked = cv2.erode(green_dilatetion, kernel, iterations = 1)
            green_masked = cv2.dilate(green_masked, kernel, iterations = 1)

            #YELLOW CLOSING
            yellow_masked = cv2.cvtColor(yellow_masked, cv2.COLOR_RGB2GRAY)

            yellow_dilatetion = cv2.dilate(yellow_masked, kernel, iterations = 2)
            yellow_masked = cv2.erode(yellow_dilatetion, kernel, iterations = 1)
            yellow_masked = cv2.dilate(yellow_masked, kernel, iterations = 1)

            #PORCENTAJE
            num_pixeles = 921600.0

            suma_red = np.count_nonzero(red_masked)
            suma_green = np.count_nonzero(green_masked)
            suma_yellow = np.count_nonzero(yellow_masked)

            self.red_density.data = (suma_red/num_pixeles)*100.0
            self.green_density.data = (suma_green/num_pixeles)*100.0
            self.yellow_density.data = (suma_yellow/num_pixeles)*100.0

            #self.img_msg_color = self.bridge.cv2_to_imgmsg(yellow_masked, 'mono8')
            #self.msk.publish(self.img_msg_color)

            #RED PUBLISHER
            self.red_density_pub.publish(self.red_density)

            #GREEN PUBLISHER
            self.green_density_pub.publish(self.green_density)

            #YELLOW PUBLISHER
            self.yellow_density_pub.publish(self.yellow_density)

        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().info('ERROR D:' + str(e))

def main(args=None):
    rclpy.init(args=args)
    m_s = final_project()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from std_msgs.msg import Int32

class color_identification_node(Node):
    def __init__(self):
        super().__init__('color_identification_node')
        self.video_source_subscription = self.create_subscription(Image, 'video_source/raw',self.videoSource_callback, rclpy.qos.qos_profile_sensor_data)

        self.red_msk = self.create_publisher(Image, 'img_properties/red/msk', rclpy.qos.qos_profile_sensor_data)
        self.red_density_pub = self.create_publisher(Float32, 'img_properties/red/density', rclpy.qos.qos_profile_sensor_data)
        self.red_x = self.create_publisher(Int32, 'img_properties/red/x', rclpy.qos.qos_profile_sensor_data)
        self.red_y = self.create_publisher(Int32, 'img_properties/red/y', rclpy.qos.qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.img_msg = Image()
        self.red_density = Float32()
        self.centro_red_x = Int32()
        self.centro_red_y = Int32()

        self.colores = 5

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New node succesfully initializated')


    def videoSource_callback(self, msg):
        img_received = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.get_logger().info('Imagen recibida :D')

        hsv_img = cv2.cvtColor(img_received, cv2.COLOR_RGB2HSV)

        dos = hsv_img [:,:,0] > 0

        if self.colores == 1:
            #Rojo
            lower_mask_value = 150
            higher_mask_value = 255
            higher_mask_value2 = 15 
            lower_saturation_mask_value = 0
            seleccion = 1
        elif self.colores == 2:
            #Azul
            lower_mask_value = 95
            higher_mask_value = 150
            higher_mask_value2 = 0
            lower_saturation_mask_value = 0
            seleccion = 1
        elif self.colores == 3:
            #Verde
            lower_mask_value = 40
            higher_mask_value = 95
            higher_mask_value2 = 0
            lower_saturation_mask_value = 20
            seleccion = 1
        elif self.colores == 4:
            #Amarillo
            lower_mask_value = 15
            higher_mask_value = 35
            higher_mask_value2 = 0
            lower_saturation_mask_value = 0
            seleccion = 1
        elif self.colores == 5:
            #Blanco
            lower_mask_value = 25
            higher_mask_value = 45
            higher_mask_value2 = 0
            lower_saturation_mask_value = 100
            value = 150
            seleccion = 2
        else:
            lower_mask_value = 0
            higher_mask_value = 0
            higher_mask_value2 = 0
            lower_saturation_mask_value = 0
            seleccion = 1


        #HUE CHANNEL
        lower_mask = hsv_img[:,:,0] > lower_mask_value
        higher_mask = hsv_img[:,:,0] < higher_mask_value

        higher_mask2 = hsv_img[:,:,0] < higher_mask_value2

        #TRANSPARENCY CHANNEL
        if seleccion == 1:
            lower_saturation_mask = hsv_img[:,:,1] > lower_saturation_mask_value
            mask_union = ((lower_mask*higher_mask)+(dos*higher_mask2))*lower_saturation_mask
        elif seleccion == 2:
            lower_saturation_mask = hsv_img[:,:,1] < lower_saturation_mask_value
            value_mask = hsv_img[:,:,2] > value
            mask_union = lower_mask*higher_mask*lower_saturation_mask*value_mask
        else:
            lower_saturation_mask = hsv_img[:,:,1] > 0
            mask_union = ((lower_mask*higher_mask)+(dos*higher_mask2))*lower_saturation_mask

        red = img_received[:,:,0]*mask_union
        green = img_received[:,:,1]*mask_union
        blue = img_received[:,:,2]*mask_union

        color_masked = np.dstack((red,green,blue))

        #CLOSING
        color_masked = cv2.cvtColor(color_masked, cv2.COLOR_RGB2GRAY)
        kernel = np.ones((5, 5), np.uint8)

        dilatetion = cv2.dilate(color_masked, kernel, iterations = 1)
        color_masked = cv2.erode(dilatetion, kernel, iterations = 1)

        #PORCENTAJE
        num_pixeles = 921600.0
        suma = 0
       
        for fila in color_masked:
            for valor in fila:
                if valor > 0:
                    suma += 1

        self.red_density.data = (suma/num_pixeles)*100.0
       # self.red_density.data = 100.0

        #CENTROIDE
        ret, thresh = cv2.threshold(color_masked, 10, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

        for c in contours:
            M = cv2.moments(c)

            if M['m00'] != 0:
                self.centro_red_x.data = int(M['m10'] / M['m00'])
                self.centro_red_y.data = int(M['m01'] / M['m00'])
                

        self.img_msg = self.bridge.cv2_to_imgmsg(color_masked, 'mono8')


    def timer_callback(self):
        self.red_msk.publish(self.img_msg)

        self.red_density_pub.publish(self.red_density)

        self.red_x.publish(self.centro_red_x)
        self.red_y.publish(self.centro_red_y)

def main(args=None):
    rclpy.init(args=args)
    m_s = color_identification_node()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


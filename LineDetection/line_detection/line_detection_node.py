import rclpy
import rclpy.qos
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist
#10
class line_detection_node(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        self.video_source_subscription = self.create_subscription(Image, 'video_source/raw',self.videoSource_callback, rclpy.qos.qos_profile_sensor_data)

        self.green_subscriber = self.create_subscription(Float32, 'img_properties/green/density', self.green_callback, rclpy.qos.qos_profile_sensor_data)
        self.red_subscriber = self.create_subscription(Float32, 'img_properties/red/density', self.red_callback, rclpy.qos.qos_profile_sensor_data) 

        self.line_publisher = self.create_publisher(Image, 'lines', rclpy.qos.qos_profile_sensor_data)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data) #20
        self.red_den_publisher = self.create_publisher(Float32, 'red_density', rclpy.qos.qos_profile_sensor_data)
        self.green_den_publisher = self.create_publisher(Float32, 'green_density', rclpy.qos.qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.img_msg = Image()
        self.img_received = Image()

        self.vel_lineal = 0.0
        self.vel_angular = 0.0

        self.red_density = Float32() #30
        self.green_density = Float32()

        self.vel_total = Twist()

        self.flag = 0
        self.percentage = Float32()
        self.percentage.data = 15.0

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New node succesfully initializated')

    def videoSource_callback(self, msg):
        self.img_received = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def green_callback(self, msg):
        self.green_density = msg
        if (self.green_density.data > self.percentage.data):
            self.flag = 1
        else:
            self.flag = self.flag

    def red_callback(self, msg):
        self.red_density = msg
        if (self.red_density.data > self.percentage.data):
            self.flag = 0
        else:
            self.flag = self.flag

    def timer_callback(self):
        try:
            self.get_logger().info('Imagen recibida :D')
            self.red_den_publisher.publish(self.red_density)
            self.green_den_publisher.publish(self.green_density)
            # Accion
            if (self.flag == 0):
                self.get_logger().info('RED FLAG')
                self.vel_angular = 0.0
                self.vel_lineal = 0.0

                self.vel_total.linear.x = self.vel_lineal
                self.vel_total.angular.z = self.vel_angular

                self.vel_publisher.publish(self.vel_total)


            elif (self.flag == 1):
                self.get_logger().info('GREEN FLAG')
                img_rec = cv2.flip(self.img_received, -1) #1280x720 #40

                img_cropped = img_rec[380:720, 330:950]

                img_mono = cv2.cvtColor(img_cropped, cv2.COLOR_RGB2GRAY)
                width = img_mono.shape[1]

                frame = cv2.GaussianBlur(img_mono, (5,5), 0)

                aperture_size = 5
                edges = cv2.Canny(frame, 200, 250, apertureSize = aperture_size) #50

                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 60, None, 20, 30)

                max_length = 0
                y = 0

                if lines is not None:
                    self.get_logger().info('Lineaaaaaaaaa')

                    for i in range(0, len(lines)):
                        l = lines[i][0]
                        cv2.line(img_mono, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
                        # Calcular la longitud de la línea
                        length = np.sqrt((l[2] - l[0]) ** 2 + (l[3] - l[1]) ** 2)
                        if length > max_length:
                            max_length = length
                            longest_line = l

                        # Guardar el punto en x más lejos en la variable x
                        x = max(longest_line[0], longest_line[2])
                        error = 1 - (x / (width / 2))

                        angular_max = 0.0
                        if (error > 0):
                            angular_max = 0.12
                        else:
                            angular_max = 0.06

                        self.vel_angular = (error)*(angular_max)
                        self.vel_lineal = 0.02

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular

                else:
                    self.vel_lineal = 0.0
                    self.vel_angular = 0.0

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular

                img_resize = cv2.resize(img_mono,(0,0),fx = 0.1, fy = 0.1) #a la mitad

                self.img_msg = self.bridge.cv2_to_imgmsg(img_mono, 'mono8')

                #Publisher Image
                self.line_publisher.publish(self.img_msg)

                #Publisher vel
                self.vel_publisher.publish(self.vel_total)

            self.get_logger().info('Salida')

        except:
            self.get_logger().info('ERROR D:')


def main(args=None):
    rclpy.init(args=args)
    m_s = line_detection_node()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

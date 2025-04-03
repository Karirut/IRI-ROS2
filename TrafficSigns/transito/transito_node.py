import rclpy
import rclpy.qos
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Bool, Int8
from geometry_msgs.msg import Twist
#10
class transito_node(Node):
    def __init__(self):
        super().__init__('transito_node')

        self.video_source_subscription = self.create_subscription(Image, 'video_source/raw',self.videoSource_callback, rclpy.qos.qos_profile_sensor_data)

        self.bandera_publisher = self.create_publisher(Int8, 'bandera_senal', rclpy.qos.qos_profile_sensor_data)
        self.senal_publisher = self.create_publisher(Int8, 'senal_transito', rclpy.qos.qos_profile_sensor_data)

        self.transito = 0
        self.bandera_senal = 0


        self.bridge = CvBridge()
        self.img_received = Image()

        # Cargar y preprocesar las imágenes de referencia
        ruta1 = '/home/puzzlebot/ros2_ws/src/transito/transito/giveaway.jpg'
        ruta2 = '/home/puzzlebot/ros2_ws/src/transito/transito/stop.jpg'
        ruta3 = '/home/puzzlebot/ros2_ws/src/transito/transito/straigth.jpg'
        ruta4 = '/home/puzzlebot/ros2_ws/src/transito/transito/turnaround.jpg'
        ruta5 = '/home/puzzlebot/ros2_ws/src/transito/transito/turnleft.jpg'
        ruta6 = '/home/puzzlebot/ros2_ws/src/transito/transito/turnright.jpg'
        ruta7 = '/home/puzzlebot/ros2_ws/src/transito/transito/workinprogress.jpg'

        self.imagenes = []
        for ruta in [ruta1, ruta2, ruta3, ruta4, ruta5, ruta6, ruta7]:
            img = cv2.imread(ruta)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = cv2.GaussianBlur(img, (5, 5), 0)
            _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self.imagenes.append(img)

        self.nombres = ['giveaway', 'stop', 'straight', 'turn around', 'left', 'right', 'work in progress']
        self.conteo = [0, 0, 0, 0, 0, 0, 0]
        self.iteracion = 0

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)

    def videoSource_callback(self, msg):
        self.img_received = self.bridge.imgmsg_to_cv2(msg, 'rgb8')


    def timer_callback(self):
        try:
            self.get_logger().info('DENTRO')
            gray = cv2.cvtColor(self.img_received, cv2.COLOR_RGB2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # Crear objeto SIFT
            sift = cv2.SIFT_create()
            keypoints_ref, descriptors_ref = sift.detectAndCompute(th, None)
            # Crear feature matcher
            bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
            puntos = []

            # Comparar cada imagen de referencia con la imagen del frame
            for img in self.imagenes:
                keypoints_img, descriptors_img = sift.detectAndCompute(img, None)
                matches = bf.match(descriptors_img, descriptors_ref)
                puntos.append(len(matches))

            maximo = max(puntos)
            if maximo > 20:
                index = puntos.index(maximo)
                self.get_logger().info('i: ' + str(index)) 

            self.get_logger().info('...')

            if index == 2:
                self.conteo[4] -= 1
                self.conteo[5] -= 1
                self.conteo[6] -= 1
                self.conteo[index] += 2
            elif index == 0 and maximo < 50:
                self.conteo[0] -= 1
            elif index == 6 and maximo < 40:
                self.conteo[6] -= 1
            else:
                self.conteo[index] += 1

            self.iteracion += 1
            val = 2

            indice = -1
            for i, valor in enumerate(self.conteo):
                if valor >= val:
                    indice = i


            if (self.iteracion > (val + 5)):
                #NO HAY SEÑAL
                self.get_logger().info('NOOOOOOOOOO')
                self.bandera_senal = 0
                self.iteracion = 0
                self.conteo = [0, 0, 0, 0, 0, 0, 0]
            else:
                if indice != -1:
                    self.get_logger().info('SEÑAL')
                    print(self.nombres[indice])
                    self.bandera_senal = 1
                    self.transito = indice
                    self.conteo = [0, 0, 0, 0, 0, 0, 0]
                    self.iteracion = 0
                else:
                    self.get_logger().info('Hola Mundo (te la creiste papiiii)')

            self.bandera_publisher.publish(self.bandera_senal)
            self.senal_publisher.publish(self.transito)

        except:
            self.get_logger().info('ERROR D:')

def main(args=None):
    rclpy.init(args=args)
    m_s = transito_node()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

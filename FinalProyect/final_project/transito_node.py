import rclpy
import rclpy.qos
import numpy as np
import cv2
import traceback
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
#10
class transito_node(Node):
    def __init__(self):
        super().__init__('transito_node')

        self.video_source_subscription = self.create_subscription(Image, 'video_source/raw',self.videoSource_callback, rclpy.qos.qos_profile_sensor_data)
        self.yellow_subscription = self.create_subscription(Int32, 'yellow_flag',self.yellow_callback, rclpy.qos.qos_profile_sensor_data)
        self.lines_subscription = self.create_subscription(Int32, 'lineas_flag',self.lines_callback, rclpy.qos.qos_profile_sensor_data)
        self.color_subscription = self.create_subscription(Int32, 'color_flag',self.color_callback, rclpy.qos.qos_profile_sensor_data)

        self.bandera_publisher = self.create_publisher(Int32, 'bandera_senal', rclpy.qos.qos_profile_sensor_data)
        self.senal_publisher = self.create_publisher(Int32, 'senal_transito', rclpy.qos.qos_profile_sensor_data)

        self.transito = Int32()
        self.transito.data = 0

        self.bandera_senal = Int32()
        self.bandera_senal.data = 0

        self.yellow_flag = Int32()
        self.yellow_flag.data = 0

        self.color_flag = Int32()
        self.color_flag.data = 0

        self.lines_flag = Int32()
        self.lines_flag.data = 0

        self.bridge = CvBridge()
        self.img_received = Image()

        # Cargar y preprocesar las imágenes de referencia
        ruta1 = '/home/puzzlebot/ros2_ws/src/transito/transito/giveaway.jpg'
        ruta2 = '/home/puzzlebot/ros2_ws/src/transito/transito/stop.jpg'
        ruta3 = '/home/puzzlebot/ros2_ws/src/transito/transito/straigth.jpg'
        ruta5 = '/home/puzzlebot/ros2_ws/src/transito/transito/turnleft.jpg'
        ruta6 = '/home/puzzlebot/ros2_ws/src/transito/transito/turnright.jpg'
        ruta7 = '/home/puzzlebot/ros2_ws/src/transito/transito/workinprogress.jpg'

        self.imagenes = []
        for ruta in [ruta1, ruta2, ruta3, ruta5, ruta6, ruta7]:
            img = cv2.imread(ruta)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = cv2.GaussianBlur(img, (5, 5), 0)
            _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self.imagenes.append(img)

        self.nombres = ['giveaway', 'stop', 'straight', 'left', 'right', 'work in progress']
        self.conteo = [0, 0, 0, 0, 0, 0]
        self.iteracion = 0

        self.indexu = Int32()
        self.indexu.data = -1

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)

        self.nombres_linea = ['giveaway', 'stop', 'work in progress']
        self.nombres_nada = ['straight', 'left', 'right']
        self.conteo_dos = [0, 0, 0]

        self.imagenes_lineas = []
        for ruta in [ruta1, ruta2,ruta7]:
            img = cv2.imread(ruta)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = cv2.GaussianBlur(img, (5, 5), 0)
            _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self.imagenes_lineas.append(img)


        self.imagenes_nada = []
        for ruta in [ruta3, ruta5, ruta6]:
            img = cv2.imread(ruta)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = cv2.GaussianBlur(img, (5, 5), 0)
            _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            self.imagenes_nada.append(img)

    def videoSource_callback(self, msg):
        self.img_received = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def yellow_callback(self, msg):
        self.yellow_flag = msg

    def color_callback(self, msg):
        self.color_flag = msg

    def lines_callback(self, msg):
        self.lines_flag = msg

    def timer_callback(self):
        try:
            self.get_logger().info('DENTRO')
            gray = cv2.cvtColor(self.img_received, cv2.COLOR_RGB2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            if (self.yellow_flag.data == 0):
                if self.lines_flag.data == 0 or self.color_flag.data == 0:
                    self.get_logger().info('paradooooooooo')
                    self.get_logger().info('flag: ' + str(self.yellow_flag.data))
                    sift = cv2.SIFT_create()
                    keypoints_ref, descriptors_ref = sift.detectAndCompute(th, None)
                    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
                    puntos = []

                    for img in self.imagenes_nada:
                        keypoints_img, descriptors_img = sift.detectAndCompute(img, None)
                        matches = bf.match(descriptors_img, descriptors_ref)
                        puntos.append(len(matches))

                    maximo = max(puntos)
                    self.get_logger().info('maximoooo: ' + str(maximo))
                    if maximo > 18:
                        self.indexu.data = puntos.index(maximo)
                        self.get_logger().info('i: ' + str(self.indexu.data))
                        self.iteracion += 1
                    else:
                        self.indexu.data = -1

                    self.get_logger().info('...')

                    if self.indexu.data == 0:
                        self.conteo_dos[2] -= 1
                        self.conteo_dos[1] -= 1
                        self.conteo_dos[self.indexu.data] += 2
                    elif self.indexu.data == 1:
                        self.conteo_dos[1] += 2
                    elif self.indexu.data == -1:
                        self.get_logger().info('A1')
                    else:
                        self.conteo_dos[self.indexu.data] += 1
                        self.get_logger().info('else...')

                    print(self.conteo_dos)
                    val = 8
                    indice = -1
                    for i, valor in enumerate(self.conteo_dos):
                        if valor >= val:
                            indice = i
                            self.get_logger().info('indice =' + str(indice))

                    if self.iteracion > val + 4:
                        self.bandera_senal.data = 0
                        self.iteracion = 0
                        self.conteo_dos = [0, 0, 0]

                    else:
                        if indice != -1:
                            self.get_logger().info('SENAL')
                            print(self.nombres_nada[indice])
                            self.bandera_senal.data = 1
                            self.transito.data = indice
                            self.conteo_dos = [0, 0, 0]
                            self.iteracion = 0
                        else:
                            self.get_logger().info('NONOSENAL')
                            self.bandera_senal.data = 0

                    self.bandera_publisher.publish(self.bandera_senal)
                    self.senal_publisher.publish(self.transito)

                elif (self.lines_flag.data == 1):
                    self.get_logger().info('movimientoooooooooooo')
                    self.get_logger().info('flag: ' + str(self.yellow_flag.data))
                    sift = cv2.SIFT_create()
                    keypoints_ref, descriptors_ref = sift.detectAndCompute(th, None)
                    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
                    puntos = []

                    for img in self.imagenes_lineas:
                        keypoints_img, descriptors_img = sift.detectAndCompute(img, None)
                        matches = bf.match(descriptors_img, descriptors_ref)
                        puntos.append(len(matches))

                    maximo = max(puntos)
                    self.get_logger().info('maximoooo: ' + str(maximo))
                    if maximo > 20:
                        self.indexu.data = puntos.index(maximo)
                        self.get_logger().info('i: ' + str(self.indexu.data))
                        self.iteracion += 1
                    else:
                        self.indexu.data = -1

                    self.get_logger().info('...')

                    if self.indexu.data == -1:
                        self.get_logger().info('A1')
                    else:
                        self.conteo_dos[self.indexu.data] += 1

                    val = 2
                    indice = -1
                    for i, valor in enumerate(self.conteo_dos):
                        if valor >= val:
                            indice = i

                    if self.iteracion > val + 2:
                        self.bandera_senal.data = 0
                        self.iteracion = 0
                        self.conteo = [0, 0, 0]
                    else:

                        if indice != -1:
                            self.get_logger().info('SENAL')
                            print(self.nombres_linea[indice])
                            self.bandera_senal.data = 1
                            self.transito.data = indice
                            self.conteo_dos = [0, 0, 0]
                            self.iteracion = 0
                        else:
                            self.get_logger().info('NONOSENAL')
                            self.bandera_senal.data = 0

                    self.bandera_publisher.publish(self.bandera_senal)
                    self.senal_publisher.publish(self.transito)

            #YELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOW
            elif (self.yellow_flag.data ==  1):
                self.get_logger().info('flag: ' + str(self.yellow_flag.data))

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
                self.get_logger().info('maximoooo: ' + str(maximo))
                if maximo > 15:
                    self.indexu.data = puntos.index(maximo)
                    self.get_logger().info('i: ' + str(self.indexu.data))
                else:
                    self.indexu.data = -1

                #self.get_logger().info(str(self.indexu.data))
                self.get_logger().info('...')

                if self.indexu.data == 0:
                    if maximo > 30:
                        self.conteo[0] += 1
                    else:
                        self.conteo[0] = self.conteo[0]

                elif self.indexu.data == 5:
                    if maximo > 30:
                        self.conteo[5] += 1
                    else:
                        self.conteo[5] = self.conteo[5]

                elif self.indexu.data == 2:
                    self.conteo[3] -= 1
                    self.conteo[4] -= 1
                    self.conteo[5] -= 1
                    self.conteo[self.indexu.data] += 2
                    self.get_logger().info('vueltas')
                elif self.indexu.data == -1:
                    self.get_logger().info('A1')
                else:
                    self.conteo[self.indexu.data] += 1

                val = 2

                indice = -1
                for i, valor in enumerate(self.conteo):
                    if valor >= val:
                        indice = i

                if indice != -1:
                    self.get_logger().info('SENAL')
                    print(self.nombres[indice])
                    self.bandera_senal.data = 1
                    self.transito.data = indice
                    self.conteo = [0, 0, 0, 0, 0, 0]
                    self.iteracion = 0
                else:
                    self.get_logger().info('NONOSENAL')
                    self.bandera_senal.data = 0

                self.bandera_publisher.publish(self.bandera_senal)
                self.senal_publisher.publish(self.transito)

        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().info('ERROR D:' + str(e))

def main(args=None):
    rclpy.init(args=args)
    m_s = transito_node()

    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
import rclpy.qos
import numpy as npy
import cv2
import traceback
import math
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int8
from geometry_msgs.msg import Twist, Pose2D

class final_project(Node):
    def __init__(self):
        super().__init__('master_node')

        self.error_subscriber = self.create_subscription(Float32, 'error', self.error_callback, rclpy.qos.qos_profile_sensor_data)
        self.bandera_subscriber = self.create_subscription(Int32, 'color_flag', self.bandera_callback, rclpy.qos.qos_profile_sensor_data)
        self.lineas_subscriber = self.create_subscription(Int32, 'lineas_flag', self.lineas_callback, rclpy.qos.qos_profile_sensor_data)
        self.posicion_subscriber = self.create_subscription(Pose2D, 'pos_global', self.posicion_callback, rclpy.qos.qos_profile_sensor_data)
        self.yellow_subscriber = self.create_subscription(Int32, 'yellow_flag', self.yellow_callback, rclpy.qos.qos_profile_sensor_data)
        self.red_subscriber = self.create_subscription(Float32, 'red_density', self.red_callback, rclpy.qos.qos_profile_sensor_data)

        self.banderaS_subscriber = self.create_subscription(Int32, 'bandera_senal', self.flag_senal_callback, rclpy.qos.qos_profile_sensor_data)
        self.senal_subscriber = self.create_subscription(Int32, 'senal_transito', self.senaltransito_callback, rclpy.qos.qos_profile_sensor_data)

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)

        self.error = Float32()

        # FLAGS
        self.color_flag = Int32()
        #self.color_flag.data = 1

        self.lineas_flag = Int32()
        self.lineas_flag.data = 0

        self.senal_flag = Int32()
        self.senal_flag.data = 0

        self.senal = Int32()
        self.senal.data = 0

        self.yellow_flag = Int32()
        self.yellow_flag.data = 0

        self.red_density = Float32()
        self.red_density.data = 0.0

        self.vel_lineal = 0.0
        self.vel_angular = 0.0
        self.vel_total = Twist()

        self.list = [0.0, math.pi, (3.0*math.pi)/4.0]

        self.posicion = Pose2D()

        self.timer_period = 0.1
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New node succesfully initializated')


    def error_callback(self,msg):
        self.error = msg

    def bandera_callback(self, msg):
        self.color_flag = msg

    def lineas_callback(self, msg):
        self.lineas_flag = msg

    def flag_senal_callback(self, msg):
        self.senal_flag = msg

    def senaltransito_callback(self, msg):
        self.senal = msg

    def posicion_callback(self, msg):
        self.posicion = msg

    def yellow_callback(self, msg):
        self.yellow_flag = msg

    def red_callback(self, msg):
        self.red_density = msg

    def timer_callback(self):
        try:
            cont = 0
            self.get_logger().info('IMAGEN')
            if (self.senal_flag.data == 0) and (self.lineas_flag.data == 1) and (self.yellow_flag.data == 0):
                self.get_logger().info('PASEO POR EL CAMPO')
                if (self.color_flag.data == 0):
                    self.get_logger().info('RED FLAG')
                    self.vel_angular = 0.0
                    self.vel_lineal = 0.0

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

                elif (self.color_flag.data == 1):
                    self.get_logger().info('GREEN FLAG')
                    angular_max = 0.0
                    if (self.error.data > 0.1):
                        angular_max = 0.12
                    else:
                        angular_max = 0.06

                    self.vel_angular = (self.error.data)*(angular_max)
                    self.vel_lineal = 0.02

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

            elif self.senal_flag.data == 0 and self.lineas_flag.data == 0 and self.yellow_flag.data == 0:
                self.get_logger().info('NADA DE NADA')
                self.vel_total.linear.x = 0.0
                self.vel_total.angular.z = 0.0
                self.vel_publisher.publish(self.vel_total)

            elif (self.senal_flag.data == 1) and (self.lineas_flag.data == 1) and (self.yellow_flag.data == 0):
                self.get_logger().info('LINEAS Y SENAL')
                if (self.color_flag.data == 1):
                #GIVEAWAY y WORKINPROGRESS
                    if (self.senal.data == 0) or (self.senal.data == 2):
                        self.get_logger().info('INICIO GIVE o WORK')
                        angular_max = 0.0
                        if (self.error.data > 0.0):
                            angular_max = 0.06
                        else:
                            angular_max = 0.03

                        self.vel_angular = (self.error.data)*(angular_max)
                        self.vel_lineal = 0.01

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        if cont < 300:
                            cont += 1
                            self.senal.data = 0
                            self.senal_flag.data = 1
                        else:
                            cont = 0
                        self.get_logger().info('Termino GIVE o WORK')
                elif (self.color_flag.data == 0):
                    self.vel_lineal = 0.0
                    self.vel_angular = 0.0
                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

                #WORK IN PROGRESS
                    '''
                    elif (self.senal.data == 5):
                        self.get_logger().info('INICIO WORK IN PROGRESS')
                        angular_max = 0.0
                        if (self.error.data > 0.0):
                            angular_max = 0.06
                        else:
                            angular_max = 0.03

                        self.vel_angular = (self.error.data)*(angular_max)
                        self.vel_lineal = 0.01

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        if cont < 600:
                            cont += 1
                            self.senal.data = 5
                            self.senal_flag.data = 1
                        else:
                            cont = 0

                        self.get_logger().info('termino WORK IN PROGESS')
                    '''
            elif (self.senal_flag.data == 1) and (self.lineas_flag.data == 0) and (self.yellow_flag.data == 0):
                self.get_logger().info('FLECHAS')
                '''
                self.vel_lineal = 0.0
                if self.posicion.theta not in self.list:
                    for i in range(len(self.list)):
                        list_n[i] = self.posicion.theta - self.list[i]
                        abs_list[i] = abs(list_n[i])
                        min = min(abs_list)
                        ind = abs_list.index(min)
                        self.vel_angular = (list_n[ind] * 0.01)
                else:
                    self.vel_angular = 0.0

                self.vel_total.linear.x = self.vel_lineal
                self.vel_total.angular.z = self.vel_angular
                self.vel_publisher.publish(self.vel_total)
                '''
                #STRAIGHT
                if (self.senal.data == 0) and (self.color_flag.data == 1):
                    self.get_logger().info('INICIO STRAIGHT')
                    for i in range(600):
                        self.vel_angular = 0.0
                        self.vel_lineal = 0.04

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        self.get_logger().info('STRAIGHT: ' + str(i))

                    self.get_logger().info('termino STRAIGHT')

                #LEFT
                elif (self.senal.data == 1) and (self.color_flag.data == 1):
                    self.get_logger().info('INICIO LEFT')
                    for i in range(200):
                        self.vel_angular = 0.08
                        self.vel_lineal = 0.08

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        self.get_logger().info('LEFT: ' + str(i))

                    self.get_logger().info('termino LEFT')

                #RIGHT
                elif (self.senal.data == 2) and (self.color_flag.data == 1):
                    self.get_logger().info('INICIO RIGHT')
                    for i in range(240):
                        self.vel_angular = -0.06
                        self.vel_lineal = 0.08

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        self.get_logger().info('RIGHT: ' + str(i))

                    self.get_logger().info('termino RIGHT')
                else:
                    self.get_logger().info('NO RECONOSCO SENAL')
                    self.vel_angular = 0.0
                    self.vel_lineal = 0.0

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

            #AMARILLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
            elif (self.yellow_flag.data == 1):
                self.get_logger().info('YELLOW FLAG')
                '''
                self.vel_lineal = 0.0
                if self.posicion.theta not in self.list:
                    for i in range(len(self.list)):
                        list_n[i] = self.posicion.theta - self.list[i]
                        abs_list[i] = abs(list_n[i])
                        min = min(abs_list)
                        ind = abs_list.index(min)
                        self.vel_angular = (list_n[ind] * 0.01)
                else:
                    self.vel_angular = 0.0

                self.vel_total.linear.x = self.vel_lineal
                self.vel_total.angular.z = self.vel_angular
                self.vel_publisher.publish(self.vel_total)
                '''

                if (self.red_density.data > 4.5 or self.senal.data == 1):
                    self.vel_lineal = 0.0
                    self.vel_angular = 0.0

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

                elif (self.senal_flag.data == 0) or (self.senal.data == 2):
                    self.vel_lineal = 0.02
                    self.vel_angular = 0.0

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

                elif (self.senal.data == 3):
                    for i in range(200):
                        self.vel_angular = 0.08
                        self.vel_lineal = 0.08

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        self.get_logger().info('LEFT: ' + str(i))

                elif (self.senal.data == 4):
                    for i in range(180):
                        self.vel_angular = -0.06
                        self.vel_lineal = 0.08

                        self.vel_total.linear.x = self.vel_lineal
                        self.vel_total.angular.z = self.vel_angular
                        self.vel_publisher.publish(self.vel_total)

                        self.get_logger().info('RIGHT: ' + str(i))

                elif (self.senal.data == 0) or (self.senal.data == 5):
                    self.vel_angular = 0.0
                    self.vel_lineal = 0.01

                    self.vel_total.linear.x = self.vel_lineal
                    self.vel_total.angular.z = self.vel_angular
                    self.vel_publisher.publish(self.vel_total)

            else:
                self.get_logger().info('AYUDAAAAAAAAAAA')
                self.vel_lineal = 0.02
                self.vel_angular = 0.0

                self.vel_total.linear.x = self.vel_lineal
                self.vel_total.angular.z = self.vel_angular
                self.vel_publisher.publish(self.vel_total)

            # Publishsers
            #self.vel_publisher.publish(self.vel_total)
            self.get_logger().info('FINITO')

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

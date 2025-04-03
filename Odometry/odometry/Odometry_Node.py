import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import math 
import time


class Odometry_Node(Node):
    def __init__(self):
        super().__init__('Odometry_Node')
        # 10 es para evitar el cuello de botella.
        #cambiar vels
        self.velright_subscription = self.create_subscription(Float32, 'VelocityEncR', self.velocidadR_callback, rclpy.qos.qos_profile_sensor_data)
        self.velleft_subscription = self.create_subscription(Float32, 'VelocityEncL', self.velocidadL_callback, rclpy.qos.qos_profile_sensor_data)
        # Se crean publishers para la senal y el tiempo
        self.distancia_publisher = self.create_publisher(Float32, 'distancia', rclpy.qos.qos_profile_sensor_data)
        self.vel_publisher = self.create_publisher(Float32, 'vel_lineal', rclpy.qos.qos_profile_sensor_data)
        self.orientacion_publisher = self.create_publisher(Float32, 'orientacion', rclpy.qos.qos_profile_sensor_data)
        self.velangular_publisher = self.create_publisher(Float32, 'vel_angular', rclpy.qos.qos_profile_sensor_data)
        self.pos_global_publisher = self.create_publisher(Pose2D, 'pos_global', rclpy.qos.qos_profile_sensor_data)

        # Se dejan default en None para despues hacer las comparaciones   

        self.vel_lineal = Float32()
        self.vel_angular = Float32()

        self.velrigth_value = Float32()
        self.velleft_value = Float32()

        self.distancia = Float32()
        self.orientacion = Float32()

        self.distancia.data = 0.0
        self.orientacion.data = 0.0

        self.vel_x = Float32()
        self.vel_y = Float32()

        self.pos_x = Float32()
        self.pos_y = Float32()

        self.pos_x.data = 0.10
        self.pos_y.data = 2.40

        self.posGlobal = Pose2D()

        self.r = Float32()
        self.wl = Float32()

        self.r = 0.05
        self.wl = 0.18

        self.timer_period = 0.1 # 10 Hz
        self.proc = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('New Signal generator node successfully initialized')

    # Nota: se realiza un callback para cada publisher y para cada subscription
    def velocidadR_callback(self, msg):
        self.velrigth_value.data = msg.data

        #self.get_logger().info(f'Velocidad: {msg.data}')


    def velocidadL_callback(self, msg):
        self.velleft_value.data = msg.data 

    # callback del publisher (nueva senal procesada)
    def timer_callback(self):
        self.vel_lineal.data = self.r*((self.velrigth_value.data + self.velleft_value.data)/2)
        self.vel_angular.data = self.r*((self.velrigth_value.data - self.velleft_value.data)/self.wl)

        self.distancia.data = self.distancia.data + (self.vel_lineal.data * 0.1)
        self.orientacion.data = self.orientacion.data + (self.vel_angular.data * 0.1)
        if ((self.orientacion.data > 2* math.pi) or (self.orientacion.data) < (-2)*math.pi):
            self.orientacion.data = 0.0

        self.vel_x.data = self.vel_lineal.data * math.cos(self.orientacion.data)
        self.vel_y.data = self.vel_lineal.data * math.sin(self.orientacion.data)

        self.pos_x.data = self.pos_x.data + (self.vel_x.data * 0.1)
        self.pos_y.data = self.pos_y.data + (self.vel_y.data * 0.1)

        self.posGlobal.x = self.pos_x.data
        self.posGlobal.y = self.pos_y.data
        self.posGlobal.theta = self.orientacion.data 

        #publicacion
        self.vel_publisher.publish(self.vel_lineal)
        self.velangular_publisher.publish(self.vel_angular)
        self.distancia_publisher.publish(self.distancia)
        self.orientacion_publisher.publish(self.orientacion)
        self.pos_global_publisher.publish(self.posGlobal)

   
                                                
def main(args=None):
    rclpy.init(args=args)
    m_s = Odometry_Node()
    
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

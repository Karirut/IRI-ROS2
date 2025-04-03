import rclpy
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32
import math
import numpy as np 
import yaml

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pura_persecucion_node')
    
        self.declare_parameters(namespace = '',
                               parameters = [
                                   ('AstarP1_EM.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP1_EM.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP1_MC.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP1_MC.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP2_EM.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP2_EM.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP2_MC.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('AstarP2_MC.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP1_p120.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP1_p120.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP1_p8.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP1_p8.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP2_p120.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP2_p120.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP2_p8.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('PRMP2_p8.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P1largo.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P1largo.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P1corto.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P1corto.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P2largo.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P2largo.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P2corto.pos_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('RRT_P2corto.pos_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                   ('constantes.lookahead', rclpy.Parameter.Type.DOUBLE),
                                   ('constantes.max_linear_speed', rclpy.Parameter.Type.DOUBLE),
                               ])
        
        self.sub_odometria = self.create_subscription(Pose2D,'pos_global', self.pose_callback, rclpy.qos.qos_profile_sensor_data)
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.publisher_pruebas = self.create_publisher(Float32, 'pruebas', rclpy.qos.qos_profile_sensor_data)
        
        self.pose = Pose2D()

        self.trayectoria = 1
        
        # variables
        self.max_angular_speed = 0.0
        self.alpha = 0.0 
        self.steering_angle = 0.0 
        self.k = 0.0 
        self.lookahead_distance = 0.0    
           
        
        self.distance = 0.0
        self.closest_index = 0
        self.closest_point_x = 0.0
        self.closest_point_y = 0.0 
        self.limite = Float32()
        self.vel = Twist()

        
        # variables timer 
        self.timer_period = Float32()
        self.timer_period = 0.1
    

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # Encontramos en punto más cercano en nuetro radio con la distancia euclediana 
    def find_closest_index(self):
        
        self.closest_point_x = [self.pose.x - i for i in self.pos_x]
        self.closest_point_y = [self.pose.y - i for i in self.pos_y]
        
        self.distance = np.hypot(self.closest_point_x, self.closest_point_y)
        
        if np.argmin(self.distance)+ 1 < len(self.pos_x):
            self.closest_index = np.argmin(self.distance) +1
        else:
            len(self.pos_x)-1
        
        return self.closest_index       
    
    # Obteniendo los datos de odometrias (x,y,theta)
    def pose_callback(self, msg):
        self.pose = msg
        
      
    # Publicar los datos de velocidad 
    def timer_callback(self):
        
        # Obtener datos 
        if self.trayectoria == 1:
            self.pos_x = self.get_parameter('PRMP2_p8.pos_x').get_parameter_value().double_array_value
            self.pos_y = self.get_parameter('PRMP2_p8.pos_y').get_parameter_value().double_array_value
            self.lookahead = self.get_parameter('constantes.lookahead').get_parameter_value().double_value
            self.max_linear_speed = self.get_parameter('constantes.max_linear_speed').get_parameter_value().double_value
            self.get_logger().info('Ready set go cuadrado:D!!')
        else:
            self.pos_x = 0
            self.pos_y = 0
            self.lookahead = 0
            self.max_linear_speed = 0
        
        # Posición de x y del punto más cercano  
        self.closest_index = self.find_closest_index()
        
        self.closest_point_x = self.pos_x[self.closest_index]
        self.closest_point_y = self.pos_y[self.closest_index]

        # Calculamos alpha con respecto al punto más cercano y nuetra posición 
        self.alpha = np.arctan2((self.closest_point_y - self.pose.y),(self.closest_point_x - self.pose.x)) - self.pose.theta

        # Calcula la distancia entre nuestra posición y el waypoint
        self.lookahead_distance = math.sqrt((self.closest_point_x - self.pose.x)**2 + (self.closest_point_x - self.pose.y)**2)
         

        # Obtenemos k (arco deseado) y δ (ángulo del coche) 
        self.k = (2.0 * 0.18 * math.sin(self.alpha))/(self.lookahead)
            
        # Calculamos la velocidad angular 
        self.max_angular_speed = np.arctan2(self.k,1)

        # Limite para que el robot pare llegando al punto final 
        self.limite.data = math.sqrt((self.pos_x[len(self.pos_x)-1] - self.pose.x)**2 + (self.pos_y[len(self.pos_y)-1] - self.pose.y)**2)
        
        
        if self.limite.data > 0.05:
            self.max_angular_speed = self.max_angular_speed
            self.max_linear_speed = self.max_linear_speed
            #self.get_logger().info('VIVEEEEE :D!!')
            print(self.limite.data)
        else:
            self.max_linear_speed = 0.0
            self.max_angular_speed = 0.0
            #self.get_logger().info('NO VIVEEEEE :D!!')
        
        self.vel.linear.x = self.max_linear_speed
        self.vel.angular.z = self.max_angular_speed
        self.publisher_vel.publish(self.vel)
        self.publisher_pruebas.publish(self.limite)

def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main() 

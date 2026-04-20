#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry                   # -- LIBRERIAS
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from aruco_opencv_msgs.msg import ArucoDetection
import tf_transformations

class Bug3(Node):
    def __init__(self):
        super().__init__('bug3')
        self.get_logger().info('Bug 3 Inicializado...')

        #
        #       -- PUBLICADORES --
        #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Tópico para velocidades del puzzlebot
        self.servo_pub = self.create_publisher(Float32, '/ServoAngle', 10) # Tópico para mover servo

        # 
        #       -- SUSCRIPCIONES --
        #
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Tópico para recibir odometría
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10) # Tópico para recibir datos de LiDAR
        self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_callback, 10) # Tópico para recibir datos de la cámara
        self.create_timer(0.1, self.state_machine) # Timer para repetir máquina de estados

        self.state = 'go_to_goal' # Se inicia en el estado go to goal de la máquina de estados principal
        self.current_pose = []  # [x, y, yaw]
        self.target_pose = [1.0, 0.0] # Primer valor de trayectoria
        self.got_new_target = True # Nuevo target inicializado con True

        self.tolerance = 0.1         # Tolerancia de llegada al punto
        self.Kv = 0.06   # Ganancia lineal para go_to_goal
        self.Kw = 0.15   # Ganancia angular para go_to_goal

        self.line_to_goal = self.compute_line() # Se llama inicialmente al método go to goal
        self.start_point = None # No inicializa con ningun start point
        self.closest_point = None # Punto más cercano al objeto
        self.min_dist_to_goal = float('inf') # La distancia es infinita para que la condicion inicial sea True

        self.ranges = []  # lectura completa del scan
        self.robot_view = {} 
        self.d0 = 0.40  # Tolerancia del LIDAR para obstáculo

        self.first_time_flag = True # Bandera de primer mensaje

        self.marker_visible = False # Bandera de la cámara
        self.last_angle = 0.0 # ángulo relativo al marcador (rad)
        self.last_distance = float('inf') # distancia en Z al marcador (m)
        self.last_marker_id = None # id del marcador detectado

        self.desired_dist = 0.20         # Threshold base: 20 cm
        self.dist_margin = 0.02          # Margen de error
        self.K_ang = 0.8 # Ganancia angular para pick and place
        self.K_lin = 0.1 # Ganancia lineal para pick and place
        self.max_lin_speed = 0.10   # limitacion lineal
        self.max_ang_speed = 0.6    # limitacion angular
        self.min_aruco_detection_dist = 2.0 # Distancia mínima para detección del aruco
        self.angle_tolerance = 0.2 # Tolerancia de ángulo

        self.servo_angle = -20.0 # Iniclialización de altura del servo
        self.servo_pub.publish(Float32(data=self.servo_angle)) # Se publica altura inicial

        self.pick_phase = 'idle' # Pick inicializa en Idle
        self.pick_move_start_time = None

        self.drop_phase = 'idle' # Place inicializa en ilde
        self.drop_start_time = None

        self.pick_place_trigger = False

    # SE TOMAN TODOS LOS VALORES DEL LIDAR Y SE CREA UN DICCIONARIO ESPECIFICANDO SU POSICIÓN DEL OBJETO DETECTADO 
    def lidar_callback(self, data: LaserScan):
        self.ranges = list(data.ranges)
        for i in range(len(self.ranges)):
            if self.ranges[i] > data.range_max:
                self.ranges[i] = data.range_max + 0.01
            elif self.ranges[i] < data.range_min:
                self.ranges[i] = data.range_min - 0.01

        self.robot_view = {
            'front': min(min(self.ranges[0:66]), min(self.ranges[1012:1078])), # REVISA FRENTE EN 44°
            'front_left': min(self.ranges[66:198]), # REVISA FRENTE A LA IZQUIERDA EN 44°
            'front_right': min(self.ranges[858:1012]), # REVISA FRENTE A LA DERECHA EN 44°
            'left': min(self.ranges[198:330]), # REVISA IZQUIERDA EN 44°
            'right': min(self.ranges[726:858]) # REVISA DERECHA EN 44°
        }

    # SE RECIBEN VALORES DE ODOMETRIA EN X, Y y YAW
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = [x, y, yaw]

    # SE RECIBEN VALORES DE LA CÁMARA Y SE ESTIMA SU POSICIÓN EN X, Z y el ángulo al objetivo
    def aruco_callback(self, msg: ArucoDetection):
        self.marker_visible = False
        self.last_marker_id = None

        for marker in msg.markers:
            if marker.marker_id == 0:
                p = marker.pose.position
                if p.z > 0.0:
                    self.last_distance = p.z
                    self.last_angle = math.atan2(p.x, p.z)
                    self.marker_visible = True
                    self.last_marker_id = 0
                break

        for marker in msg.markers:
            if marker.marker_id in (1, 5):
                p = marker.pose.position
                if p.z > 0.0:
                    self.last_distance = p.z
                    self.last_marker_id = marker.marker_id
                break

    # SE DIBUJA LA LÍNEA DE BUG2 
    def compute_line(self):
        if len(self.current_pose) < 2 or len(self.target_pose) < 2:
            return (0.0, 0.0)
        dx = self.target_pose[0] - self.current_pose[0]
        dy = self.target_pose[1] - self.current_pose[1]
        mag = math.hypot(dx, dy)
        if mag == 0:
            return (0.0, 0.0)
        return (dx / mag, dy / mag)

    # FUNCIÓN PARA MEDIR DISTANCIA EUCLIDIANA
    def distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # SE CALCULA EL VALOR DE LA MLINE Y SE ESTABLECEN VALORES DE GROSOR PARA DETECCIÓN
    def on_m_line(self):
        if self.line_to_goal is None or self.start_point is None:
            return False
        dx, dy = self.line_to_goal
        px = self.current_pose[0] - self.start_point[0]
        py = self.current_pose[1] - self.start_point[1]
        cross = dx * py - dy * px
        return abs(cross) < 0.1

    # SE ESTABLECE CUANDO HAYA LLEGADO AL GOAL
    def at_goal(self):
        return self.distance(self.current_pose, self.target_pose) < self.tolerance

    # SI SE TIENE UN OBSTACULO SE NOTIFICA
    def obstacle_ahead(self):
        return self.robot_view.get('front', float('inf')) < self.d0

    # SE ESTABLECEN VELOCIDADES ANGULARES Y LINEALES
    def moveRobot(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    # SE DEFINEN VELOCIDADES EN 0
    def stop_robot(self):
        self.moveRobot(0.0, 0.0)
        if self.first_time_flag:
            self.first_time_flag = False

    # SE DEFINE IR AL GOAL DEPENDIENDO DE LA ODOMETRÍA Y EL VALOR DEL GOAL
    def go_to_goal(self):
        if self.first_time_flag:
            self.first_time_flag = False

        dx = self.target_pose[0] - self.current_pose[0]
        dy = self.target_pose[1] - self.current_pose[1]
        angle_to_goal = math.atan2(dy, dx) - self.current_pose[2]
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))

        v = 0.0 if abs(angle_to_goal) > 0.1 else self.Kv
        w = self.Kw * angle_to_goal
        self.moveRobot(v, w)

    # SE HACE UN SEGUIMIENTO DE PARED POR AMBOS LADOS 
    def follow_wall(self, direction):
        if self.first_time_flag:
            self.get_logger().info("Following wall... ")
            self.first_time_flag = False

        if direction == 'left':
            t2, r2 = math.radians(45), self.robot_view.get('front_left', 1.0)
            t1, r1 = math.radians(90), self.robot_view.get('left', 1.0)
        else:
            t2, r2 = math.radians(-45), self.robot_view.get('front_right', 1.0)
            t1, r1 = math.radians(-90), self.robot_view.get('right', 1.0)

        P1x, P1y = r1 * math.cos(t1), r1 * math.sin(t1)
        Ux_tan = r2 * math.cos(t2) - P1x
        Uy_tan = r2 * math.sin(t2) - P1y
        norm_tan = math.hypot(Ux_tan, Uy_tan)
        if norm_tan == 0:
            return
        Ux_tan_n, Uy_tan_n = Ux_tan / norm_tan, Uy_tan / norm_tan

        dot = P1x * Ux_tan_n + P1y * Uy_tan_n
        Ux_per = P1x - dot * Ux_tan_n
        Uy_per = P1y - dot * Uy_tan_n
        norm_per = math.hypot(Ux_per, Uy_per)
        if norm_per == 0:
            return
        Ux_per_n, Uy_per_n = Ux_per / norm_per, Uy_per / norm_per

        dwall, betha, Kfw = 0.50, 0.95, 1.2
        Ex_per = Ux_per - dwall * Ux_per_n
        Ey_per = Uy_per - dwall * Uy_per_n

        angle_per = math.atan2(Ey_per, Ex_per)
        angle_tan = math.atan2(Uy_tan_n, Ux_tan_n)
        fw_angle = betha * angle_tan + (1 - betha) * angle_per
        fw_angle = math.atan2(math.sin(fw_angle), math.cos(fw_angle))

        v = 0.07 if abs(fw_angle) > 0.1 else 0.06
        w = Kfw * fw_angle
        self.moveRobot(v, w)

    # SE DEFINE LA MÁQUINA DE PICK AND PLACE
    def pick_and_place(self):
        if self.pick_phase == 'advancing':
            elapsed = time.time() - self.pick_move_start_time
            if elapsed < 2.0:
                self.moveRobot(0.1, 0.0)                # SI ESTA EN ESTADO "advancing" DETERMINA QUE YA SE TIENE EL ÁNGULO ADECUADO Y LA DISTANCIA ADECUADA.
            else:                                       # SE ESTABLECE UNA VELOCIDAD POR UN TIEMPO ADECUADO PARA LLEGAR Y CARGAR EL OBJETIVO
                self.moveRobot(0.0, 0.0)                # SI SE TERMINA LA RUTINA AVANZA AL SIGUIENTE ESTADO "raising_servo"
                self.pick_phase = 'raising_servo'
            return

        if self.pick_phase == 'raising_servo':
            if self.servo_angle > -60.0:                # SI EL ÁNGULO ES MAYOR QUE -60 ENTONCES SUBIR LENTAMENTE EL SERVO PARA EVITAR APAGADO DE HACKERBOARD
                self.servo_angle -= 5.0              
                self.servo_pub.publish(Float32(data=self.servo_angle))
                time.sleep(0.5)
            else:
                self.get_logger().info("Etapa pick completada... ")     # CUANDO TERMINA CICLO DE SUBIDA DEL CUBO ENTONCES COMIENZA LA ETAPA DE IR AL SIGUIENTE PUNTO
                self.target_pose = [-1.2, 0.0]                          # EN -1.2, 0. EN DONDE SE REINICIAN VARIABLES Y SE ESTABLECEN OTRAS COMO EL ESTADO PRINCIPAL go_to_goal       
                self.line_to_goal = self.compute_line() 
                self.start_point = self.current_pose[:2]
                self.min_dist_to_goal = float('inf')
                self.state = 'go_to_goal'
                self.first_time_flag = True
                self.pick_phase = 'idle'
                self.pick_place_trigger = False
            return
        
    def drop_and_reverse(self):
        if self.drop_phase == 'lower_to_minus_20':
            if self.servo_angle < -20.0:                                # EL SERVO EN ESTA ETAPA ES CUANDO SE LLEGA AL PUNTO -1.2, 0. EN DONDE EL SERVO BAJARÁ LENTAMENTE LA CAJA
                self.servo_angle += 5.0
                self.servo_pub.publish(Float32(data=self.servo_angle))
                time.sleep(0.5)
            else:
                self.drop_phase = 'reverse_5s'                          # CUANDO TERMINE PASA AL SIGUIENTE ESTADO DE REVERSA
                self.drop_start_time = time.time()
            return

        if self.drop_phase == 'reverse_5s':
            elapsed = time.time() - self.drop_start_time
            if elapsed < 5.0:                                           # EL ROBOT APLICA VELOCIDADES NEGATIVAS PARA RETROCEDER SIN AFECTAR LA POSICIÓN DE LA CAJA
                self.moveRobot(-0.05, 0.0)
            else:
                self.moveRobot(0.0, 0.0)
                self.drop_phase = 'finish_drop'                         # SE DETIENE Y PASA AL ESTADO "finish_drop"
            return

        if self.drop_phase == 'finish_drop':
            self.get_logger().info("Entrega completada... ")
            self.target_pose = [0.0, 0.0]
            self.line_to_goal = self.compute_line()                     # EN EL ESTADO FINISH DROP SE ENVIA LA POSICIÓN DESEADA AL PUNTO 0,0 Y SE VUELVEN A INICIALIZAR VARIABLES
            self.start_point = self.current_pose[:2]
            self.min_dist_to_goal = float('inf')
            self.state = 'go_to_goal'
            self.first_time_flag = True
            self.drop_phase = 'idle'
            self.pick_place_trigger = False
            return

    # MÁQUINA DE ESTADOS PRINCIPAL
    def state_machine(self):
        if len(self.current_pose) < 3 or self.robot_view == {}:  # ESTADO INICIAL EN DONDE SI NO SE DETECTA POSICIÓN DESEADA NI UN OBJETO NO SE MUEVE
            return

        if self.state == 'pick_and_place':  # ENTRA EN EL ESTADO PICK AND PLACE
            self.pick_and_place()
            return
        if self.state == 'drop_and_reverse': # ENTRA EN EL ESTADO DROP AND REVERSE 
            self.drop_and_reverse()
            return

        if ((self.state == 'go_to_goal' or self.state == 'follow_wall')
                and self.marker_visible
                and self.last_marker_id == 0
                and self.last_distance < self.min_aruco_detection_dist  # SI EL ESTADO ES GO TO GOAL O HAY UN OBSTÁCULO Y APARTE SE TIENE UNA CONDICIÓN COMO QUE ESTA DETECTANDO
                and not self.pick_place_trigger                         # EL ARUCO DE LA CAJA Y QUE LA DISTANCIA ES CONSIDERABLE ENTONCES ENTRA AL ESTADO "FOLLOW ARUCO" UNA SOLA VEZ
                and self.target_pose == [1.0, 0.0]):
            self.state = 'follow_aruco'
            self.first_time_flag = True

        if self.state == 'go_to_goal':
            if self.at_goal():
                self.stop_robot()
                if self.target_pose == [1.0, 0.0]:                  # SI ES GO TO GOAL YA SE RECOGIÓ O DEJO LA CAJA, POR LO QUE SE ACTIVA EL GO TO GOAL. SI VA DE REGRESO SE ACTIVAN
                    pass                                            # LOS ESTADOS DE IR AL PUNTO -1,0 Y BAJAR EL SERVO
                elif self.target_pose == [-1.2, 0.0]:
                    self.state = 'drop_and_reverse'
                    self.drop_phase = 'lower_to_minus_20'
                    self.first_time_flag = True
                else:
                    if self.target_pose == [0.0, 0.0]:              # SI LA POSE DE TARGET ES 0,0 SE DETIENE EL ROBOT
                        self.state = 'stop_robot'
                    else:
                        self.state = 'stop_robot'
                self.first_time_flag = True
                return

            if self.obstacle_ahead():
                self.closest_point = self.current_pose[:2]                          # SI HAY UN OBSTACULO SE DETERMINA EL VALOR EL HIT POINT Y CLOSEST POINT 
                self.min_dist_to_goal = self.distance(self.current_pose, self.target_pose)
                self.state = 'follow_wall'
                self.first_time_flag = True
                return

            self.go_to_goal()

        elif self.state == 'follow_wall':
            dist_to_goal = self.distance(self.current_pose, self.target_pose)
            if dist_to_goal < self.min_dist_to_goal:
                self.min_dist_to_goal = dist_to_goal                                # SI EL ESTADO ES FOLLOW WALL ENTONCES SE CALCULA LA DISTANCIA AL GOAL, CLOSEST POINT 
                self.closest_point = self.current_pose[:2]                          # Y SE VERIFICA QUE SE ENCUENTRA SOBRE LA TRAYECTORIA QUE DESEAMOS PARA LLEGAR AL GOAL
            if self.on_m_line() and not self.obstacle_ahead():
                self.state = 'go_to_goal'
                self.first_time_flag = True
                return
            direction = 'left' if self.robot_view['left'] < self.robot_view['right'] else 'right' # SE ESTABLECE QUE LOS VALORES DE POR DONDE RODEA EL OBJETO DEPENDEN DEL LUGAR DEL OBSTÁCULO EN EL LiDAR
            self.follow_wall(direction)

        elif self.state == 'stop_robot':  # SE DETIENE EL ROBOT
            self.stop_robot()

        elif self.state == 'follow_aruco': # ESTADO DE SEGUIR ARUCO 0
            if self.first_time_flag:
                self.first_time_flag = False

            if not (self.marker_visible and self.last_marker_id == 0):
                self.state = 'go_to_goal'
                self.first_time_flag = True
                return

            if self.last_distance <= (self.desired_dist + self.dist_margin):
                self.state = 'pick_and_place'
                self.pick_place_trigger = True
                self.pick_phase = 'advancing'
                self.pick_move_start_time = time.time()
                self.first_time_flag = True
                self.moveRobot(0.0, 0.0)
                return
            angle = math.atan2(math.sin(self.last_angle), math.cos(self.last_angle))
            w = - self.K_ang * angle

            if w > self.max_ang_speed:
                w = self.max_ang_speed
            elif w < -self.max_ang_speed:
                w = -self.max_ang_speed

            lin_err = self.last_distance - self.desired_dist
            v = self.K_lin * lin_err
            if v > self.max_lin_speed:
                v = self.max_lin_speed
            elif v < 0.0:
                v = 0.0

            if abs(angle) >= self.angle_tolerance:
                v = 0.0

            self.moveRobot(v, w)
        else:
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = Bug3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

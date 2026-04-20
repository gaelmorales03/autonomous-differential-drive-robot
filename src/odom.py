#!/usr/bin/env python3

import rclpy  # Librería principal de ROS 2 para Python
import math   # Biblioteca para funciones matemáticas
import numpy as np  # Biblioteca para cálculo numérico y manejo de matrices
from rclpy.node import Node  # Clase base para nodos ROS 2
from std_msgs.msg import Float32  # Mensaje estándar de tipo float de 32 bits
from nav_msgs.msg import Odometry  # Mensaje para odometría
from sensor_msgs.msg import JointState  # Mensaje para estados de articulaciones
from aruco_opencv_msgs.msg import ArucoDetection  # Mensaje para detección de marcadores ArUco
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Calidad de servicio para suscripciones y publicaciones
import tf_transformations  # Utilidades para transformaciones de quaterniones y matrices

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odom_covarianza')  # Inicializa el nodo con nombre 'odom_covarianza'
        self.get_logger().info("Nodo de odometría con corrección ArUco iniciado")  # Mensaje informativo al iniciar
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # Perfil QoS para suscripciones
        self.pub_odom = self.create_publisher(Odometry, '/odom', 1)  # Publicador de mensajes Odometry en tópico /odom
        self.pub_js = self.create_publisher(JointState, 'estimated/joint_states', 1)  # Publicador de JointState para articulaciones estimadas
        # Suscripción a velocidades angulares de rueda derecha e izquierda (Float32)
        self.create_subscription(Float32, '/VelocityEncR', self.wR_cb, best_effort_qos)
        self.create_subscription(Float32, '/VelocityEncL', self.wL_cb, best_effort_qos)
        # Suscripción a detecciones ArUco con cola de tamaño 10
        self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_cb, 10)
        self.timer = self.create_timer(0.05, self.update_odometry)  # Temporizador que llama a update_odometry cada 0.05 segundos
        self.r = 0.0505  # Radio nominal de rueda (en metros)
        self.l = 0.183   # Distancia entre ruedas (ancho base) en metros
        self.Kr = 0.0297  # Constante de ruido para rueda derecha
        self.Kl = 0.0326  # Constante de ruido para rueda izquierda
        self.wL = 0.0  # Velocidad angular inicial rueda izquierda
        self.wR = 0.0  # Velocidad angular inicial rueda derecha
        self.prev_time = None  # Tiempo previo para cálculo de delta tiempo
        self.x = 0.0  # Posición X inicial estimada
        self.y = 0.0  # Posición Y inicial estimada
        self.theta = 0.0  # Orientación inicial (yaw) estimada
        self.P = np.diag([0.01, 0.01, 0.08])  # Matriz de covarianza inicial (incertidumbre en x, y, theta)
        self.R = np.diag([0.00000065626, 0.000000052843])  # Matriz de covarianza del ruido de medición
        self.I = np.eye(3)  # Matriz identidad 3x3 para cálculos matriciales
        # Posiciones conocidas en el mapa de marcadores ArUco por ID (x, y, theta)
        self.MarkersPosition = {
            1: {'map': [-1.5,  0.0,  1.56]},
            2: {'map': [ 1.3, 0.0, -1.56]},
            3: {'map': [ 0.0,  -1.03,  1.56]},
            4: {'map': [ 0.0,   1.03,  0.0]},
            5: {'map': [-1.5,  -0.15,  1.56]},
        }
        self.markers = []  # Lista para almacenar marcadores detectados en cada callback
        self.odom_msg = Odometry()  # Mensaje de odometría para publicar
        self.odom_msg.header.frame_id = 'world'  # Marco de referencia global
        self.odom_msg.child_frame_id = 'estimated/base_footprint'  # Marco del robot estimado
        self.js_msg = JointState()  # Mensaje para estado de articulaciones
        self.js_msg.name = ['estimated/wheel_right_joint', 'estimated/wheel_left_joint']  # Nombres de articulaciones

    def wL_cb(self, msg):
        self.wL = msg.data  # Callback para actualizar velocidad angular de rueda izquierda

    def wR_cb(self, msg):
        self.wR = msg.data  # Callback para actualizar velocidad angular de rueda derecha

    def aruco_cb(self, msg: ArucoDetection):
        self.markers = msg.markers  # Callback para actualizar la lista de marcadores detectados

    def update_odometry(self):
        now = self.get_clock().now()  # Obtiene el tiempo actual del nodo
        if self.prev_time is None:
            self.prev_time = now  # Si es la primera vez, solo guarda el tiempo y regresa
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9  # Calcula delta tiempo en segundos
        if dt == 0.0:
            return  # Si no pasó tiempo, no hace nada
        self.prev_time = now  # Actualiza el tiempo previo
        # Calcula velocidad lineal v y angular w del robot usando velocidades de rueda y radio/distancia base
        v = (self.wR + self.wL) * self.r / 2.0
        w = (self.wR - self.wL) * self.r / self.l

        # Matriz jacobiana Hk para propagación de covarianza en modelo de movimiento
        Hk = np.array([
            [1.0, 0.0, -dt * v * math.sin(self.theta)],
            [0.0, 1.0,  dt * v * math.cos(self.theta)],
            [0.0, 0.0,  1.0]
        ])
        # Matriz de ruido del proceso (basado en velocidades angulares ponderadas)
        self.L = np.diag([self.Kr * abs(self.wR), self.Kl * abs(self.wL)])
        # Matriz Fk que relaciona ruido de entrada con estados
        Fk = 0.5 * self.r * dt * np.array([
            [ math.cos(self.theta),  math.cos(self.theta)],
            [ math.sin(self.theta),  math.sin(self.theta)],
            [ 2.0 / self.l,         -2.0 / self.l]
        ])
        # Covarianza del ruido de proceso Qk
        Qk = Fk @ self.L @ Fk.T
        # Propaga covarianza P con modelo y ruido de proceso
        self.P = Hk @ self.P @ Hk.T + Qk
        # Actualiza estado de posición x, y y orientación theta
        self.x += dt * v * math.cos(self.theta)
        self.y += dt * v * math.sin(self.theta)
        self.theta = math.atan2(math.sin(self.theta + w * dt), math.cos(self.theta + w * dt))

        # Corrección con detección ArUco para cada marcador detectado
        for m in self.markers:
            MarkerID = m.marker_id  # ID del marcador detectado
            if MarkerID not in self.MarkersPosition:
                continue  # Ignora si no está en posiciones conocidas
            mx, my, _ = self.MarkersPosition[MarkerID]['map']  # Posición en mapa del marcador
            q = m.pose.orientation  # Orientación del marcador detectada (quaternion)
            p = m.pose.position  # Posición del marcador detectada
            # Construye matriz de rotación a partir del quaternion
            RotationCM = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[0:3, 0:3]
            TranslationCM = np.array([p.x, p.y, p.z])  # Vector de traslación del marcador
            MatrixMC = np.eye(4)  # Matriz homogénea 4x4 identidad
            MatrixMC[0:3, 0:3] = RotationCM  # Inserta rotación
            MatrixMC[0:3, 3] = TranslationCM  # Inserta traslación
            # Matrices de rotación para ajustar la orientación del marcador (corrección)
            Ry = tf_transformations.rotation_matrix(math.radians(90), (0, 1, 0))[0:3, 0:3]
            Rz = tf_transformations.rotation_matrix(math.radians(-90), (0, 0, 1))[0:3, 0:3]
            transformationCR = np.eye(4)  # Matriz homogénea para corrección
            transformationCR[0:3, 0:3] = Ry @ Rz  # Aplica rotaciones Ry y Rz
            transformationCR[0:3, 3] = np.array([0.07, 0.0, 0.07])  # Traslación de corrección
            # Combina corrección con matriz del marcador para obtener posición de la cámara en marco del robot
            T_RM = transformationCR @ MatrixMC
            xR, yR = T_RM[0, 3], T_RM[1, 3]  # Extrae posición X y Y corregida
            # Calcula distancia y ángulo desde la odometría estimada a la posición del marcador
            dx = mx - self.x
            dy = my - self.y
            odom_dist = math.hypot(dx, dy)  # Distancia desde odometría a marcador
            estimated_ang = math.atan2(dy, dx) - self.theta  # Ángulo estimado relativo a orientación robot
            cam_dist = math.hypot(xR, yR)  # Distancia medida por la cámara al marcador
            cam_ang = math.atan2(yR, xR)  # Ángulo medido por la cámara al marcador
            
            # Vector de observación Zk con distancia y ángulo medidos
            Zk = np.array([cam_dist, cam_ang])
            # Matriz Jacobiana Gk que relaciona observación con estado
            Gk = np.array([
                [-dx / odom_dist, -dy / odom_dist, 0.0],
                [ dy / (odom_dist ** 2), -dx / (odom_dist ** 2), -1.0]
            ])
            # Matriz de covarianza de la innovación
            S = Gk @ self.P @ Gk.T + self.R
            # Ganancia de Kalman para corregir estado
            KalmanGain = self.P @ Gk.T @ np.linalg.inv(S)
            # Diferencia angular normalizada para evitar saltos 2pi
            d_bearing = (Zk[1] - estimated_ang + math.pi) % (2 * math.pi) - math.pi
            dz = np.array([Zk[0] - odom_dist, d_bearing])  # Vector diferencia entre medición y estimación
            delta = KalmanGain @ dz  # Incremento a aplicar al estado
            # Actualiza estado con corrección Kalman
            self.x += delta[0]
            self.y += delta[1]
            self.theta += delta[2]
            # Actualiza covarianza posterior
            self.P = (self.I - KalmanGain @ Gk) @ self.P

        # Convierte ángulo theta a quaternion para publicar en mensaje Odometry
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        now_msg = now.to_msg()  # Obtiene tiempo en formato ROS2
        self.odom_msg.header.stamp = now_msg  # Estampa de tiempo para odometría
        # Actualiza posición y orientación en mensaje odometría
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        # Actualiza matriz de covarianza del pose con covarianza estimada P
        cov = self.odom_msg.pose.covariance
        cov[0] = self.P[0, 0]
        cov[1] = self.P[0, 1]
        cov[5] = self.P[0, 2]
        cov[6] = self.P[1, 0]
        cov[7] = self.P[1, 1]
        cov[11] = self.P[1, 2]
        cov[30] = self.P[2, 0]
        cov[31] = self.P[2, 1]
        cov[35] = self.P[2, 2]
        self.pub_odom.publish(self.odom_msg)  # Publica mensaje de odometría actualizado
        self.js_msg.header.stamp = now_msg  # Estampa de tiempo para JointState
        self.js_msg.position = [self.wR * dt, self.wL * dt]  # Posiciones estimadas de ruedas (basado en velocidad y dt)
        self.pub_js.publish(self.js_msg)  # Publica estado de articulaciones

def main(args=None):
    rclpy.init(args=args)  # Inicializa sistema ROS2
    node = OdometryNode()  # Crea instancia del nodo de odometría
    try:
        rclpy.spin(node)  # Ejecuta el nodo hasta que se interrumpa (Ctrl+C)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por usuario")  # Mensaje al detener el nodo manualmente
    node.destroy_node()  # Destruye el nodo
    rclpy.shutdown()  # Apaga ROS2

if __name__ == '__main__':
    main() 


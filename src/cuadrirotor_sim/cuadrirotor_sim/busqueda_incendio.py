"""Búsqueda del incendio con la cámara y YOLO, y órbita sobre el foco encontrado.

Cadena: imagen -> detección (detector_yolo) -> estimación de la
posición del foco (aquí) -> referencia (aquí) -> control (controlador_pd/sta
con mision:=externa). Este nodo nunca usa la posición verdadera del foco: solo
lo que reporta el detector.

Estados:
  despegue      sube en vertical a la altura de búsqueda (sobre las copas).
  patrulla      recorre en zigzag la ruta, con la cámara hacia adelante.
  hacia_humo    YOLO confirmó humo: vuela en esa dirección y acumula rumbos.
  aproximacion  hay estimación del foco: va al punto del anillo de órbita más
                cercano, a la altura de búsqueda, mirando al foco.
  descenso      baja dentro del claro hasta la altura de órbita.
  orbita        órbita circular con la cámara apuntando al foco estimado.

Estimación del foco:
  - Con llamas ('fuego'): el rayo de la cámara por el centro de la caja se corta
    con el plano z = 1.1 m (media altura de las llamas); se usa la mediana de
    los últimos cortes.
  - Con humo ('humo'): la columna es vertical sobre el foco, así que cada
    detección da un rumbo; con rumbos desde puntos distintos se triangula por
    mínimos cuadrados.
"""
from collections import deque
import math

from geometry_msgs.msg import PointStamped, Quaternion, Transform, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from vision_msgs.msg import Detection2DArray

from cuadrirotor_sim.dinamica import envolver, quat_a_rot

RUTA = [-35.0, -30.0, 45.0, -30.0, 45.0, -10.0, -35.0, -10.0,
        -35.0, 10.0, 45.0, 10.0, 45.0, 30.0, -35.0, 30.0]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class BusquedaIncendio(Node):

    def __init__(self):
        super().__init__('busqueda_incendio')
        p = self.declare_parameters('', [
            ('altura_busqueda', 15.0), ('altura_orbita', 3.0),
            ('radio', 4.0), ('omega', 0.3),
            ('velocidad', 2.5), ('velocidad_vertical', 1.0), ('aceleracion', 1.5),
            ('ruta', RUTA),
            ('confianza', 0.45), ('confirmar', 3),
            ('camara_xyz', [0.12, 0.0, -0.06]), ('camara_pitch', 0.5236),
            ('camara_hfov', 1.3), ('camara_ancho', 640), ('camara_alto', 480),
            ('periodo', 0.02),
        ])
        self.v = {x.name: x.value for x in p}
        self.ruta = np.reshape(self.v['ruta'], (-1, 2))
        self.f = (self.v['camara_ancho'] / 2) / math.tan(self.v['camara_hfov'] / 2)
        self.Rc = rot_y(self.v['camara_pitch'])

        self.odom = deque(maxlen=400)       # (t, p, R) para emparejar con las imágenes
        self.estado = None
        self.cmd_p = self.cmd_v = None
        self.cmd_yaw = 0.0
        self.i_ruta = 0
        self.racha = {'fuego': 0, 'humo': 0}
        self.cortes_fuego = deque(maxlen=15)
        self.rumbos_humo = deque(maxlen=40)  # (posición xy de la cámara, azimut)
        self.az_humo = None
        self.t_humo = 0.0
        self.foco = None
        self.entrada = None
        self.t_orbita = None

        self.pub = self.create_publisher(MultiDOFJointTrajectoryPoint, '/cuadrirotor/consigna', 10)
        self.pub_foco = self.create_publisher(PointStamped, '/cuadrirotor/foco_estimado', 10)
        self.pub_estado = self.create_publisher(String, '/cuadrirotor/busqueda/estado', 10)
        self.pub_inicio = self.create_publisher(Empty, '/cuadrirotor/mision/inicio', 1)
        self.create_subscription(Odometry, '/cuadrirotor/odometria', self._odom, 20)
        self.create_subscription(Detection2DArray, '/cuadrirotor/detecciones', self._detecciones, 10)
        self.create_timer(self.v['periodo'], self._paso)

    # ------------------------------------------------------------ utilidades
    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cambiar(self, estado, texto):
        self.estado = estado
        self.get_logger().info(f'[{estado}] {texto}')
        self.pub_estado.publish(String(data=estado))

    def _odom(self, msg):
        o = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom.append((t, np.array([o.position.x, o.position.y, o.position.z]),
                          quat_a_rot(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)))

    def _pose_en(self, t):
        return min(self.odom, key=lambda e: abs(e[0] - t))[1:]

    def _rayo(self, p, R, u, v):
        """Origen y dirección (mundo) del rayo de la cámara por el píxel (u, v)."""
        W, H = self.v['camara_ancho'], self.v['camara_alto']
        Rcam = R @ self.Rc
        d = Rcam @ np.array([1.0, -(u - W / 2) / self.f, -(v - H / 2) / self.f])
        return p + R @ np.array(self.v['camara_xyz']), d / np.linalg.norm(d)

    # ------------------------------------------------------------- detección
    def _detecciones(self, msg):
        if not self.odom or self.estado in (None, 'despegue', 'descenso', 'orbita'):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, R = self._pose_en(t)
        vistos = set()
        for d in msg.detections:
            h = d.results[0].hypothesis
            if h.score < self.v['confianza']:
                continue
            c = d.bbox.center.position
            o, r = self._rayo(p, R, c.x, c.y)
            if h.class_id == 'fuego' and r[2] < -0.05:
                s = (1.1 - o[2]) / r[2]
                corte = (o + s * r)[:2]
                # Compuerta: con un foco ya estimado por llamas, un corte a mas de 4 m
                # es un falso positivo (p. ej. un arbol tomado por fuego) y no se usa.
                ref = getattr(self, 'foco_llamas', None)
                if ref is not None and np.linalg.norm(corte - ref) > 4.0:
                    continue
                self.cortes_fuego.append(corte)
            elif h.class_id == 'humo':
                self.rumbos_humo.append((o[:2].copy(), math.atan2(r[1], r[0])))
                self.az_humo = math.atan2(r[1], r[0])
                self.t_humo = self._ahora()
            vistos.add(h.class_id)
        for k in self.racha:
            self.racha[k] = self.racha[k] + 1 if k in vistos else 0
        self._actualizar_estimacion()

    def _actualizar_estimacion(self):
        est = None
        if len(self.cortes_fuego) >= 5:
            est = np.median(np.array(self.cortes_fuego), axis=0)
            fuente = 'llamas'
            if getattr(self, 'foco_llamas', None) is None:
                self.foco_llamas = est.copy()
        elif len(self.rumbos_humo) >= 6:
            az = np.array([a for _, a in self.rumbos_humo])
            if np.ptp(np.unwrap(az)) > 0.25:
                A, b = np.zeros((2, 2)), np.zeros(2)
                for c, a in self.rumbos_humo:
                    n = np.array([math.cos(a), math.sin(a)])
                    P = np.eye(2) - np.outer(n, n)
                    A += P
                    b += P @ c
                est = np.linalg.solve(A, b)
                fuente = 'humo (triangulación)'
        if est is None:
            return
        nuevo = self.foco is None
        self.foco = est
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.point.x, m.point.y, m.point.z = float(est[0]), float(est[1]), 0.0
        self.pub_foco.publish(m)
        if nuevo:
            self.get_logger().info(f'Foco estimado con {fuente}: ({est[0]:.1f}, {est[1]:.1f})')

    # ------------------------------------------------------------ consignas
    def _seguir(self, objetivo, yaw_obj, dt):
        """Avanza la consigna hacia el objetivo con velocidad y aceleración limitadas."""
        e = objetivo - self.cmd_p
        vmax = np.array([self.v['velocidad']] * 2 + [self.v['velocidad_vertical']])
        amax = self.v['aceleracion']
        v_des = np.sign(e) * np.minimum(vmax, np.sqrt(2 * amax * np.abs(e)))
        exy = np.linalg.norm(e[:2])
        if exy > 1e-6:   # en el plano, avanzar en línea recta hacia el objetivo
            vxy = min(self.v['velocidad'], math.sqrt(2 * amax * exy))
            v_des[:2] = e[:2] / exy * vxy
        a = np.clip((v_des - self.cmd_v) / dt, -amax, amax)
        self.cmd_v = self.cmd_v + a * dt
        self.cmd_p = self.cmd_p + self.cmd_v * dt
        dyaw = envolver(yaw_obj - self.cmd_yaw)
        self.cmd_yaw = envolver(self.cmd_yaw + np.clip(dyaw, -0.6 * dt, 0.6 * dt))
        return a, np.linalg.norm(e)

    def _publicar(self, p, v, a, yaw):
        m = MultiDOFJointTrajectoryPoint()
        m.transforms = [Transform(translation=Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                  rotation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)))]
        m.velocities = [Twist(linear=Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2])))]
        m.accelerations = [Twist(linear=Vector3(x=float(a[0]), y=float(a[1]), z=float(a[2])))]
        self.pub.publish(m)

    def _paso(self):
        if not self.odom:
            return
        dt = self.v['periodo']
        p_dron = self.odom[-1][1]
        if self.estado is None:
            self.cmd_p, self.cmd_v = p_dron.copy(), np.zeros(3)
            R = self.odom[-1][2]
            self.cmd_yaw = math.atan2(R[1, 0], R[0, 0])
            self._cambiar('despegue', f"Subiendo a {self.v['altura_busqueda']} m")

        hb = self.v['altura_busqueda']
        if self.estado == 'despegue':
            obj = np.array([self.cmd_p[0], self.cmd_p[1], hb])
            a, err = self._seguir(obj, self.cmd_yaw, dt)
            if err < 0.05 and abs(p_dron[2] - hb) < 0.5:
                self._cambiar('patrulla', 'Recorriendo el bosque')
        elif self.estado == 'patrulla':
            wp = self.ruta[self.i_ruta]
            obj = np.array([wp[0], wp[1], hb])
            yaw = math.atan2(obj[1] - self.cmd_p[1], obj[0] - self.cmd_p[0])
            a, err = self._seguir(obj, yaw, dt)
            if err < 1.0:
                self.i_ruta = (self.i_ruta + 1) % len(self.ruta)
            if self.foco is not None:
                self._ir_al_foco()
            elif self.racha['humo'] >= self.v['confirmar']:
                self._cambiar('hacia_humo', f'Humo a la vista, rumbo {math.degrees(self.az_humo):.0f}°')
        elif self.estado == 'hacia_humo':
            obj = np.array([self.cmd_p[0] + 30 * math.cos(self.az_humo),
                            self.cmd_p[1] + 30 * math.sin(self.az_humo), hb])
            a, _ = self._seguir(obj, self.az_humo, dt)
            if self.foco is not None:
                self._ir_al_foco()
            elif self._ahora() - self.t_humo > 6.0:
                self._cambiar('patrulla', 'Se perdió el humo; sigue el recorrido')
        elif self.estado == 'aproximacion':
            self.entrada = self._punto_entrada()
            obj = np.array([*self.entrada, hb])
            yaw = math.atan2(self.foco[1] - self.cmd_p[1], self.foco[0] - self.cmd_p[0])
            a, err = self._seguir(obj, yaw, dt)
            if err < 0.3:
                self._cambiar('descenso', f'Sobre el anillo de órbita; foco en '
                                          f'({self.foco[0]:.1f}, {self.foco[1]:.1f})')
        elif self.estado == 'descenso':
            obj = np.array([*self.entrada, self.v['altura_orbita']])
            yaw = math.atan2(self.foco[1] - self.entrada[1], self.foco[0] - self.entrada[0])
            a, err = self._seguir(obj, yaw, dt)
            if err < 0.05 and abs(envolver(yaw - self.cmd_yaw)) < 0.05:
                self.t_orbita = self._ahora()
                self.th0 = math.atan2(self.entrada[1] - self.foco[1], self.entrada[0] - self.foco[0])
                self.pub_inicio.publish(Empty())
                self._cambiar('orbita', 'Orbitando el foco')
        elif self.estado == 'orbita':
            self._orbita()
            return
        self._publicar(self.cmd_p, self.cmd_v, a, self.cmd_yaw)

    def _ir_al_foco(self):
        self._cambiar('aproximacion', f'Hacia el foco estimado ({self.foco[0]:.1f}, {self.foco[1]:.1f})')

    def _punto_entrada(self):
        """Punto del anillo de órbita más cercano al dron."""
        d = self.cmd_p[:2] - self.foco
        n = np.linalg.norm(d)
        u = d / n if n > 1e-3 else np.array([1.0, 0.0])
        return self.foco + self.v['radio'] * u

    def _orbita(self):
        R, w, Tr = self.v['radio'], self.v['omega'], 4.0
        tau = self._ahora() - self.t_orbita
        if tau < Tr:
            th, dth, ddth = w * tau**2 / (2 * Tr), w * tau / Tr, w / Tr
        else:
            th, dth, ddth = w * (tau - Tr / 2), w, 0.0
        th += self.th0
        c, s = math.cos(th), math.sin(th)
        fx, fy = self.foco
        p = np.array([fx + R * c, fy + R * s, self.v['altura_orbita']])
        v = np.array([-R * s * dth, R * c * dth, 0.0])
        a = np.array([-R * (c * dth**2 + s * ddth), R * (-s * dth**2 + c * ddth), 0.0])
        self._publicar(p, v, a, th + math.pi)


def main():
    rclpy.init()
    nodo = BusquedaIncendio()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

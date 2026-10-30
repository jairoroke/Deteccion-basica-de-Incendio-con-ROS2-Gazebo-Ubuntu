"""Misión de observación del incendio ejecutada por ArduPilot SITL.

Usa la misma referencia que los controladores de ROS 2 (dinamica.Referencia):
traslado al foco y órbita apuntando la cámara hacia él. ArduCopter vuela en
modo GUIDED y recibe la posición, velocidad y guiñada deseadas por MAVLink
(SET_POSITION_TARGET_LOCAL_NED). El estado verdadero se toma de la odometría de
Gazebo y se guarda en el mismo formato CSV que controlador_pd, para comparar
con scripts/metricas.py.

Se ejecuta con el Python del entorno de ArduPilot (~/venv-ardupilot), que tiene
pymavlink y ve los paquetes de ROS 2.

Marcos: Gazebo usa x este, y norte, z arriba (ENU); ArduPilot usa norte, este,
abajo (NED). Por eso N = y, E = x, D = -z y la guiñada NED es pi/2 - psi.
"""
import csv
import math
import time

from nav_msgs.msg import Odometry
import numpy as np
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from cuadrirotor_sim.dinamica import Referencia, quat_a_rot, rot_a_euler

# Posición, velocidad y guiñada; se ignoran aceleración y tasa de guiñada.
MASCARA = 0b0000_1001_1100_0000


class MisionArduPilot(Node):

    def __init__(self):
        super().__init__('mision_ardupilot')
        p = self.declare_parameters('', [
            ('conexion', 'udp:127.0.0.1:14551'),
            ('mision', 'orbita'), ('altura', 3.0), ('foco', [8.0, 5.0]),
            ('radio', 4.0), ('omega', 0.3),
            ('periodo_comando', 0.05),
            ('archivo_csv', ''),
        ])
        v = {x.name: x.value for x in p}
        self.v = v
        self.args_ref = dict(mision=v['mision'], altura=v['altura'], foco=tuple(v['foco']),
                             radio=v['radio'], omega=v['omega'])
        self.odom = None
        self.estado = 'conectando'
        self.ref = None
        self.t0 = None
        self.t_estado = time.monotonic()
        self.ultimo = (None, None, 0.0)
        self.prearmado = False

        self.mav = mavutil.mavlink_connection(v['conexion'], source_system=250)
        self.get_logger().info(f"Esperando a ArduPilot en {v['conexion']} ...")

        self.pub_inicio = self.create_publisher(Empty, '/cuadrirotor/mision/inicio', 1)
        self.create_subscription(Odometry, '/cuadrirotor/odometria', self._odom, 10)
        self.create_timer(v['periodo_comando'], self._paso)
        self.create_timer(0.005, self._registrar)

        self.csv = None
        if v['archivo_csv']:
            self.archivo = open(v['archivo_csv'], 'w', newline='')
            self.csv = csv.writer(self.archivo)
            self.csv.writerow(['t', 'x', 'y', 'z', 'xd', 'yd', 'zd', 'phi', 'theta', 'psi',
                               'psid', 'T', 'tx', 'ty', 'tz', 'w0', 'w1', 'w2', 'w3'])

    # ---------------------------------------------------------------- estado
    def _odom(self, msg):
        self.odom = msg

    def _posicion(self):
        o = self.odom.pose.pose.position
        return np.array([o.x, o.y, o.z])

    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cambiar(self, estado, texto=None):
        self.estado = estado
        self.t_estado = time.monotonic()
        self.get_logger().info(texto or estado)

    def _leer_mavlink(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                return
            tipo = m.get_type()
            if tipo == 'STATUSTEXT':
                self.get_logger().info(f'ArduPilot: {m.text}')
            elif tipo == 'SYS_STATUS':
                self.prearmado = bool(m.onboard_control_sensors_health
                                      & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)

    # --------------------------------------------------------------- máquina
    def _paso(self):
        self._leer_mavlink()
        esperado = time.monotonic() - self.t_estado

        if self.estado == 'conectando':
            # mavutil fija target_system al recibir el primer latido del dron.
            if self.mav.target_system != 0:
                self.mav.mav.request_data_stream_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                self._cambiar('preparando', 'Conectado; esperando que el EKF esté listo')
        elif self.estado == 'preparando':
            if self.prearmado and self.odom is not None:
                self.mav.set_mode('GUIDED')
                self._cambiar('armando', 'Modo GUIDED; armando')
        elif self.estado == 'armando':
            if self.mav.motors_armed():
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.v['altura'])
                self._cambiar('despegando', f"Armado; despegando a {self.v['altura']} m")
            elif esperado > 2.0:
                self.mav.arducopter_arm()
                self.t_estado = time.monotonic()
        elif self.estado == 'despegando':
            if self._posicion()[2] > self.v['altura'] - 0.15 and esperado > 3.0:
                self.t0 = self._ahora()
                self.ref = Referencia(self._posicion(), **self.args_ref)
                self.pub_inicio.publish(Empty())
                self._cambiar('mision', f"Altura alcanzada; inicia la misión '{self.v['mision']}'")
        elif self.estado == 'mision':
            t = self._ahora() - self.t0
            pd, vd, _, psid = self.ref(t)
            self.ultimo = (pd, psid, t)
            self.mav.mav.set_position_target_local_ned_send(
                0, self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASCARA,
                pd[1], pd[0], -pd[2], vd[1], vd[0], -vd[2], 0, 0, 0,
                math.pi / 2 - psid, 0)

    def _registrar(self):
        if not self.csv or self.estado != 'mision' or self.odom is None:
            return
        pd, psid, _ = self.ultimo
        if pd is None:
            return
        o = self.odom.pose.pose.orientation
        phi, th, psi = rot_a_euler(quat_a_rot(o.x, o.y, o.z, o.w))
        t = self._ahora() - self.t0
        self.csv.writerow([f'{t:.4f}', *self._posicion(), *pd, phi, th, psi, psid,
                           0, 0, 0, 0, 0, 0, 0, 0])

    def destroy_node(self):
        if self.csv:
            self.archivo.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = MisionArduPilot()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

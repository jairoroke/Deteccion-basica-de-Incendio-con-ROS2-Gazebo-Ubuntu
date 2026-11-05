"""Detector de incendios con YOLO26 sobre la cámara del dron.

Lee /cuadrirotor/camara, corre el modelo (entrenado con scripts/entrenar_yolo.py
sobre imágenes del simulador, clases 'fuego' y 'humo') y publica:
  /cuadrirotor/detecciones   vision_msgs/Detection2DArray, con la marca de tiempo
                             de la imagen, para emparejarla con la pose del dron.
  /cuadrirotor/camara_yolo   la imagen con cada objeto encuadrado y etiquetado
                             (marcado.py); se ve en el panel "Detección YOLO".

Se ejecuta con el Python de ~/.venvs/yolo (ultralytics y PyTorch), que también
ve los paquetes de ROS 2. Con dispositivo = 'auto' usa la GPU si PyTorch tiene
CUDA y si no la CPU.
"""
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cuadrirotor_sim.marcado import detecciones_de, marcar


class DetectorYolo(Node):

    def __init__(self):
        super().__init__('detector_yolo')
        p = self.declare_parameters('', [
            ('modelo', ''), ('periodo', 0.1), ('dispositivo', 'auto'),
            # Umbral de confianza general y, si es mayor que cero, el de cada clase.
            ('confianza', 0.35), ('confianza_fuego', 0.0), ('confianza_humo', 0.0),
        ])
        v = {x.name: x.value for x in p}
        if not os.path.exists(v['modelo']):
            raise RuntimeError(f"No existe el modelo YOLO: {v['modelo']!r}")
        self.modelo = YOLO(v['modelo'])
        self.conf = v['confianza']
        self.umbral = {'fuego': v['confianza_fuego'] or self.conf,
                       'humo': v['confianza_humo'] or self.conf}
        self.disp = v['dispositivo']
        if self.disp == 'auto':
            self.disp = '0' if torch.cuda.is_available() else 'cpu'
        self.ultima = None
        self.create_subscription(Image, '/cuadrirotor/camara', self._imagen, 2)
        self.pub = self.create_publisher(Detection2DArray, '/cuadrirotor/detecciones', 10)
        self.pub_img = self.create_publisher(Image, '/cuadrirotor/camara_yolo', 2)
        # Calentar el modelo para que la primera imagen no tarde.
        self.modelo.predict(np.zeros((480, 640, 3), np.uint8), device=self.disp, verbose=False)
        self.create_timer(v['periodo'], self._procesar)
        self.get_logger().info(f"YOLO listo: {os.path.basename(v['modelo'])}, clases {self.modelo.names}, "
                               f"dispositivo {self.disp}, umbrales {self.umbral}")

    def _imagen(self, msg):
        self.ultima = msg

    def _procesar(self):
        msg, self.ultima = self.ultima, None
        if msg is None:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        t0 = time.perf_counter()
        r = self.modelo.predict(img[..., ::-1], conf=min(self.umbral.values()), device=self.disp,
                                verbose=False)[0]
        ms = (time.perf_counter() - t0) * 1000
        detecciones = detecciones_de(r, self.modelo.names, self.umbral, self.conf)

        salida = Detection2DArray(header=msg.header)
        for nombre, conf, (x0, y0, x1, y1) in detecciones:
            d = Detection2D(header=msg.header)
            d.bbox.center.position.x, d.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            d.bbox.size_x, d.bbox.size_y = x1 - x0, y1 - y0
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = nombre, conf
            d.results.append(h)
            salida.detections.append(d)
        self.pub.publish(salida)
        dibujo = marcar(img, detecciones, ms)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding='rgb8',
                    step=msg.width * 3, data=dibujo.tobytes())
        self.pub_img.publish(out)


def main():
    rclpy.init()
    nodo = DetectorYolo()
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

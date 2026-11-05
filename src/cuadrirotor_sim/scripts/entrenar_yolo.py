#!/usr/bin/env python3
"""Ajusta YOLO26 a las clases 'fuego' y 'humo' con datasets del simulador.

Por omisión parte de los pesos preentrenados en COCO (yolo26n.pt, el modelo más
rápido de la familia, el recomendado para drones) y entrena con uno o varios
datasets de scripts/capturar_dataset.py. Copia el mejor modelo a
yolo/incendio_yolo26n.pt del paquete (o a --salida), que es el que usa detector_yolo.

Uso (con el Python de ~/.venvs/yolo):
    # Desde COCO, un dataset:
    ~/.venvs/yolo/bin/python scripts/entrenar_yolo.py ~/datasets/incendio_bosque [épocas]
    # Ajuste fino del modelo del paquete con imágenes de los dos mundos y las
    # primeras capas congeladas:
    ~/.venvs/yolo/bin/python scripts/entrenar_yolo.py ~/datasets/mundo_incendio ~/datasets/mundo_bosque \\
        --desde yolo/incendio_yolo26n.pt --epocas 10 --congelar 10 --lr0 0.001

Usa la GPU si PyTorch tiene CUDA y si no la CPU (--dispositivo para elegir).
"""
import argparse
import os
import shutil

import torch
from ultralytics import YOLO

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def datos_combinados(datasets, carpeta):
    """data.yaml con las particiones train y val de todos los datasets."""
    if len(datasets) == 1:
        return os.path.join(datasets[0], 'data.yaml')
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, 'datos_combinados.yaml')
    with open(ruta, 'w') as f:
        f.write('train:\n' + ''.join(f'  - {d}/images/train\n' for d in datasets))
        f.write('val:\n' + ''.join(f'  - {d}/images/val\n' for d in datasets))
        f.write('names:\n  0: fuego\n  1: humo\n')
    return ruta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('datasets', nargs='+', help='carpetas de capturar_dataset.py (y, al final, las épocas)')
    ap.add_argument('--epocas', type=int, default=None)
    ap.add_argument('--desde', default='yolo26n.pt', help='pesos iniciales')
    ap.add_argument('--congelar', type=int, default=0, help='capas iniciales que no se entrenan')
    ap.add_argument('--lr0', type=float, default=None, help='tasa de aprendizaje inicial')
    ap.add_argument('--lote', type=int, default=None)
    ap.add_argument('--dispositivo', default='auto')
    ap.add_argument('--salida', default=os.path.join(RAIZ, 'yolo', 'incendio_yolo26n.pt'))
    a = ap.parse_args()
    if a.datasets[-1].isdigit():   # forma antigua: entrenar_yolo.py <dataset> [épocas]
        a.epocas = a.epocas or int(a.datasets.pop())
    epocas = a.epocas or 60
    datasets = [os.path.abspath(os.path.expanduser(d)) for d in a.datasets]
    gpu = torch.cuda.is_available()
    disp = (0 if gpu else 'cpu') if a.dispositivo == 'auto' else a.dispositivo
    proyecto = os.path.join(datasets[0], 'entrenamientos')
    datos = datos_combinados(datasets, proyecto)
    desde = a.desde if os.path.isabs(a.desde) or not os.path.exists(os.path.join(RAIZ, a.desde)) \
        else os.path.join(RAIZ, a.desde)
    print(f'Entrenando desde {desde} con {datos} en {disp}: {epocas} épocas, '
          f'{a.congelar} capas congeladas', flush=True)

    extra = {'lr0': a.lr0, 'warmup_epochs': 1} if a.lr0 else {}
    modelo = YOLO(desde)
    modelo.train(data=datos, epochs=epocas, imgsz=640, batch=a.lote or (32 if gpu else 16), device=disp,
                 workers=min(8, os.cpu_count() or 1), project=proyecto, name='yolo26n_incendio',
                 exist_ok=True, patience=20, freeze=a.congelar or None,
                 # Aumentos: el simulador tiene luz fija; variar color y escala.
                 hsv_h=0.015, hsv_s=0.5, hsv_v=0.4, scale=0.5, mosaic=1.0, fliplr=0.5, **extra)
    mejor = os.path.join(proyecto, 'yolo26n_incendio', 'weights', 'best.pt')
    metricas = YOLO(mejor).val(data=datos, imgsz=640, device=disp, verbose=False,
                               project=proyecto, name='validacion', exist_ok=True)
    print(f'mAP50 = {metricas.box.map50:.3f}, mAP50-95 = {metricas.box.map:.3f}')
    for i, nombre in metricas.names.items():
        print(f'  {nombre}: mAP50-95 = {metricas.box.maps[i]:.3f}')
    os.makedirs(os.path.dirname(os.path.abspath(a.salida)), exist_ok=True)
    shutil.copy(mejor, a.salida)
    print(f'Modelo copiado a {a.salida}')


if __name__ == '__main__':
    main()

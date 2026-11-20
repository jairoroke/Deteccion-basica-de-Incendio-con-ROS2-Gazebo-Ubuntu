#!/usr/bin/env python3
"""Revisa un modelo YOLO (.pt) antes de usarlo en la simulación.

Comprueba lo que necesita el detector de este proyecto:
  1. que el archivo cargue con ultralytics y sea un modelo de detección;
  2. que sus clases se llamen 'fuego' y 'humo' (busqueda_incendio usa esos nombres);
  3. cómo detecta en imágenes de la cámara del dron en Gazebo (yolo/imagenes_prueba/,
     o las que se den con --imagenes), cuánto tarda y si hay detecciones falsas.
Guarda las imágenes marcadas igual que el panel "Detección YOLO" de Gazebo.

Si las clases tienen otro nombre (p. ej. 'fire' y 'smoke'), --renombrar escribe una
copia del modelo con los nombres que espera el proyecto.

Uso (desde src/cuadrirotor_sim, con el Python de ~/.venvs/yolo):
    ~/.venvs/yolo/bin/python scripts/probar_modelo.py ~/Descargas/best.pt
    ~/.venvs/yolo/bin/python scripts/probar_modelo.py best.pt --imagenes mis_fotos/*.jpg
    ~/.venvs/yolo/bin/python scripts/probar_modelo.py best.pt --renombrar fire=fuego smoke=humo \\
        --salida yolo/modelo_equipo1.pt
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
from PIL import Image

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from cuadrirotor_sim.cajas import detecciones_de  # noqa: E402
from cuadrirotor_sim.marcado import marcar  # noqa: E402

CLASES = ('fuego', 'humo')
PRUEBA = os.path.join(RAIZ, 'yolo', 'imagenes_prueba')
# Nombres que suelen tener las clases en otros datasets, para sugerir cómo renombrarlas.
PISTAS = {'fuego': ('fire', 'flame', 'llama', 'fuego'), 'humo': ('smoke', 'humo')}


def sugerir_renombre(nombres, faltan):
    """Cambios 'viejo=nuevo' para las clases que faltan, adivinados por el nombre."""
    cambios = {}
    for clase in faltan:
        for n in nombres:
            if n not in cambios and n not in CLASES and any(p in n.lower() for p in PISTAS[clase]):
                cambios[n] = clase
                break
    # Si no se adivinó, por posición: la primera clase libre para cada una que falta.
    libres = [n for n in nombres if n not in cambios and n not in CLASES]
    for clase in faltan:
        if clase not in cambios.values() and libres:
            cambios[libres.pop(0)] = clase
    return ' '.join(f'{v}={n}' for v, n in cambios.items())


def esperado(ruta):
    """Clases que deberían verse en una imagen de prueba, según su nombre."""
    n = os.path.basename(ruta)
    if 'sin_fuego' in n:
        return set()
    return {c for c in CLASES if c in n}


def renombrar(origen, destino, cambios):
    """Copia del modelo con las clases renombradas, p. ej. {'fire': 'fuego'}."""
    import torch
    ck = torch.load(origen, map_location='cpu', weights_only=False)
    for clave in ('model', 'ema'):
        m = ck.get(clave)
        if m is None:
            continue
        nombres = m.names if isinstance(m.names, dict) else dict(enumerate(m.names))
        faltan = set(cambios) - set(nombres.values())
        if faltan:
            sys.exit(f'El modelo no tiene las clases {sorted(faltan)}; sus clases son {list(nombres.values())}')
        m.names = {i: cambios.get(n, n) for i, n in nombres.items()}
    os.makedirs(os.path.dirname(os.path.abspath(destino)), exist_ok=True)
    torch.save(ck, destino)
    print(f'Modelo con clases renombradas guardado en {destino}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('modelo', help='archivo de pesos (.pt) entrenado con ultralytics')
    ap.add_argument('--imagenes', nargs='+', help='imágenes para probar (por omisión, yolo/imagenes_prueba)')
    ap.add_argument('--confianza', type=float, default=0.35, help='umbral, el mismo que usa el detector')
    ap.add_argument('--renombrar', nargs='+', metavar='VIEJO=NUEVO', help='p. ej. fire=fuego smoke=humo')
    ap.add_argument('--salida', help='con --renombrar: dónde guardar el modelo renombrado')
    ap.add_argument('--resultados', default='prueba_modelo', help='carpeta para las imágenes marcadas')
    a = ap.parse_args()

    if not os.path.isfile(a.modelo):
        sys.exit(f'No existe el archivo {a.modelo}')
    try:
        import torch
        import ultralytics
        from ultralytics import YOLO
    except ImportError:
        sys.exit('Falta ultralytics: corre este script con ~/.venvs/yolo/bin/python (ver el README, Instalar)')

    modelo_pt = a.modelo
    if a.renombrar:
        if not a.salida:
            sys.exit('Con --renombrar indica también --salida, p. ej. --salida yolo/modelo_equipo1.pt')
        cambios = dict(r.split('=', 1) for r in a.renombrar)
        renombrar(a.modelo, a.salida, cambios)
        modelo_pt = a.salida

    problemas = []
    print(f'\n== 1. Modelo: {modelo_pt}  ({os.path.getsize(modelo_pt) / 1e6:.1f} MB, '
          f'ultralytics {ultralytics.__version__})')
    try:
        modelo = YOLO(modelo_pt)
    except Exception as e:   # versión de ultralytics distinta, archivo dañado, etc.
        sys.exit(f'No se pudo cargar el modelo: {e}\n'
                 'Si se entrenó con otra versión de ultralytics, actualízala en el entorno de YOLO:\n'
                 '    ~/.venvs/yolo/bin/pip install -U ultralytics "numpy<2" "opencv-python<4.12" "setuptools<80"')
    print(f'   tarea: {modelo.task}')
    if modelo.task != 'detect':
        problemas.append(f"es un modelo de '{modelo.task}', no de detección (cajas): entrena con yolo26n.pt o similar")
    nombres = list(modelo.names.values())
    print(f'   clases: {modelo.names}')
    try:
        imgsz = dict(modelo.ckpt.get('train_args', {})).get('imgsz')
    except Exception:
        imgsz = None
    if imgsz:
        print(f'   entrenado con imágenes de {imgsz} px (la cámara del dron da 640x480)')

    print('\n== 2. Nombres de las clases')
    faltan = [c for c in CLASES if c not in nombres]
    if faltan:
        problemas.append(f'faltan las clases {faltan}: busqueda_incendio solo usa "fuego" y "humo"')
        print(f'   FALTA: {faltan}. Renombra con, por ejemplo (revisa que cada nombre vaya con su clase):')
        print(f'   --renombrar {sugerir_renombre(nombres, faltan)} --salida yolo/mi_modelo.pt')
    else:
        print("   OK: tiene 'fuego' y 'humo'")
        extra = [n for n in nombres if n not in CLASES]
        if extra:
            print(f'   (las clases {extra} se dibujan en blanco y la búsqueda las ignora)')

    rutas = a.imagenes or sorted(glob.glob(os.path.join(PRUEBA, '*.jpg')))
    if not rutas:
        sys.exit('No hay imágenes para probar')
    disp = 0 if torch.cuda.is_available() else 'cpu'
    print(f"\n== 3. Prueba con {len(rutas)} imágenes (umbral {a.confianza}, {'GPU' if disp == 0 else 'CPU'})")
    os.makedirs(a.resultados, exist_ok=True)
    modelo.predict(np.zeros((480, 640, 3), np.uint8), device=disp, verbose=False)   # calentar
    tiempos = []
    for ruta in rutas:
        img = np.array(Image.open(ruta).convert('RGB'))
        t0 = time.perf_counter()
        r = modelo.predict(img[..., ::-1], conf=a.confianza, device=disp, verbose=False)[0]
        ms = (time.perf_counter() - t0) * 1000
        tiempos.append(ms)
        det = detecciones_de(r, modelo.names, {}, a.confianza)
        Image.fromarray(marcar(img, det, ms)).save(os.path.join(a.resultados, os.path.basename(ruta)))
        vistas = {n for n, _, _ in det}
        texto = ', '.join(f'{n} {c:.2f}' for n, c, _ in det) or 'nada'
        linea = f'   {os.path.basename(ruta):28s} -> {texto}'
        if a.imagenes is None:   # imágenes de prueba: se sabe qué debería verse
            esp = esperado(ruta)
            faltan_aqui = esp - vistas                          # no vio lo que hay
            falsas = not esp and bool(vistas & set(CLASES))     # vio fuego o humo donde no hay
            mal = bool(faltan_aqui) or falsas
            linea += '   ' + ('OK' if not mal else f'REVISAR (se esperaba: {", ".join(sorted(esp)) or "nada"})')
            if mal:
                problemas.append(f'{os.path.basename(ruta)}: detectó {texto}, se esperaba '
                                 f'{", ".join(sorted(esp)) or "nada"}')
        print(linea)
    print(f'   tiempo medio: {np.mean(tiempos):.0f} ms por imagen')
    print(f'   imágenes marcadas en: {os.path.abspath(a.resultados)}')

    print('\n== Resultado')
    if problemas:
        for p in problemas:
            print(f'   - {p}')
        print('   Corrige lo anterior antes de usar el modelo en la simulación (ver el tutorial).')
        sys.exit(1)
    print(f'   El modelo está listo. Para usarlo en Gazebo:\n'
          f'   ros2 launch cuadrirotor_sim simulacion.launch.py mision:=busqueda controlador:=sta '
          f'modelo_yolo:={os.path.abspath(modelo_pt)}')


if __name__ == '__main__':
    main()

"""Marcado de las detecciones de YOLO sobre la imagen de la cámara.

No depende de ROS: lo usa detector_yolo y puede probarse por separado. Una
detección es (clase, confianza, (x0, y0, x1, y1)) en píxeles (ver cajas.py).

Sobre la imagen se dibuja:
  - cada objeto con su caja y una etiqueta con la clase y la confianza;
  - el objetivo (el fuego más confiable o, si no hay fuego, el humo más
    confiable) encuadrado con esquinas gruesas, con una línea desde el centro de
    la imagen hasta su centro;
  - una línea de estado con el tiempo de inferencia y el conteo por clase.
"""
import cv2

from cuadrirotor_sim.cajas import objetivo

COLORES = {'fuego': (255, 60, 0), 'humo': (0, 200, 255)}   # RGB
FUENTE = cv2.FONT_HERSHEY_SIMPLEX


def _etiqueta(img, texto, x, y, color):
    """Texto con fondo del color de la clase, arriba de (x, y) o abajo si no cabe."""
    (w, h), base = cv2.getTextSize(texto, FUENTE, 0.5, 1)
    y0 = y - h - base - 4 if y - h - base - 4 >= 0 else y
    x = min(max(x, 0), img.shape[1] - w - 6)
    cv2.rectangle(img, (x, y0), (x + w + 6, y0 + h + base + 4), color, -1)
    tinta = (0, 0, 0) if sum(color) > 400 else (255, 255, 255)
    cv2.putText(img, texto, (x + 3, y0 + h + 2), FUENTE, 0.5, tinta, 1, cv2.LINE_AA)


def _esquinas(img, x0, y0, x1, y1, color, largo, grosor):
    for (cx, cy, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(img, (cx, cy), (cx + dx * largo, cy), color, grosor, cv2.LINE_AA)
        cv2.line(img, (cx, cy), (cx, cy + dy * largo), color, grosor, cv2.LINE_AA)


def marcar(img, detecciones, ms=None, titulo='YOLO26'):
    """Dibuja las detecciones sobre una copia de img (RGB, uint8) y la devuelve."""
    out = img.copy()
    H, W = out.shape[:2]
    for clase, conf, (x0, y0, x1, y1) in detecciones:
        c = COLORES.get(clase, (255, 255, 255))
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        cv2.rectangle(out, (x0, y0), (x1, y1), c, 2)
        _etiqueta(out, f'{clase} {conf:.2f}', x0, y0, c)

    obj = objetivo(detecciones)
    cx, cy = W // 2, H // 2
    cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
    if obj is not None:
        clase, _, (x0, y0, x1, y1) = obj
        c = COLORES.get(clase, (255, 255, 255))
        m = 4   # las esquinas van un poco por fuera de la caja
        x0, y0 = max(int(x0) - m, 0), max(int(y0) - m, 0)
        x1, y1 = min(int(x1) + m, W - 1), min(int(y1) + m, H - 1)
        largo = max(6, min(x1 - x0, y1 - y0) // 4)
        _esquinas(out, x0, y0, x1, y1, c, largo, 4)
        ox, oy = (x0 + x1) // 2, (y0 + y1) // 2
        cv2.line(out, (cx, cy), (ox, oy), c, 1, cv2.LINE_AA)
        cv2.drawMarker(out, (ox, oy), c, cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)

    conteo = '  '.join(f'{k}: {sum(d[0] == k for d in detecciones)}' for k in COLORES)
    estado = f'{titulo}  {ms:.0f} ms  |  {conteo}' if ms is not None else f'{titulo}  |  {conteo}'
    (w, h), base = cv2.getTextSize(estado, FUENTE, 0.55, 1)
    cv2.rectangle(out, (0, 0), (w + 12, h + base + 10), (0, 0, 0), -1)
    cv2.putText(out, estado, (6, h + 5), FUENTE, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out

"""Cajas de detección de YOLO: filtrado, objetivo y recorte por el borde.

No depende de ROS ni de OpenCV: lo usan detector_yolo, marcado y busqueda_incendio.
Una detección es (clase, confianza, (x0, y0, x1, y1)) en píxeles.
"""


def detecciones_de(resultado, nombres, umbral, umbral_defecto=0.35):
    """Detecciones de un resultado de ultralytics, filtradas con el umbral de cada clase.

    umbral: {clase: confianza mínima}; las clases que no aparecen usan umbral_defecto.
    """
    salida = []
    cajas = resultado.boxes
    for caja, clase, conf in zip(cajas.xyxy.cpu().numpy(), cajas.cls.cpu().numpy(),
                                 cajas.conf.cpu().numpy()):
        nombre = nombres[int(clase)]
        if conf >= umbral.get(nombre, umbral_defecto):
            salida.append((nombre, float(conf), tuple(float(c) for c in caja)))
    return salida


def objetivo(detecciones):
    """Detección principal: el fuego más confiable o, si no hay, el humo más confiable."""
    for clase in ('fuego', 'humo'):
        candidatas = [d for d in detecciones if d[0] == clase]
        if candidatas:
            return max(candidatas, key=lambda d: d[1])
    return None


def toca_borde(caja, ancho, alto, margen=2.0):
    """Lados de la imagen que toca la caja (x0, y0, x1, y1): subconjunto de 'izq', 'der', 'arr', 'aba'.

    Una caja recortada por el borde no tiene el centro del objeto, así que su
    centro no sirve para ubicarlo.
    """
    x0, y0, x1, y1 = caja
    return {lado for lado, toca in (('izq', x0 <= margen), ('der', x1 >= ancho - 1 - margen),
                                    ('arr', y0 <= margen), ('aba', y1 >= alto - 1 - margen)) if toca}

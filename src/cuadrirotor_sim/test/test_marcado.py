import numpy as np
import pytest

pytest.importorskip('cv2')
from cuadrirotor_sim.marcado import COLORES, marcar  # noqa: E402

FUEGO = ('fuego', 0.9, (300, 200, 360, 260))
HUMO = ('humo', 0.95, (100, 20, 200, 120))


def test_marcar_encuadra_sin_modificar_la_imagen_original():
    img = np.full((480, 640, 3), 90, np.uint8)
    out = marcar(img, [FUEGO, HUMO], ms=16)
    assert out.shape == img.shape and out.dtype == np.uint8
    assert (img == 90).all()
    # El borde de la caja del objetivo queda del color del fuego.
    x0, y0, x1, y1 = FUEGO[2]
    assert tuple(out[(y0 + y1) // 2, x0]) == COLORES['fuego']
    # Sin detecciones solo se dibuja la mira central y la línea de estado.
    vacia = marcar(img, [])
    assert (vacia[300:, :] == 90).sum() > 0.99 * vacia[300:, :].size

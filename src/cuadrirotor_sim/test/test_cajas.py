import numpy as np

from cuadrirotor_sim.cajas import detecciones_de, objetivo, toca_borde

FUEGO_BAJO = ('fuego', 0.5, (10, 10, 50, 50))
FUEGO_ALTO = ('fuego', 0.9, (300, 200, 360, 260))
HUMO = ('humo', 0.95, (100, 20, 200, 120))


def test_objetivo_prefiere_el_fuego_mas_confiable():
    assert objetivo([HUMO, FUEGO_BAJO, FUEGO_ALTO]) == FUEGO_ALTO
    assert objetivo([HUMO]) == HUMO
    assert objetivo([]) is None


class _Tensor:
    def __init__(self, a):
        self.a = np.asarray(a, dtype=float)

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class _Resultado:
    """Imita el resultado de ultralytics: boxes.xyxy, boxes.cls y boxes.conf."""

    def __init__(self, cajas, clases, confs):
        self.boxes = type('B', (), dict(xyxy=_Tensor(cajas), cls=_Tensor(clases), conf=_Tensor(confs)))()


def test_detecciones_filtra_con_el_umbral_de_cada_clase():
    r = _Resultado([[0, 0, 10, 10], [5, 5, 20, 20], [1, 2, 3, 4]], [0, 1, 1], [0.4, 0.4, 0.7])
    d = detecciones_de(r, {0: 'fuego', 1: 'humo'}, {'fuego': 0.35, 'humo': 0.5})
    assert [(n, round(c, 2)) for n, c, _ in d] == [('fuego', 0.4), ('humo', 0.7)]
    assert d[1][2] == (1.0, 2.0, 3.0, 4.0)


def test_toca_borde():
    assert toca_borde((100, 100, 200, 200), 640, 480) == set()
    assert toca_borde((0, 300, 50, 479), 640, 480) == {'izq', 'aba'}
    assert toca_borde((600, 0.5, 638, 40), 640, 480) == {'der', 'arr'}

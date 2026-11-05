import pytest

pytest.importorskip('MAVProxy')
from cuadrirotor_sim.geo import enu_a_geodesica  # noqa: E402
from cuadrirotor_sim.mavproxy_trayectoria import distancia, encuadre  # noqa: E402


def test_encuadre_centra_la_ruta_con_margen():
    # Rectángulo de 100 m (este) por 60 m (norte) con centro en (10, 20) del mundo.
    ruta = [enu_a_geodesica(x, y, 0.0)[:2] for x, y in ((-40, -10), (60, -10), (60, 50), (-40, 50))]
    centro, ancho = encuadre(ruta)
    assert distancia(centro, enu_a_geodesica(10, 20, 0.0)[:2]) < 0.5
    assert ancho == pytest.approx(1.6 * 100, rel=0.01)


def test_encuadre_minimo_para_rutas_cortas():
    ruta = [enu_a_geodesica(x, 0, 0.0)[:2] for x in (0, 5)]
    assert encuadre(ruta)[1] == 80.0

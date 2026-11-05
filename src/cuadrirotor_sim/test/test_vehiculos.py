"""Coherencia de los vehículos de scripts/generar_modelo.py con los archivos del paquete."""
import importlib.util
import os
import re

import numpy as np
import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location(
    'generar_modelo', os.path.join(RAIZ, 'scripts', 'generar_modelo.py'))
gm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gm)


def leer(*ruta):
    with open(os.path.join(RAIZ, *ruta)) as f:
        return f.read()


@pytest.mark.parametrize('nombre', list(gm.VEHICULOS))
def test_archivos_generados_al_dia(nombre):
    # Si falla: correr python3 scripts/generar_modelo.py y no editar a mano los generados.
    v = gm.VEHICULOS[nombre]
    assert leer('models', nombre, 'model.sdf') == gm.sdf_vehiculo(nombre, v)
    assert leer('config', f'vehiculo_{nombre}.yaml') == gm.yaml_vehiculo(nombre, v)
    if v.get('ardupilot'):
        assert leer('models', f'{nombre}_ardupilot', 'model.sdf') == gm.sdf_ardupilot(nombre, v)


def test_masa_inercia_de_una_caja_y_dos_masas_puntuales():
    m, c, I = gm.masa_inercia([('caja', 2.0, (0.1, 0.0, 0.0), (0.3, 0.2, 0.1))])
    assert m == 2.0 and np.allclose(c, [0.1, 0.0, 0.0])
    assert np.allclose(I, 2.0 / 12 * np.array([0.2**2 + 0.1**2, 0.3**2 + 0.1**2, 0.3**2 + 0.2**2]))
    m, c, I = gm.masa_inercia([('a', 1.0, (0.0, 0.0, 1.0), (0, 0, 0)),
                               ('b', 1.0, (0.0, 0.0, -1.0), (0, 0, 0))])
    assert np.allclose(c, 0.0) and np.allclose(I, [2.0, 2.0, 0.0])


def test_tarot650pro_centro_de_masa_bajo_el_centro_del_marco():
    v = gm.VEHICULOS['tarot650pro']
    assert np.hypot(*gm._CDM_PRO[:2]) < 1e-3
    assert -0.1 < v['cdm_z'] < 0.0
    # El presupuesto incluye el armazón completo del fabricante (476 g).
    armazon = sum(p[1] for p in gm.PIEZAS_TAROT650PRO
                  if p[0] in ('placas centrales', 'brazo y soporte de motor', 'tren de aterrizaje'))
    assert armazon == pytest.approx(0.476)


def fraccion_estacionario(v):
    """Empuje de vuelo estacionario por rotor entre el empuje máximo."""
    masa = v['masa_cuerpo'] + len(v['rotores']) * v['masa_rotor']
    return masa * 9.81 / len(v['rotores']) / (v['kf'] * v['w_max'] ** 2)


@pytest.mark.parametrize('nombre', list(gm.VEHICULOS))
def test_vuelo_estacionario_con_margen(nombre):
    # Relación empuje/peso entre 2 y 6: margen para controlar sin saturar.
    assert 1 / 6 < fraccion_estacionario(gm.VEHICULOS[nombre]) < 0.5


@pytest.mark.parametrize('nombre', [n for n, v in gm.VEHICULOS.items() if v.get('ardupilot')])
def test_parametros_de_ardupilot_coinciden_con_el_modelo(nombre):
    parm = leer('config', 'ardupilot', f'{nombre}.parm')
    hover = float(re.search(r'^MOT_THST_HOVER\s+([\d.]+)', parm, re.M).group(1))
    assert hover == pytest.approx(fraccion_estacionario(gm.VEHICULOS[nombre]), abs=0.01)


@pytest.mark.parametrize('nombre', [n for n, v in gm.VEHICULOS.items() if 'mallas' in v])
def test_mallas_existen(nombre):
    v = gm.VEHICULOS[nombre]
    carpeta = os.path.join(RAIZ, 'models', v.get('mallas_de', nombre), 'meshes')
    nombres = [m for m, _ in v['mallas']]
    if v.get('helices_cad'):
        nombres += [f'helice_{i}' for i in range(len(v['rotores']))]
    for m in nombres:
        assert os.path.isfile(os.path.join(carpeta, f'{m}.stl')), m

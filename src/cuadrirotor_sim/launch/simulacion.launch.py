"""Lanza Gazebo con el escenario del incendio, el puente ROS 2 y el controlador.

Ejemplos:
  ros2 launch cuadrirotor_sim simulacion.launch.py
  ros2 launch cuadrirotor_sim simulacion.launch.py mision:=orbita viento:=true
  ros2 launch cuadrirotor_sim simulacion.launch.py gui:=false csv:=/tmp/corrida.csv
  ros2 launch cuadrirotor_sim simulacion.launch.py controlador:=sta esquema:=implicito
  ros2 launch cuadrirotor_sim simulacion.launch.py mundo:=bosque mision:=orbita
  ros2 launch cuadrirotor_sim simulacion.launch.py vehiculo:=tarot650 mundo:=bosque
  ros2 launch cuadrirotor_sim simulacion.launch.py vehiculo:=tarot650 mundo:=bosque controlador:=ardupilot mision:=orbita
  ros2 launch cuadrirotor_sim simulacion.launch.py vehiculo:=tarot650 mundo:=bosque controlador:=sta mision:=busqueda

Con controlador:=ardupilot el dron lo pilota ArduCopter SITL (~/ardupilot, o la
ruta de ARDUPILOT_DIR) con el Python de ~/venv-ardupilot (o ARDUPILOT_VENV), y
el nodo mision_ardupilot le manda la misma referencia que a los controladores
de ROS 2.
"""
import os
import signal
import tempfile
import time

import yaml

from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                          get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def terminar_restos(event, context):
    """Al cerrar, termina lo que el launch no alcanza a cerrar solo.

    'gz sim' corre detrás de un envoltorio de ruby ('sh -c' de gz_sim.launch.py) que
    no reenvía la señal, y sim_vehicle.py no siempre cierra ArduCopter ni MAVProxy.
    Con Ctrl+C en una terminal no hace falta (la señal llega a todo el grupo), pero sí
    al cerrar la ventana de la terminal o cuando solo el launch recibe la señal. Solo
    toca los procesos de esta simulación: el gz sim con un mundo de cuadrirotor_sim
    (y sus hijos), el SITL de ArduCopter y el MAVProxy conectado a él.
    """
    procesos = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/stat') as f:
                ppid = int(f.read().rsplit(')', 1)[1].split()[1])
            if os.stat(f'/proc/{pid}').st_uid == os.getuid():
                procesos[int(pid)] = (ppid, cmd)
        except (OSError, ValueError, IndexError):
            continue
    raices = {p for p, (_, c) in procesos.items()
              if (c.startswith('gz sim') and 'cuadrirotor_sim' in c)
              or 'build/sitl/bin/arducopter' in c
              or ('mavproxy.py' in c and 'tcp:127.0.0.1:5760' in c)}
    todos, nuevos = set(raices), set(raices)
    while nuevos:
        nuevos = {p for p, (pp, _) in procesos.items() if pp in nuevos} - todos
        todos |= nuevos
    todos.discard(os.getpid())
    for sig in (signal.SIGTERM, signal.SIGKILL):
        vivos = []
        for p in todos:
            try:
                os.kill(p, sig)
                vivos.append(p)
            except OSError:
                pass
        if not vivos:
            break
        time.sleep(2.0)


def aparecer_vehiculo(context, share):
    """Agrega el vehículo elegido al mundo, siempre con el nombre 'cuadrirotor'."""
    vehiculo = LaunchConfiguration('vehiculo').perform(context)
    with open(os.path.join(share, 'config', f'vehiculo_{vehiculo}.yaml')) as f:
        z0 = yaml.safe_load(f)['/**']['ros__parameters']['z_inicial']
    modelo = vehiculo
    if LaunchConfiguration('controlador').perform(context) == 'ardupilot':
        modelo = f'{vehiculo}_ardupilot'
        if not os.path.isdir(os.path.join(share, 'models', modelo)):
            raise RuntimeError(f"El vehículo '{vehiculo}' no tiene variante para ArduPilot")
    # En la búsqueda el dron sale lejos del foco, en un claro al borde del bosque.
    x0, y0 = LaunchConfiguration('x0').perform(context), LaunchConfiguration('y0').perform(context)
    if LaunchConfiguration('mision').perform(context) == 'busqueda':
        x0, y0 = x0 or '-40', y0 or '-35'
    return [Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', LaunchConfiguration('mundo').perform(context),
                   '-file', os.path.join(share, 'models', modelo, 'model.sdf'),
                   '-name', 'cuadrirotor', '-x', x0 or '0', '-y', y0 or '0', '-z', str(z0)])]


def busqueda(context, share):
    """Detector YOLO26 y planificador de la búsqueda del incendio."""
    if LaunchConfiguration('mision').perform(context) != 'busqueda':
        return []
    if LaunchConfiguration('controlador').perform(context) == 'ardupilot':
        raise RuntimeError('La misión de búsqueda funciona con controlador:=pd o sta')
    vehiculo = LaunchConfiguration('vehiculo').perform(context)
    venv = os.path.expanduser(os.environ.get('YOLO_VENV', '~/.venvs/yolo'))
    python = os.path.join(venv, 'bin', 'python3')
    modelo = LaunchConfiguration('modelo_yolo').perform(context) or \
        os.path.join(share, 'yolo', 'incendio_yolo26n.pt')
    for ruta, que in ((python, 'entorno de Python de YOLO'), (modelo, 'modelo YOLO')):
        if not os.path.exists(ruta):
            raise RuntimeError(f'No se encontró el {que}: {ruta}')
    detector = ExecuteProcess(
        cmd=[python, '-m', 'cuadrirotor_sim.detector_yolo', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', f'modelo:={modelo}'],
        output='screen', name='detector_yolo')
    planificador = Node(
        package='cuadrirotor_sim', executable='busqueda_incendio', output='screen',
        parameters=[os.path.join(share, 'config', 'parametros.yaml'),
                    os.path.join(share, 'config', f'vehiculo_{vehiculo}.yaml'),
                    {'use_sim_time': True}])
    return [detector, planificador]


def ardupilot(context, share):
    """ArduCopter SITL (sim_vehicle.py con MAVProxy) y el nodo de misión."""
    if LaunchConfiguration('controlador').perform(context) != 'ardupilot':
        return []
    vehiculo = LaunchConfiguration('vehiculo').perform(context)
    ap = os.path.expanduser(os.environ.get('ARDUPILOT_DIR', '~/ardupilot'))
    venv = os.path.expanduser(os.environ.get('ARDUPILOT_VENV', '~/venv-ardupilot'))
    python = os.path.join(venv, 'bin', 'python3')
    for ruta, que in ((os.path.join(ap, 'build', 'sitl', 'bin', 'arducopter'), 'ArduCopter SITL compilado'),
                      (python, 'entorno de Python de ArduPilot')):
        if not os.path.exists(ruta):
            raise RuntimeError(f'No se encontró el {que}: {ruta}')
    trabajo = os.path.join(tempfile.gettempdir(), f'cuadrirotor_sim_{os.getuid()}', 'ardupilot')
    os.makedirs(trabajo, exist_ok=True)

    sitl = [python, os.path.join(ap, 'Tools', 'autotest', 'sim_vehicle.py'),
            '-v', 'ArduCopter', '-f', 'gazebo-iris', '--model', 'JSON', '--no-rebuild', '-w',
            '--add-param-file', os.path.join(share, 'config', 'ardupilot', f'{vehiculo}.parm'),
            '--out', 'udp:127.0.0.1:14551', '--mavproxy-args=--non-interactive']
    if LaunchConfiguration('gui').perform(context) == 'true':
        sitl += ['--console', '--map']
    entorno = {'PATH': os.path.join(venv, 'bin') + os.pathsep + os.environ.get('PATH', ''),
               'VIRTUAL_ENV': venv}

    mision = [python, '-m', 'cuadrirotor_sim.mision_ardupilot', '--ros-args',
              '--params-file', os.path.join(share, 'config', 'parametros.yaml'),
              '-p', 'use_sim_time:=true',
              '-p', f"mision:={LaunchConfiguration('mision').perform(context)}"]
    csv = LaunchConfiguration('csv').perform(context)
    if csv:
        mision += ['-p', f'archivo_csv:={csv}']
    return [ExecuteProcess(cmd=sitl, cwd=trabajo, additional_env=entorno, output='screen',
                           name='ardupilot_sitl'),
            ExecuteProcess(cmd=mision, additional_env=entorno, output='screen',
                           name='mision_ardupilot')]


def modelos_con_rutas_absolutas(share):
    """Copia de foco_incendio con rutas absolutas a sus texturas.

    Gazebo solo encuentra la imagen de rango de colores de las partículas
    (<color_range_image>) con una ruta absoluta; la ruta depende de dónde está
    instalado el paquete, así que se resuelve al lanzar.
    """
    origen = os.path.join(share, 'models', 'foco_incendio')
    destino = os.path.join(tempfile.gettempdir(), f'cuadrirotor_sim_{os.getuid()}', 'modelos')
    os.makedirs(os.path.join(destino, 'foco_incendio'), exist_ok=True)
    with open(os.path.join(origen, 'model.sdf')) as f:
        sdf = f.read().replace('model://foco_incendio/', origen + '/')
    with open(os.path.join(destino, 'foco_incendio', 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(origen, 'model.config')) as f, \
            open(os.path.join(destino, 'foco_incendio', 'model.config'), 'w') as g:
        g.write(f.read())
    return destino


def ruta_plugin_ardupilot():
    """Agrega el plugin de ardupilot_gazebo a Gazebo si está compilado en el workspace."""
    try:
        prefijo = get_package_prefix('ardupilot_gazebo')
    except PackageNotFoundError:
        return []
    return [AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                      os.path.join(prefijo, 'lib', 'ardupilot_gazebo'))]


def generate_launch_description():
    share = get_package_share_directory('cuadrirotor_sim')

    # Con GPU NVIDIA, Mesa intenta abrir el dispositivo por EGL y llena la
    # terminal de avisos 'libEGL warning'; se fuerza el EGL de NVIDIA.
    egl_nvidia = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    entorno_egl = ([SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', egl_nvidia)]
                   if os.path.exists(egl_nvidia) else [])
    mundo = [os.path.join(share, 'worlds', ''), LaunchConfiguration('mundo'), '.sdf']

    vehiculo_yaml = [os.path.join(share, 'config', 'vehiculo_'), LaunchConfiguration('vehiculo'), '.yaml']
    gui = LaunchConfiguration('gui')
    # Ventana: panel "Detección YOLO" en la búsqueda; "Cámara del dron" en las demás.
    ventana = PythonExpression(["'" + os.path.join(share, 'config', 'gazebo_gui') + "' + ('_yolo' if '",
                                LaunchConfiguration('mision'), "' == 'busqueda' else '') + '.config'"])
    gz_args = PythonExpression([
        "'-r ' + ('--gui-config ' + '", ventana, "' + ' ' if '", gui, "' == 'true' else '-s --headless-rendering ') + '",
        *mundo, "'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    puente = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(share, 'config', 'puente.yaml'),
                     'use_sim_time': True}])

    controlador = Node(
        package='cuadrirotor_sim', output='screen',
        executable=['controlador_', LaunchConfiguration('controlador')],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('controlador'), "' not in ('ninguno', 'ardupilot')"])),
        parameters=[os.path.join(share, 'config', 'parametros.yaml'), vehiculo_yaml, {
            'use_sim_time': True,
            'mision': PythonExpression(["'externa' if '", LaunchConfiguration('mision'),
                                        "' == 'busqueda' else '", LaunchConfiguration('mision'), "'"]),
            'Ts': LaunchConfiguration('Ts'),
            'archivo_csv': LaunchConfiguration('csv'),
            'esquema': LaunchConfiguration('esquema'),
        }])

    viento = Node(
        package='cuadrirotor_sim', executable='viento', output='screen',
        condition=IfCondition(LaunchConfiguration('viento')),
        parameters=[os.path.join(share, 'config', 'parametros.yaml'), {
            'use_sim_time': True,
            'sincronizar': PythonExpression(["'", LaunchConfiguration('controlador'), "' == 'ardupilot' or '",
                                             LaunchConfiguration('mision'), "' == 'busqueda'"]),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('mundo', default_value='incendio', description='incendio | bosque'),
        DeclareLaunchArgument('vehiculo', default_value='cuadrirotor', description='cuadrirotor | tarot650'),
        DeclareLaunchArgument('gui', default_value='true', description='Mostrar la interfaz de Gazebo'),
        DeclareLaunchArgument('controlador', default_value='pd', description='pd | sta | ardupilot | ninguno (para lanzarlo aparte con ros2 run)'),
        DeclareLaunchArgument('esquema', default_value='implicito',
                              description='STA: explicito | semi_implicito | implicito'),
        DeclareLaunchArgument('mision', default_value='hover',
                              description='hover | orbita | busqueda (YOLO26 encuentra el incendio)'),
        DeclareLaunchArgument('x0', default_value='', description='Posición inicial x [m]'),
        DeclareLaunchArgument('y0', default_value='', description='Posición inicial y [m]'),
        DeclareLaunchArgument('modelo_yolo', default_value='', description='Pesos de YOLO (.pt)'),
        DeclareLaunchArgument('Ts', default_value='0.005', description='Periodo de muestreo [s]'),
        DeclareLaunchArgument('viento', default_value='false', description='Aplicar perturbación de viento'),
        DeclareLaunchArgument('csv', default_value='', description='Ruta del CSV de registro'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', modelos_con_rutas_absolutas(share)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        *entorno_egl,
        *ruta_plugin_ardupilot(),
        RegisterEventHandler(OnShutdown(on_shutdown=terminar_restos)),
        gazebo, OpaqueFunction(function=aparecer_vehiculo, args=[share]), puente, controlador, viento,
        OpaqueFunction(function=ardupilot, args=[share]),
        OpaqueFunction(function=busqueda, args=[share]),
    ])

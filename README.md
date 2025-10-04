# MS2R1P - Robot Manipulador RRP

Paquete ROS 2 para control y simulación de un manipulador robótico serial con configuración RRP (Revolute-Revolute-Prismatic).

## Autor
**Herick Steven Duran Burgos**

## Descripción

Este proyecto implementa un sistema completo de control robótico que incluye:
- Parsing de trayectorias desde archivos DXF
- Planificación de trayectorias con interpolación cúbica/quínctica
- Cinemática directa e inversa
- Control de seguimiento de trayectoria
- Visualización en RViz2
## Video entrega:
https://youtu.be/NXWL8rJlf1o
## Requisitos

### Sistema
- Ubuntu 22.04
- ROS 2 Humble

### Dependencias de Python
```bash
pip3 install ezdxf numpy
```

### Dependencias de ROS 2
```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-xacro \
                 python3-numpy
```

## Estructura del Paquete
```
ms2r1p/
├── ms2r1p/
│   ├── direct_kinematics.py          # Nodo de cinemática directa
│   ├── inverse_kinematics.py         # Nodo de cinemática inversa
│   ├── dxf_parser_node.py            # Parser de archivos DXF
│   ├── trajectory_planner_node.py    # Planificador de trayectorias
│   ├── trajectory_follower.py        # Seguidor de trayectorias
│   ├── ms2r1p_state_publisher.py     # Publicador de estados de joints
│   └── dxfs/
│       ├── traj_shape_amg.dxf        # Archivo DXF de entrada
│       └── create_test_dxf.py        # Script para generar DXF de prueba
├── launch/
│   └── display_launch.py             # Launch file principal
├── model/
│   ├── ms2r1p_amg.urdf.xacro        # Descripción URDF del robot
│   └── rviz_config.rviz             # Configuración de RViz
├── package.xml
├── setup.py
└── README.md
```

## Instalación
```bash
# Crear workspace
mkdir -p ~/ros2_ws_2502/src
cd ~/ros2_ws_2502/src

# Clonar el paquete
git clone <repository-url> ms2r1p

# Compilar
cd ~/ros2_ws_2502
colcon build --packages-select ms2r1p

# Source del workspace
source install/setup.bash
```

## Uso
1. Generar DXF de prueba (opcional):
```bash
python3 ~/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/create_test_dxf.py
```

2. Lanzar el sistema completo:
```bash
ros2 launch ms2r1p display_launch.py
```

3. Ejecutar el planificador de trayectorias (en otra terminal):
```bash
source ~/ros2_ws_2502/install/setup.bash
ros2 run ms2r1p trajectory_planner_node
```

## Parámetros Configurables

### Trajectory Planner
- **interpolation_type**: Tipo de interpolación (`linear`, `cubic`, `quintic`)
- **T_segment**: Tiempo por segmento (segundos)
- **T_transition**: Tiempo de transición entre entidades (segundos)
- **T_prism_up/down**: Tiempo de movimiento prismático (segundos)

### Trajectory Follower
- **publish_period**: Periodo entre puntos (segundos, default: 0.1)
- **autoloop**: Reiniciar trayectoria al finalizar (bool, default: false)

### State Publisher
- **publish_rate**: Frecuencia de publicación (Hz, default: 10.0)
- **upper_limit_angular_1/2**: Límites superiores de joints angulares
- **lower_limit_angular_1/2**: Límites inferiores de joints angulares
- **upper_limit_linear**: Límite superior del joint prismático
- **lower_limit_linear**: Límite inferior del joint prismático

## Topics ROS 2

### Publicados
- `/dxf_path` (`nav_msgs/Path`): Path extraído del DXF
- `/dxf_pointcloud` (`sensor_msgs/PointCloud`): Nube de puntos del DXF
- `/planned_trajectory` (`nav_msgs/Path`): Trayectoria interpolada
- `/goal_pose_trajectory` (`geometry_msgs/PointStamped`): Poses objetivo
- `/scara_conf` (`geometry_msgs/Twist`): Configuración del robot (θ1, θ2, s4)
- `/joint_states` (`sensor_msgs/JointState`): Estados de las articulaciones
- `/end_effector_pose` (`geometry_msgs/PointStamped`): Pose del efector final

## Especificaciones del Robot

### Configuración
- Tipo: RRP (2 Revolute + 1 Prismatic)
- Base: Cilindro de 82 cm de altura

### Links
- Brazo 1: 1.6 m (azul)
- Brazo 2: 1.2 m (rojo)
- Prismático: 18 cm base + variable hasta 60.85 cm
- Efector Final: Cilindro verde

### Límites de Joints
- Joint 1 (`base_to_arm1_cx`): [-90°, +90°]
- Joint 2 (`arm_1_arm_2_joint_cx`): [-120°, +120°]
- Joint 3 (`arm_2_prismatic_joint_cx`): [0, -0.6085 m]

### Workspace
- Alcance máximo: 2.8 m (brazos extendidos)
- Alcance mínimo: 0.4 m (brazos plegados)

## Formato DXF
El sistema soporta las siguientes entidades DXF:
- LINE
- LWPOLYLINE
- POLYLINE
- ARC
- CIRCLE
- SPLINE

### Coordenadas
- Unidades: milímetros
- Escalado automático: División por 100 (mm → m)
- Origen: Centro de la base del robot

## Ejemplo de Workspace Válido
Para un cuadrado de 80x80 mm centrado en X=1.5 m:
- Rango X: 1.5 m a 2.3 m
- Rango Y: 0 m a 0.8 m

## Troubleshooting

**Error: "Goal position is out of reach"**  
La trayectoria contiene puntos fuera del workspace del robot. Ajusta el DXF añadiendo offset, por ejemplo:
```python
offset_x = 150  # 1.5 m después de escalar
```

**Robot no se mueve**  
Verifica que todos los nodos estén corriendo:
```bash
ros2 node list
```
Deberías ver:
```
/dxf_parser_node_cx
/trajectory_planner_cx
/trajectory_follower_cx
/inverse_kinematics_cx
/fk_solver_cx
/ms2r1p_state_publisher_cx
```

**RViz no muestra el robot**  
Verifica que el Fixed Frame sea `base_link_cx`:  
RViz → Global Options → Fixed Frame → `base_link_cx`

## Archivos CSV Generados
Durante la ejecución se generan:
- `dxf_waypoints_v5.csv`: Waypoints extraídos del DXF
- `planned_trajectory.csv`: Trayectoria interpolada completa

## Licencia
Apache-2.0

## Referencias
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ezdxf Documentation](https://ezdxf.readthedocs.io/)

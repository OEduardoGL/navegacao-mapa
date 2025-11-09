# Navegação por Mapa com TurtleBot3 (ROS 2 Humble + Gazebo Classic)

## Visão geral

### Conversão de mapa e mundo
1. Um mapa `map.pgm` criado no GIMP foi convertido para um mundo SDF compatível com o Gazebo Classic por meio do pacote `map2world`.
2. O arquivo final `custom_map.world` inclui o mapa e posiciona o TurtleBot3 e o marcador de objetivo nas coordenadas desejadas.

### Algoritmos
- **D\* Lite** (`tb3_map_dstar/dstar_planner_node.py`): lê `custom_map.yaml`, infla obstáculos com o `safety_radius` e mantém as listas `g/rhs` para garantir consistência entre objetivo e origem antes de extrair o trajeto final.
- **RRT** (`tb3_map_rrt/rrt_planner_node.py`): sampleia o espaço livre do mapa (após inflar o grid conforme `safety_radius`/`occupied_threshold`), constrói uma árvore Randômica Rápida de Busca e aplica atalhos + suavização (`simplify_distance`, `smoothing_iterations`, `goal_bias`, etc.) até conectar origem e objetivo.

### Seguimento de caminho
- Os seguidores `tb3_map_dstar/path_follower_node.py` e `tb3_map_rrt/path_follower_node.py` assinam `/planned_path`, calculam o deslocamento entre frames `map` e `odom` e aplicam um controle proporcional sobre `cmd_vel` para seguir os waypoints com lookahead configurável por launch.
- O mesmo nó publica:
  - `robot_pose` (`geometry_msgs/PoseStamped`) – pose estimada no frame `map`.
  - `navigation_status` (`std_msgs/String`) – estados (`waiting_for_path`, `navigating`, `goal_reached`, etc.).

## Como executar

### Pré-requisitos
- Linux com Docker e Docker Compose instalados.
- Ambiente gráfico X11 (no host, execute `xhost +local:` antes de subir os containers).

### Subindo o container
```bash
docker-compose up -d --build
```

### Acessando o container
```bash
docker exec -it tb3_nav bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tb3_map_dstar tb3_map_rrt tb3_bug_nav_classic --symlink-install
source /root/ws/install/setup.bash
```

### Executando D\* Lite
```bash
ros2 launch tb3_bug_nav_classic tb3_dstar_nav.launch.py \
  gui:=true \
  start_x:=-2.76177 start_y:=-6.63469 \
  goal_x:=-2.37407 goal_y:=13.9167 \
  safety_radius:=0.45 \
  simplify_distance:=0.25 \
  lookahead:=2
```

### Executando RRT
```bash
ros2 launch tb3_bug_nav_classic tb3_rrt_nav.launch.py \
  gui:=true \
  start_x:=-2.76177 start_y:=-6.63469 \
  goal_x:=-2.37407 goal_y:=13.9167 \
  occupied_threshold:=200 \
  safety_radius:=0.4 \
  max_iterations:=9000 \
  step_size:=0.8 \
  goal_radius:=0.7 \
  goal_bias:=0.12 \
  smoothing_iterations:=120 \
  lookahead:=0.5
```
---

## Arquitetura de pastas

```
├── src/
│   ├── tb3_bug_nav_classic/
│   │   ├── launch/tb3_{dstar,rrt}_nav.launch.py
│   │   ├── maps/custom_map.{yaml,pgm}
│   │   └── worlds/custom_map.world
│   ├── tb3_map_dstar/
│   │   ├── dstar_planner_node.py
│   │   └── path_follower_node.py
│   ├── tb3_map_rrt/
│   │   ├── rrt_planner_node.py
│   │   └── path_follower_node.py
│   └── log/
│       ├── dstar_run/ # rosbag gravado
│       └── rrt_run/   # rosbag gravado
├──Dockerfile
├──docker-compose.yml
└──entrypoint.sh
```

---

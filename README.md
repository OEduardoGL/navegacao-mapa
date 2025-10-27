# Navegação por Mapa com TurtleBot3 (ROS 2 Humble + Gazebo Classic)

## Visão geral

### Conversão de mapa e mundo
1. Um mapa `map.pgm` criado no GIMP foi convertido para um mundo SDF compatível com o Gazebo Classic por meio do pacote `map2world`.
2. O arquivo final `custom_map.world` inclui o mapa e posiciona o TurtleBot3 e o marcador de objetivo nas coordenadas desejadas.

### D\* Lite
- O nó `tb3_map_dstar/dstar_planner_node.py` carrega o mapa ocupacional (`custom_map.yaml`), infla os obstáculos (`safety_radius`) e procura um caminho livre entre start e goal.
- O algoritmo **D\* Lite** mantém custos `g` e `rhs` por célula e expande a árvore a partir do objetivo até garantir consistência no estado inicial. Depois extrai o percurso completo.

### Seguimento de caminho
- O nó `tb3_map_dstar/path_follower_node.py` assina `/planned_path`, calcula um deslocamento entre frames `map` e `odom` e aplica um controle proporcional sobre `cmd_vel` para seguir os waypoints com lookahead configurável.
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
colcon build --packages-select tb3_map_dstar tb3_bug_nav_classic --symlink-install
source /root/ws/install/setup.bash
```

### Executando a navegação D\* Lite
```bash
ros2 launch tb3_bug_nav_classic tb3_dstar_nav.launch.py \
  gui:=true \
  start_x:=-2.76177 start_y:=-6.63469 \
  goal_x:=-2.37407 goal_y:=13.9167 \
  safety_radius:=0.45 \
  simplify_distance:=0.25 \
  lookahead:=2
```

---

## Arquitetura de pastas

```
├── src/
│   ├── tb3_bug_nav_classic/
│   │   ├── launch/tb3_dstar_nav.launch.py
│   │   ├── maps/custom_map.{yaml,pgm}
│   │   └── worlds/custom_map.world
│   └── tb3_map_dstar/
│       ├── dstar_planner_node.py
│       └── path_follower_node.py
├──log/dstar_run/ # rosbag gravado
├──Dockerfile
├──docker-compose.yml
└──entrypoint.sh
```

---

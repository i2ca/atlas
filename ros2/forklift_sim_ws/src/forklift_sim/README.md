# forklift_sim

Hyster no Gazebo (gz-sim do ROS 2: Jazzy, Kilted ou Lyrical), comandada pelo mesmo CAN J1939 da empilhadeira real.

## Outra maquina / outro ROS 2

- Nada aponta para o PC de quem gerou: os mundos usam caminhos RELATIVOS a `worlds/` (instalada pelo `setup.py`)
  e URLs do Gazebo Fuel para as pecas do Depot (o Gazebo baixa para `~/.gz/fuel` na primeira vez; precisa de
  internet so' nessa vez). Os geradores (`tools/make_*_world.py`, `make_big_depot.py`) gravam assim
  (`tools/world_uris.py`) e avisam se sobrar caminho absoluto.
- O Gazebo trata arquivo faltando como erro FATAL (o mundo nao abre). O `sim.launch.py` confere: se faltar um
  asset opcional (o navio `worlds/models/ship_aaa`, que nao vai no repositorio -- ver `worlds/models/LEIA-ME.txt`),
  tira so' aquele visual e avisa.
- Python: `forklift_sim/gz_compat.py` acha o par `gz.transport`/`gz.msgs` de cada versao (Jazzy: 13/10, Kilted:
  14/11, Lyrical: sem numero). Os plugins (`gz-sim-*-system`, `gz_ros2_control-system`) tem o mesmo nome nessas
  distros. O Humble (Gazebo Fortress, `ignition-*`) nao e' suportado.
- Build: `colcon build` com o `setup.bash` da distro; no astro, os process escolhem a distro em `SET ROS2_DISTRO`.

```
forklift_clamp_teleop / astro ─IPC─> forklift_driver ─vcan0 (0x0CFFC0EC)─┬─> forklift_can_simulator (VCU, motor, transmissao)
                                                                         └─> can_bridge ─> Gazebo (mastro e garra)
/cmd_vel ──────────────────────────────────────────────────────────────────> can_bridge ─> Gazebo (base)
```

- `can_bridge` le o joystick da garra do vcan0 e move as juntas como hidraulico:
  a alavanca define a **velocidade** da junta, sem realimentacao de posicao.
  Para se nao chegar frame por 0,3 s, ou se a VCU simulada marcar "garra
  desconectada" (0x18FF1027).
- O mapeamento eixo -> junta esta em `config/can_bridge.yaml`
  (`axisN_function: lift|sideshift|clamp|none`, `axisN_sign`). **O padrao (1 = elevar,
  2 = deslocamento lateral, 3 = garra) e um palpite**: conferir na empilhadeira.
- `lift` sobe primeiro o carro (`carriage_joint`) e depois o mastro (`mast_lift_joint`);
  `sideshift` move a base da garra (`clamp_base_joint`, +-0,3 m);
  `clamp` fecha os dois bracos juntos (`clamp_1/2_joint`, 0 = aberta, 0,45 m = fechada).
- A base anda de forma cinematica (`VelocityControl` do Gazebo) pela velocidade e
  pela marcha que a empilhadeira publica no CAN (CCVS `18FEF127`, ETC2 `18F00527`, os
  mesmos que o `vehicle_driver` le'). A direcao nao esta' no CAN: a guinada vem do
  `/cmd_vel` (angular.z), e o `/cmd_vel` inteiro so' vale sem velocidade no CAN. As rodas
  dianteiras e a direcao traseira giram junto, so para a visualizacao.

## Modelo

`../Hyster_description` e uma copia do `~/atlas/ros2/gazebo_legacy_model` (cores,
cilindros de inclinacao fixos, rodas cilindricas, deslocamento lateral, `base_footprint`).
Na copia so foi tirado do `Hyster.gazebo` o bloco de plugins do Gazebo Classic
(ackermann, `libhyster_joint_position.so`, joint_states), que nao carregam no gz sim.
O mastro nao inclina neste modelo (`mast_tilt_joint` e fixo).

## Mundos

- `world:=empty` (padrao): chao vazio.
- `world:=depot`: galpao de 30 x 15 m com prateleiras, paletes e pilares (`worlds/depot.sdf`).
  Baseado no "tugbot_depot" de **MovAi** e no modelo "Depot" de **OpenRobotics**, do Gazebo
  Fuel, licenca CC-BY 4.0 (atribuicao no cabecalho do .sdf). Sem o robo Tugbot. Os modelos
  vem por URL do Fuel e ficam no cache `~/.gz/fuel` (o Depot tem ~100 MB). Foi feito para robos
  pequenos: os corredores entre as prateleiras (~1 m) nao passam a Hyster; ela nasce na area
  livre da ponta oeste (-3.8, 1.1).
- `world:=big_depot`: o mesmo galpao, bem maior: **113 x 79 m**, teto a 12 m, 4 x 4 blocos de
  prateleiras e caixas com corredores de 6 m (`worlds/big_depot.sdf`). E' GERADO por
  `tools/make_big_depot.py` a partir das pecas do Depot no cache (`--tiles-x`, `--tiles-y`,
  `--aisle`, `--height` mudam o tamanho); sem pilares, vigas e ventiladores, para sobrar espaco.
  O BOXSET dos blocos usa `worlds/meshes/depot_boxset.dae`, uma copia gerada sem as pecas que no
  original ficavam presas nas paredes (ventiladores de parede, que ficariam soltas no ar).
  A empilhadeira nasce no corredor transversal (0.2, 8.8). Dentro de cada bloco os corredores
  continuam com ~1 m (como no original): a Hyster anda pelos corredores largos.
- `world:=<caminho.sdf>`: qualquer outro mundo. A posicao inicial muda com `x:=` `y:=` `yaw:=`.

## Rodar

```
cd ~/atlas/ros2/forklift_sim_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_sim sim.launch.py world:=big_depot   # gui:=false para headless
```

Argumentos: `world` (empty), `gui` (true), `camera` (carry, o padrao: a camera da janela nasce
8 m atras e 3,8 m acima e vai junto com a empilhadeira so' por translacao -- nao gira nas curvas,
e zoom/rotacao do mouse ficam livres; `camera_follow`. free_look/follow: os modos do Gazebo, presos
ao veiculo; free: solta), `can` (true, sobe o `can_bridge`),
`can_interface` (vcan0; `none` = so `/cmd_vel`, sem CAN), `bridge_config` (yaml do `can_bridge`).

Cadeia completa: criar o vcan0 e subir o lado astro como em
`~/astro/src/forklift_driver/README.md`, depois este launch.

Andar: `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"`

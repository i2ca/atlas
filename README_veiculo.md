# Empilhadeira (Hyster) em simulação

Tudo para testar o astro na empilhadeira **sem o veículo**, sem alterar nenhum código
original do astro.

```
forklift_gui ─IPC motion_command + Go/Stop──> vehicle_driver (ford_escape_hybrid_dbc)
     │                                              │ vcan0: 18FF50F0 acelerador/freio/marcha/direcao
     │                                              v
     │                                   forklift_can_simulator  (34 IDs da empilhadeira)
     │                                              │ vcan0: CCVS, ETC2, EEC1, 18FF51F0 chave/angulo
     │                                              v
     │                               vehicle_driver ─IPC odometria/status─> forklift_gui
     │
     └─IPC astro_forklift_clamp_command──> forklift_driver ─vcan0 0CFFC0EC─> simulador (VCU)
                                                                          └─> Gazebo (garra)
Gazebo (~/atlas/ros2/forklift_sim_ws): base pela velocidade/marcha do CAN, garra pelo joystick
```

| Peça | Onde | O que é |
|---|---|---|
| `vehicle_driver` | `bin/ford_escape_hybrid_dbc` | o **original**, compilado sem `-DUSE_JAUS_PROTOCOL` numa cópia fora da árvore (`tools/build_vehicle_driver_dbc.sh`). Fala direto no CAN pelo DBC. |
| `forklift_driver` | `src/forklift_driver` | garra (o `vehicle_driver` não tem sinal para os 4 eixos) + status do CAN. |
| `forklift_can_simulator` | `src/forklift_can_simulator` | o lado da empilhadeira no `vcan0`. |
| `forklift_gui` | `src/forklift_gui` | painel: garra, movimento, autônomo, status, joystick. |
| `forklift_lidar_sim` | `src/forklift_lidar_sim` | LiDAR do Gazebo → `lidar0` do astro, no lugar do `hesai_driver`. |
| `forklift_gz_pose` | `src/forklift_gz_pose` | Gazebo → GPS (`GGA` nr 1 + `HDT`, 20 Hz) e xsens (`XSENS_QUAT`, 125 Hz), como os drivers reais. |
| Gazebo | `~/atlas/ros2/forklift_sim_ws` | Hyster 3D lendo o mesmo `vcan0`. |

## Arquivos

- `parameters_ini_new/astro.ini` — cópia do da IARA com:
  `vehicle_driver_can vcan0`, `vehicle_driver_dbc_path`,
  `vehicle_driver_g_param_driver_velocity_dbc off` (com `on` o modo DBC não atualiza a marcha
  lida e o `vehicle_driver` acha que está em neutro), `vehicle_driver_g_param_driver_gear_control_by_rpm off`,
  `robot_PID_velocity_throttle_gap 10` (zona morta do acelerador), `robot_max_velocity 2.5`,
  `robot_PID_velocity_backward_*` iguais aos de frente (os da IARA deixavam a ré fraca),
  `robot_PID_steering_kp 1500` / `ki 800` (com os da IARA a roda passava ~6° do centro).
- `parameters_ini_new/empilhadeira.dbc` — o DBC único (cópia de `src/forklift_driver/dbc`):
  leitura do CAN real + escrita do `vehicle_driver` (placa de interceptação **[SIM]**) + garra.
- `parameters_ini_new/can_bridge.yaml` — o que cada eixo do joystick da garra move no Gazebo.
- `process_new/process-empilhadeira-sim-garra.ini` — sobe tudo.
- `tools/build_vehicle_driver_dbc.sh` — gera o `bin/ford_escape_hybrid_dbc`.

## Subir

```
cd $ASTRO_HOME/bin
../data/empilhadeira_sim/tools/build_vehicle_driver_dbc.sh      # uma vez
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./central                                                        # outro terminal
./proccontrol ../data/empilhadeira_sim/process_new/process-empilhadeira-sim-garra.ini
```

O roteiro do teste está no cabeçalho do process.

## Como a empilhadeira se comporta (simulador)

- Marchas **1, 0, -1**. O neutro (0) só existe com o **freio de mão puxado**; solto, fica em 1 ou -1.
- Só anda com o acelerador acima de **~10 %**.
- Inverter o sentido andando: a transmissão freia e só inverte quase parada (< 0,3 m/s).
- Na real, acelerador e marcha não passam pelo CAN (são por interceptação elétrica). Os frames
  da placa de interceptação (`18FF50F0`, `18FF51F0`) são uma **proposta**; trocar os IDs quando
  a placa existir.

## LiDAR simulado (OT-128 da IARA)

O Gazebo tem um `gpu_lidar` na Hyster, montado e configurado pelo **mesmo `astro.ini`** do
`param_daemon` (o launch lê o .ini): pose = `sensor_board_1_*` composto com `lidar0_*`, grade vertical
cobrindo os `lidar0_vertical_angles`, `1 / (20 Hz * lidar0_time_between_shots)` = 1800 tiros por
volta (20 Hz, como no log da IARA), alcance `lidar0_min_sensing / range_division_factor` a `lidar0_max_range`.

O `forklift_lidar_sim` (`src/forklift_lidar_sim`) faz o papel do `hesai_driver`: para cada tiro e canal
pega o raio no ângulo vertical `lidar0_vertical_angles[j]` e no horizontal `tiro + lidar0_horizontal_angles[j]`
e publica `astro_velodyne_variable_scan_message0`, com as mesmas unidades (m × `range_division_factor`),
mais a IMU do LiDAR (`imu_lidar`, `lidar0_imu_frequency`) e o texto de log com a informação do sensor,
como o `hesai_driver`.

`lidar0_*` é a cópia exata do da IARA. A `sensor_board_1` está **em cima da cabine** (teto de proteção:
x −1,0, z 2,46): o LiDAR fica a 2,81 m do chão. Para mudar, edite `sensor_board_1_*` / `lidar0_*` e
reinicie o `gazebo` e o `lidar0` (os dois leem o .ini ao subir). O mastro, na frente, tampa parte da vista.

Testado em 20/10: 20 Hz, 1800 × 128, IMU do LiDAR a ~100 Hz (acc z 9,80), chão em z ≈ −0,02 m no
referencial do carro, prateleiras do corredor a ~3 m dos lados.

## Sensores e localização (como no veículo real)

Os simuladores publicam **só o que o veículo publica** (as mesmas mensagens dos logs da IARA); a
`globalpos` sai da cadeia original do astro:

    forklift_gz_pose  --GGA nr 1 + HDT (20 Hz)-->  gps_xyz  --+
                      --XSENS_QUAT (125 Hz)----------------------+--> fused_odometry --> localize_ackerman -lidar 0
    forklift_lidar_sim --variable scan 0 (20 Hz) + imu_lidar----------------------------->      |  (globalpos)
                                                                     mapper (modo normal) <---------+

- GPS: antena = carro × `sensor_board_1` × `gps_nmea_1_*`; lat/lon pela origem −20,2813 / −40,3056
  (UTM 7756851,0 N / 363662,6 E, zona 24). HDT na convenção do driver (o `gps_xyz` devolve o yaw do carro).
- xsens: `xsens_*` = embaixo do LiDAR (na `sensor_board_1`, z 0,05); quaternion da orientação do
  Gazebo e giro do sensor `imu` do Gazebo; `xsens_magnetic_declination 0`. A **aceleração** sai da
  pose verdadeira (2ª derivada do ponto do xsens + gravidade, no referencial do sensor): a do sensor do
  Gazebo vem suja pela física da base (andando, z médio 13,9 m/s²), e o SC-LIO-SAM integrava isso em
  z de ±2 m e pitch de ±3°. `-acc_from_pose off` volta à do Gazebo. Logs gravados antes de 20/10
  16:49 (ex.: `teste_1`) têm a aceleração ruim.
- `localize_ackerman -lidar 0` (sem `-mapping_mode`, inicia pelo GPS) e `mapper` em **modo normal**
  (`-map_path` só, como na navegação da IARA): só o mapa local online, **não grava mapa 2D**
  (`mapper_update_and_merge_with_mapper_saved_maps off`). O localize espera o `mss` antes de subir;
  o `mss` aponta para `map_level0/`, que só tem o `info.txt`.

- O `localize_ackerman` **sem** `-mapping_mode` precisa da `astro_base_ackerman_odometry`
  (módulo `base_ackerman`, a partir do v/phi do `vehicle_driver`): sem ela descarta todo scan e a
  `globalpos` nunca sai. Por isso o `base_ackerman` está no process (no `-mapping_mode on` não fazia falta).
- `viewer_3D` sobe com `env __GL_SYNC_TO_VBLANK=0` (driver NVIDIA): com vsync e a janela coberta ele cai
  para ~1 quadro/s dentro do laço do IPC e os ícones FORD ESCAPE/GPS/IMU/ODOM ficam vermelhos. A
  correção no código (`desliga_vsync`) só existe no branch `ype_homologacao_v_1_1_3d_localization_ubuntu26`.

O `viewer_3D` só desenha o LiDAR depois de receber GPS e posiciona tudo pela `globalpos`.

## Modelo 3D no viewer_3D

`models/hyster.obj` (+ `.mtl`) gerado do mesmo modelo do Gazebo por
`~/atlas/ros2/forklift_sim_ws/src/forklift_sim/tools/make_viewer3d_model.py --ini <astro.ini>`, que
também escreve os `carmodel_*` (arquivo, tamanho e posição) no .ini. Simplificado para ~19 mil
triângulos (o `viewer_3D` desenha o modelo a cada quadro).

## Frente e ré

O `vehicle_driver` **não** usa o sinal do `v` para escolher a marcha: ele segue o sentido do
caminho do `behavior_selector` (`path_goals_and_annotations`, última pose com v < 0 = ré),
estável por 3 s, e ainda espera 3 s depois de trocar a marcha. Sem o planner, a `forklift_gui`
anuncia esse caminho com o sentido do seu comando. Inverter o sentido leva uns 6 s parada.

A velocidade do CCVS não tem sinal, e o modo DBC não aplica o sentido da marcha nela (a
odometria sairia positiva na ré). Por isso o `Velocity` que o `vehicle_driver` lê vem com sinal
da placa **[SIM]** (`18FF51F0`), e a do CCVS se chama `CCVSSpeed` (é a que o `forklift_driver`
mostra).

## O que o vehicle_driver exige para andar

O mesmo que no carro: **Go** (`navigator_ackerman_go`, que também solta o freio de mão), o
`behavior_selector` fora de `Stopped`, a placa de interceptação com o computador no comando
(`ManualOverride` 0 — o simulador já sobe assim; tecla `m` = o motorista assume) e 3 s depois
de a marcha ficar pronta. Sem o planner no ar, a `forklift_gui` anuncia o `Free_Running`
(caixa "Anunciar behavior_selector", já marcada).

**Manual** = você pela `forklift_gui`/joystick. **Autônomo** = o astro com mapa e rota (a fazer);
usa os mesmos caminhos (`motion_command`, Go/Stop, `clamp_command`).

Testado em 19/10 num central isolado: comando de 1,0 m/s -> acelerador 31-35 % -> 0,92 m/s em
8 s; v = 0 -> freio 34 % -> 75 % (segurando parado).

**Direção:** na real ela não está no CAN (também vai por interceptação). Na simulação a placa
**[SIM]** recebe o `SteeringEffort` do PID de direção do `vehicle_driver` (`18FF50F0`) e devolve o
`SteeringAngle` (`18FF51F0`, 50 Hz); o simulador tem um atuador de 5 rad/s (o PID usa 20-30 %) até ±0,6 rad e o
Gazebo vira as rodas por esse ângulo. Testado: φ pedido 0,5 rad -> assenta em ~1 s e volta ao centro em ~1 s.
O máximo da `forklift_gui` é 0,5 rad, abaixo do batente (0,6): o `vehicle_driver` amplia o φ com a
velocidade, e pedindo além do batente o integral do PID acumula e a roda ficava ~5 s presa antes de voltar.

## Mapas 3D e 2D (SC-LIO-SAM), como a VDU da IARA

1. **Gravar o log:** no `process-empilhadeira-sim-garra.ini`, ligue o módulo `logger` pelo
   `proccontrol_gui`, ande pelo galpão (voltar ao ponto de partida ajuda o loop closure) e desligue.
   Ele grava em `/dados/logs_emp/log_emp-<data>.txt` (+ `<log>_lidar0/`), com os `logger_*` do
   astro.ini (iguais aos da IARA): LiDAR, IMU do LiDAR, GGA/HDT, XSENS_QUAT, ROBOTVELOCITY_ACK,
   FORD_ESCAPE_STATUS.
2. Ponha o nome do log em `SET LOG_NAME=` (sem `.txt`) nos três process abaixo. Tudo cai em
   `geodata/<LOG_NAME>/`.
3. `process-vdu-01-sclio-mapa3d.ini` — mapa 3D (`geodata/<LOG_NAME>/sc_lio_sam/`). Dê Play; no fim do
   playback, **desligue só o SC-LIO-SAM** pelo `proccontrol_gui` (é quando ele grava).
4. `process-vdu-02-sclio-poses.ini` — `localizationMode`: grava `poses_opt.dat` em
   `/dados/sc_lio_sam_output/empilhadeira/`; copie para `geodata/<LOG_NAME>/sc_lio_sam/`.
5. `process-vdu-03-map2d.ini` — `graphslam_publish` (poses) → `mapper -mapping_mode on` →
   `geodata/<LOG_NAME>/map_files/`; no fim, `complete_map`.

`parameters_ini_new/param.yaml` é a cópia do da IARA; só mudou o que é do veículo (linhas
`EMPILHADEIRA:`): `wheelbase` 2,235, `imuGravity` 9,80 (Gazebo), extrínseca xsens→LiDAR
(`extrinsicTrans [0, 0, -0.30]`, `extrinsicRPY [0, 0, π]` — LiDAR com `lidar0_yaw` π, xsens alinhado
com o carro, 0,30 m abaixo), `robot_collision_height` 4,0 (mastro) e os caminhos de saída.

O `pointcloud_node` tira a própria empilhadeira da nuvem pelo `robot_collision_file`, que agora é
`parameters_ini_new/hyster_col.txt` (5 círculos de 1,05 m cobrindo x −2,79…2,21 m; era o do Ford
Escape). O `lidarExcludeBox*` do yaml não é usado pelo código.

## Câmeras (3) com o `camera_drivers` original

| câmera | onde (carro) | mensagem | aparece em |
|---|---|---|---|
| 1 frente | borda dianteira do teto, perto do LiDAR (x −0,55, z 2,55), 14° para baixo | `camera1` | `viewer_3D`, painel esquerdo |
| 2 ré | em cima do contrapeso (x −2,70, z 1,45), olhando para trás, 17° para baixo | `camera2` | `viewer_3D`, painel direito |
| 3 pinça | presa na base da garra (`Base_das_garras_1`): sobe, desce e desliza junto | `camera3` | `camera_viewer intelbras3 3` |

    Gazebo (cameras:=true) --gz.msgs.Image--> camera_stream (a "câmera IP": ffmpeg/libx264, MPEG-TS)
        --udp://127.0.0.1:560N--> ./camera_drivers intelbrasN N --> cameraN

- Tudo nos blocos `intelbras1/2/3_*` do astro.ini (formato do `intelbras1` da IARA): pose em
  relação à `sensor_board_1`, 640×480, `fx` (campo horizontal de 80°), `sim_rate` 15 Hz e a porta no
  `ip_adress`. O launch do Gazebo lê o mesmo .ini. A 3 tem `intelbras3_link Base_das_garras_1`. O nome tem que ser `intelbras<N>`: o `camera_viewer` só corrige a lente (e não cai) com nomes `intelbras`/`hikvision`, e o `viewer_3D` busca `intelbras<id>_*`. O `intelbras1` da IARA ficou comentado (`#IARA`).
- `camera_viewer`: além do nome, ele chamava `setMouseCallback` antes de a janela existir, e o OpenCV
  4.10 desta máquina (interface Qt) aborta com "NULL window handler" no 1º quadro. Corrigido com um
  `namedWindow` antes (`src/camera_drivers/camera_viewer_draw_functions.cpp`, autorizado em 20/10).
- UDP: o `camera_drivers` e o Gazebo podem subir em qualquer ordem e reiniciar sozinhos.
- `viewer_3D_camera_left_id 1`, `right_id 2`, sem MOT e sem troca automática na ré (a 2 já fica fixa).
- `logger_camera on`: o log passa a gravar as 3 câmeras (bem maior).
- Testado em 20/10 (isolado): as 3 mensagens a 15 Hz, 640×480, imagens conferidas.

## Localização pelo SC-LIO-SAM (`process-empilhadeira-sim-sclio.ini`)

Igual ao `process-empilhadeira-sim-garra.ini`, trocando a localização pelo caminho da etapa 4 da IARA
(`process-iara-vdu-sc-globalpos.ini`):

    SC-LIO-SAM (localizationMode, mapa 3D de geodata/<GEO_NAME>/sc_lio_sam) --/odometry/imu-->
      fused_odometry_from_slam_node (UTM pelo map_world_origin.txt) --astro_fused_odometry-->
      localize_ackerman -mapping_mode on --globalpos--> mapper, viewer_3D...

- `SET GEO_NAME=teste_2`: o log cujos mapas 3D (vdu-01) e 2D (vdu-03) são usados.
- Sem o `fused_odometry` do astro (seriam dois publicadores da mesma mensagem).
- `use_sim_time:=false` (o simulador publica com o relógio do astro) e `gps_nrs:="[1]"`.
- Pose inicial `START_*` = 0: o referencial do mapa 3D é o do LiDAR onde o log começou. Vale se a
  empilhadeira nascer no mesmo lugar do log (big_depot sem x/y/yaw); senão, "2D Pose Estimate" no RViz.
- O SC-LIO-SAM precisa do `/ackermann/odom_raw` (`odometrySource: ackermann`), que vem do
  `vehicle_driver` → `base_ackerman`. Sem o `vehicle_driver` no ar, não sai `/odometry/imu` nem globalpos.
- Em localizationMode ele regrava `/dados/sc_lio_sam_output/empilhadeira/poses_opt.dat` (posesEnable);
  a cópia em `geodata/<GEO_NAME>/sc_lio_sam/` não é tocada.
- Testado em 22/10 (isolado, parado): globalpos a 20 Hz a ~8 cm da pose verdadeira do Gazebo.

## Gazebo abaixo do tempo real (odometria "andando demais")

O `forklift_can_simulator` e o astro trabalham no relógio da máquina; o Gazebo, no relógio da simulação.
Com a janela do Gazebo aberta ele roda abaixo do tempo real (medido em 22/10: fator médio 0,8, com
travadas até 0,02; o servidor usa ~85% de um núcleo). Antes, a velocidade do CAN ia direto para a base:
a empilhadeira andava ~1,5x menos no Gazebo do que a odometria dizia (36,5 m contra 24,6 m reais) e a
`globalpos` escapava (12 m em 2 min).

Correção na `can_bridge`: lê o tempo da simulação do `/world/<mundo>/stats` (gz.transport; o `/clock` do
ROS não é publicado) e manda a base e as rodas com `v / fator_de_tempo_real` (janela de 1 s, piso 0,2).
Testado forçando o Gazebo a 0,5: 1,0 m/s no CAN por 10 s reais = 10,02 m no Gazebo (antes 5,02 m).
`compensate_rtf: false` no `can_bridge.yaml` desliga. O log da ponte mostra `tempo real X (base xY)`.

Também ajuda: memória (a máquina estava com 6 GB em swap) e fechar o que não estiver usando.

## RDDF, grafo e navegação autônoma (como o ARGOS no carmen)

1. **Gravar o RDDF**: no `process-empilhadeira-sim-garra.ini` (ou no `-sclio`), com a `globalpos` boa,
   ligue o `rddf_build` pelo `proccontrol_gui`, dirija pelo caminho (no sentido em que ela vai andar
   sozinha) e desligue. Ele grava a `globalpos` a cada 0,5 m (`rddf_min_distance_between_waypoints`) em
   `rddf/rddf_emp_<data>.txt`.
2. **Gerar o grafo** (de dentro de `bin/`):

       ../data/empilhadeira_sim/tools/make_graph.sh ../data/empilhadeira_sim/rddf/rddf_emp_<data>.txt rddf_emp

   `road_network_generator` → `graphs/rddf_emp.gr`, com o mapa 2D do `teste_2` (3º argumento troca).
3. **Rodar autônomo**: `process-empilhadeira-sim-navigate.ini` (`SET GRAPH=` aponta para o grafo).
   Mesmos módulos do `process-argos-navigate.ini` (`obstacle_distance_mapper`, `route_planner`,
   `offroad_planner`, `behavior_selector`, `mpp`, `obstacle_avoider`, `navigator_gui2`). Os planners
   sobem desligados: ligue na ordem com a pose boa, escolha o destino no `navigator_gui2` e solte o
   freio de mão. A `forklift_gui` sobe com `-publish_motion off -announce_behavior off` (senão
   brigaria com o `obstacle_avoider` e o `behavior_selector`); continua servindo para a garra e o freio.
- `rddf/annotations.txt` começa vazio (formato: `NOME tipo código theta x y z`).
- Testado em 22/10 (isolado): RDDF de 44 pontos (das poses do SC-LIO-SAM) → grafo de 44 nós → o
  `route_planner` carrega (precisa do `mss` no ar). A navegação autônoma ainda não foi testada.
- `rddf_build_from_file` lê 7 colunas (`x y theta` + 4 timestamps), não o que a mensagem de uso diz.

## LiDARs 1 a 4 (provisórios, para posicionar)

O Gazebo sobe com `extra_lidars:="1 2 3 4"`: cada um é um `gpu_lidar` (+ IMU em `/lidarN/imu`) montado e
configurado pelo seu `lidarN_*` do astro.ini, como o `lidar0`, a 10 Hz e grade vertical de 0,5°
(`extra_lidar_rate`, `extra_lidar_vstep_deg`). Um `forklift_lidar_sim -lidar_id N -rate 10` por LiDAR
publica `astro_velodyne_variable_scan_messageN`.

- Modelos e ângulos ainda são os herdados da IARA (OS1-16 nos 1-3, RS16 no 4) e as poses são
  provisórias, nos cantos da empilhadeira (relativas à `sensor_board_1`).
- **Posicionar**: o `viewer_3D` assina a pose dos `lidarN_x/y/z/roll/pitch/yaw` (atualiza na hora
  quando o parâmetro muda no `param_daemon`). Ajuste até a nuvem ficar onde o sensor vai, passe os
  valores e os modelos, e o Gazebo é refeito com eles (o sensor do Gazebo só muda ao reiniciar o `gazebo`).
- Testado em 22/10 (isolado): lidar0 20 Hz + lidar1-4 10 Hz, sem perda; fator de tempo real 1,00 sem janela.

## Editor 3D de LiDARs (`tools/lidar_pose_editor.py`)

    $ASTRO_HOME/data/empilhadeira_sim/tools/lidar_pose_editor.py

A Hyster (o mesmo `models/hyster.obj` do `viewer_3D`) com todos os `lidarN` do astro.ini, câmeras,
xsens e GPS, para posicionar cada LiDAR vendo onde fica:

- **Mouse**: esquerdo gira, direito/meio arrasta, roda zoom; **Shift+clique na empilhadeira cola o
  LiDAR selecionado ali** (+ altura do suporte, nivelado); duplo clique seleciona.
- **Teclado** (vista em foco): W/S x, A/D y, R/F z, Q/E yaw, T/G roll, Y/H pitch no passo escolhido;
  1-5 vistas; Tab próximo LiDAR; Ctrl+Z desfaz; Ctrl+S salva.
- Pose no **carro** ou na **sensor_board_1**; Copiar/Colar; Espelhar esq↔dir de outro LiDAR; Voltar ao .ini.
- **Modelo**: campos `model`, `shot_size`, `vertical_angles`... e "Copiar modelo de" outro LiDAR do .ini.
- **Feixes e sombra**: raios do selecionado (com os `vertical_angles` dele); em vermelho os que batem na
  própria empilhadeira, com o % bloqueado (recalcula sozinho ao mover).
- **Ao vivo**: cada mudança vai para o `param_daemon` (`stdbuf -oL param_tool set`; o `param_tool` fica
  preso no `IPC_dispatch()` e com o `printf` no buffer, por isso o `stdbuf`) e o `viewer_3D` move a nuvem.
  Sem o sistema no ar, desliga sozinho depois de 3 falhas.
- **Salvar no .ini** grava todos os alterados (backup `astro.ini.bak-lidar_editor` na 1ª vez). Depois,
  reinicie `gazebo` e `lidarN` no `proccontrol_gui` para a simulação usar.

## Modelos dos LiDARs (22/10)

| LiDAR | modelo | onde | taxa |
|---|---|---|---|
| 0 | Pandar XT-32 (bloco do ARGOS, `data/argos/parameters/astro-argos.ini` lidar5) | teto | 20 Hz, 1990 tiros |
| 1-3 | Hesai JT128 (hemisférico, curto alcance) | cantos, invertidos (roll 180°) | 10 Hz, 900 tiros |
| 4 | Hesai JT128 | **preso na garra** (`lidar4_link Base_das_garras_1`) | 10 Hz |

- JT128 (hesaitech.com/product/jt128): 128 canais, vertical −4,4°…90,5° (0,74°), horizontal 0,4°,
  1.152.000 pts/s (= 900 × 128 × 10 Hz), 40 m @10% / 60 m máx. A Hesai não publica a tabela de
  ângulos por canal: os 128 `vertical_angles` são uniformes nessa faixa (trocar quando tiver o arquivo
  de calibração do sensor real). Alcance mínimo 0,05 m (`min_sensing 13` com `range_division_factor 250`).
- XT-32: copiado do ARGOS, inclusive `min_sensing 1000` / `range_division_factor 250` = **só enxerga a
  partir de 4 m**. Os parâmetros do OT-128 da IARA que não existem no XT-32 (`horizontal_angles`,
  `channel_group*`) ficaram comentados com `#OT128`. O `param.yaml` do SC-LIO-SAM foi para `N_SCAN 32`,
  `Horizon_SCAN 2000`, `lidarMaxRange 70` (como o do ARGOS). Os mapas 3D/2D do `teste_2` foram feitos
  com o OT-128; para o SC-LIO-SAM, refazer o mapa 3D com o XT-32.
- Gazebo: `lidar_vstep_deg 0.25` (lidar0) e `extra_lidar_vstep_deg 0.75` (JT128); campo vertical
  limitado a ±90° (o canal de 90,5° usa a linha da borda no `forklift_lidar_sim`). `lidarN_link` prende
  o sensor num link do modelo (a pose do .ini é com a garra em repouso).
- O astro (viewer_3D, mapper...) ainda trata o lidar4 como fixo no carro: a nuvem dele só fica certa
  com a garra em repouso. Medir altura/fechamento da garra por ele fica para depois.
- No editor 3D: caixa "preso na garra", "Usar modelo pronto" (XT-32 / JT128) e `ray_order` ajustado ao
  `shot_size` ao salvar.

## Log com os 5 LiDARs e a VDU multi-LiDAR (22/10)

- `log_emp-2026-10-22_20-47-07` saiu **sem `XSENS_QUAT`** (o `forklift_gz_pose` ficou sem a IMU do Gazebo
  naquela sessão, com a janela aberta). `tools/add_xsens_to_log.py <log>` recria o XSENS a 100 Hz pelo
  GPS/HDT gravados (pose verdadeira no simulador) numa cópia `<log>_xsens.txt`, com links para as pastas
  `_lidarN`/`_cameraN`. As 3 etapas da VDU usam essa cópia.
- `forklift_gz_pose` agora publica o `XSENS_QUAT` mesmo sem a IMU do Gazebo: giro pela derivada do
  quaternion da pose verdadeira (testado: curva de 0,3 rad/s → 0,298), aceleração pela pose como antes.
- Etapa 3 (mapa 2D) com todos os LiDARs: `./mapper ... -mapper_lidar1 on ... -mapper_lidar4 on` (sobrescreve
  o .ini só nesse processo). Acrescentados `mapper_lidar1..4_mode/min_height_from_ground/max_height_from_ground`
  (iguais ao lidar0) ao astro.ini, que o mapper exige quando o LiDAR está ligado.
- JT128 agora a **20 Hz** (o sensor faz 10 ou 20): 450 disparos/volta, 0,8° na horizontal; o
  `time_between_shots` é o mesmo (taxa de pontos fixa), muda só `extra_lidar_rate:=20` e `-rate 20`.
- Etapa 3: o `lidar4` ficou **fora** do mapa 2D (preso na garra, móvel). Chão = LiDARs 0-3.
- **Mapa de teto (level 1)** na etapa 3: `mapper_l1` (`./mapper ... -level 1`, só o `lidar0`) grava em
  `geodata/<log>/map_files_l1/`; o `mss` sobe com `-level0 map_files -level1 map_files_l1`; no fim,
  `complete_map_l1`. Faixa: `mapper_lidar0_min/max_height_from_ground_level1` 8,0 / 14,0 m (teto do
  big_depot entre 11,3 e 12,8 m, prateleiras até ~7,1 m). Os JT128 invertidos não veem o teto.

## Etapa 4: RDDF pelo playback (`process-vdu-04-rddf-playback.ini`)

Sem Gazebo: `playback` do log + `graphslam_publish` (as mesmas poses achatadas da etapa 3) → `globalpos`
→ `rddf_build` (sobe desligado; ligue no início do trecho, desligue no fim) → `rddf/rddf_emp_<data>.txt`;
`navigator_gui2` com o mapa da etapa 3. Depois: `tools/make_graph.sh <rddf> rddf_emp <map_files do log>`
ou o mapeditor2.
- Poses do SC-LIO-SAM com z/pitch escorregando (±2,8 m, ±3°) neste log: a etapa 3 usa
  `sc_lio_sam/poses_opt.dat` achatado (z = roll = pitch = 0; chão do simulador é plano) e o original fica
  em `poses_opt_original.dat`. Mapa 2D resultante: chão limpo (sem chão virando obstáculo), teto com a
  estrutura do telhado.

## Mundo do porto (`GZ_WORLD=port`)

`~/atlas/ros2/forklift_sim_ws/src/forklift_sim/tools/make_port_world.py` gera `worlds/port.sdf` a partir do
`big_depot.sdf`, no estilo do terminal de celulose da foto do local:
- galpão com a **frente sul aberta** (colunas) e cobertura avançando 12 m sobre o pátio; paredes N/L/O com a
  textura de parede do Depot em ladrilhos;
- pátio de **concreto** (textura gerada `port_concrete.png`, placas de 6 m), faixa de pedestre vermelha, tracejados,
  cones, placas, postes; **cais** com meio-fio amarelo (colisão) e cabeços; **água** (textura gerada `port_water.png`);
- **navio graneleiro** com guindastes de bordo: modelo "Ship AAA" baixado pelo Miguel (`worlds/models/ship_aaa`,
  escala 7,7 ≈ 180 × 31 m). O casco tinha faces viradas para dentro (via-se o interior do navio): o
  `ship_aaa.obj` usado tem **dupla face** nas 148 peças grandes afetadas (ver `LEIA-ME.txt` lá);
- **caminhões** "Cargo Truck" (J-Toastie, poly.pizza, CC BY 3.0 — `worlds/models/ATTRIBUTION.txt`);
- **unidades de celulose** (`worlds/models/pulp_unit`): 8 fardos de 250 kg (94,5 × 69 × 45 cm, 3 arames) =
  **2 t, 0,945 × 1,39 × 1,80 m**, 7 arames de 3 mm (ficha técnica UPM Euca, BEKP de eucalipto — a Portocel não
  publica a sua; confirmar com a operação). Textura gerada: celulose creme com camadas das folhas, juntas
  entre os fardos, arames e marcação de lote. Sob a cobertura: 5 blocos de 10 × 4, 3 de altura (5,4 m); no
  cais: 3 blocos de 6 × 2, 2 de altura. Uma caixa de colisão por bloco.
- Nasce no pátio (x 5, y −32); dentro do galpão: `x:=0.2 y:=8.8`. Os mapas do big_depot não servem lá fora.

## GPU, joystick do operador, colisão e fardos com física (23/10)

- **GPU**: o notebook é híbrido e o OpenGL ia para a Intel (RAM alta, NVIDIA parada, Gazebo a 0,2 do tempo
  real). As linhas `gazebo` e `viewer_3D` dos process agora começam com
  `env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ...`. Confira com `nvidia-smi`.
- **Joystick = operador no CAN** (como o ojTorc, sem o `vehicle_driver`): frame [SIM] `18FF52F0`
  (grupo `operator` do DBC). Back assume/devolve; Y só solta/puxa o freio de mão (não dá Go); X frente/ré;
  analógico direito acelera (cima) / freia (baixo). Ver `src/forklift_gui/README.md`. O `forklift_driver`
  sobe com `-require_autonomous off` para a garra andar também com o operador no comando.
- **CAUSA dos problemas de 23/10** (nao virava, patinava, inclinava 35 graus e saia do chao, garra que nao subia):
  o DetachableJoint do Gazebo instalado (10.5) NAO tem `initial_attach` -- ao nascer, a empilhadeira ficava presa
  as 6 unidades com fisica (12 t no braco da garra, a 20-60 m). O `can_bridge` agora solta todas na partida e
  confere o estado ("6 unidades soltas da garra ..." no log). Testado isolado: curva no batente a 2,9 m/s gira
  0,69 rad/s (a geometria da 0,70) com inclinacao 0,02 graus; a pega levanta a unidade a 1,2 m e solta.
- **Base**: anda como antes (velocidade imposta, rodas com atrito 1,0). O nivelamento/bloqueio de
  `forklift_sim/gz_body.py` ficou DESLIGADO (`base_physics: true` no `can_bridge.yaml` liga): em 23/10 ele
  travava a empilhadeira nas curvas fechadas.
- **Fardos com física** (teste): 6 unidades soltas no cais (`pulp_dyn_0..5`, 2 t cada, em fila com 1 m de folga,
  x 10,5 a 20,2 e y -56,7). As outras pilhas sairam do mundo (eram 642 malhas). Com 72 unidades soltas a fisica
  sozinha levava o Gazebo a 0,7 do tempo real. Aproxime com a face de 0,945 m entre os braços (a de
  1,39 m não cabe na abertura de 1,29 m) e feche a garra: ela para ao encostar e, 0,4 s depois, prende a
  unidade (DetachableJoint no `Garra_1_1`, uma por unidade, gerada pelo `sim.launch.py`). As de cima vão
  pelo atrito. Abrindo 2 cm, solta. O log do `can_bridge` mostra "garra PEGOU / SOLTOU".

## Outra máquina / outro ROS 2

No topo dos `process-empilhadeira-sim-*.ini`:

- `SET ROS2_DISTRO=lyrical`: a distro do ROS 2 (`/opt/ros/<distro>/setup.bash`) usada para subir o Gazebo
  (`jazzy`, `kilted` e `lyrical` usam os mesmos nomes de plugin do Gazebo).
- `SET FORKLIFT_WS=$HOME/atlas/ros2/forklift_sim_ws`: onde está o workspace do simulador (repositório atlas).
- `SET GL_ENV=...`: variáveis de OpenGL do notebook com placa NVIDIA híbrida (PRIME offload). Em PC sem placa
  híbrida, deixe vazio (`SET GL_ENV=`), senão o Gazebo/viewer_3D não abrem.

Também dá para trocar sem editar o arquivo: `./proccontrol -SET ROS2_DISTRO=jazzy -SET GL_ENV= <process>` (o process por ultimo).

Os módulos `forklift_gz_pose` e `forklift_lidar_sim` acham o gz-transport/gz-msgs por `pkg-config` (vendor
do `$ROS_DISTRO`, do último `/opt/ros/*` que tiver o Gazebo, ou do apt). Para forçar: `make GZ_ROS_DISTRO=jazzy`.

## Poses do SC-LIO-SAM: achatar antes da VDU-03

Na simulacao o SC-LIO-SAM deriva em z/roll/pitch (log de 23/10: z -4..+6 m, roll ate' -20 graus, pitch ate' +19).
O `graphslam_publish` passa a pose 6D ao mapper e o chao vira obstaculo no mapa 2D. Depois de copiar o
`poses_opt.dat` da VDU-02, achate (z = roll = pitch = 0, guarda o original em `poses_opt_original.dat`):

    ../data/empilhadeira_sim/tools/flatten_poses.py ../data/empilhadeira_sim/geodata/<log>/sc_lio_sam

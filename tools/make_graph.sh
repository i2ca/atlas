#!/bin/bash
# Grafo do route_planner a partir de um RDDF gravado no simulador (bin/README_COMO_FAZER_UM_GRAFO_A_PARTIR_DE_UM_RDDF.txt).
#
#   ../data/empilhadeira_sim/tools/make_graph.sh <rddf.txt> [nome_do_grafo] [map_files] [nearby_lane_range_m]
#
# Rodar de dentro de $ASTRO_HOME/bin. Gera ../data/empilhadeira_sim/graphs/<nome>.gr (padrao: o nome do
# RDDF). map_files padrao: o mapa 2D do teste_2. O RDDF pode ser o de 12 colunas do rddf_build ou o de 6
# do rddf_build_from_file (testado em 22/10 com os dois: mesmo grafo).
#
# Com o mapa ja' limpo: o road_network_generator usa o mapa para a largura da faixa (nearby_lane_width).

set -e
if [ -z "$1" ]; then
	sed -n '2,9p' "$0"
	exit 1
fi
cd "$ASTRO_HOME/bin"
RDDF=$1
NAME=${2:-$(basename "${RDDF%.txt}")}
MAP=${3:-../data/empilhadeira_sim/geodata/teste_2/map_files}
RANGE=${4:-50}
OUT=../data/empilhadeira_sim/graphs/$NAME.gr
mkdir -p ../data/empilhadeira_sim/graphs

echo "RDDF:  $RDDF ($(wc -l < "$RDDF") pontos)"
echo "mapa:  $MAP"
echo "grafo: $OUT"
./road_network_generator --rddf "$RDDF" --graph_dir "$OUT" --nearby_lane_range "$RANGE" --offline_map_dir "$MAP"
echo
echo "Pronto. No process-empilhadeira-sim-navigate.ini: SET GRAPH=$OUT"

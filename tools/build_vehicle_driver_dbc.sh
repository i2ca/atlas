#!/bin/bash
# Gera o vehicle_driver no modo DBC (sem JAUS) como bin/ford_escape_hybrid_dbc,
# SEM alterar nada do astro: copia os arquivos versionados do src/vehicle_driver para
# uma pasta de build fora da arvore e so' la' tira o -DUSE_JAUS_PROTOCOL do Makefile.
# O bin/ford_escape_hybrid (JAUS, usado pelos outros veiculos) nao e' tocado.
#
# Uso: ./build_vehicle_driver_dbc.sh        (precisa de ASTRO_HOME)
set -e
: "${ASTRO_HOME:?exporte o ASTRO_HOME}"
BUILD=$ASTRO_HOME/build/vehicle_driver_dbc
SRC=$ASTRO_HOME/src/vehicle_driver

mkdir -p $BUILD/src/vehicle_driver
# o Makefile do modulo inclui ../Makefile.conf e ../Makefile.rules
for f in Makefile.conf Makefile.rules Makefile.vars Makefile.global-vars; do
	[ -e $ASTRO_HOME/src/$f ] && ln -sf $ASTRO_HOME/src/$f $BUILD/src/$f
done
# so' os arquivos versionados (sem .o e Makefile.depend do build JAUS)
(cd $SRC && git ls-files -z . | rsync -a --from0 --files-from=- ./ $BUILD/src/vehicle_driver/)
sed -i 's/ -DUSE_JAUS_PROTOCOL//' $BUILD/src/vehicle_driver/Makefile
grep -q USE_JAUS_PROTOCOL $BUILD/src/vehicle_driver/Makefile && { echo "falhou ao tirar o USE_JAUS_PROTOCOL"; exit 1; }

cd $BUILD/src/vehicle_driver
make -s depend >/dev/null 2>&1 || true
# alvo direto: compila e linka sem os passos de instalacao do Makefile.rules
make ford_escape_hybrid
install -m 755 ford_escape_hybrid $ASTRO_HOME/bin/ford_escape_hybrid_dbc
echo "ok: $ASTRO_HOME/bin/ford_escape_hybrid_dbc"

#!/bin/bash
#
# sample start script for 3D soccer simulation
#
AGENT_BINARY="seu-spark-agent"
BINARY_DIR="./"
NUM_PLAYERS=9

export LD_LIBRARY_PATH=./libs/:$LD_LIBRARY_PATH;
for ((i=1;i<=$NUM_PLAYERS;i++)); do
echo "Running agent No. $i"
"$BINARY_DIR/$AGENT_BINARY" -s $1 -t SEURedSuntest -u $i > /dev/null 2> /dev/null&
sleep 2
done

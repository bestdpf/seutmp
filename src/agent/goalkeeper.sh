#!/bin/bash
#
# sample start script for 3D soccer simulation
#
AGENT_BINARY="seu-spark-agent"
BINARY_DIR="./"
NUM_PLAYERS=1
TEAM_NAME="SEU3D"
killall -9 "$AGENT_BINARY" &> /dev/null
export LD_LIBRARY_PATH=./libs/:$LD_LIBRARY_PATH;
echo "Running agent No. 1"
"$BINARY_DIR/$AGENT_BINARY" -s $1 -t $TEAM_NAME -u 1 > /dev/null 2> /dev/null&
sleep 2
done

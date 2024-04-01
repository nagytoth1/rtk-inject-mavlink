#!/bin/bash
SITL_PATH="$HOME/rover"

# Check if the directory does not exist
if [ ! -d "$SITL_PATH" ]; then
    # The directory does not exist, so create it
    mkdir "$SITL_PATH"
    echo "Directory '$SITL_PATH' created."
fi
cd $SITL_PATH
echo "SITL started!"
sim_vehicle.py -v Rover --no-mavproxy > /dev/null &
echo "Launching MAVProxy..."
sleep 2
mavproxy.py --master=tcp:127.0.0.1:5760 --sitl=127.0.0.1:5501 --out=udpin:0.0.0.0:14550 --out=udp:0.0.0.0:14551

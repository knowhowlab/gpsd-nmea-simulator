#!/usr/bin/env sh

python gpssim.py &

sleep 1

/usr/sbin/gpsd -G -N -n tcp://localhost:8888

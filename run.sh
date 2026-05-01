#!/bin/sh
set -e
echo "[SRNE] Pornire..."
while true; do
    python3 /srne_modbus.py || true
    echo "[SRNE] Script oprit, restart in 10 secunde..."
    sleep 10
done

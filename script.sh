#!/bin/sh

killall JLinkGDBServerCL.exe
killall socat

# Start JLink GDB Server with specified parameters
"/mnt/c/Program Files/SEGGER/JLink_V812a/JLinkGDBServerCL.exe" -singlerun -nogui -if swd -port 50000 -swoport 50001 -telnetport 50002 -device STM32H757ZI -nolocalhostonly -rtos GDBServer/RTOSPlugin_ChibiOS.so &

sleep 1

socat TCP-LISTEN:50000,bind=127.0.0.1 TCP-CONNECT:172.25.32.1:50000

fg
fg
fg


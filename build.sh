#/usr/bin/bash
gcc -O2  -o main main.c $(pkg-config --libs --cflags raylib) -lm -lpthread

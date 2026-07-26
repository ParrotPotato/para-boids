#/usr/bin/bash
gcc -O3 -mavx2 -o main main.c $(pkg-config --libs --cflags raylib) -lm -lpthread

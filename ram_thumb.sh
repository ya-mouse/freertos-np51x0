#!/bin/sh

export USE_THUMB_MODE=YES
export DEBUG=-g
export OPTIM=-O0
export RUN_MODE=RUN_FROM_RAM
export LDSCRIPT=lpc2106-ram.ld
make

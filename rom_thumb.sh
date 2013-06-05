#!/bin/sh

export USE_THUMB_MODE=YES
export DEBUG=
export OPTIM=-O3
export RUN_MODE=RUN_FROM_ROM
export LDSCRIPT=lpc2106-rom.ld
make

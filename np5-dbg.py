#!/usr/bin/python3
import os
import sys
import numpy as np
from struct import pack, unpack

def process_file(fwname, outfile):
    firmware = np.memmap(fwname, dtype=np.uint32)

    fwpart = np.array([
        np.fromstring(b'2G8P', dtype=np.uint32),
        0x03040000,                        # +0x04
        len(firmware)*4,                   # +0x08, fw length
        0x00000020,                        # +0x0c, fw offset
        np.sum(firmware, dtype=np.uint32), # +0x10, fw csum
        0x00000000,
        0x00000000,
        0x00000000,
    ], dtype=np.uint32)

    fwpart = np.append(fwpart, firmware)

    del firmware

    fwpart.tofile(outfile)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: np5-dbg firmware.bin outfile')
        sys.exit(1)
    _, firmware, outfile = sys.argv
    process_file(firmware, outfile)

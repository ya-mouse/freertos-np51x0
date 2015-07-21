#!/usr/bin/python3
import os
import sys
import numpy as np
from struct import pack, unpack

def process_file(fwname, cfgname, dataname, outfile):
    try:
        config = np.memmap(cfgname, dtype=np.uint32)
    except ValueError:
        config = np.array([], dtype=np.uint32)
    try:
        data = np.memmap(dataname, dtype=np.uint32)
    except ValueError:
        data = np.array([], dtype=np.uint32)

    np5 = np.array([
        np.fromstring(b'*FRM', dtype=np.uint32),
        0x01000000,
        0x00000000, # + 0x08, file length
        0x00020060, # + 0x0c
    ], dtype=np.uint32)

    np5 = np.lib.pad(np5, (0, 12), mode='constant')

    np5 = np.append(np5, np.array([
        0x00000002, # + 0x40, type
        0x00000060, # + 0x44, web offset
        0x00000000, # + 0x48, web length
        0x00000000, # + 0x4c
        0x00000001, # + 0x50, type
        0x00000000, # + 0x54, fw offset
        0x00000000, # + 0x58, fw length
        0x00000000, # + 0x5c
    ], dtype=np.uint32))

    web = np.lib.pad(np.fromstring(b'NPort51x0\0\0\0', dtype=np.uint32),
                     (0,5), mode='constant')
    web = np.append(web, np.array([
        0x03020000, # +0x20
        0x4e36ec31, # +0x24
        np.sum(config.view(dtype=np.uint8), dtype=np.uint32), # +0x28, dir csum
        np.sum(data.view(dtype=np.uint8), dtype=np.uint32),   # +0x2c, data csum
        0x00000100,                              # +0x30, dir offset
        len(config)*4,                           # +0x34, data offset
        0x00360040,                              # +0x38
        len(data)*4,                             # +0x3c, data length
    ], dtype=np.uint32))

    web = np.lib.pad(web, (0, 0x30), mode='constant') # Pad to 0x100

    web = np.append(web, config)
    web = np.append(web, data)

    del data
    del config

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

    np5 = np.append(np5, web)
    np5 = np.append(np5, fwpart)

    np5[0x02] = len(np5)*4        # +0x08
    np5[0x12] = len(web)*4        # +0x48
    np5[0x15] = len(web)*4 + 0x60 # +0x54
    np5[0x16] = len(fwpart)*4     # +0x58

    np5.tofile(outfile)

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print('Usage: np5 firmware.bin config.bin data.bin outfile')
        sys.exit(1)
    _, firmware, config, data, outfile = sys.argv
    process_file(firmware, config, data, outfile)

#!/bin/bash
idf.py flash -p /dev/cu.SLAB_USBtoUART
idf.py -p /dev/cu.SLAB_USBtoUART monitor
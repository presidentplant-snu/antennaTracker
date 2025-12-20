#!/bin/bash

#mavproxy.py --master /dev/blahblah --master udp:blahblah --out 127.0.0.1:13785 --default-modules output
mavproxy.py --master 127.0.0.1:14550 --out 127.0.0.1:13786 --out 127.0.0.1:13785 --default-modules output

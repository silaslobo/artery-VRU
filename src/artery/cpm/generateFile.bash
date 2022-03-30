#!/bin/bash

cd compiled && asn1c -fcompound-names -fincludes-quoted -no-gen-example ../cpm.asn && cd ..

python rmUselessFiles.py

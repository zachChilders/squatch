#! /bin/bash

idf.py build || ./scripts/init.sh && idf.py build

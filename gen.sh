#!/usr/bin/env bash

eval "$(conda shell.bash hook)"
conda activate contact

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/test1"
python contact.py --file data/tube_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/test1"
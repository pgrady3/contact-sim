#!/usr/bin/env bash

eval "$(conda shell.bash hook)"
conda activate contact

# python contact.py --file data/tube_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/tube_dense" --elastic 20 --damping 5 --bending 0.05
# python vis.py  --folder="output/" --infile="tube_dense"

# python contact.py --file data/tube_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/tube_dense_100_25_05" --elastic 100 --damping 25 --bending 0.05
# python vis.py  --folder="output/" --infile="tube_dense_100_25_05"

# python contact.py --file data/tube_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/tube_dense_5_1_05" --elastic 5 --damping 1 --bending 0.05
# python vis.py  --folder="output/" --infile="tube_dense_5_1_05"

# python contact.py --file data/boot_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/boot_dense" --elastic 50 --damping 5 --bending 0.05
# python vis.py  --folder="output/" --infile="boot_dense"

# python contact.py --file data/torus_dense.vtk --objeuler="[0, 0, 1.57]" --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.3]" --outfile="output/torus_dense" --elastic 50 --damping 5 --bending 0.05
# python vis.py  --folder="output/" --infile="torus_dense"

# python contact.py --file data/ball_dense.vtk --euler="[1.57, 0.4, 0]" --pos="[1.1, 0, 1.3]" --outfile="output/ball_dense" --elastic 50 --damping 5 --bending 0.5
# python vis.py  --folder="output/" --infile="ball_dense"

# python contact.py --file data/bread_dense.vtk --objeuler="[1.57, 0.8, -0.8]" --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.3]" --outfile="output/bread_dense" --elastic 50 --damping 5 --bending 0.05
# python vis.py  --folder="output/" --infile="bread_dense"

# python contact.py --file data/paper_roll_dense.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.3]" --outfile="output/paper_roll_dense" --elastic 50 --damping 5 --bending 0.05
python vis.py  --folder="output/" --infile="paper_roll_dense"

#Ditto is huge
# python contact.py --file data/ditto.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.3]" --outfile="output/ditto" --elastic 50 --damping 5 --bending 0.05
# python vis.py  --folder="output/" --infile="ditto"
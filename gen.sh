#!/usr/bin/env bash

eval "$(conda shell.bash hook)"
conda activate contact

# python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/hand_tube"
# python vis.py  --infile="output/hand_tube"
# python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube" --allegro
# python vis.py  --infile="output/allegro_tube"
# python contact.py --file data/boot.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/hand_boot"
# python vis.py  --infile="output/hand_boot"
# python contact.py --file data/boot.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_boot" --allegro
# python vis.py  --infile="output/allegro_boot"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_50_5_5" --allegro --elastic 50 --damping 5 --bending 5
python vis.py  --infile="output/allegro_tube_50_5_5"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_500_5_5" --allegro --elastic 500 --damping 5 --bending 5
python vis.py  --infile="output/allegro_tube_500_5_5"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_5_5_5" --allegro --elastic 5 --damping 5 --bending 5
python vis.py  --infile="output/allegro_tube_5_5_5"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_50_50_5" --allegro --elastic 50 --damping 50 --bending 5
python vis.py  --infile="output/allegro_tube_50_50_5"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_50_05_5" --allegro --elastic 50 --damping 0.5 --bending 5
python vis.py  --infile="output/allegro_tube_50_05_5"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_50_5_50" --allegro --elastic 50 --damping 5 --bending 50
python vis.py  --infile="output/allegro_tube_50_5_50"

python contact.py --file data/tube.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/allegro_tube_50_5_05" --allegro --elastic 50 --damping 5 --bending 0.5
python vis.py  --infile="output/allegro_tube_50_5_05"

#python contact.py --file data/banana_big.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/banana"
#python contact.py --file data/ball.vtk --euler="[1.57, 0.2, 0]" --pos="[1.1, 0, 1.0]" --outfile="output/ball"
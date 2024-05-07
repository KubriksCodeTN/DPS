# PD-line

## Overleaf
https://www.overleaf.com/5231953997qtgqsbpcmtwh#f5c2a8

## Desmos demos
https://www.desmos.com/calculator/ucp5rgnzow?lang=it <br>
https://www.desmos.com/calculator/rjevj8bn16?lang=it

## Usage
- make makedir
- make sequential (or make cuda)
- bin/dubins_sequential < "test/xxx_points.txt" (or dubins_cuda)

the output can be copied into desmos to visualize the path

a new test file named "xyz_points.txt", where xyz is the number of desired points, can be create under the folder "test" by running
python3 gen_points.py xyx

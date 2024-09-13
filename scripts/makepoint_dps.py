from random import uniform, randint
from math import sin, cos, pi, tan
from sys import argv
from os import mkdir, listdir

if len(argv) != 2:
    print(f"ERROR!!!\nusage: python3 {argv[0]} <number of points>")
    exit(-1)

n_points = int(argv[1])
radius = .5
min_th = pi * .75
min_edge_len = 2 * tan(min_th / 2)
x = 0
y = 0
th = uniform(0, pi)

if "test" not in listdir("../src"):
    mkdir("../src/test")
    
with open(f"../src/test/{n_points}_points.txt", "w+") as f:
    f.write(f"{n_points}\n")
    while(n_points > 0):
        th = th % (2*pi)
        f.write(f"{x},{y}\n")
        n_points = n_points - 1
        new_th = th + uniform(-min_th, min_th)
        while(abs(new_th - th) < 1.): # problem with OMPL: if starting angle ~ final angle it blows up
            new_th += 1.

        th = new_th        
        offset = randint(1, 10)
        x = x + (min_edge_len + offset) * radius * cos(th)
        y = y + (min_edge_len + offset) * radius * sin(th)
        

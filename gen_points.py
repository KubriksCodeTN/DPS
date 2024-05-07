from random import uniform
from math import sin, cos, pi, tan
from sys import argv


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
with open(f"test/{n_points}_points.txt", "w") as f:
    f.write(f"{n_points}\n")
    while(n_points > 0):
        f.write(f"{x},{y}\n")
        n_points = n_points - 1
        th = th + uniform(-min_th, min_th)
        x = x + (min_edge_len + 1) * radius * cos(th)
        y = y + (min_edge_len + 1) * radius * sin(th)
        

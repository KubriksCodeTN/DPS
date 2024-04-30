from random import uniform
from math import sin, cos, pi

n_points = 15
radius = .5
x = 0
y = 0
th = uniform(0, pi)
with open(f"{n_points}_points.txt", "w") as f:
    f.write(f"{n_points}\n")
    while(n_points > 0):
        f.write(f"{x},{y}\n")
        n_points = n_points - 1
        th = th + uniform(-pi * .75, pi * .75)
        x = x + 4 * radius * cos(th)
        y = y + 4 * radius * sin(th)
        

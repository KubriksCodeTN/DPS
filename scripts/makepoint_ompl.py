from math import sqrt
from math import sin, atan2, pi
from os import listdir, system

R = .5

def through_q1(file):
    with open(file, "r") as f:
        m = f.read().split()

        xf, yf = float(m[-1].split(",")[0]), float(m[-1].split(",")[1])
        thf = atan2(yf - float(m[-2].split(",")[1]), xf - float(m[-2].split(",")[0]))
        last_point = (xf, yf, thf)

        m = m[1:]
        x0, y0 = float(m[0].split(",")[0]), float(m[0].split(",")[1])
        x1, y1 = float(m[1].split(",")[0]), float(m[1].split(",")[1])
        th0 = atan2(y1 - y0, x1 - x0)
        l = [(x0, y0, th0)]

        q1x, q1y = 0, 0

        for i in range(1, len(m) - 1):
            A = [float(x) for x in m[i - 1].split(",")]
            B = [float(x) for x in m[i].split(",")]
            C = [float(x) for x in m[i + 1].split(",")]

            vx0 = B[0] - A[0]
            vy0 = B[1] - A[1]
            norm0 = sqrt(vx0 * vx0 + vy0 * vy0)
            unitx0 = vx0 / norm0
            unity0 = vy0 / norm0

            vxf = C[0] - B[0]
            vyf = C[1] - B[1]
            thf = atan2(vyf, vxf)
            normf = sqrt(vxf * vxf + vyf * vyf)
            unitxf = vxf / normf
            unityf = vyf / normf

            cross = vx0 * vyf - vy0 * vxf
            across = abs(cross)
            dot = vx0 * vxf + vy0 * vyf
            d = R * ((across) / (dot + norm0 * normf))
            alpha = pi - atan2(across, dot)

            # q0x = B[0] - unitx0 * d 
            # q0y = B[1] - unity0 * d
            q1x = B[0] + unitxf * d
            q1y = B[1] + unityf * d

            l.append((q1x, q1y, thf))
        #l.append((q1x, q1y)) # final new_a before the last trait
        l.append(last_point)

        return l

def mpdp_through_q1(file):
    pts = through_q1(f"../src/test/{file}")

    with open(f"../tests/dubins_ompl/test/{file}","w+") as f:
        
        f.write(f"{len(pts)}\n")
        for i in pts:
            f.write(f"{i[0]:.18f},{i[1]:.18f},{i[2]:.18f}\n") 

if __name__ == "__main__":

    system("mkdir ../tests/dubins_ompl/test")

    for f in listdir("../src/test"):
        mpdp_through_q1(f)
          

from math import sqrt
from math import sin, atan2, pi
from os import listdir, system

R = .5

def through_q0(file):
    with open(file, "r") as f:
        m = f.read().split()

        last_point = (float(m[-1].split(",")[0]), float(m[-1].split(",")[1]))

        m = m[1:]

        l = [(float(m[0].split(",")[0]), float(m[0].split(",")[1]))]

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
            normf = sqrt(vxf * vxf + vyf * vyf)
            unitxf = vxf / normf
            unityf = vyf / normf

            cross = vx0 * vyf - vy0 * vxf
            across = abs(cross)
            dot = vx0 * vxf + vy0 * vyf
            d = R * ((across) / (dot + norm0 * normf))
            alpha = pi - atan2(across, dot)

            q0x = B[0] - unitx0 * d 
            q0y = B[1] - unity0 * d
            # q1x = B[0] + unitxf * d
            # q1y = B[1] + unityf * d

            l.append((q0x, q0y))
        #l.append((q1x, q1y)) # final new_a before the last trait
        l.append(last_point)

        return l

def through_b2(file):
    with open(file, "r") as f:
        m = f.read().split()

        last_point = (float(m[-1].split(",")[0]), float(m[-1].split(",")[1]))

        m = m[1:]

        l = [(float(m[0].split(",")[0]), float(m[0].split(",")[1]))]

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
            normf = sqrt(vxf * vxf + vyf * vyf)
            unitxf = vxf / normf
            unityf = vyf / normf

            cross = vx0 * vyf - vy0 * vxf
            across = abs(cross)
            dot = vx0 * vxf + vy0 * vyf
            d = R * ((across) / (dot + norm0 * normf))
            alpha = pi - atan2(across, dot)

            q0x = B[0] - unitx0 * d 
            q0y = B[1] - unity0 * d
            q1x = B[0] + unitxf * d
            q1y = B[1] + unityf * d

            m1 = -vx0 / vy0
            m2 = -vxf / vyf

            dx = (m1 * q0x - q0y - m2 * q1x + q1y) / (m1 - m2)
            dy = m1 * (dx - q0x) + q0y
            
            vxb = B[0] - dx
            vyb = B[1] - dy
            normb = sqrt(vxb * vxb + vyb * vyb)
            
            unitxb = vxb / normb
            unityb = vyb / normb

            xx = B[0] + unitxb * R * (1 - 1 / sin(alpha / 2))
            xy = B[1] + unityb * R * (1 - 1 / sin(alpha / 2))

            l.append((xx, xy))
        #l.append((q1x, q1y)) # final new_a before the last trait
        l.append(last_point)

        return l

def through_q1(file):
    with open(file, "r") as f:
        m = f.read().split()

        last_point = (float(m[-1].split(",")[0]), float(m[-1].split(",")[1]))

        m = m[1:]

        l = [(float(m[0].split(",")[0]), float(m[0].split(",")[1]))]

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

            l.append((q1x, q1y))
        #l.append((q1x, q1y)) # final new_a before the last trait
        l.append(last_point)

        return l

def through_q0_q1(file):
    with open(file, "r") as f:
        m = f.read().split()

        last_point = (float(m[-1].split(",")[0]), float(m[-1].split(",")[1]))

        m = m[1:]

        l = [(float(m[0].split(",")[0]), float(m[0].split(",")[1]))]

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
            normf = sqrt(vxf * vxf + vyf * vyf)
            unitxf = vxf / normf
            unityf = vyf / normf

            cross = vx0 * vyf - vy0 * vxf
            across = abs(cross)
            dot = vx0 * vxf + vy0 * vyf
            d = R * ((across) / (dot + norm0 * normf))
            alpha = pi - atan2(across, dot)

            q0x = B[0] - unitx0 * d 
            q0y = B[1] - unity0 * d
            q1x = B[0] + unitxf * d
            q1y = B[1] + unityf * d

            l.append((q0x, q0y))
            l.append((q1x, q1y))

        l.append(last_point)

        return l

def mpdp_through_b2(file):
    pts = through_b2(f"../src/test/{file}")

    with open(f"../tests/mpdp/test/through_b2/{file}","w+") as f:
        
        # I need to impose the correct start and end angles
        th0 = atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        thf = atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])

        f.write(f"{len(pts)}\n")
        f.write(f"{pts[0][0]} {pts[0][1]} {th0}\n")
        for i in pts[1:-1]:
            f.write(f"{i[0]} {i[1]}\n") # no need to write ANGLE::FREE
        f.write(f"{pts[-1][0]} {pts[-1][1]} {thf}\n")

def mpdp_through_q0(file):
    pts = through_q0(f"../src/test/{file}")

    with open(f"../mpdp/test/through_q0/{file}","w+") as f:
        
        # I need to impose the correct start and end angles
        th0 = atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        thf = atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])

        f.write(f"{len(pts)}\n")
        f.write(f"{pts[0][0]} {pts[0][1]} {th0}\n")
        for i in pts[1:-1]:
            f.write(f"{i[0]} {i[1]}\n") # no need to write ANGLE::FREE
        f.write(f"{pts[-1][0]} {pts[-1][1]} {thf}\n")

def mpdp_through_q1(file):
    pts = through_q1(f"../src/test/{file}")

    with open(f"../mpdp/test/through_q1/{file}","w+") as f:
        
        # I need to impose the correct start and end angles
        th0 = atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        thf = atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])

        f.write(f"{len(pts)}\n")
        f.write(f"{pts[0][0]} {pts[0][1]} {th0}\n")
        for i in pts[1:-1]:
            f.write(f"{i[0]} {i[1]}\n") # no need to write ANGLE::FREE
        f.write(f"{pts[-1][0]} {pts[-1][1]} {thf}\n")

def mpdp_through_b(file):
    # useless since I just need to copy the points "as they are"
    #pts = through_b(f"dubins_cuda/test/{file}")
    pts = []
    with open(f"../src/test/{file}", "r") as f:
        pts = f.readlines()[1:]
        pts = [(float(p.split(",")[0]), float(p.split(",")[1])) for p in pts]

    with open(f"../tests/mpdp/test/through_b/{file}","w+") as f:
       
        # I need to impose the correct start and end angles
        th0 = atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        thf = atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])

        f.write(f"{len(pts)}\n")
        f.write(f"{pts[0][0]} {pts[0][1]} {th0}\n")
        for i in pts[1:-1]:
            f.write(f"{i[0]} {i[1]}\n") # no need to write ANGLE::FREE
        f.write(f"{pts[-1][0]} {pts[-1][1]} {thf}\n")

def mpdp_through_q0_q1(file):
    pts = through_q0_q1(f"../src/test/{file}")

    with open(f"../mpdp/test/through_q0_q1/{file}","w+") as f:
        
        # I need to impose the correct start and end angles
        th0 = atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        thf = atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])

        f.write(f"{len(pts)}\n")
        f.write(f"{pts[0][0]} {pts[0][1]} {th0}\n")
        for i in pts[1:-1]:
            f.write(f"{i[0]} {i[1]}\n") # no need to write ANGLE::FREE
        f.write(f"{pts[-1][0]} {pts[-1][1]} {thf}\n")

if __name__ == "__main__":

    system("mkdir ../tests/mpdp/test")

    system("mkdir ../tests/mpdp/test/through_b2")
    system("mkdir ../tests/mpdp/test/through_b")

    for f in listdir("../src/test"):
        mpdp_through_b2(f)
        mpdp_through_b(f)
  

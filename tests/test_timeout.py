from os import system
from statistics import mean
from random import uniform
from math import pi


def collide(x, y, object):
    # object: (bottom-left, top-right)
    return object[0][0] <= x <= object[1][0] and object[0][1] <= y <= object[1][1]

def points_ok(sx, sy, gx, gy, map_obs):
    for obj in map_obs:
        if collide(sx, sy, obj) or collide(gx, gy, obj):
            return False
    return True

map = "dubins_ompl/img/warehouse.svg"
map_top_left = (5, -5)
map_bot_right = (120, -76)

# (bottom left, top right)
map_obs = [((11.5201, -20.0169),(49.1801, -9.86222)),
((53.9015, -69.9063),(58.4651, -65.4007)),
((55.8886, -66.622),(60.4522, -62.1164)),
((60.4028, -69.1561),(64.9944, -64.6074)),
((58.2029, -72.8409),(62.7584, -68.3474)),
((11.5201, -36.4401),(49.1801, -26.2855)),
((11.7452, -53.4855),(49.4053, -43.3308)),
((11.4582, -69.6553),(49.1183, -59.5006)),
((60.7308, -59.3659),(65.1676, -55.0232)),
((59.28, -55.812),(63.7168, -51.4693)),
((65.6451, -57.6818),(70.1005, -53.3168)),
((63.9391, -53.7599),(68.3945, -49.3948)),
((56.4534, -39.2001),(60.9738, -34.7542)),
((58.2051, -35.7845),(62.7254, -31.3386)),
((60.9535, -41.8232),(65.4604, -37.3948)),
((62.4279, -37.8832),(66.9348, -33.4547)),
((60.2327, -24.5716),(64.6402, -20.2636)),
((58.4251, -20.3871),(62.9837, -15.8888)),
((64.8183, -22.3156),(69.2462, -17.9835)),
((63.2073, -18.3537),(67.6352, -14.0215)),
((0.05376, -40.2254),(4.10892, -0.14494)),
((0.03316, -81.2676),(4.08832, -41.1871)),
((97.2841, -20.5199),(101.063, -16.9131)),
((101.39, -20.5199),(105.168, -16.9131)),
((105.593, -20.5199),(109.371, -16.9131)),
((109.699, -20.6158),(113.477, -17.0091)),
((88.1345, -36.556),(91.913, -32.9493)),
((92.2404, -36.556),(96.0188, -32.9493)),
((96.4434, -36.556),(100.222, -32.9493)),
((100.742, -36.6039),(104.52, -32.9972)),
((105.162, -36.556),(108.94, -32.9493)),
((109.364, -36.556),(113.143, -32.9493)),
((88.0535, -53.2604),(91.832, -49.6537)),
((92.3277, -53.3563),(96.1062, -49.7497)),
((96.7112, -53.4547),(100.49, -49.8481)),
((100.817, -53.5507),(104.595, -49.944)),
((88.4651, -20.5199),(92.2436, -16.9131)),
((92.6681, -20.5199),(96.4466, -16.9131)),
((97.2841, -15.7581),(101.063, -12.1514)),
((101.39, -15.7581),(105.168, -12.1514)),
((105.593, -15.7581),(109.371, -12.1514)),
((109.699, -15.854),(113.477, -12.2474)),
((88.1345, -31.7942),(91.913, -28.1876)),
((92.2404, -31.7942),(96.0188, -28.1876)),
((96.4434, -31.7942),(100.222, -28.1876)),
((100.742, -31.8421),(104.52, -28.2354)),
((105.162, -31.7942),(108.94, -28.1876)),
((109.364, -31.7942),(113.143, -28.1876)),
((88.0535, -48.4986),(91.832, -44.8919)),
((92.3277, -48.5945),(96.1062, -44.9879)),
((96.7112, -48.693),(100.49, -45.0863)),
((100.817, -48.7888),(104.595, -45.1822)),
((88.4651, -15.7581),(92.2436, -12.1514)),
((92.6681, -15.7581),(96.4466, -12.1514)),
((77.0048, -34.4294),(81.0648, -0.26395)),
((76.9856, -81.1167),(81.0426, -43.3463)),
((105.166, -53.4547),(108.944, -49.8481)),
((109.272, -53.5507),(113.05, -49.944)),
((105.166, -48.693),(108.944, -45.0863)),
((109.272, -48.7888),(113.05, -45.1822)),
((4.49985, -4.08999),(40.0289, -0.03123)),
((40.5771, -4.21816),(76.4828, -0.15972)),
((4.83402, -80.8605),(40.3512, -76.8017)),
((40.8989, -80.9888),(76.4162, -76.9301)),
((81.8754, -4.49009),(119.646, -0.43311)),
((120.392, -40.6027),(124.447, -0.44879)),
((120.371, -80.7697),(124.427, -41.2832)),
((87.5942, -69.4373),(91.3727, -65.8306)),
((91.8684, -69.5332),(95.6469, -65.9266)),
((96.2519, -69.6316),(100.03, -66.025)),
((100.358, -69.7276),(104.136, -66.1209)),
((87.5942, -64.6755),(91.3727, -61.0688)),
((91.8684, -64.7715),(95.6469, -61.1648)),
((96.2519, -64.8699),(100.03, -61.2632)),
((100.358, -64.9657),(104.136, -61.3591)),
((104.707, -69.6316),(108.485, -66.025)),
((108.812, -69.7276),(112.591, -66.1209)),
((104.707, -64.8699),(108.485, -61.2632)),
((108.812, -64.9657),(112.591, -61.3591)),
((81.8754, -80.7902),(119.646, -76.7332)),
]


n_points = 100
n_rep = 5

def test_fixed_timeout():
    max_time = .1

    csv = open(f"../test_out/ompl_survival_3_seconds_fixed.csv", "w+")
    csv.write("test;exec;success_rate;avg_len\n")

    for _ in range(n_points):

        while True:
            start_x = uniform(map_top_left[0], map_bot_right[0])
            start_y = uniform(map_top_left[1], map_bot_right[1])

            goal_x = uniform(map_top_left[0], map_bot_right[0])
            goal_y = uniform(map_top_left[1], map_bot_right[1])

            if points_ok(start_x, start_y, goal_x, goal_y, map_obs):
                break

        #print(f"\nStart: ({start_x}, {start_y}), Goal: ({goal_x}, {goal_y})")


        lengths_rrt = []
        lengths_rrt_dubins = []
        lengths_my_rrt = []
        for _ in range(n_rep):
            
            th0 = pi / 2
            thf = th0

            #print(f"\t{exec}: ", end='')

            rv = system(f"../build/tests/dubins_ompl/rrt {start_x} {start_y} {goal_x} {goal_y} {map} {max_time} > temp.out 2> /dev/null")
            if rv == 0:
                # example line:
                # time: 21700062.415000 us | len: 154.422243 | th0: 0 | thf: 3.14
                with open("temp.out", "r") as f:
                    last_line = f.readlines()[-1]
                    th0 = float(last_line.split()[-4])
                    thf = float(last_line.split()[-1])
                    l = float(last_line.split()[5])
                    lengths_rrt.append(l)
                    

            rv = system(f"../build/tests/dubins_ompl/rrt_dubins {start_x} {start_y} {th0} {goal_x} {goal_y} {thf} {map} {max_time} > temp.out 2> /dev/null")
            if rv == 0:
                # example line:
                # time: 21700062.415000 us | len: 154.422243
                with open("temp.out", "r") as f:
                    l = float(f.readlines()[-1].split()[-1])
                    lengths_rrt_dubins.append(l)


            e = min(abs(map_top_left[1] - map_bot_right[1]), abs(map_top_left[0] - map_bot_right[0])) / 100
            stepsz = 6
            gamma = 10
            rv = system(f"../build/tests/rrt/main_2d {e} {stepsz} {gamma} {max_time} {start_x} {start_y} {goal_x} {goal_y} < rrt/test/map/warehouse > temp.out 2> /dev/null")
            if rv == 0:
                # example line:
                # time: 21700062.415000 us | len: 154.422243
                with open("temp.out", "r") as f:
                    l = float(f.readlines()[-1])
                    lengths_my_rrt.append(l)

        print("rrt: ", lengths_rrt)
        print("dubins: ", lengths_rrt_dubins)
        print("my_rrt: ", lengths_my_rrt)

        csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});rrt;{len(lengths_rrt) / n_rep};{mean(lengths_rrt) if len(lengths_rrt) != 0 else 0}\n")
        csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});rrt_dubins;{len(lengths_rrt_dubins) / n_rep};{mean(lengths_rrt_dubins) if len(lengths_rrt_dubins) != 0 else 0}\n")
        csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});my_rrt;{len(lengths_my_rrt) / n_rep};{mean(lengths_my_rrt) if len(lengths_my_rrt) != 0 else 0}\n")

    csv.close()

def test_multi_timeout():
    max_time = [.05, .1, .2 , .5, 1, 2]

    csv = open(f"../test_out/ompl_survival_multi_timeout.csv", "w+")
    csv.write("test;exec;timeout;time;len\n")

    for t in max_time:

        for _ in range(n_points):

            while True:
                start_x = uniform(map_top_left[0], map_bot_right[0])
                start_y = uniform(map_top_left[1], map_bot_right[1])

                goal_x = uniform(map_top_left[0], map_bot_right[0])
                goal_y = uniform(map_top_left[1], map_bot_right[1])

                if points_ok(start_x, start_y, goal_x, goal_y, map_obs):
                    break

            #print(f"\nStart: ({start_x}, {start_y}), Goal: ({goal_x}, {goal_y})")


            for _ in range(n_rep):
                th0 = pi / 2
                thf = th0
                length = -1
                time = -1

                rv = system(f"../build/tests/dubins_ompl/rrt {start_x} {start_y} {goal_x} {goal_y} {map} {t} > temp.out 2>> /dev/null")
                if rv == 0:
                    # example line:
                    # time: 21700062.415000 us | len: 154.422243 | th: 0 | thf: 1
                    with open("temp.out", "r") as f:
                        last_line = f.readlines()[-1]
                        th0 = float(last_line.split()[-4])
                        thf = float(last_line.split()[-1])
                        time = float(last_line.split()[1])
                        length = float(last_line.split()[5])
                csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});rrt;{t};{time};{length}\n")

                length = -1
                time = -1
                rv = system(f"../build/tests/dubins_ompl/rrt_dubins {start_x} {start_y} {th0} {goal_x} {goal_y} {thf} {map} {t} > temp.out 2> /dev/null")
                if rv == 0:
                    # example line:
                    # time: 21700062.415000 us | len: 154.422243
                    with open("temp.out", "r") as f:
                        last_line = f.readlines()[-1]
                        time = float(last_line.split()[1])
                        length = float(last_line.split()[-1])
                csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});rrt_dubins;{t};{time};{length}\n")

                length = -1
                time = -1
                e = min(abs(map_top_left[1] - map_bot_right[1]), abs(map_top_left[0] - map_bot_right[0])) / 100
                stepsz = 6
                gamma = 10
                rv = system(f"../build/tests/rrt/main_2d {e} {stepsz} {gamma} {t} {start_x} {start_y} {goal_x} {goal_y} < rrt/test/map/warehouse > temp.out 2> /dev/null")
                if rv == 0:
                    # example line:
                    # 154.422243
                    with open("temp.out", "r") as f:
                        last_line = f.readlines()[-1]
                        length = float(last_line)
                        time = 1
                csv.write(f"({start_x},{start_y})->({goal_x},{goal_y});my_rrt;{t};{time};{length}\n")
        
    csv.close()

if __name__ == "__main__":

    test_fixed_timeout()

    test_multi_timeout()

    system("rm temp.out")
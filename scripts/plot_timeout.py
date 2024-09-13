import matplotlib.pyplot as plt
from os import listdir, system
from csv import DictReader
from statistics import mean, stdev
from numpy import arange
from random import uniform
from math import pi

def load_fixed_timeout_csv(filename):
    outdata = []
    with open(filename, "r") as f:
        csvreader = DictReader(f, delimiter=";")
        for r in csvreader:
            r["success_rate"] = float(r["success_rate"])
            r["avg_len"] = float(r["avg_len"])
            if(r["avg_len"] > 0):
                outdata.append(r)

    # print(outdata)
    return outdata

def load_multi_timeout_csv(filename):
    outdata = []
    with open(filename, "r") as f:
        outdata = []
        csvreader = DictReader(f, delimiter=";")
        for r in csvreader:
            r["timeout"] = float(r["timeout"])
            r["time"] = float(r["time"])
            r["len"] = float(r["len"])
            outdata.append(r)
        
    # print(outdata)
    return outdata

def plot_fixed_timeout(outdata):

    outdata.sort(key=lambda e:e["avg_len"])

    rrt = [e["avg_len"] for e in outdata if e["exec"] == "rrt"]
    rrt_dubins = [e["avg_len"] for e in outdata if e["exec"] == "rrt_dubins"]
    my_rrt = [e["avg_len"] for e in outdata if e["exec"] == "my_rrt"]
   
   
    # print(rrt)
    # print(rrt_dubins)
    # print(my_rrt)

    rrt.sort()
    rrt_dubins.sort()
    my_rrt.sort()

    rrt_y = [rrt[0]]
    for i in rrt[1:]:
        rrt_y.append(rrt_y[-1] + i)

    rrt_dub_y = [rrt_dubins[0]]
    for i in rrt_dubins[1:]:
        rrt_dub_y.append(rrt_dub_y[-1] + i)

    my_rrt_y = [my_rrt[0]]
    for i in my_rrt[1:]:
        my_rrt_y.append(my_rrt_y[-1] + i)

    plt.plot(arange(1, len(rrt) +1), rrt_y, label="rrt")
    plt.plot(arange(1, len(rrt_dubins) + 1), rrt_dub_y, label="rrt_dubins")
    plt.plot(arange(1, len(my_rrt) + 1), my_rrt_y, label="my_rrt")

    plt.xlabel("N. of successful tests")
    plt.ylabel("Sum of lengths")

    plt.legend()
    plt.show()

def plot_multi_timeout_histogram(outdata):
    max_time = [.05, .1, .2 , .5, 1, 2]
    n_points = 100
    n_rep = 5

    histogram_rrt = {t:0 for t in max_time}
    for e in outdata:
        histogram_rrt[e["timeout"]] += (e["exec"] == "rrt") and e["time"] > 0

    histogram_rrt_dub = {t:0 for t in max_time}
    for e in outdata:
        histogram_rrt_dub[e["timeout"]] += (e["exec"] == "rrt_dubins") and e["time"] > 0

    histogram_my_rrt = {t:0 for t in max_time}
    for e in outdata:
        histogram_my_rrt[e["timeout"]] += (e["exec"] == "my_rrt") and e["time"] > 0

    # print(histogram_rrt)

    x = arange(len(histogram_rrt.values()))

    width = 0.1  # the width of the bars1
    multiplier = 0

    fig, ax = plt.subplots(figsize=(9, 5))

    rects = plt.bar(x, list(histogram_rrt.values()), width, label="rrt")
    #ax.bar_label(rects, fmt="%.3f")

    rects = plt.bar(x + width, list(histogram_rrt_dub.values()), width, label="rrt_dubins")
    #ax.bar_label(rects)

    rects = plt.bar(x + 2 * width, list(histogram_my_rrt.values()), width, label="my_rrt")

    # Add some text for labels, title and custom x-axis tick labels, etc.
    plt.xlabel("Timeout [s]")
    plt.ylabel(f'# of successful tests (out of {n_points * n_rep} trials for every timeout)')
    plt.title(f'Random start and goal points with different timeouts')
    plt.xticks(x + width / len(max_time), [str(t) for t in max_time])
    plt.legend()
    plt.show()

def statistical_analysis(outdata):
    out = []
    # --- multi-timeout table for mean and std_dev of lenghts ---
    t = {(e["test"], e["timeout"]):{"rrt":[], "rrt_dubins":[], "my_rrt":[]} for e in outdata}

    for e in outdata:
        if e["len"] > 0:
            t[(e["test"], e["timeout"])][e["exec"]].append(e["len"])

    i = 1
    for k, v in t.items():
        v["rrt"] = [mean(v["rrt"]), stdev(v["rrt"])] if len(v["rrt"]) > 2 else [-1, -1]
        v["rrt_dubins"] = [mean(v["rrt_dubins"]), stdev(v["rrt_dubins"])] if len(v["rrt_dubins"]) > 2 else [-1, -1]
        v["my_rrt"] = [mean(v["my_rrt"]), stdev(v["my_rrt"])] if len(v["my_rrt"]) > 2 else [-1, -1]

        out.append(f'test {i} with timeout {k[1]}:\n\
                   \trrt: mean={v["rrt"][0]} stdev={v["rrt"][1]}\n\
                   \trrt_dubins: mean={v["rrt_dubins"][0]} stdev={v["rrt_dubins"][1]}\n\
                   \tmy_rrt: mean={v["my_rrt"][0]} stdev={v["my_rrt"][1]}\n')
        i += 1

    return t, out

def plot_len_multi_timeout(data, timeout):
    t = data
    rrt = {k[1]:[v2["rrt"][0] for k2, v2 in t.items() if k2[1] == k[1] and v2["rrt"][0] > 0] for k in t.keys()}
    rrt_dubins = {k[1]:[v2["rrt_dubins"][0] for k2, v2 in t.items() if k2[1] == k[1] and v2["rrt_dubins"][0] > 0] for k in t.keys()}
    my_rrt = {k[1]:[v2["my_rrt"][0] for k2, v2 in t.items() if k2[1] == k[1] and v2["my_rrt"][0] > 0] for k in t.keys()}

    # print(rrt)
    # print(rrt_dubins)
    # print(my_rrt)

    for v in rrt.values():
        v.sort()
    for v in rrt_dubins.values():
        v.sort()
    for v in my_rrt.values():
        v.sort()

    rrt_y = {}
    for k, v in rrt.items():
        if len(v) > 0:
            rrt_y[k] = [v[0]]
        else:
            rrt_y[k] = [-1]

    for k, v in rrt.items():
        for i in v[1:]:
            rrt_y[k].append(rrt_y[k][-1] + i)

    rrt_dub_y = {}
    for k, v in rrt_dubins.items():
        if len(v) > 0:
            rrt_dub_y[k] = [v[0]]
        else:
            rrt_dub_y[k] = [-1]

    for k, v in rrt_dubins.items():
        for i in v[1:]:
            rrt_dub_y[k].append(rrt_dub_y[k][-1] + i)

    
    my_rrt_y = {}
    for k, v in my_rrt.items():
        if len(v) > 0:
            my_rrt_y[k] = [v[0]]
        else:
            my_rrt_y[k] = [-1]

    for k, v in my_rrt.items():
        for i in v[1:]:
            my_rrt_y[k].append(my_rrt_y[k][-1] + i)

    key = timeout

    # print(rrt_y[key])
    # print(rrt_dub_y[key])
    # print(my_rrt_y[key])

    plt.plot(arange(1, len(rrt_y[key]) + 1), rrt_y[key], label="rrt")
    plt.plot(arange(1, len(rrt_dub_y[key]) + 1), rrt_dub_y[key], label="rrt_dubins")
    plt.plot(arange(1, len(my_rrt_y[key]) + 1), my_rrt_y[key], label="my_rrt")

    plt.xlabel("N. of successful test")
    plt.ylabel("Sum of lenghts")
    plt.title(f"Timeout of {key} seconds")

    plt.legend()
    plt.show()



if __name__ == "__main__":

    fixed_timeout_data = load_fixed_timeout_csv("../test_out/ompl_survival_3_seconds_fixed.csv")
    plot_fixed_timeout(fixed_timeout_data)

    mutli_timeout_data = load_multi_timeout_csv("../test_out/ompl_survival_multi_timeout.csv")
    plot_multi_timeout_histogram(mutli_timeout_data)

    data, stat_analysis = statistical_analysis(mutli_timeout_data)

    timeout =[.05, .1, .2 , .5, 1, 2]
    for t in timeout:
        plot_len_multi_timeout(data, t)




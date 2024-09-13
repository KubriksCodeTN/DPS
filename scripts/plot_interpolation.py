import matplotlib.pyplot as plt
from csv import DictReader
from numpy import arange

def load_csv(filename):
    outdata = []
    with open(filename, "r") as f:
        csvreader = DictReader(f, delimiter=",")
        for r in csvreader:
            r["n_points"] = int(r["n_points"])
            r["time"] = float(r["time"])
            r["len"] = float(r["len"]) if r["len"] != "len" else -1
            r["discr"] = int(r["discr"])
            r["ref"] = int(r["ref"]) 
            outdata.append(r)

    #print([l for l in outdata if l["exec_name"] == "dubins_cuda"])
    return outdata

def plot_time(outdata):
    cuda = [(l["n_points"], l["time"]) for l in outdata if l["exec_name"] == "dubins_cuda"]
    cuda.sort(key = lambda x: int(x[0]))

    seq = [(l["n_points"], l["time"]) for l in outdata if l["exec_name"] == "dubins_seq"] 
    seq.sort(key = lambda x: int(x[0]))

    mpdp = [(l["n_points"], l["time"]) for l in outdata if l["exec_name"] == "mpdp" and l["type"] == "through_b2"] 
    mpdp.sort(key = lambda x: int(x[0]))

    ompl = [(l["n_points"], l["time"]) for l in outdata if l["exec_name"] == "dubins_ompl"]
    ompl.sort(key = lambda x: int(x[0]))

    plt.plot([str(c[0]) for c in cuda],[(float(c[1])) for c in cuda], label="cuda", marker='o')
    plt.plot([str(c[0]) for c in seq],[(float(c[1])) for c in seq], label="sequential", marker='o')
    plt.plot([str(c[0]) for c in mpdp],[(float(c[1])) for c in mpdp], label="dynamic prog", marker='o')
    plt.plot([str(c[0]) for c in ompl],[(float(c[1])) for c in ompl], label="dubins_seq + OMPL", marker='o')

    plt.xlabel("N. of points")
    plt.ylabel("Time [us]")

    plt.legend()
    plt.yscale('log')

    #print(seq)

def plot_len(outdata):
    seq = [(l["n_points"], l["len"]) for l in outdata if l["exec_name"] == "dubins_seq"]
    seq.sort(key = lambda x: int(x[0]))

    mpdp_b = [(l["n_points"], l["len"]) for l in outdata if l["exec_name"] == "mpdp" and l["type"] == "through_b"] 
    mpdp_b.sort(key = lambda x: int(x[0]))

    mpdp_b2 = [(l["n_points"], l["len"]) for l in outdata if l["exec_name"] == "mpdp" and l["type"] == "through_b2"] 
    mpdp_b2.sort(key = lambda x: int(x[0]))

    ompl = [(l["n_points"], l["len"]) for l in outdata if l["exec_name"] == "dubins_ompl"]
    ompl.sort(key = lambda x: int(x[0]))


    arr = zip((("dubins_seq", "dyn_prog_through_b", "dyn_prog_through_b2", "dubins_seq + OMPL")), (seq, mpdp_b, mpdp_b2, ompl))

    n_points = [str(c[0]) for c in seq]
    data = {algo[0]:[float(l[1]) for l in algo[1]] for algo in arr}

    for k in data.keys():
        if k != "dubins_seq":
            for i in range(len(data[k])):
                data[k][i] = data[k][i] / data["dubins_seq"][i]

    data["dubins_seq"] = [1 for _ in range(len(seq))]

    x = arange(len(n_points))
    width = 0.1  # the width of the bars1
    multiplier = 0

    fig, ax = plt.subplots(figsize=(18, 6))

    for n, l in data.items():
        offset = width * multiplier
        rects = ax.bar(x + offset, l, width, label=n)
        ax.bar_label(rects, padding=3, fmt="%.3f")
        multiplier += 1.5

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_xlabel("n. of points")
    ax.set_ylabel('Length w.r.t length found with sequential algo (%)')
    ax.set_title('Length of paths given by different algorithms')
    ax.set_xticks(x + width, n_points)
    ax.legend(ncol=2)
    ax.set_ylim(.90, 1.20)


    plt.show()

if __name__ == "__main__":
    outdata = load_csv("../test_out/interpolation_data.csv")

    plot_time(outdata)

    plot_len(outdata)




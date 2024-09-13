# given the svg from the test, plots the results 
# NOTE the filename is hardcoded, needs to be changed
# NOTE needs a command line argument: if 0 is 1 is passed doesn't account for A* times
import matplotlib.pyplot as plt
import sys
import csv

with open("data_out_map", "r") as f:
    csvf = csv.DictReader(f)
    data = []
    for row in csvf:
        data.append(row)

# TODO this is garbage
if sys.argv[1] == "1":
    for x in data:
        x["A* (us)"] = "0" 

Ns = list(dict.fromkeys(sorted([int(x["N"]) for x in data])))
times1 = [(int(x["N"]), float(x["A* (us)"]) + float(x["PD (us)"])) for x in data]
times2 = [(int(x["N"]), float(x["A* (us)"]) + float(x["Dubins (us)"])) for x in data]
times3 = [(int(x["N"]), float(x["A* (us)"]) + float(x["mpdp (us)"])) for x in data]
lens1 = [(int(x["N"]), float(x["PD (len)"])) for x in data]
lens2 = [(int(x["N"]), float(x["Dubins (len)"])) for x in data]
lens3 = [(int(x["N"]), float(x["mpdp (len)"])) for x in data]

figt1, axt1 = plt.subplots(1, 2, sharey=True)
figl1, axl1 = plt.subplots(1, 2, sharey=True)
figt2, axt2 = plt.subplots(1, 2, sharey=True)
figl2, axl2 = plt.subplots(1, 2, sharey=True)

xt1s = [[x[1] for x in times1 if x[0] == n] for n in Ns]
xt2s = [[x[1] for x in times2 if x[0] == n] for n in Ns]
xt3s = [[x[1] for x in times3 if x[0] == n] for n in Ns]

xl1s = [[x[1] for x in lens1 if x[0] == n] for n in Ns]
xl2s = [[x[1] for x in lens2 if x[0] == n] for n in Ns]
xl3s = [[x[1] for x in lens3 if x[0] == n] for n in Ns]

figt1.suptitle("times PD vs Dubins")
figl1.suptitle("lens PD vs Dubins")
figt2.suptitle("times PD vs mpdp")
figl2.suptitle("lens PD vs mpdp")

axt1[0].boxplot(xt1s, whis=(1, 99))
axt1[1].boxplot(xt2s, whis=(1, 99))

axl1[0].boxplot(xl1s, whis=(1, 99))
axl1[1].boxplot(xl2s, whis=(1, 99))


axt2[0].boxplot(xt1s, whis=(1, 99))
axt2[1].boxplot(xt3s, whis=(1, 99))

axl2[0].boxplot(xl1s, whis=(1, 99))
axl2[1].boxplot(xl3s, whis=(1, 99))

plt.show()

from os import listdir, system
from statistics import mean


def test_cuda_seq():

    bins = [
        "../build/src/dubins_cuda", 
        "../build/src/dubins_seq",
    ]

    test_folder = "../src/test"

    stat_file = open("../test_out/dps.csv", "w+")
    stat_file.write("exec_name,n_points,time,unit,len,discr,ref\n")
    # --- run the tests for our implementation----
    for exec in bins:
        system(f"mkdir ../test_out/{exec.split('/')[-1]}") # it fails if the folder's already there 

        line1 = ""

        for t in listdir(test_folder):
            times = []
            
            # exmple of first line of output
            # time elapsed: 130 us Total length: 45

            out_file = f"../test_out/{exec.split('/')[-1]}/out_{t}"
            for _ in range(10):
                print(f"{exec} < {test_folder}/{t} > {out_file}")
                system(f"{exec} < {test_folder}/{t} > {out_file}")
                with open(out_file, "r") as f:
                    line1 = f.readline()
                    time = line1.split()[2]
                    times.append(float(time))
            
            avg_time = mean(times)
            exec_name = exec.split('/')[-1]
            n_points = t.split('_')[0]
            line1 = line1.split()
            unit = line1[3]
            len = line1[6]

            # format of output line
            # exec_name,n_points,time,unit,len,discr,ref

            stat_file.write(f"{exec_name},{n_points},{avg_time},{unit},{len},0,0\n")
        

    stat_file.close()

def test_dynamic_prog():

    stat_file = open("../test_out/dubins_mpdp.csv", "w+")
    stat_file.write("exec_name,n_points,time,unit,len,discr,ref,type\n")

    line1 = ""

    system(f"mkdir ../test_out/mpdp/")
    for sub in listdir("../tests/mpdp/test"):
        system(f"mkdir ../test_out/mpdp/{sub}")

        for t in listdir(f"../tests/mpdp/test/{sub}"):
            times = []
            out_file = f"../test_out/mpdp/{sub}/out_{t}"

            for i in range(10):
                print(f"../build/tests/mpdp/mpdp_test < ../tests/mpdp/test/{sub}/{t} > {out_file}")
                system(f"../build/tests/mpdp/mpdp_test < ../tests/mpdp/test/{sub}/{t} > {out_file}")
    # example of the first line of output
    # TEST: 30 points with discr: 90 | refin: 8 | time: 471.539 ms | length_mpdp: 59.0928 | length_nostra: 59.0928
                with open(out_file, "r") as f:
                    line1 = f.readline().split('|')
                    time = line1[2].split()[1]
                    unit = line1[2].split()[2]
                    times.append(float(time) * (1000 if unit == "ms" else 1)) # to microseconds
                
            avg_time = mean(times)
            n_points = t.split('_')[0]
            len = line1[3].split()[-1]
            discr = line1[0].split()[-1]
            ref = line1[1].split()[-1]
            
            # format of output line
            # exec_name,n_points,time,unit,len,discr,ref

            stat_file.write(f"mpdp,{n_points},{avg_time},us,{len},{discr},{ref},{sub}\n")
            

    stat_file.close()

def test_ompl():

    stat_file = open("../test_out/dubins_ompl.csv", "w+")
    stat_file.write("exec_name,n_points,time,unit,len,discr,ref,type\n")

    exec = "../build/tests/dubins_ompl/dubins_seq_ompl"

    system(f"mkdir ../test_out/dubins_ompl/")
    for file in listdir("../src/test"):
        times = []

        out_file = f"../test_out/dubins_ompl/out_{file}"

        # 10 times per test and then we take the avg of the different exec times
        for _ in range(10):
            ret_val = -1
            while(ret_val != 0):
                print(f"{exec} < ../src/test/{file} > {out_file}")
                ret_val = system(f"{exec} < ../src/test/{file} > {out_file}")
            
            # exmple of first line of output
            # time elapsed: 130 us Total length: 45
            
            with open(out_file, "r") as f:
                line1 = f.readline().split()
                times.append(float(line1[2]))
                unit = line1[3]
                n_points = int(file.split('_')[0])
                len = line1[-1]

        stat_file.write(f"dubins_ompl,{n_points},{mean(times)},{unit},{len},0,0,through_q1\n")
            
    stat_file.close()

def create_csv():
    with open("../test_out/interpolation_data.csv", "w") as csv:

        with open("../test_out/dubins_mpdp.csv", "r") as stat:
            lines = stat.readlines()
            for l in lines:
                csv.write(l)

        with open("../test_out/dps.csv", "r") as stat:
            lines = stat.readlines()[1:]
            for l in lines:
                l = l[:-1] # remove final "\n"
                csv.write(l+",through_b2\n")

        with open("../test_out/dubins_ompl.csv", "r") as stat:
            lines = stat.readlines()[1:]
            for l in lines:
                csv.write(l)

if __name__ == "__main__":
    system(f"mkdir ../test_out")

    test_cuda_seq()
    test_dynamic_prog()
    test_ompl()

    create_csv()

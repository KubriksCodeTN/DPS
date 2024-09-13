with open("tt", "r") as f:
    lines = f.readlines()[:-1]

    with open("tt.desmos", "w+") as d:
        for l in lines:
            if(l[0] == '\\'): # obstacles
                d.write(l)
            
            elif(l[0].isdigit()): # trajectory
                d.write(f"({l.split()[0]},{l.split()[1]})\n")
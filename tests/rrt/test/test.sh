#!/bin/bash

# this is in bash to prove a point (i.e. python has bad command output handling)
set -e

readonly n_iters=10
readonly e="1" # TODO set tolerance

readonly times=(".1" "1" "5" "10")
readonly header="map,dim,start,end,rrt,timeout,gamma,r,len\n"

if [[ $# != 2 ]] ; then printf "\nError: please specify test input and output file\n\n" ; exit 1; fi

readonly in_file=$1
readonly out_file=$2

# input file: 6 lines
# 1st line: map name and size
# 2nd to 6th line: x0 y0 xf yf gamma_rrtstar r_rrtstar gamma_rrtdubins r_rrtdubins tho thf
readarray -t lines < $in_file
printf $header > $out_file

info=($(echo ${lines[0]} | tr " " "\n"))
map_name=${info[0]}
map_size=${info[1]}

unset lines[0]

for i in "${lines[@]}"
do
    # data
    words=($(echo $i | tr " " "\n"))
    x0=${words[0]}
    y0=${words[1]}
    xf=${words[2]}
    yf=${words[3]}
    gs=${words[4]} # gamma rrtstar
    rs=${words[5]} # r rrtstar
    gd=${words[6]} # gamma rrtdubins
    rd=${words[7]} # r rrtdubins
    th0=${words[8]}
    thf=${words[9]}
    # rrtstar
    for t in "${times[@]}"
    do
        for run in $(seq $n_iters) 
        do 
            printf "%s,%s,(%s %s),(%s %s)," ${map_name} ${map_size} ${x0} ${y0} ${xf} ${yf} >> $out_file
            printf "PD,%s,%s,%s," ${t} ${gs} ${rs} >> $out_file
            ../../../build/tests/rrt/main2d ${e} ${gs} ${rs} ${t} ${x0} ${y0} ${xf} ${yf} < map/${map_name} >> $out_file
            #sleep 10s
        done
    done
done

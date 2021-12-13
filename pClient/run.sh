#!/bin/bash

challenge="4"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        python3 mainRobControl.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 mainRobMapping.py -h "$host" -p "$pos" -r "$robname" -f "$outfile" # assuming -f is the option for the map
        ;;
    3)
        # how to call agent for challenge 3
        python3 mainRobPlanning.py -h "$host" -p "$pos" -r "$robname" -f "$outfile" # assuming -f is the option for the path
        ;;
    4)
        # how to call agent for challenge 4
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f1 "$outfile.map" -f2 "$outfile.path"
        # mv your_mapfile $outfile.map
        # mv your_pathfile $outfile.path
        ;;
        # how to call agent for challenge 4
        #java mainC4 -h "$host" -p "$pos" -r "$robname" 
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;

        # how to call agent for challenge 4
        #./mainC4 -h "$host" -p "$pos" -r "$robname"
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;
esac


#!/usr/bin/env python3

import subprocess

try:
    decision = input(str("All topics? (y/n): "))

    #storage = str("")
    #storage = str("/media/nacho/HDD/terreno2411/")
    storage = str("/media/nacho/HDD/LICEO/")
    #storage = str("/media/nacho/TOSHIBA EXT/terreno2411/")

    if decision.lower() == "y":
        #buffsize = str(input("Size for the buffsize (MB): "))
        buffsize = str(4096)
        name = str(input("Name for the bag: "))

        split = str(input("Split record? (y/n): "))

        if split.lower() == "y":
            split_decision = str(input("Size or duration? (s/d): "))

            if split_decision.lower() == "s":
                size = str(input("Size to split? (e.g. 1024): "))
                command = "rosbag record --all --split --size=" + size + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""
                
            elif split_decision.lower() == "d":
                duration = str(input("Duration to split? (e.g. 30/5m/30m): "))
                command = "rosbag record --all --split --duration=" + duration + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

            else:
                print("Invalid answer. Please answer 's' or 'd'.")

        elif split.lower() == "n":
            command = "rosbag record --all --buffsize=" + buffsize + " --output-name=" + storage + name + ""

        else:
            print("Invalid answer. Please answer 'y' or 'n'.")

    elif decision.lower() == "n":
        topics_list = []

        while True:
            topic = input("Enter a topic ('false' to finish): ")
            
            if topic.lower() == "false":
                break 

            topics_list.append(topic)

        topics = ' '.join(topics_list)

        #buffsize = str(input("Size for the buffsize (MB): "))
        buffsize = str(4096)
        name = str(input("Name for the bag: ")) 
        
        split = str(input("Split record? (y/n): "))

        if split.lower() == "y":
            split_decision = str(input("Size or duration? (s/d): "))

            if split_decision.lower() == "s":
                size = str(input("Size to split? (e.g. 1024): "))
                command = "rosbag record " + topics + " --split --size=" + size + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""
                
            elif split_decision.lower() == "d":
                duration = str(input("Duration to split? (e.g. 30/5m/30m): "))
                command = "rosbag record " + topics + " --split --duration=" + duration + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

            else:
                print("Invalid answer. Please answer 's' or 'd'.")

        elif split.lower() == "n":
            command = "rosbag record " + topics + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

    else:
        print("Invalid answer. Please answer 'y' or 'n'.")

    subprocess.call(command, shell=True)

except KeyboardInterrupt:
    print("\nRosbag record interrupted by the user.\n")
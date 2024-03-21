#!/usr/bin/env python3

import subprocess

try:
    decision = input(str("All topics? (y/n): "))

    #storage = str("")
    storage = str("/media/nacho/HDD/")

    if decision.lower() == "y":
        split = str(input("Split record? (y/n): "))

        if split.lower() == "y":
            duration = str(input("Duration to split? (e.g. 30/5m/30m): "))
            #buffsize = str(input("Size for the buffsize (MB): "))
            buffsize = str(4096)
            name = str(input("Name for the bag: "))

            command = "rosbag record --all --split --duration=" + duration + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

        elif split.lower() == "n":
            #buffsize = str(input("Size for the buffsize (MB): "))
            buffsize = str(4096)
            name = str(input("Name for the bag: "))

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

        split = str(input("Split record? (y/n): "))

        if split.lower() == "y":
            duration = str(input("Duration to split? (e.g. 30/5m/30m): "))
            #buffsize = str(input("Size for the buffsize (MB): "))
            buffsize = str(4096)
            name = str(input("Name for the bag: "))

            command = "rosbag record " + topics + " --split --duration=" + duration + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

        elif split.lower() == "n":
            #buffsize = str(input("Size for the buffsize (MB): "))
            buffsize = str(4096)
            name = str(input("Name for the bag: "))

            command = "rosbag record " + topics + " --buffsize=" + buffsize + " --output-name=" + storage + name + ""

    else:
        print("Invalid answer. Please answer 'y' or 'n'.")

    subprocess.call(command, shell=True)

except KeyboardInterrupt:
    print("\nRosbag record interrupted by the user.\n")
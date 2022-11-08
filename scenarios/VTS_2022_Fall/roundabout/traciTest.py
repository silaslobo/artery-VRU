import os, sys
import math
import sumolib

if 'SUMO_HOME' in os.environ:
     tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
     sys.path.append(tools)
else:
     sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "/usr/share/sumo/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "/home/lobo/Programs/artery/scenarios/roundabout_InTAS/roundabout/AudiKreis.sumocfg", "-d", "300"]

import traci
import traci.constants as tc

traci.start(sumoCmd)

step = 0

while step < 1000:
    traci.simulationStep()

    print (step)
    print ("Number of elements in the Simulation: ", traci.person.getIDCount())
    print ("Elements in the Simulation: ")
    for element in traci.person.getIDList():
        print (element)
    print("---------------------")

    step += 1

traci.close()


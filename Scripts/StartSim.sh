#! /bin/bash 



gnome-terminal --tab -x bash -c "./TurtleSimulator.sh" &
gnome-terminal --tab -x bash -c "./robotTransmiter.sh" &
gnome-terminal --tab -x bash -c "./robotReceiver.sh" 

#gnome-terminal --tab -x bash -c "./TurtleRVIZSim.sh" 








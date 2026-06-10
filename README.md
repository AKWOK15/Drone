# Intercept Drone
### Background
According to CNN, there were 78 school shootings in the US in 2025.  Yet, students have no way of neutralizing the shooter. Instead, all they can do is lock doors and wait for the police, giving the shooter 5-10 minutes to inflict fatalities. I want to change that.

### Goal
Fully autonomous quadcopter that detects shooters and neutralizes them. 


### V1
![Drone](media/drone.jpg)
First flight didn't go so well....
![Drone Crash](https://github.com/user-attachments/assets/d10f185f-e179-4da1-928b-fbda6e41bb4e)

### V2
https://github.com/user-attachments/assets/73565cb0-7355-4eb5-b081-c038481e7385



### Current Hardware Stack
1. Matek H743 Slim V4 Flight Controller
2. Raspberry Pi 4B 8GB
3. Flash Hobby Arthur 2207.5 2500Kv Motor
4. DYS Aria F45A 4-in-1 ESC BLHeli_32
5. Oak-D Pro Wide Camera
6. Holybro Micro M10 GPS


### Setup
1. I'm going to dockerize everything soon, sooooo until then, this isn't super reproducible 

### Run
1. In one window: ```./run_mavrouter.sh``` starts the mavrouter server so that commands from QGroundControl get routed through Raspberry Pi to Flight Controller and telemtry from Flight Controller goes to QGroundControl
2. In another terminal window: ```./run ``` runs all the ros nodes and logging
3. ```./run ``` will prompt you to choose which node to run: takeoff_rtl_node (only takes off and lands) or intercept_node (detect gun and fly into person holding it)

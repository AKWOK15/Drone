# Intercept Drone

### Goal
Build a drone that can autonomously follow another drone.


### V1
First flight didn't go so well....

https://github.com/user-attachments/assets/c05fe6c4-b6f4-47bb-8222-8aa895b77531

### V2
https://github.com/user-attachments/assets/fb5b84ec-bcd4-4d41-8906-009272763356

### V3 Coming Soon

### Current Hardware Stack
1. Matek H743 Slim V4 Flight Controller
2. Raspberry Pi 4B 8GB
3. Flash Hobby Arthur 2207.5 2500Kv Motor
4. DYS Aria F45A 4-in-1 ESC BLHeli_32
5. Oak-D Pro Wide Camera
6. Holybro Micro M10 GPS


### Setup
1. Install Docker
2. Install QGroundControl
3. Edit ```./config/mavlink-router/example.conf``` with the IP of your laptop running QGroundControl and the proper serial port for your flight controller and rename to main.conf
4. Ensure ```compose.yml``` has the correct serial port for the flight controller

### Run
1. ```docker compose up -d```
2. Create two separate terminal tabs and in both run ```docker compose exec ros2 /bin/bash```
3. In one tab, run ```./run_mavrouter.sh```. QGroundControl should detect your flight controller
4. To initiate autonomous behavior, in the other tab, run ```./run ```
5. ```./run ``` will prompt you to choose which node to run: takeoff_rtl_node (only takes off and lands) or intercept_node

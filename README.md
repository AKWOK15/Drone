# Intercept Drone
### Background
According to CNN, there were 78 school shootings in the US in 2025.  Yet, students have no way of neutralizing the shooter. Instead, all they can do is lock doors and wait for the police, giving the shooter 5-10 minutes to inflict fatalities. We want to change that.

### Goal
Fully autonomous quadcopter that detects shooters and neutralizes them. 

### V1
![Drone](media/drone.jpg)
First flight didn't go so well....
![Drone Crash](https://github.com/user-attachments/assets/d10f185f-e179-4da1-928b-fbda6e41bb4e)

### Learnings
1. Balancing functionality with footprint as opposed to just functionality. By using a Pixhawk 6c mini and a Raspberry Pi 4B, we would have had enough compute and built in sensors (excluding stereo camera) to achieve our goal, but our footprint and weight ballooned.  The bigger our drone is, the easier  it will be for the shooter to shoot it out of the air.
2. Learning how to debug hardware, software and electrical. When I first connected the LiPo battery to the ESC, all the motors sounded but only two vibrated. The two that vibrated ended up being the only ones that I could throttle. First, I checked if the ESC received enough power from the LiPo, which it did. After that, I used a multimeter to detect any shorts on the ESC, there weren’t any. Next, I knew that the motors weren’t damaged because we’d only lightly throttled them. So I isolated the issue to the ESC and tried to recalibrate it with QGC. That didn’t work. Plus, when I tried to throttle the two working motors, sometimes it would work, other times it wouldn’t. So my hypothesis is that there was a communication issue on the ESC. Either PMW signals get dropped between the ESC and Pixhawk or the ESC was damaged and couldn’t internally spread the PMW signals to the motors. More debugging required! 
3. Even if hardware data sheets say that it'll meet your specs, they can't be trusted. Ex: our second ESC had a 5V/3A BEC that we wanted to use to power our Pi, but the BEC didn't work. But if I had read online reviews prior to purchasing it, I would have realized that other people encountered the same issue.

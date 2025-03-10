## Goal

- OUTPUT: safety node for the car that avoid collisions when traveling at higher velocities
- implement iTTC (instantaneous) using LaserScan msg in the simulator


## Messages

#### LaserScan (env sensing) msg

- check API for schema and definitions
- `ranges`: array of all range measurements (1080 points)
- LaserScan readings are transmitted over /scan
- calculate iTTC using LaserScan

#### Odometry (vehicle telemetry) msg

- both the simulator node and the actual car will publish Odometry msgs
- payload includes car position, orientation and velocity (among others)

#### AckermannDriveStamped control msg

- used to send driving commands to the simulator and car
- stop the car by sending an AckermannDriveStamped msg with `speed` = 0.0

## TTC

- time to collide with obstacle at current heading and velocity
- iTTC = r / {-r_dot}+
	- r: distance between vehicles
	- r_dot: relative velocity between vehicles
	- -r_dot: 
		- if vehicles are approaching, r_dot is negative
		- if vehicles are separating, r_dot is positive
		- we care about when the vehicles are approaching
		- during this condition, we can negate r_dot to derive a positive TTC measure
	
	- {-r_dot}+:
		- {x}+: max(x, 0)
		- if x = r_dot:
			- if the vehicles are approaching, r_dot < 0 and {x}+ = 0
			- if the vehicles are separating, r_dot > 0 and {x}+ = r_dot

		- if x = -r_dot:
			- if the vehicles are approaching, -r_dot > 0 and {x}+ = -r_dot (desired)
			- if the vehicles are separating, -r_dot <= 0 and {r}+ = 0 (desired)
		
		- in other words, we ignore the case where {-r_dot}+ = 0 since we dont need AEB here


- the instantaneous range r to an obstacle is obtained from the LaserScan.ranges[] array
- LiDAR measures distance between sensor to obstacle
- r_dot is the is the rate of change of range along each beam
	- calculated in 2 ways:

	1. map the ego's current longitudinal velocity onto each beam's angle: v_x * cos(theta)
		- v_x: longitudinal velocity at current timestamp t
		- theta: angle between ego's heading angle and LiDAR beam angle
		- the beam angles can be derived from the LaserScan payload data
		- range rate interpretation: how much the range will change assuming the ego keeps its current
		  heading/speed (velocity) while the obstacle remains stationary

	2. can extrapolate range rate between two frames
		- measure range in scene over two consecutive frames
		- take the difference of the two range measurements and divide by frame interval
		- apply a negation to correctly interpret whether range measurement should be increasing or decreasing
		- if ego >-< obstacle, the range rate of the beam at the front of the vehicle should be negative
		- if ego <-> obstacle, the range rate of the beam at the front of the vehicle should be positive

- calculate an array of TTCs (one for each beam)
- if TTC drops below a threshhold, a collision is imminent

## Implementation

- write a safety node that executes AEB for the car
	- ROS2 node
	- it should subscribe to LaserScan and Odometry messages for telemetry
	- analyze LaserScan and, if necessary, publish AckermannSteering.speed = 0.0 (AEB)
	
- after calculating array of TTCs, decide how to proceed:
	- tune TTC threshholds
	- how can we best remove false positives (braking when collision isnt imminent)
		- deal with NaNs and infs in data

## Testing

- launch sim container iwth kb_telep = True in sim.yaml
- in another bash session, launch teleop_twist_keyboard node from teleop_twist package for keyboard teleop
	- this package should already be installed as a dependency of the simulator
	
- run the simulator, keyboard teleop and safety nodes, use the reset tool and drive into a wall

- topics of note:
	- LaserScan (/scan)
	- Odometry (/ego_racecar/odom): see twist.twsti.linear.x for the longitudal velocity
	- AckermannDriveStamped: /drive

## Deliverables

- implement in C++ or Python 
- use skeleton provided in repo: push the updated skeleton with safety node to github

1. your committed code should start and run in simulation smoothly after pushing
2. make a screencast running the safety node
	- drive the car using teleop along the hallways of Levine
	- show it doesnt brake when traveling straight
		- show that it doesnt generate false positives
		- ie. car doesnt suddenly stop when driving down the hallway
	
	- show that it brakes when driving into a wall (true positive)


  

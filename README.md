# SpringMassRL

Mass-Spring-Damper System with PID Control

This project simulates a mass-spring-damper system controlled by a PID controller, with real-time keyboard controls for adjusting the target position and PID parameters. The simulation is visualized using Pyglet.

## Features
	 •	Simulates a mass-spring-damper system with physics-based motion
	 •	Uses a PID controller to adjust the mass position
	 •	Allows real-time tuning of PID values (kp, ki, kd)
	 •	Interactive keyboard controls to move the target position

Installation
	1.	Clone the repository:

    git clone https://github.com/your-username/mass-spring-damper.git
    cd mass-spring-damper


  2.	Install dependencies:

     pip install numpy torch pyglet


  3.	Run the simulation:

     python mass_spring_damper.py



## How It Works
	   •	The system follows a mass-spring-damper model:
	   •	The mass moves based on spring and damping forces
	   •	A PID controller applies a force to reach the target position
	   •	The simulation runs at 60 FPS
	   •	The target position and PID values can be adjusted in real time

## Controls

     Key	Action
     Up Arrow (↑)	Increase target position
     Down Arrow (↓)	Decrease target position
     W	Increase kp (proportional gain)
     S	Decrease kp (proportional gain)
     A	Increase ki (integral gain)
     D	Decrease ki (integral gain)
     Q	Increase kd (derivative gain)
     E	Decrease kd (derivative gain)

## Notes
	   •	The mass bounces if it reaches the ground
	   •	The target position is visualized as a yellow line
	   •	Adjusting kp, ki, and kd affects how the mass moves toward the target

# Self-Balancing Monowheel

A one-wheeled vehicle that tries to stay upright using a reaction wheel. We built it as a team of four for a dual enrollment physics capstone, with an ESP32, an MPU-6050 IMU, and a PID controller written in Arduino C++. The parts we bought cost about $100. The Arduino code is in this repo.

Demo video: https://www.youtube.com/watch?v=1DhUtAXYsVc

## Where it ended up

The PID loop does what it's supposed to. The IMU measures the roll angle, and the reaction wheel spins the right way whenever the vehicle tilts. The problem is that our reaction wheel couldn't produce enough torque to actually hold the vehicle up, so it doesn't fully balance itself. It can still be driven, and it travels about 20 m in a straight line.

## Where the idea came from

We started from a video of a self-balancing bicycle. When the bike tilts, a reaction wheel spins in that direction, and conservation of angular momentum pushes the bike back upright. Bicycles are big and awkward in tight spaces, so we wanted to see whether the same trick could work on a single wheel.

## How it's built

The vehicle is an outer wheel and an inner frame. The outer wheel is the part that rolls. The inner frame holds the motors, the battery, and the electronics that handle balancing.

The IMU can only tell if the vehicle is tilting if the inner frame stays upright while the wheel turns around it, so we put gears on the inside of the outer wheel. As the wheel spins, the frame stays put. One worry was the frame falling out sideways, so instead of straight gears we used arrow-shaped ones, which still turn freely but resist left and right movement.

## How it balances

The reaction wheel is a pinwheel that spins in the plane perpendicular to the direction of travel. When it speeds up one way, conservation of angular momentum pushes the outer wheel the other way. We pick a set-point, which is the roll angle we want to return to, and the error is the difference between the current roll and that set-point. Spinning the reaction wheel in the direction of the error rotates the main wheel back toward equilibrium.

The motor power comes from a PID controller. The P term is proportional to the current error. The I term is proportional to the sum of past errors, which removes steady-state error. The D term is proportional to how fast the error is changing, and it works a bit like a brake. Each gain has to be tuned for the specific build, so we made the coefficients adjustable while testing, which meant we didn't have to recompile and re-upload the code every time we wanted to try new values.

## Driving it

A PlayStation controller drives the vehicle. The controls are basic: forward and reverse, with no turning. The controller can also switch the PID on and off, but that doesn't really help with balancing, so it's mostly a cosmetic feature.

## Physics

We found the center of mass with the suspension method, which means hanging the object from a string at different points and seeing where the lines cross. For the moment of inertia, we weighed the individual pieces and treated each one as a simple shape, for example modeling the outer wheel as a hollow cylinder. Then we used the parallel axis theorem to get the total.

## My part

There were four of us. I did the coding, including all of the PID logic, with help from a teammate who knew Arduino better on the other parts. I also helped with assembly and did the physics analysis, although a teammate collected the measurements.

## If we rebuilt it

Torque was the limiting factor. A stronger motor, a gear reduction on the reaction wheel, or a heavier reaction wheel with more of its mass near the rim would all be worth trying.

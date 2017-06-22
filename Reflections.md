
## PID Control Project Reflections and Summary
In this Project It was expected that A PID controller would be written and tuned to control the steering angle of a Car in the Term 2 Simulator. A reflection and justification on parameters, their effects, nuances and final parameter selection was expected, which this writeup shall attempt to address

#### Contents
* PID controller as a concept
* PID applied to the Self Driving Car
* Implementation methodology
* Tuning Methodology
* Final Parameter Selection
* Future work and thoughts

### PID controller as a concept
The PID controller is a tried and tested method to achieve a target state by applying corrections based on the following error metrics:
* Absolute Instantaneous Error (P) : Difference between current state and target state
* Instantaneous Change in Error (D) : Instantaneous rate of change of Cross track Error
* Overall Accumulated Error (I) : Accumulated Error

#### Coefficients 
* Kp: Is the Proportional Correction coefficient. It is responsible for the Direction and magnitude of the correction. However due to the momentum principle in control problems, Just using Kp will result in overshoot of the target state.
* Ki: Is the Integral Correction Coefficient. It is responsible for the elimination of the residual steady-state error that occurs with a pure proportional controller while increasing the speed of achieving target state. The momentum principle can plague the I controller as well so it is also prone to overshoot and oscillation
* Kd: Is the Differential Correction Coefficient. It is reponsbile for Dampening of the oscillation by being sensitive to the rate of change of instantaneous error rather than just pure magnitude or direction. Simply put, Kd will delay the achievement of target state but will also dampen and suppress the overshoot and oscillations.

The Mathematics governing the PID Controller is expressed as 
![Equation](./writeup_helpers/Equation.svg "Equation")

where the terms hold their obvious meanings.


### PID applied to the Self Driving Car
PID control has myriad applications in the Autonomous Mobile Robots and Self Driving Cars Domain. This project required control of a Self Driving Car in the Simulator using the PID loop on the steering. Another sentiment expressed in the project writeups was to see how fast could the car go in the simulator while not going off track. Towards this I also added a PID controller on the Target speed. Details of this are explained in the Subsequent Section. A steering gain parameter was determined heuristically to both seed the initial values of the Steering PID and calculate the Maximum Target Speed in relation to current Steering angle.

### Implementation Methodology
As discussed above, the salient features of the implementation presented here are
* A Steering PID Controller that controls the Steering angle
* A Velocity PID Controller that controls the throttle Value
* A Twiddle Loop that monitors error over a fixed number of iterations (determined heuristically) and adjusts Kp, Ki and Kd
* Steering gain parameter that correlates absolute correction with steering input value

### Tuning Methodology
The Following steps were followed during the tuning Iterations

* A Steering gain was tuned heuristically to get the car to drive safely for one lap using the Kp, Ki, Kd values in there sensible order of magnitudes namely 0.1, 0.01, 1.0
* After the absolute values of the Kp, Ki, Kd were calculated that could complete one lap at 10 mph, the speed was gradually increased till the car could not complete one lap.
* Twiddle on both Velocity and Steering controllers was enabled and the iterations for checking error were adjusted till the car could drive safely
* Twiddle was disabled and Speed target was increased till car went off the track.
* Twiddle was re-enabled till car could drive safely.
* Finally Steering Twiddle was disabled to fine tune the Velocity PID independently
* Then Velocity twiddle was disabled to fine tune the Steering PID independently


### Final Parameter Selection
#### Effect of Coefficients
* If Kp is too high, Car will oscillate too much both in the steering as well as the speed domain. There will be constant overshoot which can result in vehicle going offtrack at points where the margin for error is low. If Kp is too low the Car will have a tendency to understeer at turns and will go off track in certain cases.

* If Kd is too high, There is a lot of jitter in the system about the target position, If it is too low , The overshoot problem occuring from Kp, and Ki will not be dampened enough.

* If Ki is too high, it can lead to historical error accumulation overpowering the instantaneous error and the rate of change components (Kp,Kd). If it is too low or absent, the controller will not be able to smoothen the correction curve. 

#### Final Parameter Choice
Since 2 PIDs were used which sort of impact each other, the tuning had to progress both simultaneously as well as in alternate cycles once driving at higher speeds was attained.

The Final Parameter set was as follows 
* Steering Kp, Ki, Kd : (0.097221000, 0.000018, 1.992889)
* Velocity Kp, Ki, Kd : (0.109170, 0.000754, 0.841226)
* Steering gain : 2.86

Both Velocity as well as Steering PID Kp coefficients were kept on the higher side to be able to go as fast as possible. Speed as high as 93mph was attained on straight sections on some of the lap runs. Videos recorded have been attached as reference.


```python

```

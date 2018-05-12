# The Controller Project

The quadroter for the project has the following configuration:

<img src="drone.png" width=400 />

## Command and Control

### Run Command

Based on the above configuration, we have the following system of equations for computing individual thrusts from the total thrust and angular momentums:

<img src="https://latex.codecogs.com/gif.latex?\begin{cases}&space;&&space;c&space;=&space;F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;&plus;&space;F_3&space;\\&space;&&space;\tau_x&space;=&space;F_0&space;-&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_y&space;=&space;F_0&space;&plus;&space;F_1&space;-&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_z&space;=&space;-F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\end{cases}" title="\begin{cases} & c = F_0 + F_1 + F_2 + F_3 \\ & \tau_x = F_0 - F_1 + F_2 - F_3 \\ & \tau_y = F_0 + F_1 - F_2 - F_3 \\ & \tau_z = -F_0 + F_1 + F_2 - F_3 \end{cases}" />

Where:

<img src="https://latex.codecogs.com/gif.latex?\tau_x&space;=&space;\frac{T_x}{l},&space;\&space;\tau_y&space;=&space;\frac{T_y}{l},&space;\&space;\tau_z&space;=&space;\frac{T_z}{\kappa},&space;\&space;l&space;=&space;\sqrt{L}&space;\text{&space;for&space;convenience}" title="\tau_x = \frac{T_x}{l}, \ \tau_y = \frac{T_y}{l}, \ \tau_z = \frac{T_z}{\kappa}, \ l = \sqrt{L} \text{ for convenience}" />

We solve the system for individual forces and get our run command code:

<img src="https://latex.codecogs.com/gif.latex?\begin{cases}&space;&&space;F_0&space;=&space;\frac{1}{4}(\tau_x&space;&plus;&space;\tau_y&space;-&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_1&space;=&space;\frac{1}{4}(-\tau_x&space;&plus;&space;\tau_y&space;&plus;&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_2&space;=&space;\frac{1}{4}(\tau_x&space;-&space;\tau_y&space;&plus;&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_3&space;=&space;\frac{1}{4}(-\tau_x&space;-&space;\tau_y&space;-&space;\tau_z&space;&plus;&space;c)&space;\end{cases}" title="\begin{cases} & F_0 = \frac{1}{4}(\tau_x + \tau_y - \tau_z + c) \\ & F_1 = \frac{1}{4}(-\tau_x + \tau_y + \tau_z + c) \\ & F_2 = \frac{1}{4}(\tau_x - \tau_y + \tau_z + c) \\ & F_3 = \frac{1}{4}(-\tau_x - \tau_y - \tau_z + c) \end{cases}" />

### Body Rate Controller

Three moment commands are based on the formula:

<img src="https://latex.codecogs.com/gif.latex?p_c&space;=&space;I&space;\cdot&space;(p_t&space;-&space;p_a)" title="p_c = I \cdot (p_t - p_a)" />

for each axis.

We tune the `kpPQR` parameter

### Roll Pitch Controller

<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}&space;p_c&space;\\&space;q_c&space;\\&space;\end{pmatrix}&space;=&space;\frac{1}{R_{33}}\begin{pmatrix}&space;R_{21}&space;&&space;-R_{11}&space;\\&space;R_{22}&space;&&space;-R_{12}&space;\end{pmatrix}&space;\times&space;\begin{pmatrix}&space;\dot{b}^x_c&space;\\&space;\dot{b}^y_c&space;\end{pmatrix}" title="\begin{pmatrix} p_c \\ q_c \\ \end{pmatrix} = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c \end{pmatrix}" />

where:

<img src="https://latex.codecogs.com/gif.latex?\dot{b_c}&space;=&space;u&space;/&space;{c}" title="\dot{b_c} = u / c" />, is the tilt angle command.

We need to constrain this by the `(-maxTiltAngle, maxTiltAngle)` range. `CONSTRAIN` macro is handy for these operations. The rotation matrix is already given, so this is easy to implement

### Position Controls (Lateral and Altitude)

Both the altitude and the lateral position controllers are PD controllers, we can write them in a "cascaded" form where [Sergei Lupashin's paper](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) on the ratio of velocity vs position gains applies:

<img src="https://latex.codecogs.com/gif.latex?\ddot{x}&space;=&space;K_v(K_p&space;(u&space;-&space;x)&space;&plus;&space;(\dot{x_t}-&space;\dot{x}))&space;&plus;&space;\ddot{x_{ff}}" title="\ddot{x} = K_v(K_p (u - x) + (\dot{x_t}- \dot{x})) + \ddot{x_{ff}}" />

Both altitude and lateral controllers are based on the same formula (with the appropriate _x_, _y_, _z_ substituted). Altitude controller has an additional integration component (which makes it a PID controller).

In all cases, the velocity command portion needs to be limited. This portion is given by:

<img src="https://latex.codecogs.com/gif.latex?\dot{x_c}&space;=K_p&space;(u&space;-&space;x)&space;&plus;&space;\dot{x_t}" title="\dot{x_c} =K_p (u - x) + \dot{x_t}" />

Tuning of all positional and velocity parameters is done based on the paper above (keeping a ratio of about 3 - 4 between velocities and positions gains)

#### Lateral Position Controller

We constrain the velocity and acceleration command portions of this controller by magnitude, using `magXY()` function of `V3F` class.

#### Altitude Controller

<img src="https://latex.codecogs.com/gif.latex?\ddot{z_c}&space;=&space;K_v(K_p&space;(u&space;-&space;z)&space;&plus;&space;(\dot{z_t}-&space;\dot{z}))&space;&plus;&space;\ddot{z_{ff}}&space;&plus;&space;K_i\int_{0}^{t}e(\tau)d\tau" title="\dot{z_c} = K_v(K_p (u - z) + (\dot{z_t}- \dot{z})) + \ddot{z_{ff}} + \int_{0}^{t}e(\tau)d\tau" />

where

<img src="https://latex.codecogs.com/gif.latex?e(\tau)&space;=&space;u&space;-&space;z" title="e(\tau) = u - z" />, the positional error term.

Different mass copters are able to run with the addition of the integral term
### Yaw Controller

<img src="https://latex.codecogs.com/gif.latex?\dot{\psi_c}&space;=&space;K_{\psi}(\psi_t&space;-&space;\psi),&space;\&space;\text{where&space;}&space;-\pi&space;\leq&space;\psi&space;\leq&space;\pi" title="\dot{\psi} = K_{\psi}(\psi_t - \psi), \ \text{where } -\pi \leq \psi \leq \pi" />

we tune `kpYaw`.

## Scenarios

All scenarios are passing with the `QuadControlParams.txt` currently checked in.

## Scenario 2

![Scenario 2](Scenario2.gif)

## Scenario 3

![Scenario 3](Scenario3.gif)

## Scenario 4

![Scenario 4](Scenario4.gif)

## Scenario 5

![Scenario 5](Scenario5.gif)

## Tests

![tests](tests.png)
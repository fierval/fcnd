# The Controller Project

The quadroter for the project has the following configuration:

<img src="drone.png" width=400 />

## Run Command

Based on the above configuration, we have the following system of equations for computing individual thrusts from the total thrust and angular momentums:

<img src="https://latex.codecogs.com/gif.latex?\begin{cases}&space;&&space;c&space;=&space;F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;&plus;&space;F_3&space;\\&space;&&space;\tau_x&space;=&space;F_0&space;-&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_y&space;=&space;F_0&space;&plus;&space;F_1&space;-&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_z&space;=&space;-F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\end{cases}" title="\begin{cases} & c = F_0 + F_1 + F_2 + F_3 \\ & \tau_x = F_0 - F_1 + F_2 - F_3 \\ & \tau_y = F_0 + F_1 - F_2 - F_3 \\ & \tau_z = -F_0 + F_1 + F_2 - F_3 \end{cases}" />

Where:

<img src="https://latex.codecogs.com/gif.latex?\tau_x&space;=&space;\frac{T_x}{l},&space;\&space;\tau_y&space;=&space;\frac{T_y}{l},&space;\&space;\tau_z&space;=&space;\frac{T_z}{\kappa},&space;\&space;l&space;=&space;\sqrt{L}&space;\text{&space;for&space;convenience}" title="\tau_x = \frac{T_x}{l}, \ \tau_y = \frac{T_y}{l}, \ \tau_z = \frac{T_z}{\kappa}, \ l = \sqrt{L} \text{ for convenience}" />

We solve the system for individual forces and get our run command code:

<img src="https://latex.codecogs.com/gif.latex?\begin{cases}&space;&&space;F_0&space;=&space;\frac{1}{4}(\tau_x&space;&plus;&space;\tau_y&space;-&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_1&space;=&space;\frac{1}{4}(-\tau_x&space;&plus;&space;\tau_y&space;&plus;&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_2&space;=&space;\frac{1}{4}(\tau_x&space;-&space;\tau_y&space;&plus;&space;\tau_z&space;&plus;&space;c)&space;\\&space;&&space;F_3&space;=&space;\frac{1}{4}(-\tau_x&space;-&space;\tau_y&space;-&space;\tau_z&space;&plus;&space;c)&space;\end{cases}" title="\begin{cases} & F_0 = \frac{1}{4}(\tau_x + \tau_y - \tau_z + c) \\ & F_1 = \frac{1}{4}(-\tau_x + \tau_y + \tau_z + c) \\ & F_2 = \frac{1}{4}(\tau_x - \tau_y + \tau_z + c) \\ & F_3 = \frac{1}{4}(-\tau_x - \tau_y - \tau_z + c) \end{cases}" />

## Body Rate Controller

Three moment commands are based on the formula:

<img src="https://latex.codecogs.com/gif.latex?p_c&space;=&space;I&space;\cdot&space;(p_t&space;-&space;p_a)" title="p_c = I \cdot (p_t - p_a)" />

for each axis.

We tune the `kpPQR` parameter

## Roll Pitch Controller

<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}&space;p_c&space;\\&space;q_c&space;\\&space;\end{pmatrix}&space;=&space;\frac{1}{R_{33}}\begin{pmatrix}&space;R_{21}&space;&&space;-R_{11}&space;\\&space;R_{22}&space;&&space;-R_{12}&space;\end{pmatrix}&space;\times&space;\begin{pmatrix}&space;\dot{b}^x_c&space;\\&space;\dot{b}^y_c&space;\end{pmatrix}" title="\begin{pmatrix} p_c \\ q_c \\ \end{pmatrix} = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c \end{pmatrix}" />

where:

<img src="https://latex.codecogs.com/gif.latex?\dot{b_c}&space;=&space;u_1&space;/&space;{c}" title="\dot{b_c} = u_1 / {c}" />, is the tilt angle command.

We need to constrain this by the `(-maxTiltAngle, maxTiltAngle)` range. `CONSTRAIN` macro is handy for these operations. The rotation matrix is already given, so this is easy to implement

## Position Controls (Lateral and Altitude)

Both the altitude and the lateral position controllers are cascading P controllers, where [Sergei Lupashin's paper](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) on the ratio of velocity vs position gains applies.

Both controllers are based on the same formula (with the appropriate _x_, _y_, _z_ substituted). Altitude controller has an additional integration component (which makes it a PID controller).

### Lateral Position Controller


# The Controller Project

The quadroter for the project has the following configuration:

<img src="drone.png" width=400 />

## Run Command

Based on the above configuration, we have the following system of equations for computing individual thrusts from the total thrust and angular momentums:

<img src="https://latex.codecogs.com/gif.latex?\begin{cases}&space;&&space;c&space;=&space;F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;&plus;&space;F_3&space;\\&space;&&space;\tau_x&space;=&space;F_0&space;-&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_y&space;=&space;F_0&space;&plus;&space;F_1&space;-&space;F_2&space;-&space;F_3&space;\\&space;&&space;\tau_z&space;=&space;-F_0&space;&plus;&space;F_1&space;&plus;&space;F_2&space;-&space;F_3&space;\end{cases}" title="\begin{cases} & c = F_0 + F_1 + F_2 + F_3 \\ & \tau_x = F_0 - F_1 + F_2 - F_3 \\ & \tau_y = F_0 + F_1 - F_2 - F_3 \\ & \tau_z = -F_0 + F_1 + F_2 - F_3 \end{cases}" />

Where:

<img src="https://latex.codecogs.com/gif.latex?\tau_x&space;=&space;\frac{T_x}{l},&space;\&space;\tau_y&space;=&space;\frac{T_y}{l},&space;\&space;\tau_z&space;=&space;\frac{T_z}{\kappa},&space;\&space;l&space;=&space;\sqrt{L}&space;\text{&space;for&space;convenience}" title="\tau_x = \frac{T_x}{l}, \ \tau_y = \frac{T_y}{l}, \ \tau_z = \frac{T_z}{\kappa}, \ l = \sqrt{L} \text{ for convenience}" />

We solve the system for the individual forces and get our run command code.


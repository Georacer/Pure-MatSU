# Pure-MatSU
Pure-Matlab Simulation for UAVs

**Currently designed for fixed-wing UAVs only**

This simulator is written in pure Matlab, i.e. does not make use of Simulink.

It is a free-running simulation, meaning that it will execute as fast as it can, according to the capabilities of your system. It is not locked to a real-time clock.

It provides for fixed time-step with Forward-Euler integration, or the use of Matlab's ode45 and ode15s solvers.

Please use the version releases to ensure that your downloaded version is as bug-free as possible and has updated documentation.

## User Instructions

Follow these instructions to use this simulator:

 1. **Load path**: Run the `load_path` script to setup the execution path
 - **Edit simulation options**: Edit the `simulation_options` function and adjust the simulation parameters to your needs. 

   Ample documentation is provided inline. This is also where you can set the initialization options.

   You can enable live plots and graphic output, but keep in mind that this places a significant burden to the system and slows the simulation down.

   Finally, this is where you can select the simulated vehicle, by setting the `sim_options.vehicle` parameter to one of the contents of the `vehicles` folder, but only one model is currently available.

 - (optional) **Edit vehicle parameters**: To change the model parameters of a vehicle, open its corresponding function from the `vehicles` folder. Currenlty only the `skywalker_2013` model is provided. Change any parameter value to your liking.

   Specifically the `skywalker_2013` is modeled like the corresponding model from [last_letter](http://georacer.github.io/last_letter/parameters/aircraftParams.html) and more modeling information can be found in the [uav-modeling](https://github.com/Georacer/uav-modeling/blob/master/preamble.pdf) documenation.

 - **Run simulation**: Run the `simulation` function (`sim_output = simulation(simulation_options())`) to generate the simulation output in the `sim_output` workspace struct variable. The vehicle states trajectory is saved in the `array_states` member. The input history is saved in the `array_inputs` member.

 - **Plot resulting trajectory**: Run `plot_trajectory(sim_output, sim_options)` to plot the resulting trajectory. Read the function documentation for more graphics options.

## Writing a new controller

You can select the controller which will be applied on the simulated vehicle in the `simulation_options` function.

A controller is a function which is given the `VehicleState` instance and generates a 4x1 vector with the control inputs to the vehicle. The contents of this vector are:

 1. Aileron input - range: [-1,1]
 1. Elevator input - range: [-1,1]
 1. Throttle input - range: [0,1]
 1. Rudder input - range: [-1,1]

and obviously correspond to vehicles with the airplane form.

By default, the option is set to 0, corresponding to the `StaticOutput` controller. This controller generates a constant, zero output for all four inputs.

To write your own controller, first create a new controller function in the `controllers` folder. Use the `StaticOutput` controller as a template.
Your controller class must have at least a constructor to initialize itself and a method `ctrl_output = calc_output(sim_vehicle, t)` with the same signature as the one in `StaticOutput`.

Then, edit the `Controller` class to assign an index to your controller and add an option for it in the class constructor. This is also where you will setup any initialization options for your controller.
Finally, edit the `simulation_options` function and set your new controller index in the `sim_options.controller.type` field, along with any other required parameters.
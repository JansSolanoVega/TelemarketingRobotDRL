# Telemarketing_Robot for autonomous navigation in dynamic scenarios

## Dependencies
* ROS Noetic
* Ubuntu 20
* Navigation package installed

## Usage
1.**Run GAZEBO Simulation:**
```sh
$ roslaunch telemarketing_gazebo telemarketing_store_automatizado.launch
```

2.**Run Traditional Local Planners (DWA, TEB):**
```sh
$ roslaunch telemarketing_navigation automatizado_telemarketing_navigation.launch
```

    **Variable Parameters:**
    - `algoritmoNavegacion`: Choose traditional local planners (dwa, teb).
    - `numero_intentos`: Set the number of navigations per simulation.
    - `numero_obstaculos`: Set the number of dynamic agents (4, 8).
    - `velocidad`: Specify the speed of dynamic agents (0.2, 0.3, 0.4).
    - `tipo_movimiento`: Define the type of social behavior for dynamic agents(0_Aleatorio, 1_Cruce, 2_MismaDireccion, 3_MundoObstaculosEstaticos, 4_MundoVacio).

3.**Run Mapless Local Planner (DRL):**
```sh
$ roslaunch telemarketing_drl automatizado_drl_navigation_test.launch
```

    **Variable Parameters:**
    - `algoritmoNavegacion`: Choose traditional local planners (SAC_Mapless, PPO_Mapless, DQN_Mapless).
    - `numero_intentos`: Set the number of navigations per simulation.
    - `numero_obstaculos`: Set the number of dynamic agents (4, 8).
    - `velocidad`: Specify the speed of dynamic agents (0.2, 0.3, 0.4).
    - `tipo_movimiento`: Define the type of social behavior for dynamic agents(0_Aleatorio, 1_Cruce, 2_MismaDireccion, 3_MundoObstaculosEstaticos, 4_MundoVacio).

4.**Run DRL Training on 3D simulator:**
```sh
$ roslaunch telemarketing_openai start_training.launch
```

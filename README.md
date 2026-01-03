# Optimization of PI Controller Parameters for DC Motor Speed Control Using PSO and GA

This project focuses on the design and implementation of an optimized Proportional-Integral (PI) controller for Direct Current (DC) motor speed regulation. 
By leveraging metaheuristic optimization algorithms—specifically **Particle Swarm Optimization (PSO)** and **Genetic Algorithm (GA)**—the project aims to overcome the limitations 
of conventional tuning methods to achieve superior dynamic performance.

## 1. Problem Statement
Precise speed control is essential in applications like robotics and industrial automation. While PI controllers are widely used for their simplicity and reliability, 
their performance depends heavily on the tuning of parameters ($K_p$ and $K_i$). Traditional methods, such as Ziegler-Nichols, often fail to provide satisfactory results 
in systems affected by nonlinearities or external disturbances. This project utilizes PSO and GA to automatically determine optimal parameters to minimize errors and improve response.



## 2. Objectives
* **Modeling:** Develop a mathematical model and transfer function for the DC motor relating input voltage to angular speed.
* **Controller Design:** Implement a PI controller and analyze performance using conventional tuning methods.
* **Optimization:** Apply PSO and GA to find optimal values of $K_p$ and $K_i$ to minimize error-based performance indices (IAE, ISE, ITAE).
* **Simulation:** Develop a MATLAB/Simulink model to compare classical and optimized control performances.
* **Experimental Validation:** Implement the optimized PI controller on a real DC motor using Arduino hardware.
* **Performance Comparison:** Evaluate improvement in rise time, settling time, overshoot, and steady-state error.

## 3. System Architecture

### Simulation Phase (MATLAB/Simulink)
1. Obtain the motor transfer function using differential equations.
2. Implement PSO and GA algorithms to optimize $K_p$ and $K_i$ on a selected objective function (e.g., minimizing ITAE).
3. Compare time-domain responses for Conventional PI, PSO-optimized PI, and GA-optimized PI.
4. Analyze robustness and disturbance rejection capabilities.

### Experimental Phase (ESP32 Platform)
1. Implement the optimized PI control in real-time.
2. Measure motor speed using an encoder or Hall-effect sensor.
3. Interface the motor with an L298N driver module and collect data via serial communication.
4. Validate simulation results by comparing them with experimental outcomes under different operating conditions.



## 4. Materials and Tools

| Category | Component | Description / Function |
| :--- | :--- | :--- |
| **Control Hardware** | Arduino Uno or Mega | Microcontroller for implementing the controller |
| **Motor System** | DC Motor (12V or 24V) | Main actuator for speed control |
| **Motor Driver** | L298N/L293D Module | Interface circuit to control motor direction and speed |
| **Speed Sensor** | Rotary Encoder / Hall Sensor | For real-time measurement of motor speed |
| **Power Supply** | 12V DC Power Adapter | To power the motor and Arduino system |
| **Simulation Software** | MATLAB/Simulink | Modeling, control design, and optimization environment |
| **Prototyping** | Breadboard & Wires | For prototyping and circuit assembly |

## 5. Expected Outcomes
* Optimized PI controller parameters obtained through PSO and GA.
* Enhanced speed tracking performance with reduced overshoot and minimal steady-state error.
* Demonstrated improvement of metaheuristic optimization techniques over classical tuning.
* Successful hardware validation confirming the accuracy of the simulation results.

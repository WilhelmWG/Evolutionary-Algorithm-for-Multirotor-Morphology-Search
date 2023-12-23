# About The Project
This project is an implementation of general multirotor dynamics and an evolutionary algorithm to search for and generate novel Micro Aerial Vehicles (MAVs). The designs are evolved to perform for a set of trajectories designed to mimic traversing an underground environment similar to what is expected in the DARPA subterranean challenge. Their fitness is evaluated based on their ability to follow these trajectories while maintaining designated full orientation. This involves minimizing the errors between the true trajectory and the desired one while simultaneously minimizing the amount of battery power expended.

# File contents
- GA.py contains the majority of parameters and function required to make a pygad.GA instance.
- MultiRotorDynamics.py defines the MultiRotor class, its dynamics and functions for simulating it following a trajectory. In addition, there is some code defining an IMU orientation estimator and a depth camera object. Neither of these are utilised in the simulation.
- MotorRotorAnalysis.py defines dictionaries that contain all necessary information on motor-rotor combinations and batteries used in this project.
- Trajs.py defines all trajectories the MultiRotor tracks when evaluating its fitness.
- old_main.py runs a predefined MultiRotor for a single trajectory, primarily used for testing
plotting.py contains functions for plotting 2D and 3D representations of trajectories and a function for making a 3D model of a MultiRotor object.
- utils.py contains some helpful utility functions for linear algebra
- tests.py defines tests for the utility functions
- load_and_run.py loads a MultiRotor from a .txt file and simulates it tracking one trajectory and plots its performance and 3D model, used for evaluating results from a genetic algorithm run.
- main.py runs a genetic algorithm, writes the best solution and GA_instance to files and plots the fitness, genes and 3D model of the MultiRotor.
- The /data folder contains the best solution for the evolutionary runs described in the project report. They can be tested using load_and_run.py. 

# Getting Started

## Prerequisites

To run the files in this repository a python environment for version 3.11.5 can be created with Anaconda using the EvolveEnv.yml file using the following command.

```sh
conda env create -f EvolveEnv.yml
```

## Usage
One key example of using this repository is as follows. 
1. Modify the parameters of GA.py to what you want to test for. 
2. Modify Trajs.py if new or modified trajectories are desired.
3. Run main.py
4. Modify load_and_run.py to point to the file path created by main.py.
5. Run load_and_run.py to see the performance of the best solution of the genetic algorithm.

Running load_and_run.py can also be used to view the 3D models of each solution in the /data folder for a better view than in the project report.


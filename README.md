# Selection of parameters for the automatic transmission model

## A project written in Python that implements a genetic algorithm for selecting parameters for an automatic transmission model.

This project includes a simplified physical model of an automatic transmission, which takes into account such parameters as pressure in solenoids and friction in fractional disks. A genetic algorithm is used to select the model parameters. The time it takes for the model to reach 100 km/h is used as the objective function. The project includes two parts:

* Simplified physical model of an conventional automatic transmission (torque converter) written in Python
* Genetic algorithm for selection of parameters

## How to install this Python project

1. clone this project
2. make sure you have Python 3.10 or higher
3. if you don't have Python 3.10 or higher, install it from the official website (https://www.python.org/downloads/)
4. go to the project repository on your device, open a terminal and enter the command 'pip install -r requirements.txt' to install the required libraries
5. To run the project, run the file gen_algoritm.py
6. the found solution (selected parameters) will be written to the file decision.csv
7. the operation of the model with the found parameters is presented by two graphs, a graph of engine speed, input shaft and output shaft, and a speed graph, both graphs will appear during the project operation, when the algorithm calculates the results (the graphs are written to the transmission_simulation.png file)
8. The parameters of individuals in each generation are written to the file results.csv

# CCPP
Multi-UAV Path Planning for Sweep Coverage with Connectivity Constraints
The project contains two mixed-integer linear formulation to plan the path of a multi-UAV system for atleast 1-degree periodic coverage while ensuring connectivity to the base station throughout the flight.
The first algorithm enables the UAVs to maximize area coverage within a given number of time-steps, and the second algorithm enables
the UAVs to achieve a desired coverage level while minimizing the required number of movements. The focus of this work is on ensuring continuous multi-hop connectivity of the UAVs with the base station using inter-UAV communication links.
The formulations are coded and implemented in MATLAB (R2020b). To run the codes successfully one need to install IBM ILOG CPLEX version 12.10.0 optimization software. Provide path of milpcplex files in matlab and then run the code. 
The path address will look like: C:\Program Files\IBM\ILOG\CPLEX_Studio1210\cplex\matlab\x64_win64

# IsaacSim Playground

this is a repo in which I am learning to write script in order to run simulations with Isaac Sim.

# Objective
The objective is to learn how IsaacSim, PhysX and omniverse Api is working. 

# Requirements
IsaacSim 6.0.1: https://docs.isaacsim.omniverse.nvidia.com/latest/index.html

# Learning topics
In this project, I want to learn the following topics:
- In IsaacSim, using a script in headless mode, I want to load a predefined scene. Then learn how to load a USD object that I have on my server, it is a complex solid. Finally I want to define a replicator to take some pictures about the scene just to check the results.
- In IsaacSim, I want to load several instance of the same USD object but represented by all the possible collider:
    - MeshSimplification
    - ConvexHull
    - ConvexDecomposition
    - SDF
  Once they are loaded, side by side, The aim is to analyze how they are bouncing on the floor. That means I need to learn how to add physics to these objects and how to spawn the side by side at a certain height. I should then add a replicator, synced with the physical steps and taking pictures of the falling objects from the side.
- In IsaacSim, I want to learn how to define a conveyor belt on which I will spawn a objet (again from the same USD). I also want to take pictures using a replicator.

# Warnings
This codebase is running on a remote server that does not have any screen so all the code needs to run in a headless mode.
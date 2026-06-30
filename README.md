# IsaacSim Playground

this is a repo in which I am learning to write script in order to run simulations with Isaac Sim.

# Objective
The objective is to learn how IsaacSim, PhysX and omniverse Api is working. 

# Requirements
IsaacSim 6.0.1: https://docs.isaacsim.omniverse.nvidia.com/latest/index.html

# Wanted simulation script
In this project, I want to learn the following topics:
- freefall.py: Create a simple simulation where I am generating synthetic observations of a complex solid (potato) fallin on the ground of the basesample scene of IsaacSim. An exemple is defined here: '/home/adrien/Isaacsim_playground/samples/freefall.py'
- colliders.py: Create a simular simulation but where potato prims instantiated with different colliders are falling on the ground side by side. The camera should this time look at the falling object from the side.
- object_throw.py: A simulation where the same potato is thrown from the side with an initial velocity and initial angular velocity and rebounding on a vertical wall then on the ground. I should be able to modulate the initial velocity. A camera should see the whole trajectory.
- conveyor.py: A simulation where the same potato is rolling then falling from the conveyor belt (using IsaacSim extention: https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/ext_isaacsim_asset_gen_conveyor.html)

# Compliant scripts
The four scripts above are implemented in `./simulation_scripts/` and meet all Mandatory rules (headless, PathTracing, parametrized, PNG + MP4, shared logic in `common/`, stereo cameras):
- `./simulation_scripts/freefall.py` — compliant freefall (the original `samples/freefall.py` is a legacy non-compliant version: `headless: False`, no `common/` reuse — kept as a learning artifact, do not copy as a template).
- `./simulation_scripts/colliders.py` — three potatoes side-by-side, `convexHull` / `convexDecomposition` / `sdf`, side-view stereo rig.
- `./simulation_scripts/object_throw.py` — tunable launch velocity, rebounds off a vertical wall then the ground, wide stereo rig.
- `./simulation_scripts/conveyor.py` — uses the `isaacsim.asset.gen.conveyor` extension (`create_conveyor_belt`); potato transported then falls off the end.

Each writes `_output/<scenario>/{Left,Right}/rgb/rgb_*.png` + `left.mp4`, `right.mp4`, `stereo_sbs.mp4`.
Run with: `conda deactivate && . .env && $ISAAC_SIM/python.sh ./simulation_scripts/<script>.py`

# Mandatory
Here are requirements that are to be verified in all the written scripts:
- The simulation run headless
- The used rendering mode should be PathTracing. 
- The scripts should be easily parametrized through changes in constant or config files
- The script should save png informations and create mp4 of the simulations
- All the common codes should be written in central files in '/home/adrien/Isaacsim_playground/common'
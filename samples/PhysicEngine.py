
# Start by launching the simulation App
from isaacsim import SimulationApp
SimulationApp({"headless": True})

# Import Isaac dependencies
from isaacsim.core.simulation_manager import SimulationManager

engines = SimulationManager.get_available_physics_engines(verbose=True)

print("Available physics engines:")
for engine in engines:
    print(f" - {engine}")

# Ending the simulation App
SimulationApp.close()
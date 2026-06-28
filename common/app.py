"""SimulationApp lifecycle."""
from isaacsim import SimulationApp


def launch():
    return SimulationApp({"headless": True})


def shutdown(app):
    app.close()

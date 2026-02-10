import traci
import os
import xml.etree.ElementTree as ET
import ALL_FUNCTIONS as Func

# ======================================================
# SUMO Configuration
# ======================================================
SUMO_CFG = "run.sumocfg"
sumo_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")

seed = 3
sumo_cmd = [sumo_binary, "-c", SUMO_CFG, "--start", "--seed", str(seed)]

# ======================================================
# Read simulation end time from configuration file
# ======================================================
tree = ET.parse(SUMO_CFG)
root = tree.getroot()
end_time = 1600
for time_elem in root.findall('time'):
    end_value = time_elem.get('end')
    if end_value:
        end_time = float(end_value)

# ======================================================
# Start SUMO
# ======================================================
traci.start(sumo_cmd)

step = 0
target_vehicle = None
stopped = False

# ======================================================
# Main simulation loop
# ======================================================
while step < end_time:
    traci.simulationStep()

    veh_ids = traci.vehicle.getIDList()

    # Only call the function, do not implement stopping logic
    target_vehicle, stopped = Func.handle_first_vehicle_braking(step, veh_ids, target_vehicle, stopped)

    step += 1

# ======================================================
# Close simulation
# ======================================================
traci.close()
print("Simulation finished")

# ======================================================
# Rename trajectory.xml -> b_1_base_trajectory.xml
# ======================================================
old_name = "trajectory.xml"
new_name = "b_1_base_trajectory.xml"

if os.path.exists(old_name):
    os.rename(old_name, new_name)
    print(f"File renamed to: {new_name}")
else:
    print("trajectory.xml file not found")
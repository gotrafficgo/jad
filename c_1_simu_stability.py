import traci
import os
import xml.etree.ElementTree as ET

# -------------------------------
# SUMO Configuration
# -------------------------------
SUMO_CFG = "run.sumocfg"
sumo_binary = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")

seed = 3
sumo_cmd_base = [sumo_binary, "-c", SUMO_CFG, "--start", "--seed", str(seed)]

# -------------------------------
# Read simulation end time from config
# -------------------------------
tree = ET.parse(SUMO_CFG)
root = tree.getroot()
end_time = 1600
for time_elem in root.findall('time'):
    end_value = time_elem.get('end')
    if end_value:
        end_time = float(end_value)

# -------------------------------
# Parameter definitions
# -------------------------------
SLOWDOWN_DISTANCE = 4000     # Start slowing down 4000m before the end of the road
SLOWDOWN_DURATION = 5        # Decelerate to target speed within 5 seconds (adjustable)

# -------------------------------
# List of target speeds to simulate
# -------------------------------
TARGET_SPEED_LIST = [30, 40, 50, 60]  # km/h

# -------------------------------
# Batch simulation
# -------------------------------
for TARGET_SPEED_KMH in TARGET_SPEED_LIST:
    TARGET_SPEED = TARGET_SPEED_KMH / 3.6  # Convert km/h → m/s
    print(f"\n========== Simulation TARGET_SPEED = {TARGET_SPEED_KMH} km/h ==========")
    
    # Start SUMO
    traci.start(sumo_cmd_base)
    
    step = 0
    TARGET_VEHICLE = None
    slowdown_triggered = False
    speed_fixed = False

    while step < end_time:
        traci.simulationStep()

        # Lock the first vehicle that appears
        if TARGET_VEHICLE is None:
            veh_ids = traci.vehicle.getIDList()
            if veh_ids:
                TARGET_VEHICLE = veh_ids[0]
                print(f"[Step {step}] Controlled vehicle: {TARGET_VEHICLE}")

        if TARGET_VEHICLE and TARGET_VEHICLE in traci.vehicle.getIDList():
            edge_id = traci.vehicle.getRoadID(TARGET_VEHICLE)
            if edge_id != "":
                lane_id = traci.vehicle.getLaneID(TARGET_VEHICLE)
                lane_length = traci.lane.getLength(lane_id)
                veh_pos = traci.vehicle.getLanePosition(TARGET_VEHICLE)

                distance_to_end = lane_length - veh_pos  # Distance to the end of the lane

                # ① Reach SLOWDOWN_DISTANCE → start natural slowdown
                if distance_to_end <= SLOWDOWN_DISTANCE and not slowdown_triggered:
                    current_speed = traci.vehicle.getSpeed(TARGET_VEHICLE)
                    print(f"[Step {step}] Start slowing down: from {3.6*current_speed:.2f} km/h to {TARGET_SPEED_KMH} km/h, distance to end {distance_to_end:.2f} m")
                    traci.vehicle.slowDown(
                        vehID=TARGET_VEHICLE,
                        speed=TARGET_SPEED,
                        duration=SLOWDOWN_DURATION
                    )
                    slowdown_triggered = True

                # ② After reaching target speed → force maintain
                if slowdown_triggered and not speed_fixed:
                    if abs(traci.vehicle.getSpeed(TARGET_VEHICLE) - TARGET_SPEED) < 0.5:
                        print(f"[Step {step}] Reached {TARGET_SPEED_KMH} km/h, start maintaining speed")
                        traci.vehicle.setSpeed(TARGET_VEHICLE, TARGET_SPEED)
                        speed_fixed = True

                # ③ Speed fixed → maintain each step
                if speed_fixed:
                    traci.vehicle.setSpeed(TARGET_VEHICLE, TARGET_SPEED)

        step += 1

    # -------------------------------
    # Close simulation
    # -------------------------------
    traci.close()
    print(f"Simulation ended: TARGET_SPEED = {TARGET_SPEED_KMH} km/h")

    # -------------------------------
    # Rename trajectory file
    # -------------------------------
    old_name = "trajectory.xml"
    new_name = f"c_1_stability_trajectory_{TARGET_SPEED_KMH}.xml"

    if os.path.exists(old_name):
        os.rename(old_name, new_name)
        print(f"File renamed to: {new_name}")
    else:
        print("trajectory.xml file not found")
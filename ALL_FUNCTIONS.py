import xml.etree.ElementTree as ET
import traci
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection





# ======================================================
# jam-absorption strategy
# ======================================================
def plan_jad(jad_speed, wave_speed,
             A, E, F, vt, vw):

    At = A[0]
    Ax = A[1]
    Et = E[0]
    Ex = E[1]
    Ft = F[0]
    Fx = F[1]   

    """
    [D] 
    Dt &= \frac{vt At - WAVE_SPEED Ft + (Fx - Ax)}{vt - WAVE_SPEED},
    Dx &= x_A + vt (Dt - At)
    """
    Dt = (vt * At - wave_speed * Ft + (Fx - Ax)) / (vt - wave_speed)
    Dx = Ax + vt * (Dt - At)
    D = (Dt, Dx)

    """
    [C] 
    Ct &= \frac{vw Dt - WAVE_SPEED Et + (Ex - Dx)}{vw - WAVE_SPEED},
    Cx &= Dx + vw (Ct - Dt)
    """
    Ct = (vw * Dt - wave_speed * Et + (Ex - Dx)) / (vw - wave_speed)
    Cx = Dx + vw * (Ct - Dt)
    C = (Ct, Cx)  

    """
    [B] 
    Bt &= \frac{vt At - JAD_SPEED Ct + (Cx - Ax)}{vt - JAD_SPEED}, 
    Bx &= Ax + vt (Bt - At)
    """    
    Bt = (vt * At - jad_speed * Ct + (Cx - Ax)) / (vt - jad_speed)
    Bx = Ax + vt * (Bt - At)
    B = (Bt, Bx)

    return B, C, D



# ======================================================
# Compute vertices of the feasible region of A
# ======================================================
def get_feasible_region_of_A(E, F, v_t, v_w, v_star, w, x_u):
    """
    Return three vertices of the feasible triangle:
    P1 = (t_E, x_max)
    P2 = (t_E, x_u)
    P3 = (t_max, x_u)

    E, F: tuples (t, x)
    v_t, v_w, v_star, w: scalars
    x_u: lower bound on x
    """

    t_E, x_E = E
    t_F, _ = F  # x_F is not needed because x_E = x_F

    # Directly compute gamma
    numerator = (
        w * w * v_t * t_E
        - w * w * v_t * t_F
        - w * w * v_w * t_E
        + w * w * v_w * t_F
        - w * v_star * v_t * t_E
        + w * v_star * v_t * t_F
        + w * v_star * v_w * t_E
        - w * v_star * v_w * t_F
    )

    denominator = (
        v_t * w
        - v_t * v_w
        - v_star * w
        + v_star * v_w
    )

    gamma = x_E - w * t_E + numerator / denominator

    # Upper vertex on B-line
    x_max = w * t_E + gamma

    # Lower bound
    x_min = x_u

    # Intersection of B-line and x = x_u
    t_max = (x_min - gamma) / w

    P1 = (t_E, x_max)
    P2 = (t_E, x_min)
    P3 = (t_max, x_min)

    return P1, P2, P3



# ======================================================
# Get simulation end time
# ======================================================
def get_simulation_end_time(cfg_path):
    """
    Read the simulation end time from a SUMO cfg file.
    If not specified, use the default value 3600.
    """
    tree = ET.parse(cfg_path)
    root = tree.getroot()

    end_time = 3600  # default value

    time_elem = root.find('time')
    if time_elem is not None:
        end_elem = time_elem.find('end')
        if end_elem is not None:
            end_time = float(end_elem.get('value'))

    return end_time



# ======================================================
# Control the first vehicle to perform a natural stop
# ======================================================
def handle_first_vehicle_braking(step, veh_ids, target_vehicle, stopped):
    """
    Control the first vehicle to perform one "natural stop" at a specified time
    Used to create downstream disturbance
    """

    STOP_START_TIME = 150          # Simulation time (step) when stopping begins
    STOP_DURATION = 30             # Stop duration (seconds)
    STOP_DISTANCE_TO_END = 500     # Distance from the stop point to the end of the road (m)

    # If no target vehicle has been selected yet, choose the first vehicle in the current traffic flow
    if target_vehicle is None and veh_ids:
        target_vehicle = veh_ids[0]

    # If the target vehicle is still in the network
    if target_vehicle and target_vehicle in veh_ids:

        # Reached the set time and has not stopped yet
        if step == STOP_START_TIME and not stopped:
            edge_id = traci.vehicle.getRoadID(target_vehicle)
            lane_id = f"{edge_id}_0"
            edge_length = traci.lane.getLength(lane_id)

            # Stop position: STOP_DISTANCE_TO_END from the end of the road
            stop_pos = edge_length - STOP_DISTANCE_TO_END

            # Use setStop to let the vehicle stop naturally
            traci.vehicle.setStop(
                vehID=target_vehicle,
                edgeID=edge_id,
                pos=stop_pos,
                duration=STOP_DURATION
            )
            stopped = True

    return target_vehicle, stopped



# ======================================================
# Check for vehicle insertion opportunities
# ======================================================
def check_insertion_opportunity_at_ramp(ramp, threshold_insert, 
                                        step, veh_ids, last_position):
    """
    Check if any vehicle crosses the ramp in this step
    and meets the insertion condition (headway > THRESHOLD)

    Returns:
    - last_position (updated)
    - info: None or dict containing all information needed for insertion
    """

    for focal_id in veh_ids:

        pos = traci.vehicle.getPosition(focal_id)[0]

        if focal_id in last_position:
            prev_pos = last_position[focal_id]

            # Check if the vehicle crosses the ramp
            if prev_pos < ramp <= pos:

                focal_x = pos
                lane_id = traci.vehicle.getLaneID(focal_id)
                focal_v = traci.vehicle.getSpeed(focal_id)

                # Skip if the vehicle is almost stationary
                if focal_v < 0.1:
                    last_position[focal_id] = pos
                    continue

                # -----------------------------
                # Find the leader in the same lane
                # -----------------------------
                leader_x = None
                leader_id = None

                for other in veh_ids:
                    if other == focal_id:
                        continue
                    if traci.vehicle.getLaneID(other) != lane_id:
                        continue

                    pos_other = traci.vehicle.getPosition(other)[0]
                    if pos_other > focal_x:
                        if leader_x is None or pos_other < leader_x:
                            leader_x = pos_other
                            leader_id = other

                if leader_id is None:
                    last_position[focal_id] = pos
                    continue

                leader_v = traci.vehicle.getSpeed(leader_id)
                dx = leader_x - focal_x
                headway = dx / focal_v

                # -----------------------------
                # Meets insertion condition
                # -----------------------------
                if headway > threshold_insert:
                    insertion_info = {
                        "step": step,
                        "lane_id": lane_id,
                        "headway": headway,
                        "focal_id": focal_id,
                        "focal_x": focal_x,
                        "focal_v": focal_v,
                        "leader_id": leader_id,                                                
                        "leader_x": leader_x,                        
                        "leader_v": leader_v
                    }

                    print(
                        f"[Insertion opportunities] "
                        f"({step}, {ramp})"
                    )

                    last_position[focal_id] = pos
                    return last_position, insertion_info

        last_position[focal_id] = pos

    return last_position, None



# ======================================================
# Insert a vehicle
# ======================================================
def insert_vehicle_at_ramp(jad_plan, 
                           step, insertion_info, inserted_count):
    """
    Insert a vehicle according to insertion_info
    """

    x_new = (insertion_info["focal_x"] + insertion_info["leader_x"]) / 2
    v_new = (insertion_info["focal_v"] + insertion_info["leader_v"]) / 2
    lane_id = insertion_info["lane_id"]

    new_id = f"inserted_{inserted_count}"

    print(
        f"[Insertion opportunity] "
        f"step={step}, "
        f"focal={insertion_info['focal_id']}, "
        f"leader={insertion_info['leader_id']}, "
        f"headway={insertion_info['headway']:.2f}s"
    )

    traci.vehicle.add(
        vehID=new_id,
        routeID="route0",
        typeID="car",
        depart=str(step)
    )

    traci.vehicle.setSpeedMode(new_id, 0)
    traci.vehicle.moveTo(new_id, lane_id, x_new)
    traci.vehicle.setSpeed(new_id, v_new)

    traci.vehicle.setAccel(new_id, 3.0)
    traci.vehicle.setDecel(new_id, 5.0)

    jad_plan[new_id] = {
        "phase": 1,
        "phase_start": step,
        "init_speed": v_new
    }

    print(
        f"   -> New vehicle {new_id} inserted at "
        f"x={x_new:.2f}, v={v_new:.2f}\n"
    )

    return inserted_count + 1



# ======================================================
# Control the three-phase behavior of inserted vehicles
# ======================================================
def control_inserted_vehicles(jad_plan, jad_speed, 
                              step, Duration_AB, Duration_BC):
    """
    Execute three-phase control for inserted vehicles:
    Phase 1: Maintain insertion speed
    Phase 2: Force deceleration
    Phase 3: Resume SUMO automatic car-following
    """

    JAD_SAFE_GAP = 8.0  # Minimum safe gap (m)

    to_remove = []

    for vid, info in jad_plan.items():

        # Vehicle has left the network
        if vid not in traci.vehicle.getIDList():
            to_remove.append(vid)
            continue

        phase = info["phase"]
        start = info["phase_start"]

        # ---------------------------------
        # Front vehicle safety check
        # ---------------------------------
        lane_id = traci.vehicle.getLaneID(vid)
        pos = traci.vehicle.getPosition(vid)[0]

        front_pos = None
        for other in traci.vehicle.getIDList():
            if other == vid:
                continue
            if traci.vehicle.getLaneID(other) != lane_id:
                continue

            pos_other = traci.vehicle.getPosition(other)[0]
            if pos_other > pos:
                if front_pos is None or pos_other < front_pos:
                    front_pos = pos_other

        # If the safe gap is insufficient, immediately resume automatic control
        if front_pos is not None:
            gap = front_pos - pos
            if gap < JAD_SAFE_GAP:
                print(
                    f"[Step {step}] {vid} unsafe gap {gap:.2f}m, resuming automatic car-following early"
                )
                traci.vehicle.setSpeedMode(vid, 31)
                traci.vehicle.setSpeed(vid, -1)
                to_remove.append(vid)
                continue

        # -----------------------------
        # Phase 1: Maintain initial speed
        # -----------------------------
        if phase == 1:
            if step - start < Duration_AB:
                traci.vehicle.setSpeedMode(vid, 0)
                traci.vehicle.setSpeed(vid, info["init_speed"])
            else:
                info["phase"] = 2
                info["phase_start"] = step

        # -----------------------------
        # Phase 2: Force low-speed driving
        # -----------------------------
        elif phase == 2:
            if step - start < Duration_BC:
                traci.vehicle.setSpeedMode(vid, 0)
                traci.vehicle.setSpeed(vid, jad_speed)
            else:
                info["phase"] = 3
                info["phase_start"] = step

        # -----------------------------
        # Phase 3: Resume natural car-following
        # -----------------------------
        elif phase == 3:
            print(f"[Step {step}] {vid} Phase 3: resuming natural car-following")
            traci.vehicle.setSpeedMode(vid, 31)
            traci.vehicle.setSpeed(vid, -1)
            to_remove.append(vid)

    # Clean up vehicles that have completed control
    for vid in to_remove:
        jad_plan.pop(vid, None)



# ======================================================
# Section Detection Logic
# ======================================================
def detector(step, veh_ids, last_pos, location, sg_state, sg_max_speed, sg_min_duration):
    """
    Use cumulative travel distance getDistance()
    to detect whether a vehicle passes a specified section.

    If sg_state is provided, perform
    stop-and-go detection based on cross-section speed
    (applicable to any detector)
    """

    events = []
    sg_event = None

    # ======================================================
    # 1. Detect vehicle crossing the section
    # ======================================================
    for vid in veh_ids:
        pos = traci.vehicle.getDistance(vid)

        if vid in last_pos:
            prev = last_pos[vid]

            if prev < location <= pos:
                speed = traci.vehicle.getSpeed(vid)

                # if location == DETECTOR_LOC_DOWNSTREAM:                    
                #     print(f"[step={step}] veh={vid} >>> CROSS detector @ {location} | speed={speed:.2f}")

                events.append({
                    "step": step,
                    "veh_id": vid,
                    "speed": speed,
                    "location": location
                })

        last_pos[vid] = pos

    # ======================================================
    # 2. Stop-and-Go Detection
    # ======================================================
    if sg_state is not None and events:

        # Initialize state (only once)
        if "in_low_speed" not in sg_state:
            sg_state["in_low_speed"] = False
            sg_state["t_start"] = None
            sg_state["speeds"] = []
            sg_state["v_start"] = None 

        # Representative speed of current step at this detector
        v_mean = sum(e["speed"] for e in events) / len(events)

        # ---------- Enter / continue low speed ----------
        if v_mean < sg_max_speed:
            if not sg_state["in_low_speed"]:
                sg_state["in_low_speed"] = True
                sg_state["t_start"] = step
                sg_state["speeds"] = [v_mean]
                sg_state["v_start"] = v_mean
            else:
                sg_state["speeds"].append(v_mean)

        # ---------- Speed recovery, check if it constitutes S&G ----------
        else:
            if sg_state["in_low_speed"]:
                t_start = sg_state["t_start"]
                t_end = step
                duration = t_end - t_start
                v_end = v_mean

                if duration >= sg_min_duration:
                    sg_event = {
                        "location": location,
                        "t_start": t_start,
                        "t_end": t_end,
                        "duration": duration,
                        "v_start": sg_state["v_start"],  
                        "v_end": v_end,                  
                        "v_min": min(sg_state["speeds"]),
                        "v_mean": sum(sg_state["speeds"]) / len(sg_state["speeds"])
                    }

                # Reset state (regardless of whether it constitutes S&G)
                sg_state["in_low_speed"] = False
                sg_state["t_start"] = None
                sg_state["speeds"] = []

    return last_pos, events, sg_event



# ======================================================
# Save results to CSV files
# ======================================================
def save_result(jad_speed, wave_speed, Et_offset, records_up, records_down,
                A, B, C, D, E, F,
                P1, P2, P3,
                v_t, v_w):

    def point_or_empty(P):
        if P is None:
            return "", ""
        return P[0], P[1]

    # Upstream
    with open(f"d_1_jad_detector_upstream_{int(jad_speed*3.6)}_{int(Et_offset)}.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["veh_id", "step", "speed", "location"])
        for rec in records_up:
            writer.writerow([
                rec["veh_id"],
                rec["step"],
                rec["speed"],
                rec["location"]
            ])

    # Downstream
    with open(f"d_1_jad_detector_downstream_{int(jad_speed*3.6)}_{int(Et_offset)}.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["veh_id", "step", "speed", "location"])
        for rec in records_down:
            writer.writerow([
                rec["veh_id"],
                rec["step"],
                rec["speed"],
                rec["location"]
            ])

    # Process points uniformly
    A_t, A_x = point_or_empty(A)
    B_t, B_x = point_or_empty(B)
    C_t, C_x = point_or_empty(C)
    D_t, D_x = point_or_empty(D)
    E_t, E_x = point_or_empty(E)
    F_t, F_x = point_or_empty(F)
    P1_t, P1_x = point_or_empty(P1)
    P2_t, P2_x = point_or_empty(P2)
    P3_t, P3_x = point_or_empty(P3)

    # JAD strategy
    with open(f"d_1_jad_strategy_{int(jad_speed*3.6)}_{int(Et_offset)}.csv", "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "A_t", "A_x",
            "B_t", "B_x",
            "C_t", "C_x",
            "D_t", "D_x",
            "E_t", "E_x",
            "F_t", "F_x",
            "P1_t", "P1_x",
            "P2_t", "P2_x",
            "P3_t", "P3_x",
            "v_t", "v_w",
            "wave_speed",
            "jad_speed"
        ])

        writer.writerow([
            A_t, A_x,
            B_t, B_x,
            C_t, C_x,
            D_t, D_x,
            E_t, E_x,
            F_t, F_x,
            P1_t, P1_x,
            P2_t, P2_x,
            P3_t, P3_x,
            v_t,
            v_w,
            wave_speed,
            jad_speed
        ])



# -------------------------------
# Record vehicle travel times
# -------------------------------
def record_travel_times(travel_times, current_step):
    veh_ids = traci.vehicle.getIDList()

    # Record entry
    for veh_id in veh_ids:
        if veh_id not in travel_times:
            travel_times[veh_id] = {'enter': current_step, 'leave': None}

    # Record exit
    for veh_id in list(travel_times.keys()):
        if travel_times[veh_id]['leave'] is None and veh_id not in veh_ids:
            travel_times[veh_id]['leave'] = current_step



# -------------------------------
# Append to CSV (only write travel_time)
# -------------------------------
def append_travel_times_to_csv(csv_file, travel_times, seed):
    file_exists = os.path.exists(csv_file)

    # Collect all valid travel times
    travel_time_list = []
    for veh_id, times in travel_times.items():
        if times['leave'] is not None:
            travel_time = times['leave'] - times['enter']
            travel_time_list.append(travel_time)

    with open(csv_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write header only if file does not exist
        if not file_exists:
            writer.writerow(['seed'])

        # Write a whole row: seed + all vehicle travel times
        writer.writerow([seed] + travel_time_list)

    print(f"[INFO] Travel times for SEED={seed} have been appended to {csv_file}")



# ------------------------------
# Plot reference lines
# ------------------------------
def plot_reference_lines(ax, times, detector_downstream, detector_upstream, ramp):

    right_x = times.max()
    font_size_ref = 10
    offset_y = 0.37

    def draw_ref(y, label, offset=0):
        ax.axhline(y=y/1000, color="white", linestyle="--", linewidth=0.6)
        ax.text(
            right_x, y/1000 - offset,
            label,
            va="bottom",
            ha="right",
            color="white",
            fontsize=font_size_ref
        )

    draw_ref(detector_downstream, "detector\ndownstream", offset_y)
    draw_ref(detector_upstream,   "detector\nupstream",   offset_y)
    draw_ref(ramp,                "ramp",                 0) 



# ------------------------------
# JAD Strategy
# ------------------------------
def plot_jad_strategy(JAD_FILE, ax):

    # ---- Point Style ----
    POINT_COLOR = "black"
    POINT_SIZE  = 40
    POINT_Z     = 10
    POINT_FONT  = 14    

    # ---- Feasible Region Style ----
    REGION_COLOR     = "black"
    REGION_ALPHA     = 0.2
    REGION_LW        = 2
    REGION_Z         = 2
    REGION_FONT_SIZE = 10    

    def is_valid(v):
        return v != "" and v is not None

    if os.path.exists(JAD_FILE):
        with open(JAD_FILE, "r") as f:
            rows = list(csv.DictReader(f))

        if rows:
            row = rows[-1]

            # ---- Label Points A~F ----
            for p in ["A", "B", "C", "D", "E", "F"]:
                if is_valid(row.get(f"{p}_t")) and is_valid(row.get(f"{p}_x")):
                    t = float(row[f"{p}_t"])
                    x = float(row[f"{p}_x"]) / 1000
                    ax.scatter(t, x, color=POINT_COLOR, s=POINT_SIZE, zorder=POINT_Z)
                    ax.text(t, x, f" {p}", fontsize=POINT_FONT,
                            va="bottom", ha="left", color=POINT_COLOR)

            # ---- Feasible Region Triangle ----
            P_points = []
            for p in ["P1", "P2", "P3"]:
                if is_valid(row.get(f"{p}_t")) and is_valid(row.get(f"{p}_x")):
                    P_points.append((float(row[f"{p}_t"]), float(row[f"{p}_x"])))

            if len(P_points) == 3:
                triangle_xy = [(t, x/1000) for t, x in P_points]

                triangle = Polygon(
                    triangle_xy,
                    closed=True,
                    facecolor=REGION_COLOR,
                    alpha=REGION_ALPHA,
                    edgecolor=REGION_COLOR,
                    linewidth=REGION_LW,
                    zorder=REGION_Z
                )
                ax.add_patch(triangle)

                xs_tri = [p[0] for p in triangle_xy]
                ys_tri = [p[1] for p in triangle_xy]

                ax.text(
                    min(xs_tri) - 10,
                    max(ys_tri) + 0.02,
                    "Feasible\nRegion of A",
                    fontsize=REGION_FONT_SIZE,
                    color="black",
                    ha="right",
                    va="top"
                )



# ------------------------------
# Plot trajectories
# ------------------------------
def plot_trajectories(ids, lc_for_cbar, times, xs, ax):

    ALPHA_ORIGINAL = 0.5
    LineWidth_ORIGINAL = 0.6
    LineWidth_INSERTED = 2.0    

    cmap = plt.colormaps["jet_r"]
    norm = mcolors.Normalize(vmin=0, vmax=90)

    unique_ids = np.unique(ids)

    for vid in unique_ids:
        mask = (ids == vid)
        t_list = times[mask]
        x_list = xs[mask] / 1000

        if len(t_list) < 2:
            continue

        v_list = np.diff(x_list * 1000) / np.diff(t_list) * 3.6
        v_list = np.insert(v_list, 0, v_list[0])

        points = np.array([t_list, x_list]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        if vid.startswith("inserted_"):
            for seg in segments:
                ax.plot(seg[:, 0], seg[:, 1],
                        color="black",
                        linewidth=LineWidth_INSERTED,
                        zorder=10)
            continue

        lc = LineCollection(segments,
                            cmap=cmap,
                            norm=norm,
                            array=v_list,
                            linewidths=LineWidth_ORIGINAL,
                            alpha=ALPHA_ORIGINAL)
        ax.add_collection(lc)

        if lc_for_cbar is None:
            lc_for_cbar = lc

    return lc_for_cbar




# ------------------------------
# Load trajectory data
# ------------------------------
def load_trajectory(XML_FILE):
    """Read SUMO trajectory XML"""
    times, ids, xs = [], [], []

    tree = ET.parse(XML_FILE)
    root = tree.getroot()

    for timestep in root.findall("timestep"):
        t = float(timestep.get("time"))
        for veh in timestep.findall("vehicle"):
            times.append(t)
            ids.append(veh.get("id"))
            xs.append(float(veh.get("x")))

    return np.array(times), np.array(ids), np.array(xs)



    
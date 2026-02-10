import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
import matplotlib.colors as mcolors

# ==========================
# Parameters
# ==========================
FIG_SIZE = (5.5, 5)
ALPHA_ORIGINAL = 0.5
LW_ORIGINAL = 0.6

# ==========================
# Plotting function
# ==========================
def plot_for_speed(XML_FILE):
    times, ids, xs = [], [], []

    tree = ET.parse(XML_FILE)
    root = tree.getroot()

    for timestep in root.findall("timestep"):
        t = float(timestep.get("time"))
        for veh in timestep.findall("vehicle"):
            times.append(t)
            ids.append(veh.get("id"))
            xs.append(float(veh.get("x")))

    times = np.array(times)
    ids = np.array(ids)
    xs = np.array(xs)

    unique_ids = np.unique(ids)
    cmap = plt.colormaps["jet_r"]
    norm = mcolors.Normalize(vmin=0, vmax=90)

    # ---------- Create figure ----------
    fig = plt.figure(figsize=FIG_SIZE)

    # Main plot
    ax = fig.add_axes([0.1, 0.1, 0.85, 0.75])  # Leave top space for colorbar
    lc_for_cbar = None

    for vid in unique_ids:
        mask = (ids == vid)
        t_list = times[mask]
        x_list = xs[mask] / 1000  # Convert to km

        if len(t_list) < 2:
            continue

        v_list = np.diff(x_list*1000)/np.diff(t_list)*3.6  # km/h
        v_list = np.insert(v_list, 0, v_list[0])

        points = np.array([t_list, x_list]).T.reshape(-1,1,2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        lc = LineCollection(segments, cmap=cmap, norm=norm, array=v_list,
                            linewidths=LW_ORIGINAL, alpha=ALPHA_ORIGINAL)
        ax.add_collection(lc)

        if lc_for_cbar is None:
            lc_for_cbar = lc

    # Auto scale
    ax.autoscale()
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Space (km)", fontsize=12)
    ax.set_xlim(200, 1600)
    ax.set_ylim(0, 8)

    # ---------- Top colorbar ----------
    cbar_ax = fig.add_axes([0.1, 0.88, 0.85, 0.03])  # Top space to fully display colorbar
    cbar = plt.colorbar(lc_for_cbar, cax=cbar_ax, orientation='horizontal')
    cbar.set_label("Speed (km/h)", labelpad=-7)
    cbar.set_ticks([0, 30, 60, 90])
    cbar.ax.xaxis.set_ticks_position('top')
    cbar.ax.xaxis.set_label_position('top')

    # ---------- Save & show ----------
    plt.savefig("base_trajectory.png", dpi=150, bbox_inches="tight")
    plt.show()
    plt.close()


# ==========================
# Call plotting
# ==========================
plot_for_speed("b_1_base_trajectory.xml")
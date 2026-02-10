# ======================================================
# ===================== Parameters ====================
# ======================================================

# Blocking speed corresponding to each file
FILES_AND_SPEEDS = [
    ("c_1_stability_trajectory_30", 30),
    ("c_1_stability_trajectory_40", 40),
    ("c_1_stability_trajectory_50", 50),
    ("c_1_stability_trajectory_60", 60)
]

X_MIN = 200
X_MAX = 1600

FIG_SIZE = (7, 6)  # Four subplots + top colorbar
Y_MIN = 0
Y_MAX = 8
XTICK_STEP = 400
YTICK_STEP = 1

# ======================================================
# ===================== Main ==========================
# ======================================================

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
import matplotlib.colors as mcolors

# Create a large figure with 4 subplots
fig, axes = plt.subplots(2, 2, figsize=FIG_SIZE, sharex=True, sharey=True)
axes = axes.flatten()

# Collect global speeds
all_speeds_global = []
vehicle_data_list = []

print(" ")

# ------------------------------
# Parse XML and extract data
# ------------------------------
for FILE_NAME, speed in FILES_AND_SPEEDS:
    print(f"Processing: {FILE_NAME}.xml")
    
    times, ids, xs = [], [], []

    tree = ET.parse(FILE_NAME + ".xml")
    root = tree.getroot()

    for timestep in root.findall("timestep"):
        t = float(timestep.get("time"))
        for veh in timestep.findall("vehicle"):
            times.append(t)
            ids.append(veh.get("id"))
            xs.append(float(veh.get("x")) / 1000.0)

    times = np.array(times)
    ids = np.array(ids)
    xs = np.array(xs)

    vehicle_data = []
    for vid in np.unique(ids):
        mask = (ids == vid)
        t_v = times[mask]
        x_v = xs[mask]

        if len(t_v) > 1:
            v_v = np.diff(x_v) / np.diff(t_v) * 3600
            v_v = np.insert(v_v, 0, v_v[0])
        else:
            v_v = np.array([0.0])

        all_speeds_global.extend(v_v)
        vehicle_data.append((t_v, x_v, v_v))

    vehicle_data_list.append(vehicle_data)

# ------------------------------
# Unified colormap
# ------------------------------
cmap = plt.colormaps["jet_r"]
norm = mcolors.Normalize(vmin=0, vmax=90)  # Fix max speed to 90 km/h

# ------------------------------
# Plot subplots
# ------------------------------

# Subplot labels
subplot_labels = ["(a)", "(b)", "(c)", "(d)"]

for idx, ((FILE_NAME, speed), vehicle_data) in enumerate(zip(FILES_AND_SPEEDS, vehicle_data_list)):
    segments = []
    colors = []

    for t_v, x_v, v_v in vehicle_data:
        points = np.array([t_v, x_v]).T.reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        segments.extend(segs)
        colors.extend(v_v[:-1])

    lc = LineCollection(
        segments, cmap=cmap, norm=norm, array=np.array(colors),
        linewidths=0.4, alpha=0.5
    )
    axes[idx].add_collection(lc)

    ax = axes[idx]
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_xticks(np.arange(X_MIN, X_MAX + 1, XTICK_STEP))
    ax.set_yticks(np.arange(Y_MIN, Y_MAX + 0.001, YTICK_STEP))
    
    # Control subplot axis labels
    if idx == 0:  # Top-left subplot, no x-label
        ax.set_xlabel("")
    else:
        ax.set_xlabel("Time (s)", fontsize=12)
    
    if idx == 1:  # Top-right subplot, no labels
        ax.set_xlabel("")
        ax.set_ylabel("")
    else:
        ax.set_ylabel("Space (km)", fontsize=12)

    if idx == 3:  # Bottom-right subplot, no y-label
        ax.set_ylabel("")

    # Add text in bottom-left corner: Blocking speed + label
    ax.text(
        0.03, 0.05,
        f"{subplot_labels[idx]} {speed} km/h",
        color='black', fontsize=12,
        ha='left', va='bottom',
        transform=ax.transAxes,
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="none", pad=2)
    )   

# ------------------------------
# Add top horizontal colorbar (slightly narrower)
# ------------------------------
cbar_ax = fig.add_axes([0.12, 0.92, 0.78, 0.02])  # [left, bottom, width, height]
cbar = plt.colorbar(lc, cax=cbar_ax, orientation='horizontal', ticklocation='top')
cbar.set_label("Speed (km/h)", labelpad=-7)
cbar.set_ticks([0, 30, 60, 90])

# ------------------------------
# Adjust subplot spacing for compact layout
# ------------------------------
fig.subplots_adjust(top=0.88, hspace=0.2, wspace=0.2)

# ------------------------------
# Save PNG (rasterized, smaller file, LaTeX-friendly)
# ------------------------------
png_name = "stability_trajectory.png"
plt.savefig(png_name, format="png", dpi=150, bbox_inches="tight")  # DPI adjustable
print(f"Figure saved as: {png_name}")

plt.show()
plt.close()
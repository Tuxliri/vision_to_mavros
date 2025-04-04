#!/usr/bin/env python3
import imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
import numpy as np
from moms_apriltag import ApriltagBoard


# === CONFIGURATION ===
tag_family = "tag36h11"
rows = 4
cols = 3
tag_size_m = 0.16        # meters (side length)
spacing_m = 0.04         # center-to-center spacing
dpi = 300
a3_inches = (11.7, 16.5)  # A3 in inches (landscape)

# === GENERATE BOARD (no spacing arg) ===
board = ApriltagBoard.create(rows, cols, tag_family, tag_size_m)
board_image = np.array(board.board)

# === SAVE AS IMAGE ===
imageio.imwrite("apriltag_board.png", board_image)
print("✅ Saved board as apriltag_board.png")

# === SAVE AS A3 PDF ===
fig = plt.figure(figsize=a3_inches)
plt.imshow(board_image, cmap='gray')
plt.axis('off')
plt.tight_layout()
fig.savefig("apriltags_a3.pdf", dpi=dpi)
print("✅ Saved board as apriltags_a3.pdf")


# === GENERATE YAML in required inline format ===
tag_layout_entries = []

for r in range(rows):
    for c in range(cols):
        tag_idx = r * cols + c
        tag_id = board.ids[tag_idx]
        x = round(c * (tag_size_m + spacing_m), 4)
        y = round(-r * (tag_size_m + spacing_m), 4)
        z = 0.0

        tag_layout_entries.append({
            "id": tag_id,
            "size": tag_size_m,
            "x": x,
            "y": y,
            "z": z,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0
        })

yaml_data = {
    "standalone_tags": [],
    "tag_bundles": [
        {
            "name": "apriltag_grid",
            "layout": tag_layout_entries
        }
    ]
}

# === SAVE YAML ===
with open("tag_bundle.yaml", "w") as f:
    yaml.dump(
        yaml_data,
        f,
        sort_keys=False,
        default_flow_style=True,  # inline formatting
        width=1000
    )
print("✅ Saved YAML tag bundle as tag_bundle.yaml")

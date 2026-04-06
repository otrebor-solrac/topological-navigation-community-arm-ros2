#!/usr/bin/env python3
import math
import os

def calc_pose(q2_deg, q3_deg, Ox=150, Oy=500):
    q2 = math.radians(q2_deg)
    q3 = math.radians(q3_deg)
    L_lower = 280.0
    L_lever = 120.0
    L_pleuel = 280.0
    L_forearm = 300.0

    # Codo
    Cx = Ox + L_lower * math.cos(q2)
    Cy = Oy - L_lower * math.sin(q2)
    # Rev 12
    R12x = Ox + L_lever * math.cos(q3)
    R12y = Oy - L_lever * math.sin(q3)
    # Cierre
    Ciex = R12x + L_pleuel * math.cos(q2)
    Ciey = R12y - L_pleuel * math.sin(q2)
    # Punta
    Px = Cx - L_forearm * math.cos(q3)
    Py = Cy + L_forearm * math.sin(q3)

    return (Cx, Cy, R12x, R12y, Ciex, Ciey, Px, Py)

def draw_pose_svg(q2, q3, title, output_path):
    Cx, Cy, R12x, R12y, Ciex, Ciey, Px, Py = calc_pose(q2, q3)
    Ox, Oy = 150, 500

    svg = f"""<svg width="800" height="600" xmlns="http://www.w3.org/2000/svg">
  <rect width="100%" height="100%" fill="#fafafa"/>
  <text x="400" y="40" font-family="sans-serif" font-size="24" text-anchor="middle" font-weight="bold">{title}</text>

  <!-- LOWER SHANK -->
  <line x1="{Ox}" y1="{Oy}" x2="{Cx}" y2="{Cy}" stroke="#4a90e2" stroke-width="12" stroke-linecap="round"/>
  <text x="{Ox + 40}" y="{Oy - 140}" font-family="sans-serif" font-size="20" fill="#4a90e2" font-weight="bold">lower_shank_140</text>

  <!-- UPPER SHANK -->
  <line x1="{Ciex}" y1="{Ciey}" x2="{Px}" y2="{Py}" stroke="#f5a623" stroke-width="18" stroke-linecap="round"/>
  <text x="{Px}" y="{Py + 30}" font-family="sans-serif" font-size="18" fill="#f5a623" font-weight="bold">upper_shank_140</text>
  
  <!-- ARM LEVER -->
  <line x1="{Ox}" y1="{Oy}" x2="{R12x}" y2="{R12y}" stroke="#417505" stroke-width="12" stroke-linecap="round"/>
  <text x="{R12x - 100}" y="{R12y - 20}" font-family="sans-serif" font-size="18" fill="#417505" font-weight="bold">arm_lever</text>
  
  <!-- PLEUEL -->
  <line x1="{R12x}" y1="{R12y}" x2="{Ciex}" y2="{Ciey}" stroke="#2980b9" stroke-width="6" stroke-dasharray="10,6" stroke-linecap="round"/>
  <text x="{Ciex - 40}" y="{(R12y + Ciey)/2}" font-family="sans-serif" font-size="18" fill="#2980b9" font-weight="bold">pleuel_140</text>

  <!-- Puntos pivote -->
  <circle cx="{Ox}" cy="{Oy}" r="10" fill="#d0021b" />
  <circle cx="{Cx}" cy="{Cy}" r="8" fill="#333" />
  <circle cx="{R12x}" cy="{R12y}" r="8" fill="#333" />
  <circle cx="{Ciex}" cy="{Ciey}" r="10" fill="#9013fe" />
</svg>"""

    with open(output_path, "w") as f:
        f.write(svg)
    print(f"Graficado: {output_path}")

if __name__ == "__main__":
    doc_dir = os.path.dirname(__file__)
    # Pose 1: q2=60, q3=20
    draw_pose_svg(90, 40, "Pose 1: q2=90°, q3=40°", os.path.join(doc_dir, "Pose_1.svg"))
    # Pose 2: q2=60, q3=80
    draw_pose_svg(90, 0, "Pose 2: q2=90°, q3=0°", os.path.join(doc_dir, "Pose_2.svg"))

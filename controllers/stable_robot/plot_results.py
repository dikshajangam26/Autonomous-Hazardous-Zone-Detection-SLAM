import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import numpy as np

# CONFIGURATION
CSV_FILE = 'robot_data.csv'
REFRESH_INTERVAL_MS = 500  # Faster refresh (0.5s) to catch fast events


plt.style.use('ggplot')
fig, axes = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle('Real-Time Robot Telemetry', fontsize=16)

def animate(i):
    if not os.path.exists(CSV_FILE) or os.stat(CSV_FILE).st_size == 0:
        return 

    try:
        # 2. Read the CSV
        df = pd.read_csv(CSV_FILE)
        df = df.tail(100) 
        if df.empty: return

        # --- PLOT 1: Trajectory ---
        ax1 = axes[0, 0]
        ax1.clear()
        ax1.plot(df['Pos_X'], df['Pos_Y'], 'b-', label='Path')
        ax1.plot(df['Pos_X'].iloc[-1], df['Pos_Y'].iloc[-1], 'ro', label='Current') 
        ax1.set_title(f"Live Map (X, Y)")
        ax1.set_xlim([-25, 25]); ax1.set_ylim([-25, 25]) 
        ax1.grid(True)
        ax1.legend(loc='upper right')

        # --- PLOT 2: Sensors ---
        ax2 = axes[0, 1]
        ax2.clear()
        ax2.plot(df['Time_s'], df['Lidar_Front'], 'r-', label='Front')
        ax2.plot(df['Time_s'], df['Lidar_Left'], 'g--', label='Left')
        ax2.plot(df['Time_s'], df['Lidar_Right'], 'b--', label='Right')
        ax2.set_title("Obstacle Distances (m)")
        ax2.set_ylim([0, 10]) 
        ax2.legend(loc='upper right')

        # --- PLOT 3: Hazard Meter ---
        ax3 = axes[1, 0]
        ax3.clear()

        min_dist = df[['Lidar_Front', 'Lidar_Left', 'Lidar_Right']].min(axis=1).fillna(20.0)
        hazard_score = (3.0 - min_dist).clip(lower=0)

        ax3.fill_between(df['Time_s'], hazard_score, color='red', alpha=0.6)
        ax3.plot(df['Time_s'], hazard_score, color='darkred', label='Current Hazard')
        
        ax3.set_title("Proximity Hazard Level")
        ax3.set_ylabel("Danger Level")

        current_peak = hazard_score.max() if not hazard_score.empty else 0.0
        if np.isnan(current_peak): current_peak = 0.0
        new_limit = max(3.5, current_peak * 1.1)
        ax3.set_ylim([0, new_limit])

        ax3.axhline(y=2.6, color='yellow', linestyle='--', linewidth=2, label='Turn Threshold')

        ax3.legend(loc='upper right', facecolor='white', framealpha=1.0, shadow=True)

        
        # --- PLOT 4: Logic State ---
        ax4 = axes[1, 1]
        ax4.clear()
        
        # Get the last 10 data points (approx. 3 seconds of history)
        recent_states = df['State'].tail(10).values
        
        # PRIORITY SYSTEM: Show the most dangerous state from recent history
        if any("EMERGENCY" in str(s) for s in recent_states):
            display_state = "EMERGENCY!"
            box_color = "red"
            text_color = "white"
        elif any("WORKER" in str(s) for s in recent_states):
            display_state = "WORKER DETECTED"
            box_color = "yellow"
            text_color = "black"
        elif any("U_TURN" in str(s) for s in recent_states):
            display_state = "TURNING"
            box_color = "lightblue"
            text_color = "black"
        else:
            display_state = "FORWARD"
            box_color = "white"
            text_color = "black"
        
        ax4.text(0.5, 0.5, display_state, fontsize=22, weight='bold',
                 ha='center', va='center', color=text_color,
                 bbox=dict(boxstyle="round,pad=1", fc=box_color, ec="black"))
        ax4.set_title("Current Logic State")
        ax4.axis('off') 

    except Exception as e:
        print(f"Plotting Error: {e}")


ani = FuncAnimation(fig, animate, interval=REFRESH_INTERVAL_MS, cache_frame_data=False)
plt.tight_layout()
plt.show()

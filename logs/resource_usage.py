import json
import matplotlib.pyplot as plt
import pandas as pd

# ==== CONFIG ====
log_path = "sysmon_20251018_151326.jsonl"  # your log file
window = 3  # smoothing window (in samples)

# ==== LOAD DATA ====
data = []
with open(log_path, "r") as f:
    for line in f:
        try:
            data.append(json.loads(line))
        except json.JSONDecodeError:
            continue

df = pd.DataFrame(data)
if df.empty:
    raise ValueError("No valid data found in log file.")

# Compute time offset and smoothing
df["time"] = df["timestamp"] - df["timestamp"].iloc[0]
df["cpu_smooth"] = df["cpu_percent"].rolling(window, center=True).mean()
df["mem_smooth"] = df["memory_mb"].rolling(window, center=True).mean()

# ==== CONVERT TO NUMPY (fix for multi-dimensional issue) ====
t = df["time"].to_numpy()
cpu = df["cpu_smooth"].to_numpy()
mem = df["mem_smooth"].to_numpy()

# ==== PLOT ====
plt.style.use("ggplot")
fig, ax1 = plt.subplots(figsize=(10, 5))

# CPU usage
ax1.plot(t, cpu, color="#1f77b4", linewidth=2, label="CPU (%)")
ax1.set_xlabel("Time (s)", fontsize=12)
ax1.set_ylabel("CPU Usage (%)", color="#1f77b4", fontsize=12)
ax1.tick_params(axis="y", labelcolor="#1f77b4")
ax1.grid(True, linestyle="--", alpha=0.5)

# Memory usage (secondary axis)
ax2 = ax1.twinx()
ax2.plot(t, mem, color="#ff7f0e", linewidth=2, label="Memory (MB)")
ax2.set_ylabel("Memory (MB)", color="#ff7f0e", fontsize=12)
ax2.tick_params(axis="y", labelcolor="#ff7f0e")

fig.suptitle("CPU and Memory Utilization Over Time", fontsize=14, weight="bold")
plt.tight_layout()
plt.savefig("resource_usage.png", dpi=200)
plt.show()

print("✅ Saved as resource_usage.png")

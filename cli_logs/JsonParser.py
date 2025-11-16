import ast
import json
import os
import re

# ==================== CONFIG =====================
INPUT_LOG = "Drone2.log"
OUTPUT_JSON = "parsed_missions_drone2.json"
# =================================================

def extract_mission_blocks(log_path):
    """Extract full JSON-like mission dictionaries from log, even if very long."""
    missions = []
    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    pattern = re.compile(r"Received message from Unity:\s*(\{.*?\})(?=\s*\[DroneInstance|\Z)", re.S)
    matches = pattern.findall(text)

    for match in matches:
        # Verify balanced braces
        open_count = match.count("{")
        close_count = match.count("}")
        if open_count == close_count:
            missions.append(match)
        else:
            # Fallback to manual balance scan if mismatch
            brace_depth = 0
            for i, ch in enumerate(match):
                if ch == "{": brace_depth += 1
                elif ch == "}": brace_depth -= 1
                if brace_depth == 0 and i > 0:
                    missions.append(match[:i+1])
                    break

    return missions


def parse_log_to_json(log_path):
    missions_raw = extract_mission_blocks(log_path)
    print(f"[INFO] Found {len(missions_raw)} mission blocks in {log_path}")

    missions = []

    for i, raw in enumerate(missions_raw, 1):
        try:
            mission_data = ast.literal_eval(raw)
        except Exception as e:
            print(f"[WARN] Mission {i} failed literal_eval: {e}")
            with open(f"failed_mission_{i}.txt", "w", encoding="utf-8") as f:
                f.write(raw)
            continue

        # --- Extract waypoints or points ---
        points = []
        if "waypoints" in mission_data and len(mission_data["waypoints"]) > 0:
            points = [
                {"x": wp["coords"][0], "y": wp["coords"][1], "z": wp["coords"][2]}
                for wp in mission_data["waypoints"]
                if "coords" in wp
            ]
        elif "points" in mission_data and len(mission_data["points"]) > 0:
            points = mission_data["points"]

        mission_entry = {
            "type": mission_data.get("type", "map_area"),
            "mission_id": mission_data.get("mission_id", f"Unknown_{i}"),
            "points": points,
            "waypoints": [],
            "grid_spacing": mission_data.get("grid_spacing", 0.0),
            "vertical_step": mission_data.get("vertical_step", 0.0),
            "priority": mission_data.get("priority", 1),
            "preempt": mission_data.get("preempt", False),
        }

        missions.append(mission_entry)
        print(f"[OK] Parsed {mission_entry['mission_id']} with {len(points)} waypoints")

    final_data = {"missions": missions}
    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(final_data, f, indent=2)

    print(f"\n[SUMMARY] Extracted {len(missions)} missions → {OUTPUT_JSON}")
    return final_data


if __name__ == "__main__":
    if not os.path.exists(INPUT_LOG):
        print(f"[ERROR] Log file not found: {INPUT_LOG}")
    else:
        parse_log_to_json(INPUT_LOG)

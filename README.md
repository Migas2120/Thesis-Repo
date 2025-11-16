
# Thesis-Repo
ROS server for drone companion computers, enabling communication with a Mixed Reality interface for Experimentation.
---

## ⚙️ Installation

### Prerequisites

To successfully set up and run this project, your environment must meet the following criteria:

1.  **Windows Subsystem for Linux (WSL 2):**
    * You must be running **Windows 10 or 11**.
    * The project requires the **Ubuntu-20.04** distribution running on **WSL 2**.
    * **Verification:** Check your distribution list and version by running this command in PowerShell or Command Prompt:
        ```bash
        wsl -l -v
        ```
    * Ensure the output shows `Ubuntu-20.04` with a version of `2`.

2.  **Robot Operating System (ROS):**
    * **ROS Noetic** must be installed within your Ubuntu-20.04 environment.
    * If not installed, please follow the official [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

3.  **ArduPilot SITL (Software In The Loop) & ROS Integration:**
    * You must have the ArduPilot SITL simulation environment installed.
    * **Crucially**, you must configure SITL for ROS communication, which involves setting up the `apm.launch` file and dependencies.
    * Please follow the dedicated **ArduPilot ROS-SITL setup documentation** here:
        [ArduPilot ROS-SITL Setup](https://ardupilot.org/dev/docs/ros-sitl.html)

## 📦 Project Dependencies

This project relies on both **ROS Noetic packages** (managed by Catkin) and **external Python libraries** (managed by `pip`).

### 1. ROS Dependencies (Catkin)

These packages are declared in your `package.xml` and are required for building and running the ROS nodes.

* **Action:** Ensure these packages are installed in your Ubuntu-20.04 environment, typically by installing the corresponding `ros-noetic-` packages via `apt` or by using `rosdep` in your workspace.

| Package Name | Purpose | Dependency Type |
| :--- | :--- | :--- |
| `geometry_msgs` | Standard ROS messages for common geometric primitives. | Build & Exec |
| `mavros_msgs` | Messages for MAVLink communication with ArduPilot. | Build & Exec |
| `rospy` | The ROS client library for Python. | Build & Exec |
| `std_msgs` | Standard ROS message types. | Build & Exec |

### 2. Python Dependencies (`pip`)

These external Python libraries are required by the project's Python scripts. The required versions are listed in the `requirements.txt` file.

* **Action:** To install these, navigate to the project root and run:
    ```bash
    pip install -r requirements.txt
    ```

| Library | Version | Notes |
| :--- | :--- | :--- |
| `Shapely` | `2.1.2` | For geometric operations and manipulation. |
| `trimesh` | `4.6.12` | For working with 3D models and meshes. |
| `matplotlib` | `3.1.2` | For generating plots and visualizations. |
| `numpy` | `1.24.4` / `1.17.4` | Fundamental package for numerical computing. (Note: Multiple versions listed) |
| `pandas` | `2.0.3` | For data manipulation and analysis. |
| `psutil` | `7.1.3` | For retrieving process and system utilization. |
| `sensor_msgs` | `1.13.2` | ROS message definitions for sensor data. |



## ⚙️ Project Setup & Building

These steps guide you through creating your ROS workspace and compiling the project packages.

### 1. Workspace Creation 

First, create your Catkin workspace directory structure in your Ubuntu-20.04 environment:

```bash
mkdir -p ~/your_Workspace/src
cd ~/your_Workspace
```

### 2. Repository Cloning & File Transfer

Clone this repository and move the necessary package files into your workspace’s source directory (`src`).

1. **Clone the repository temporarily:**
   ```bash
   # Option 1 — Clone using SSH (recommended)
   git clone git@github.com:Migas2120/Thesis-Repo.git

   # Option 2 — Clone using HTTPS
   git clone https://github.com/Migas2120/Thesis-Repo.git

2. **Move the repository contents into your Catkin workspace:**
   ```bash
   cd Thesis-Repo
   mv * ~/your_Workspace/src/
   ```

### 3. Building the Workspace

Once all packages are placed inside the `src` directory, return to the root of your Catkin workspace and build the project using `catkin_make`.

1. **Navigate to your workspace root:**
   ```bash
   cd ~/your_Workspace
   ```

2. **Build the project:**
   ```bash
   catkin_make
   ```

3. **Source the setup file to update your environment:**
   ```bash
   source devel/setup.bash
   ```
4. **Tip: You can add the source command to your ~/.bashrc file so that your environment is automatically configured every time you open a new terminal:**
   ```bash
   echo "source ~/your_Workspace/devel/setup.bash" >> ~/.bashrc
   ```

### 4. Running the Project

To execute the full simulation and ROS communication setup, you will need to open **four separate terminals** — each responsible for a specific process.

---

#### Terminal 1 — Start the ROS Master

In the first terminal, launch the ROS core service:

```bash
roscore
```

#### Terminal 2 — Launch the ArduPilot SITL

In the second terminal, start the ArduPilot Software In The Loop (SITL) simulation.

If you have the ArduPilot directory cloned, run:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map
```

**Note:** In some cases, you may need to add the `--out=127.0.0.1:14550` argument to ensure proper communication between SITL and other components.

#### Terminal 3 — Launch MAVROS (apm.launch)

In the third terminal, start the MAVROS bridge using the provided launch file:

```bash
roslaunch mavros apm.launch
```

#### Terminal 4 — Run the ROS Server (This Project)

Finally, in the fourth terminal, navigate to your workspace and launch this project’s ROS node:

```bash
cd ~/your_Workspace
source devel/setup.bash
roslaunch ros1_server ros1_server.launch ip:="MetaQuestIp"
```
**Note:** The program will still run even if the `ip` parameter is not set or is incorrect.  
However, telemetry data will **not** be transmitted to the Meta Quest device in that case.

---

## ⚙️ Running on WSL (Windows Subsystem for Linux)

If you are running this project on **WSL**, you may need to perform additional setup steps to ensure proper network communication between WSL and the Meta Quest interface.

Because of how WSL handles networking, inbound connections from external devices are not automatically routed to the WSL instance. To fix this, you’ll need to:

1. **Allow inbound traffic** by creating a Windows Firewall rule:
   - Open **Windows Defender Firewall with Advanced Security**.
   - Go to **Inbound Rules → New Rule**.
   - Select **Port**, then specify port **65432**.
   - Allow the connection and apply it to all profiles.

2. **Find your WSL IP address** by running the following command inside WSL:
   ```bash
   hostname -I
   ```
   
3. **Create a port forwarding rule** using `netsh` on Powershell (will require Admin privileges) so Windows forwards traffic to WSL:
   ```bash
   netsh interface portproxy add v4tov4 listenport=65432 listenaddress=0.0.0.0 connectport=65432 connectaddress=<WSL_IP>
   ```
   If you ever need to delete this rule (for example, after an error or IP change), run:
   
   ```bash
   netsh interface portproxy delete v4tov4 listenport=65432 listenaddress=0.0.0.0
   ```


















   

# RoboRacer-VLA: A Vision-Language-Action Model

## Installation

Use the setup commands below to get started:

```bash
# 1. Create and activate conda environment
conda create -n openvla python=3.10 -y
conda activate openvla

# 2. Install PyTorch (adjust for your platform):
#    See https://pytorch.org/get-started/locally/ for details
conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia -y

# 3. Clone and install the OpenVLA repo
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .

# 4. Install Flash Attention 2 for training
pip install packaging ninja
ninja --version; echo $?          # should return exit code "0"
pip install "flash-attn==2.5.5" --no-build-isolation

# 5. Install Flask for the server
pip install flask
```

---

## Inference / Usage

This repository contains two main components:

1. **Server** (`server.py`)  
2. **Client** (`client.py`) for the RoboRacer Platform

### Prerequisites

- **ROS 2** (F1TENTH Stack) installed and sourced  
- **RealSense ROS 2 package** ([realsense2_camera](https://github.com/IntelRealSense/realsense-ros))  
- Python 3  
- Bluetooth gamepad/controller paired with your system  

### 1. Running the Server

On your control machine (or server):

```bash
python3 server.py
```

The server will start and listen for incoming VLA instruction messages from clients.

### 2. Deploying & Running the Client

1. **Copy** `client.py` onto your RoboRacer Platform (e.g., Jetson Xavier).
2. Edit client.py and set the SERVER_URL variable to your server’s IP address (e.g. http://192.168.1.2:5000/predict).
3. **Launch the F1TENTH stack**:

   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

4. **Launch the RealSense camera**:

   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

5. **Ensure your Bluetooth controller is connected** and recognized by the OS.  
6. **Run the client**:

   ```bash
   python3 client.py
   ```

### 3. Publishing Custom Instructions

At any time, publish a natural-language instruction to the `/vla_instruction` topic. Example:

```bash
ros2 topic pub /vla_instruction std_msgs/String \
  "{ data: 'Drive the car through the gap between the two cones directly ahead, staying centered in the opening and holding a steady speed.' }"
```

- Replace the text in `data:` with your own instruction.  
- The client will receive and forward it to the server for processing.

---

## Notes & Troubleshooting

- **Bluetooth Controller**  
  - Verify pairing/connection before launching the client (`lsusb` or `bluetoothctl`).  

- **ROS 2 Topics**  
  - Check that `/vla_instruction` is live and messages appear:
    ```bash
    ros2 topic echo /vla_instruction
    ```

- **Networking**  
  - Ensure the RoboRacer platform can reach the control/server machine (IP, firewall, etc.).

---

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

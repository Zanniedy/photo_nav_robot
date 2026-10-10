# Robot Debug UI

Web interface for ROS2 robot mapping and navigation. Runs on the robot machine; open a browser on any LAN machine to use it.

## Setup (on robot)

### 1. Install ROS dependencies

```bash
sudo apt install ros-humble-rosbridge-server ros-humble-slam-toolbox \
  ros-humble-nav2-bringup ros-humble-nav2-map-server
```

### 2. Install Python dependencies

```bash
cd ~/robot-debug-ui/backend
pip install -r requirements.txt
```

### 3. Build frontend (once, or after code changes)

```bash
cd ~/robot-debug-ui/frontend
npm install
npm run build
```

### 4. Configure

Edit `backend/config.py`:

| Variable | Description |
|----------|-------------|
| `ROBOT_IP` | Robot's LAN IP address (e.g. `192.168.1.100`) |
| `ROS_DOMAIN_ID` | Must match your ROS2 DDS domain |
| `MAPS_DIR` | Where maps are stored (default: `~/maps`) |
| `WORKSPACE_SETUP` | Path to your workspace's `install/setup.bash` |

## Running

### Manual

```bash
cd ~/robot-debug-ui
./start.sh
```

Open `http://<robot-ip>:8000` in a browser.

### Autostart on boot (systemd)

```bash
sudo cp ~/robot-debug-ui/robot-ui.service /etc/systemd/system/robot-ui@.service
sudo systemctl daemon-reload
sudo systemctl enable robot-ui@$(whoami)
sudo systemctl start robot-ui@$(whoami)
```

Check status: `sudo systemctl status robot-ui@$(whoami)`
View logs: `sudo journalctl -u robot-ui@$(whoami) -f`

## Usage

| Mode | How to start | What you can do |
|------|-------------|-----------------|
| 建图 | 点击"建图"按钮 | 实时查看地图构建，输入名称后点保存 |
| 导航 | 点击"导航" → 选地图 → 点击"启动" | 在地图上点击发送目标点 |

**Map view controls:**
- Scroll wheel: zoom in/out
- Middle-click or right-click drag: pan
- Left-click (navigation mode only): send goal pose to robot

## Map storage

Maps are saved to `~/maps/<name>/` (one directory per map, containing `map.pgm` and `map.yaml`).

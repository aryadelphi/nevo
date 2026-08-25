# NEVO (Development and Update)

Proyek pengembangan robot pelayan otonom NEVO (Terintegrasi Fasilitas Antar). Pengembangan ini secara bertahap bermigrasi dari **ROS 2 Humble** menuju **ROS 2 Jazzy**, dengan roadmap masa depan menuju **ROS 2 Kilted Kaiju**.

---
## 📦 Package Documentation

Berikut adalah penjelasan struktur package utama yang ada di dalam workspace ini:

### 📂 `nevo_bot`
* **Deskripsi:** Package inti yang menampung seluruh konfigurasi tingkat tinggi untuk navigasi, pemetaan, dan persepsi robot.
* **Fitur Utama:** * Pemetaan 2D menggunakan `slam_toolbox`.
  * Eksplorasi otonom menggunakan `m_explore_lite`.
  * Navigasi *point-to-point* menggunakan **Nav2 Stack** (AMCL, MPPI Controller, A* Planner).
  * Penghindaran rintangan dinamis 3D berbasis **Spatio-Temporal Voxel Layer (STVL)**.
  * Manajemen prioritas keselamatan menggunakan `twist_mux`.
* **Dependency:** `nav2_bringup`, `slam_toolbox`, `robot_localization`, `twist_mux`, `spatio_temporal_voxel_layer`.

### 📂 `nevo_ws_bridge`
* **Deskripsi:** Package yang berfungsi sebagai jembatan komunikasi (bridge) antara komputasi tingkat tinggi di Raspberry Pi dengan perangkat keras tingkat rendah.
* **Fitur Utama:** * Menerjemahkan `cmd_vel` menjadi instruksi Serial ke Arduino Nano.
  * Mengakuisisi data feedback dari Wheel Encoder.
  * Pemantauan daya baterai *real-time* (menggunakan `ina226_bridge_node.py`).
* **Dependency:** `rclpy`, `pyserial`.

### 📂 `nevo_robot_api` / Tambahan Sensor
* **Deskripsi:** Kumpulan modul dan driver untuk integrasi sensor pihak ketiga dan fungsi teleoperasi.
* **Fitur Utama:** * Integrasi Depth Camera (`kinect_ros2`).
  * Integrasi sensor laser 2D (`rplidar_ros`).
  * Ekstraksi odometri dari laser (`rf2o_laser_odometry`).
* **Dependency:** `sensor_msgs`, `geometry_msgs`.

---

## 🛠️ Tech Stack

Daftar teknologi dan perangkat keras yang digunakan dalam pengembangan NEVO:

- **Middleware & OS:** ROS 2 (Jazzy Jalisco), Ubuntu 24.04 LTS Server
- **Language:** Python 3 & C++
- **Hardware (Compute):** Raspberry Pi 4 (8GB RAM), Arduino Nano
- **Hardware (Sensors):** RPLidar A1, Kinect V1, MPU6050 / BNO055 (IMU), Motor Encoder, INA226
- **Algorithms:** Extended Kalman Filter (EKF), Pose Graph Optimization (PGO), Model Predictive Path Integral (MPPI)

---

## 🚀 Getting Started

### Prerequisites
* Ubuntu 24.04 LTS
* ROS 2 Jazzy terinstal (*Desktop-Full Recommended*)
* Colcon build tools (`sudo apt install python3-colcon-common-extensions`)

### Installation

1. **Clone the repository:**
```bash
mkdir -p ~/nevo_ws/src
cd ~/nevo_ws/src
git clone [https://github.com/yourusername/nevo-dev.git](https://github.com/yourusername/nevo-dev.git)
Install Dependencies (rosdep):

Bash
cd ~/nevo_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
Build the Workspace:

Bash
colcon build --symlink-install
Source the Environment:

Bash
source install/setup.bash

Make sure to change the `yourusername` placeholder in the clone URL to your actual GitHub username! 

Once you save this in `nano`, you can commit and push it from your terminal:

```bash
git add README.md
git commit -m "docs: finalize README format and tech stack details"
git push origin main
# PX4-ROS2-Kamikaze-Rover-Gazebo

An autonomous **red-target tracking (Kamikaze)** system for differential rovers using **PX4 Autopilot**, **ROS 2**, **Gazebo**, and **OpenCV**.

---
![kamikaze_demo](https://github.com/user-attachments/assets/4613838b-e36a-4da0-be95-a5cf1709fb20)

## 📌 Project Overview

This project demonstrates a modular **"Kamikaze" mission** where a ground rover autonomously detects a **red buoy** in a Gazebo simulation and aggressively intercepts it at high speed.

The system follows a **decoupled architecture**, separating:

* **Perception (Vision)**
* **Control (Actuation)**

into independent **ROS 2 nodes**, ensuring scalability, testability, and modularity.

---

## 🧰 Technical Stack

| Component          | Technology                 |
| ------------------ | -------------------------- |
| Autopilot          | PX4 Autopilot (v1.14+)     |
| Middleware         | Micro XRCE-DDS Agent       |
| Simulation         | Gazebo (Garden / Harmonic) |
| Robotics Framework | ROS 2 (Humble)             |
| Vision             | OpenCV (Python)            |

---

## 🧠 System Architecture & Communication

The system uses a **Publish / Subscribe** communication model.

### 🔍 Vision Node (Perception)

* Subscribes to:

  ```
  /front_camera/image
  ```
* Source: Gazebo camera via ROS–Gazebo bridge
* Processing:

  * HSV color masking
  * Red object detection
  * Target centroid extraction
* Publishes:

  ```
  /vision/target_info
  ```

---

### 🎮 Controller Node (Actuation)

* Subscribes to:

  ```
  /vision/target_info
  ```
* Computes:

  * Horizontal steering error
  * Proportional control law
* Publishes to PX4:

  ```
  /fmu/in/manual_control_input
  ```
* Message type:

  ```
  px4_msgs/msg/ManualControlSetpoint
  ```

---

## 🏗️ Node Interaction Diagram

```
Gazebo Camera
      │
      ▼
/front_camera/image
      │
      ▼
Vision Node (OpenCV)
      │
      ▼
/vision/target_info
      │
      ▼
Controller Node (P-Control)
      │
      ▼
/fmu/in/manual_control_input
      │
      ▼
PX4 Rover Controller
```
<img width="603" height="259" alt="rqt_graph" src="https://github.com/user-attachments/assets/7236ff13-87df-4910-b219-3d7f0e138020" />

---

## 🚀 Execution Guide

To run the mission, open **5 separate terminals** and execute the following steps **in order**.

---

### 1️⃣ PX4 SITL & Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands make px4_sitl gz_rover_differential
```

---

### 2️⃣ Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

---

### 3️⃣ ROS–Gazebo Bridge (Camera)

```bash
ros2 run ros_gz_bridge parameter_bridge \
/front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

---

### 4️⃣ Vision Node (Perception)

```bash
python3 vision_node.py
```

---

### 5️⃣ Controller Node (Action)

```bash
python3 controller_node.py
```

---

## 🇹🇷 Türkçe Açıklama

### Proje Özeti

Bu proje, Gazebo simülasyon ortamında çalışan diferansiyel sürüşlü bir rover'ın **kırmızı bir dubayı otonom olarak tespit etmesini** ve **yüksek hızla hedefe yönelmesini (Kamikaze görevi)** amaçlamaktadır.

Sistem, **Algılama (Görüntü İşleme)** ve **Kontrol (Hareket)** bileşenlerini birbirinden bağımsız **ROS 2 düğümleri** olarak tasarlayan **modüler bir mimariye** sahiptir.

---

### Teknik Altyapı

* **Otopilot:** PX4 Autopilot
* **Haberleşme:** Micro XRCE-DDS Agent
* **Simülasyon:** Gazebo
* **Robotik Framework:** ROS 2 (Humble)
* **Görüntü İşleme:** Python & OpenCV

---

### Sistem Mimarisi

* **Vision Node (Algılama):**

  * `/front_camera/image` topic'inden görüntüyü alır
  * HSV maskeleme ile kırmızı hedefi tespit eder
  * Hedef koordinatlarını `/vision/target_info` topic'ine yayınlar

* **Controller Node (Kontrol):**

  * Hedef koordinatlarını dinler
  * Sapma (hata) hesabı yapar
  * PX4'ün anlayacağı `ManualControlSetpoint` mesajını üretir

---
---

## 📷 PX4 Rover’a Kamera Ekleme (SDF)

PX4 rover modeline ön kamera eklemek için **SDF dosyasında** (`model.sdf`) aşağıdaki iki adım uygulanmalıdır. Kamera, ROS 2–Gazebo bridge üzerinden `/front_camera/image` topic’ine görüntü basacak şekilde yapılandırılmıştır.

---

### 1️⃣ Kamera Linki (Görsel ve Teknik Özellikler)

Bu bölüm kameranın:

* Robot üzerindeki **konumunu**
* **Çözünürlüğünü**
* **Görüş açısını (FOV)**
* Yayınladığı **ROS topic**’ini

tanımlar.

📌 **Bu kodu `<model>` etiketi içinde**, diğer `<link>` tanımlarının yanına ekleyin.

```xml
<link name="camera_link">
  <pose relative_to="base_link">0.35 0 0.25 0 0 0</pose>

  <sensor name="front_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>/front_camera/image</topic>

    <camera>
      <horizontal_fov>1.396</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</link>
```

---

### 2️⃣ Kamera Eklemi (Bağlantı – Joint)

Bu kod, yukarıda oluşturulan `camera_link`’i robotun ana gövdesine (`base_link`) **sabit (fixed)** şekilde bağlar.

📌 **Bu kodu `<joint>` tanımlarının olduğu bölüme ekleyin.**

```xml
<joint name="camera_joint" type="fixed">
  <parent>base_link</parent>
  <child>camera_link</child>
</joint>
```




## ✨ Author

**İbrahim Köse**
PX4 · ROS 2 · Autonomous Systems

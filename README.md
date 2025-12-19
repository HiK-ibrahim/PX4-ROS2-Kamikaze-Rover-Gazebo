# PX4-ROS2-Kamikaze-Rover-Gazebo

An autonomous red-target tracking system for differential rovers using PX4 Autopilot, ROS 2, and OpenCV.

## English Description

### Project Overview
This project demonstrates a modular "Kamikaze" mission where a ground rover autonomously detects a red buoy in a Gazebo simulation and intercepts it at high speed. The system is designed with a decoupled architecture, separating Perception (Vision) and Control (Actuation) into independent ROS 2 nodes.

### Technical Stack
- **Autopilot**: PX4 Autopilot (v1.14+)
- **Middleware**: Micro XRCE-DDS Agent
- **Simulation**: Gazebo (Garden/Harmonic)
- **Communication**: ROS 2 (Humble)
- **Vision**: OpenCV with Python

### System Architecture & Communication
The system utilizes a Publish/Subscribe model to ensure modularity:

1. **Vision Node**: Subscribes to `/front_camera/image` (GZ Bridge), processes frames via HSV masking, and publishes target coordinates to `/vision/target_info`.
2. **Controller Node**: Subscribes to `/vision/target_info`, calculates steering error (Proportional Control), and publishes `ManualControlSetpoint` to PX4 via `/fmu/in/manual_control_input`.

## Türkçe Açıklama

### Proje Özeti
Bu proje, Gazebo simülasyon ortamındaki bir diferansiyel rover'ın, kırmızı bir dubayı otonom olarak tespit etmesini ve yüksek hızla hedefe yönelmesini (Kamikaze görevi) gerçekleştirmektedir. Sistem, Algılama (Görüntü İşleme) ve Kontrol (Hareket) birimlerini birbirinden bağımsız ROS 2 düğümleri (nodes) olarak ayıran modüler bir mimariyle tasarlanmıştır.

### Teknik Altyapı
- **Otopilot**: PX4 Autopilot
- **Haberleşme Katmanı**: Micro XRCE-DDS Agent
- **Simülasyon**: Gazebo
- **Robotik Framework**: ROS 2 (Humble)
- **Görüntü İşleme**: Python & OpenCV

### Sistem Mimarisi ve Haberleşme
Sistem, modülerliği sağlamak için Yayınla/Abone Ol (Publish/Subscribe) modelini kullanır:

1. **Vision Node (Algılama)**: `/front_camera/image` topic'inden gelen görüntüyü alır, HSV maskeleme ile hedefi bulur ve koordinatları `/vision/target_info` topic'ine basar.
2. **Controller Node (Kontrol)**: Hedef koordinatlarını dinler, sapma miktarını (Hata) hesaplar ve PX4'ün anlayacağı `ManualControlSetpoint` mesajını `/fmu/in/manual_control_input` topic'ine iletir.

## Execution Guide / Çalıştırma Rehberi

To run the mission, follow these steps in 5 separate terminals:  
Görevi başlatmak için aşağıdaki adımları 5 ayrı terminalde sırasıyla uygulayın:

1. **PX4 SITL & Gazebo**  
   ```bash
   cd ~/PX4-Autopilot
   PX4_GZ_WORLD=baylands make px4_sitl gz_rover_differential


   Micro XRCE-DDS AgentBashMicroXRCEAgent udp4 -p 8888
ROS-Gazebo BridgeBashros2 run ros_gz_bridge parameter_bridge /front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image
Vision Node (Perception)Bashpython3 vision_node.py
Controller Node (Action)Bashpython3 controller_node.py

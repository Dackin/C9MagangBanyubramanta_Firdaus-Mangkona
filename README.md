# C9MagangBanyubramanta – Firdaus Mangkona

Repositori ini berisi rangkuman proyek dan tugas saya selama mengikuti program magang di **Banyubramanta**. Setiap minggu berfokus pada integrasi teknologi seperti ROS2, OpenCV, YOLO, serial communication, dan simulasi Gazebo.

```mermaid
flowchart TD
    %% Definisi Gaya (Styling)
    classDef hardware fill:none, stroke:none, color:#ff4444, font-weight:bold
    classDef topic fill:none, stroke:none, color:#4488ff, font-weight:bold
    classDef software fill:none, stroke:none, color:#44dd44, font-weight:bold
    classDef nodeBox fill:transparent, stroke:#889, stroke-width:2px, color:#ffffff

    %% Definisi Node & Topic
    Joystick["Joystick"]:::hardware
    Joy_Node["Joy_Node"]:::topic
    ROV_AUV["ROV/AUV ?"]:::topic
    MissionPlanner(["Mission Planner<br>w/ Behavior Tree"]):::nodeBox

    OpenVino(["OpenVino<br>(obj detect)"]):::nodeBox
    detected_obj["detected_obj"]:::topic

    OpenCV(["OpenCV<br>(color detect)"]):::nodeBox
    masked_obj["masked_obj"]:::topic

    image_raw["image_raw"]:::topic
    cmd_vel["cmd_vel"]:::topic
    MicroROS["Micro-ROS"]:::software

    %% Grup STM 32 (Lingkaran Merah)
    subgraph STM32_Group [" "]
        direction TB
        STM32["STM 32"]:::hardware --> Thruster["Thruster"]:::hardware
        Camera["Camera"]:::hardware
    end
    style STM32_Group fill:none, stroke:#ff4444, stroke-width:3px, rx:50, ry:50

    %% Koneksi / Alur Flowchart
    Joystick --> Joy_Node
    Joy_Node --> ROV_AUV
    ROV_AUV --> MissionPlanner

    OpenVino --> detected_obj
    detected_obj --> ROV_AUV

    image_raw --> OpenVino
    image_raw --> OpenCV

    OpenCV --> masked_obj
    masked_obj --> MissionPlanner

    MissionPlanner --> cmd_vel
    cmd_vel --> MicroROS

    MicroROS --> STM32
    Camera --> image_raw
```
---

## Ringkasan Mingguan

### Week 1 – ROS2 Basic Controller
Pada minggu pertama, saya mempelajari dasar komunikasi antar node di ROS2 dan membuat **controllerNode** yang:
- Subscribe ke **joyNode**
- Publish ke **/cmd_vel**
- Mengontrol pergerakan robot/ROV menggunakan joystick

---

### Week 2 – OpenCV Color Detection
Minggu kedua fokus pada pengolahan citra menggunakan **OpenCV**, meliputi:
- Membaca input video
- Melakukan **color filtering** menggunakan HSV masking
- Menampilkan hasil deteksi warna objek

---

### Week 3 – ROS2 + OpenCV + YOLO Integration
Integrasi lanjutan antara ROS2 dan computer vision:
- Menggabungkan OpenCV dan YOLO untuk mendeteksi objek
- Melakukan masking video
- Memproses dan mengirim hasil deteksi antar node ROS2

---

### Week 4.1 – Serial Communication to STM32
Pada bagian pertama Week 4, saya:
- Mengintegrasikan ROS2 controller package dengan **serial communication**
- Mengirim data pergerakan ke **STM32**
- Menguji penerimaan data melalui serial monitor

---

### Week 4.2 – Gazebo Simulation for ROV
Pada bagian kedua Week 4:
- Menghubungkan controller package dengan **Gazebo Simulation**
- Mensimulasikan pergerakan dan kontrol **ROV**
- Menguji komunikasi antar node dalam lingkungan virtual

---

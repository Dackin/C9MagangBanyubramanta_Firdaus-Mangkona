# C9MagangBanyubramanta – Firdaus Mangkona

Repositori ini berisi rangkuman proyek dan tugas saya selama mengikuti program magang di **Banyubramanta**. Setiap minggu berfokus pada integrasi teknologi seperti ROS2, OpenCV, YOLO, serial communication, dan simulasi Gazebo.

flowchart TD
    IMU:::sensor --> VectorNav:::package
    VectorNav --> |Orientation| SensorFusion[Sensor Fusion]:::package
    VectorNav --> |Angular Velocity| SensorFusion

    PressureSensor[Pressure Sensor]:::sensor --> PeripheralArduinoIn[Peripheral Arduino]:::intermediateHardware
    Voltage[Voltage Sensor]:::sensor --> PeripheralArduinoIn

    DVL:::sensor --> OffboardCommsIn[Offboard Comms]:::package
    Gyro:::sensor --> OffboardCommsIn[Offboard Comms]:::package
    IVC:::sensor --> OffboardCommsIn[Offboard Comms]:::package
    PeripheralArduinoIn --> |Depth| OffboardCommsIn
    PeripheralArduinoIn --> |Voltage| OffboardCommsIn

    OffboardCommsIn --> |Linear Velocity| SensorFusion
    OffboardCommsIn --> |Depth| SensorFusion

    FrontCamera[Front Camera]:::sensor --> CV[Computer Vision]:::package
    BottomCamera[Bottom Camera]:::sensor --> CV

    SensorFusion --> |State| Controls:::package
    SensorFusion --> |State| TaskPlanning[Task Planning]:::package
    CV --> |Object Detections| TaskPlanning
    Ping360:::sensor --> Sonar:::package
    Sonar --> |Object Poses| TaskPlanning

    Hydrophones:::sensor --> Acoustics:::package
    Acoustics --> |Pinger Positions| TaskPlanning

    TaskPlanning --> |Desired State| Controls
    TaskPlanning --> |Servo Commands| OffboardCommsOut[Offboard Comms]:::package
    Controls --> |Thruster Allocations| OffboardCommsOut
    OffboardCommsOut --> |Pulse Widths| ThrusterArduino[Thruster Arduino]:::intermediateHardware
    OffboardCommsOut --> |Servo Angles| PeripheralArduinoOut[Peripheral Arduino]:::intermediateHardware
    ThrusterArduino --> Thrusters:::outputs

    PeripheralArduinoOut --> MarkerDropperServo[Marker Dropper Servo]:::outputs
    PeripheralArduinoOut --> TorpedoServo[Torpedo Servo]:::outputs

    classDef sensor fill:#d94, color:#fff
    classDef package fill:#00c, color:#fff
    classDef outputs fill:#080, color:#fff
    classDef intermediateHardware fill:#990, color:#fff

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

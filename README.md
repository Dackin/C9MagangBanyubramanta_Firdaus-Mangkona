# C9MagangBanyubramanta_Firdaus-Mangkona

Repositori ini berisi kumpulan proyek dan tugas yang diselesaikan selama program magang saya di Banyubramanta. Fokus utama pekerjaan ini adalah pada pengembangan robotika menggunakan ROS 2, Computer Vision dengan OpenCV, dan integrasi simulasi.

---

Berikut adalah rincian pekerjaan yang diselesaikan setiap minggunya, beserta lokasi kode di dalam repositori ini.

### Minggu 1: Kontroler Dasar ROS 2
* **Lokasi:** `src/controller`
* **Deskripsi:** Membuat node ROS 2 (`controllerNode`) yang berfungsi sebagai jembatan antara input joystick dan kontrol robot. Node ini berlangganan (subscribe) ke topik dari `joyNode` (input joystick) dan mempublikasikan (publish) pesan `Object.msg` ke topik `/cmd_vel` untuk menggerakkan robot.

### Minggu 2: Deteksi Warna OpenCV
* **Lokasi:** `opencv/`
* **Deskripsi:** Sebuah program C++ / Python yang menggunakan library OpenCV untuk melakukan deteksi objek berdasarkan warna. Program ini dapat memproses file video atau input kamera, mengisolasi objek dengan warna tertentu (misalnya, merah, biru, hijau), dan menampilkannya.

### Minggu 3: Integrasi ROS 2, OpenCV, dan YOLO/OpenVINO
* **Lokasi:** `src/opencvros2` dan `src/openvinoros2`
* **Deskripsi:** Mengintegrasikan pipeline Computer Vision yang lebih kompleks dengan ROS 2. Proyek ini membuat beberapa node yang saling berkomunikasi:
    1.  Satu node mem-publish data video/kamera.
    2.  Node perantara melakukan *masking* atau pra-pemrosesan gambar menggunakan OpenCV.
    3.  Node terakhir menjalankan deteksi objek (object detection) menggunakan model YOLO (atau diakselerasi dengan OpenVINO) pada gambar yang telah diproses.

### Minggu 4: Integrasi Serial dan Simulasi

Pada minggu keempat, fokusnya adalah menghubungkan node kontroler yang dibuat pada Minggu 1 ke dua target berbeda: perangkat keras (hardware) dan simulasi.

* **1. Komunikasi Serial (Hardware Interface)**
    * **Lokasi:** `src/sercom`
    * **Deskripsi:** Mengintegrasikan `controllerNode` ROS 2 dengan komunikasi serial (menggunakan `Boost.Asio` di C++). Node ini berlangganan ke topik `/cmd_vel` dan menerjemahkan perintah tersebut menjadi string atau format data tertentu yang kemudian dikirim melalui port serial. Ini adalah langkah penting untuk mengizinkan ROS 2 berkomunikasi dengan perangkat keras fisik seperti mikrokontroler (Arduino, ESP32, dll.).

* **2. Simulasi Gazebo (Virtual Interface)**
    * **Lokasi:** (Sertakan lokasi jika ada, misal `src/simulation_launch`)
    * **Deskripsi:** Menghubungkan `controllerNode` ROS 2 dengan robot virtual di dalam simulasi Gazebo. Ini memungkinkan pengujian logika kontrol dan fungsionalitas node dalam lingkungan virtual yang aman dan terkendali sebelum diterapkan pada robot fisik.

---

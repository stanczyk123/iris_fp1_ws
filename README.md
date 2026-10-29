# IRIS Internship — Final Project 1: Autonomous Car Simulation

Program *autonomous car* untuk simulasi Gazebo (FIRA Autonomous Cars Simulator),
dibangun di atas ROS 2 Jazzy. Mobil memakai **Ackermann steering** dan
dikendalikan lewat **UDP**, bukan ROS topic.

Kemampuan yang sudah berjalan:

- Mengirim dan menerima paket UDP ke/dari simulasi
- Kendali manual lewat keyboard (WASD)
- Odometry dan *motion-to-point* dari perhitungan kinematika
- *Thresholding* dan *Bird-Eye View* dari kamera depan
- Pencarian titik belok dari gambar BEV
- Mengikuti lajur secara otonom
- Berhenti otomatis saat LiDAR mendeteksi halangan di depan

---

## Arsitektur

Tiap package punya satu tanggung jawab, dan semuanya terhubung lewat ROS topic.
`vision` tidak tahu apa-apa soal UDP, `communication` tidak tahu apa-apa soal
kamera. Pembagian ini yang membuat mode otonom penuh cukup dijalankan dengan
menyalakan ketiga package bersamaan.

```
                kamera                     LiDAR
         (sensor_msgs/Image)        (sensor_msgs/LaserScan)
                  |                           |
          +-------v---------------------------v-------+
          |                 vision                    |
          |  vision_node : threshold -> BEV -> target |
          |  lidar_node  : sektor depan -> jarak      |
          +-------+---------------------------+-------+
                  | /turn_target              | /obstacle_distance
                  |                           |
          +-------v---------------------------v-------+
          |                 master                    |
          |  odometry + kinematika + pengaman         |
          +---------------------+---------------------+
                                | /velocity_command
                                |          ^ /car_feedback
          +---------------------v----------+----------+
          |              communication                |
          |  udp_sender_node  / udp_receiver_node     |
          |  keyboard_teleop_node (manual)            |
          +---------------------+---------------------+
                                | UDP 12151 / 12152
                       +--------v--------+
                       | SIMULASI GAZEBO |
                       +-----------------+
```

---

## Prasyarat

```bash
sudo apt install -y podman ros-jazzy-cv-bridge libopencv-dev
sudo systemctl disable ufw      # firewall dapat memblokir koneksi ROS
```

Build workspace:

```bash
cd ~/iris_fp1_ws
colcon build
source install/setup.bash
```

---

## Cara menjalankan

Setiap terminal baru wajib diawali dua baris ini:

```bash
cd ~/iris_fp1_ws
source install/setup.bash
```

Jalankan **berurutan**, satu node per terminal. Nyalakan `master_node`
paling akhir, saat mobil masih diam — titik (0,0) odometry ditentukan dari
posisi mobil ketika node itu dinyalakan.

### Terminal 1 — Simulasi

```bash
cd ~/iris_fp1_ws
./run_simulation.sh
```

Tunggu Gazebo terbuka penuh. Pastikan `Real Time Factor` sekitar 1.00
(kalau 0, simulasi sedang *pause* — tekan tombol play di bawah).

### Terminal 2 — UDP receiver

```bash
cd ~/iris_fp1_ws
ros2 run communication udp_receiver_node --ros-args -p listen_port:=12152
```

Tunggu sampai muncul `Paket pertama diterima!`

### Terminal 3 — UDP sender

```bash
cd ~/iris_fp1_ws
ros2 run communication udp_sender_node --ros-args -p sim_port:=12151
```

### Terminal 4 — LiDAR

```bash
cd ~/iris_fp1_ws
ros2 run vision lidar_node
```

### Terminal 5 — Vision

```bash
cd ~/iris_fp1_ws
ros2 run vision vision_node
```

Empat jendela OpenCV akan muncul. Titik hijau di jendela *Bird Eye View*
adalah target belok yang sedang diikuti.

### Terminal 6 — Master

```bash
cd ~/iris_fp1_ws
ros2 run master master_node --ros-args -p mode:=follow
```

---

## Mode kendali

### Otonom — mengikuti lajur

```bash
ros2 run master master_node --ros-args -p mode:=follow
```

Butuh `vision_node` berjalan.

### Motion-to-point — menuju koordinat

```bash
ros2 run master master_node --ros-args -p mode:=point -p target_x:=10.0 -p target_y:=0.0
```

Target dinyatakan relatif terhadap arah hadap awal mobil: `target_x` adalah
berapa meter maju, `target_y` berapa meter ke kiri. Tidak butuh `vision_node`.

Untuk memakai koordinat dunia, tambahkan `-p target_relatif:=false`.

### Manual — keyboard

```bash
ros2 run communication keyboard_teleop_node
```

| Tombol | Fungsi |
|---|---|
| `W` / `S` | tambah / kurang PWM (`S` juga untuk mundur) |
| `A` / `D` | belok kiri / kanan |
| `X` | luruskan setir |
| `SPASI` | berhenti |
| `Q` | keluar |

Terminal harus dalam keadaan fokus agar tombol terbaca.

> **Jangan menjalankan `keyboard_teleop_node` bersamaan dengan `master_node`.**
> Keduanya mempublikasikan ke `/velocity_command`, dan perintahnya akan bentrok.

---

## Parameter

### `master_node`

| Parameter | Default | Keterangan |
|---|---|---|
| `mode` | `point` | `point` atau `follow` |
| `target_x`, `target_y` | 10.0, 0.0 | target mode point (meter) |
| `target_relatif` | `true` | relatif arah awal mobil, atau koordinat dunia |
| `steering_gain` | 30.0 | penguatan steering mode point |
| `toleransi` | 0.5 | jarak yang dianggap sudah sampai (meter) |
| `follow_gain` | 45.0 | derajat steering saat target di tepi gambar |
| `obstacle_slow` | 10.0 | mulai melambat di jarak ini (meter) |
| `obstacle_stop` | 5.0 | berhenti total di jarak ini (meter) |
| `pwm_jalan` | 90 | PWM jelajah |
| `pwm_min` | 50 | PWM minimum saat bergerak |

### `vision_node`

Parameter HSV dan Bird-Eye View diatur lewat *slider* di jendela **Tuning**.
Tekan `s` untuk mencetak nilai saat ini ke terminal, `q` untuk keluar.

Nilai hasil kalibrasi yang sudah menjadi default:

| Parameter | Nilai |
|---|---|
| HSV | H 0–179, S 0–40, V 180–255 |
| BEV atas / bawah lebar | 20% / 95% |
| BEV atas / bawah Y | 60% / 95% |

### `lidar_node`

| Parameter | Default | Keterangan |
|---|---|---|
| `scan_topic` | `/catvehicle/front_laser_points` | topic LaserScan |
| `fov_deg` | 40.0 | lebar sektor depan yang diperiksa |
| `jarak_max_valid` | 30.0 | di atas ini dianggap tidak ada halangan |

---

## Protokol UDP

Struktur paket **tanpa padding**, little-endian. Struct memakai
`__attribute__((packed))` dan diverifikasi dengan `static_assert` saat compile.

### Perintah kecepatan — program ke simulasi (port 12151)

| Field | Tipe | Rentang |
|---|---|---|
| Header | `char[4]` | wajib `IRIS` |
| Motor PWM | `int32` | −255 … 255 |
| Steering degree | `int32` | −90 … 90 |

Total 12 byte.

### Feedback — simulasi ke program (port 12152)

| Field | Tipe |
|---|---|
| Header | `char[4]` (`IRIS`) |
| Left wheel encoder | `int32` |
| Right wheel encoder | `int32` |
| X velocity (cm/s) | `int32` |
| Y velocity (cm/s) | `int32` |

Total 20 byte.

---

## Topic

| Topic | Tipe | Dari | Ke |
|---|---|---|---|
| `/velocity_command` | `geometry_msgs/Twist` | master, teleop | udp_sender |
| `/car_feedback` | `std_msgs/Int32MultiArray` | udp_receiver | master |
| `/turn_target` | `geometry_msgs/Point` | vision_node | master |
| `/obstacle_distance` | `std_msgs/Float32` | lidar_node | master |
| `/odom_2d` | `geometry_msgs/Pose2D` | master | — |

Pada `/velocity_command`, `linear.x` dipakai sebagai PWM dan `angular.z`
sebagai sudut steering dalam derajat.

---

## Catatan desain

**Heading diambil dari `atan2(vel_y, vel_x)`, bukan dari selisih encoder.**
Rumus odometry *differential drive* tidak cocok untuk mobil ini: belokan datang
dari roda depan, sehingga selisih putaran roda kiri-kanan terlalu kecil dan
tenggelam oleh derau. Kecepatan yang dikirim simulasi ternyata dinyatakan dalam
koordinat dunia — besarnya tetap sementara arahnya berputar saat mobil
berbelok — sehingga arah vektornya langsung dapat dipakai sebagai heading,
tanpa konstanta yang perlu dikalibrasi.

**Tanda steering dibalik di mode point, tidak di mode follow.** Error sudut pada
odometry memakai konvensi dunia (positif berarti target di kiri), sedangkan
simulasi menerima steering positif sebagai belok kanan. Offset piksel dari BEV
sudah berada dalam kerangka mobil (positif berarti target di kanan), jadi tidak
perlu dibalik.

**Pengaman halangan diterapkan di dalam `kirim()`.** Fungsi itu satu-satunya
jalan keluar perintah, sehingga mode `point` maupun `follow` sama-sama
terlindungi tanpa logika ganda, dan tidak ada jalur yang bisa melewatinya.
Pengereman hanya berlaku saat PWM positif, agar mobil masih bisa mundur untuk
lepas bila terlanjur mepet.

**Ambang berhenti 5 meter, bukan 1 meter.** `range_min` LiDAR adalah 1.5 m —
apa pun yang lebih dekat dari itu tidak terbaca sama sekali. Ambang yang terlalu
kecil membuat halangan justru menghilang dari pandangan tepat sebelum tabrakan.

**Nomor port dan topic dibuat sebagai ROS parameter, bukan konstanta.** Nilainya
dapat diganti dari baris perintah tanpa *rebuild*, yang berguna saat spesifikasi
simulasi masih berubah.

---

## Menyinkronkan dengan repositori IRIS

```bash
cd ~/iris_fp1_ws
git fetch upstream
git merge upstream/main
./update_template_workspace.sh
./update_simulation.sh
```

Jika simulasi tetap memakai versi lama setelah update, hapus container dan
image-nya agar `run_simulation.sh` menarik ulang:

```bash
podman rm -f fira_simulation_ros2_docker
podman rmi fira_simulation_ros2_docker
./run_simulation.sh
```
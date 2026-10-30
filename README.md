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
- Mengenali **jenis garis** kiri dan kanan: putus-putus atau solid
- Mengikuti lajur secara otonom
- **Menghindari halangan dengan FSM** — berpindah lajur, bukan sekadar berhenti
- Garis solid diperlakukan sebagai **batas keras**: mobil mundur dan menata
  ulang arah bila hampir melewatinya
- Dashboard web lokal untuk menjalankan seluruh node dari satu tombol

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
          |                jenis garis kiri/kanan     |
          |  lidar_node  : koridor depan & samping    |
          +-------+---------------------------+-------+
                  | /turn_target              | /obstacle_distance
                  | /lane_info                | /obstacle_sectors
                  |                           |
          +-------v---------------------------v-------+
          |                 master                    |
          |  odometry + kinematika + FSM penghindaran |
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

> **Build dan jalankan dari terminal Ubuntu biasa (`Ctrl+Alt+T`), bukan dari
> terminal bawaan VS Code.** Terminal VS Code versi Snap menambahkan `/snap/`
> ke `LD_LIBRARY_PATH`, dan node akan gagal dengan
> `undefined symbol: __libc_pthread_init`. VS Code tetap dipakai untuk
> mengedit kode — hanya menjalankannya yang harus dari terminal biasa.

---

## Cara menjalankan — dashboard

Cara tercepat. Satu terminal, satu tombol.

```bash
cd ~/iris_fp1_ws
source install/setup.bash
python3 demo_dashboard.py
```

Lalu buka **http://localhost:8080** di browser (`http`, bukan `https`).

Dashboard ini hanya memakai pustaka standar Python — tidak ada yang perlu
di-install. Ia menjalankan perintah `ros2 run` yang persis sama dengan cara
manual di bawah, jadi tidak ada perilaku yang berubah.

### Isi halaman

| Bagian | Fungsi |
|---|---|
| **START SEMUA** | menyalakan kelima node berurutan dengan jeda yang benar |
| **Stop semua** | menghentikan semuanya, `master_node` lebih dulu agar mobil berhenti |
| **Nyalakan simulasi Gazebo** | menghapus container lama lalu menjalankan `run_simulation.sh` |
| **Keadaan mobil** | keadaan FSM, jarak halangan, PWM, setir, offset, geseran, jenis garis |
| **Node** | lampu status, PID, dan log masing-masing node; tiap node bisa dinyalakan sendiri |

Keadaan FSM ditampilkan besar dengan warnanya sendiri: `IKUTI` hijau,
`HINDAR` kuning, `KEMBALI` biru, `KOREKSI` ungu, `BERHENTI` merah.

### Langkah demo

1. Jalankan dashboard, buka `http://localhost:8080`.
   Kalau muncul spanduk merah **Lingkungan belum siap**, beresin dulu isinya —
   biasanya workspace belum di-*source*.
2. Klik **Nyalakan simulasi Gazebo**, tunggu Gazebo terbuka penuh.
3. Di Gazebo: pastikan `Real Time Factor` ≈ 1.00 (kalau 0, tekan play),
   lalu `Edit → Reset Model Poses`, dan taruh kotak halangan minimal
   25 meter di depan mobil.
4. Kembali ke browser, tekan **START SEMUA** saat mobil masih diam.
5. Mengulang percobaan: `Stop semua` → reset pose di Gazebo → `START SEMUA`.
   Simulasi Gazebo tidak perlu dimatikan.

`Ctrl+C` di terminal dashboard mematikan seluruh node dengan rapi.

---

## Cara menjalankan — manual

Cara lama, enam terminal. Berguna sebagai cadangan dan untuk menelusuri
masalah satu per satu.

Setiap terminal baru wajib diawali dua baris ini:

```bash
cd ~/iris_fp1_ws
source install/setup.bash
```

Jalankan **berurutan**, satu node per terminal. Nyalakan `master_node`
paling akhir, saat mobil masih diam — titik (0,0) odometry ditentukan dari
posisi mobil ketika node itu dinyalakan.

| # | Perintah | Tunggu sampai |
|---|---|---|
| 1 | `./run_simulation.sh` | Gazebo terbuka penuh, RTF ≈ 1.00 |
| 2 | `ros2 run communication udp_receiver_node --ros-args -p listen_port:=12152` | muncul `Paket pertama diterima!` |
| 3 | `ros2 run communication udp_sender_node --ros-args -p sim_port:=12151` | — |
| 4 | `ros2 run vision lidar_node --ros-args -p lebar_koridor:=2.2` | muncul scan pertama |
| 5 | `ros2 run vision vision_node` | empat jendela OpenCV terbuka |
| 6 | `ros2 run master master_node --ros-args -p mode:=follow` | — |

Bila `run_simulation.sh` menolak jalan karena container lama tertinggal:

```bash
podman rm -f fira_simulation_ros2_docker
```

---

## Mode kendali

### Otonom — mengikuti lajur

```bash
ros2 run master master_node --ros-args -p mode:=follow
```

Butuh `vision_node` berjalan. Penghindaran halangan ikut aktif bila
`lidar_node` juga berjalan.

Mematikan penghindaran, hanya mengikuti lajur:

```bash
ros2 run master master_node --ros-args -p mode:=follow -p hindar_aktif:=false
```

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

## Penghindaran halangan (FSM)

```
                     ┌───────────────┐
      ┌─────────────>│  IKUTI_LAJUR  │<─────────────┐
      │              └───────┬───────┘              │
      │       halangan < jarak_hindar               │
      │                      v                      │
      │              ┌───────────────┐              │
      │              │  MENGHINDAR   │              │
      │              └───────┬───────┘              │
      │       jalur depan bersih (histeresis 1.6x)  │
      │                      v                      │
      │              ┌───────────────┐              │
      │              │    KEMBALI    │──────────────┘
      │              └───────────────┘   halangan terlewati
      │
      │   terlalu dekat garis solid — menyela keadaan mana pun
      │              ┌───────────────┐
      └──────────────│    KOREKSI    │  mundur lalu menata ulang
                     └───────────────┘

                     ┌───────────────┐
                     │   BERHENTI    │  tidak ada sisi yang boleh dilewati
                     └───────────────┘
```

Dua mekanisme menjaga FSM tidak bergetar: **histeresis** (ambang masuk dan
keluar sengaja berbeda) dan **dwell** (setiap keadaan wajib bertahan minimal
`dwell_min` detik sebelum boleh berpindah, kecuali menuju `BERHENTI`).

Arah menghindar dipilih dari **posisi halangan**, bukan dari sisi mana yang
paling lega — mobil menyingkir menjauhi halangan. Sisi yang dibatasi garis
solid ditolak mentah-mentah.

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
| `lebar_gambar` | 800 | lebar gambar BEV, piksel |
| `obstacle_slow` | 8.0 | mulai melambat di jarak ini (meter) |
| `obstacle_stop` | 5.0 | berhenti total di jarak ini (meter) |
| `pwm_jalan` | 90 | PWM jelajah |
| `pwm_min` | 50 | PWM minimum saat bergerak |

Penghindaran:

| Parameter | Default | Keterangan |
|---|---|---|
| `hindar_aktif` | `true` | menyalakan seluruh FSM penghindaran |
| `jarak_hindar` | 15.0 | mulai menyingkir di jarak ini (meter) |
| `jarak_hindar_lagi` | 9.0 | ambang lebih ketat untuk halangan berikutnya saat mobil sedang pulang |
| `jarak_bebas` | 7.0 | halangan dianggap masih di samping bila lebih dekat dari ini |
| `jarak_lewat` | 6.0 | jarak tempuh menahan geseran setelah halangan terlewati |
| `offset_hindar` | 300.0 | besar geseran target, piksel BEV |
| `bias_hindar` | 18.0 | dorongan setir langsung saat menghindar, derajat |
| `laju_ramp` | 500.0 | kecepatan perubahan geseran, piksel/detik |
| `dwell_min` | 1.5 | lama minimum bertahan di satu keadaan, detik |
| `stop_saat_hindar` | 1.8 | ambang rem saat bermanuver (meter) |
| `sabar_berhenti` | 2.5 | lama menunggu sebelum mundur dari `BERHENTI`, detik |
| `pwm_mundur` | -60 | PWM saat mundur |

Garis dan lajur:

| Parameter | Default | Keterangan |
|---|---|---|
| `hormati_garis` | `true` | menolak menyeberangi garis solid |
| `putus_di_kiri` | `true` | susunan garis pada lajur yang benar |
| `margin_garis` | 70.0 | jarak aman dari garis solid, piksel BEV |
| `ambang_garis` | 55.0 | lebih dekat dari ini memicu `KOREKSI` |
| `jarak_koreksi` | 4.0 | jarak mundur maksimum saat `KOREKSI`, meter |
| `pulang_lajur` | `true` | kembali ke lajur asal setelah melewati halangan |
| `batas_pulang` | 25.0 | jarak tempuh maksimum untuk mendorong pulang, meter |
| `batas_tahan` | 25.0 | batas aman menahan geseran, meter |
| `cacah_mantap` | 20 | siklus (50 ms) pembacaan garis harus konsisten sebelum keputusan berubah |

### `vision_node`

Parameter HSV dan Bird-Eye View diatur lewat *slider* di jendela **Tuning**.
Tekan `s` untuk mencetak nilai saat ini ke terminal, `q` untuk keluar.

Nilai hasil kalibrasi yang sudah menjadi default:

| Parameter | Nilai | Keterangan |
|---|---|---|
| HSV | H 0–179, S 0–40, V 180–255 | garis putih, tidak peduli rona |
| BEV atas / bawah lebar | 60% / 100% | lebar trapesium sumber |
| BEV atas / bawah Y | 70% / 83% | batas bawah 83% memotong rumah sensor LiDAR di kap mobil |
| Zoom out | 20% | memberi margin, sehingga area di luar trapesium ikut terlihat |
| Baris pandang | 62% | tinggi pita tempat target belok dicari |
| Tinggi pita | 30 px | — |
| Lebar lajur | 32 | setengah lebar lajur, piksel BEV |
| Minimum piksel | 60 | ambang sebuah puncak dianggap garis |

Garis disebut **solid** bila lebih dari 80% jendela penelusurannya terisi;
selebihnya **putus-putus**. Jendela yang menyentuh tepi gambar dilewati, agar
garis yang terpotong frame tidak salah dinilai putus-putus.

### `lidar_node`

| Parameter | Default | Keterangan |
|---|---|---|
| `scan_topic` | `/catvehicle/front_laser_points` | topic LaserScan |
| `lebar_koridor` | 2.6 | lebar koridor depan, meter (**dipakai 2.2** saat demo) |
| `lebar_samping` | 4.0 | batas luar sektor samping, meter |
| `margin_samping` | 0.9 | celah antara koridor dan sektor samping, meter |
| `wheelbase` | 2.6 | jarak sumbu roda, meter |
| `faktor_setir` | 0.45 | derajat setir → derajat roda depan |
| `sudut_positif_kiri` | `true` | konvensi sudut scan |
| `jarak_min_valid` | 0.15 | di bawah ini diabaikan |
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
| `/velocity_command` | `geometry_msgs/Twist` | master, teleop | udp_sender, lidar_node |
| `/car_feedback` | `std_msgs/Int32MultiArray` | udp_receiver | master |
| `/turn_target` | `geometry_msgs/Point` | vision_node | master |
| `/lane_info` | `std_msgs/Float32MultiArray` | vision_node | master |
| `/obstacle_distance` | `std_msgs/Float32` | lidar_node | master |
| `/obstacle_sectors` | `std_msgs/Float32MultiArray` | lidar_node | master |
| `/odom_2d` | `geometry_msgs/Pose2D` | master | — |

Pada `/velocity_command`, `linear.x` dipakai sebagai PWM dan `angular.z`
sebagai sudut steering dalam derajat.

`/turn_target` memakai `x` sebagai posisi target dalam piksel, `y` sebagai
baris pita pencarian, dan `z` sebagai penanda sah atau tidak.

`/lane_info` berisi tujuh elemen: target x, sah atau tidak, jenis garis kiri,
jenis garis kanan, lebar lajur terukur, posisi garis kiri, posisi garis kanan.
Jenis garis: `0` tidak ada, `1` putus-putus, `2` solid.

`lidar_node` ikut berlangganan `/velocity_command` agar tahu sudut setir saat
ini, dan dapat melengkungkan koridornya mengikuti jalur yang akan ditempuh.

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

**Halangan dicari dalam koridor persegi, bukan kerucut sudut.** Kerucut melebar
seiring jarak, sehingga pagar di tepi jalan ikut terbaca sebagai halangan di
depan. Koridor selebar mobil menjawab pertanyaan yang sebenarnya: apakah ada
sesuatu di jalur yang akan dilewati. Koridor itu ikut **melengkung** mengikuti
jari-jari belok hasil model sepeda `R = wheelbase / tan(sudut_roda)`, sehingga
saat menikung mobil tidak panik oleh dinding di sisi luar tikungan.

**Garis solid adalah batas geometris, bukan sekadar pertimbangan.** Target
steering selalu dipotong agar tidak pernah melewati garis solid yang terbaca,
dan bila mobil tetap terlalu mepet, keadaan `KOREKSI` menyela keadaan mana pun:
mobil mundur lurus lalu membiarkan pengikut lajur menata arahnya lagi. Mundur
dilakukan dengan setir lurus, karena menyetir saat mundur membuat buritan
melebar ke arah yang sulit diperkirakan.

**Keputusan "di lajur seberang" diredam dulu.** Pembacaan garis berkedip-kedip:
satu detik terbaca benar, detik berikutnya terbalik, lalu hilang sama sekali.
Keputusan baru berubah setelah bacaannya konsisten selama `cacah_mantap`
siklus. Tanpa peredam ini, satu kedipan "normal" di saat yang salah pernah
membuat FSM berhenti mengoreksi padahal mobil masih berada di lajur seberang.

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
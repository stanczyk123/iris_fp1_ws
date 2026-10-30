#!/usr/bin/env python3
"""
Dashboard demo IRIS FP1 - Autonomous Car Simulation
===================================================

Menjalankan seluruh node dari satu tombol, dan menampilkan keadaan mobil
secara langsung: keadaan FSM, jarak halangan, jenis garis kiri-kanan,
PWM dan sudut setir.

Dijalankan DI LAPTOP SENDIRI, bukan di web. Server kecil ini yang
men-spawn proses `ros2 run`, jadi harus dijalankan dari terminal yang
environment ROS-nya sudah di-source.

    cd ~/iris_fp1_ws
    source install/setup.bash
    python3 demo_dashboard.py

Lalu buka http://localhost:8080 di browser.

PENTING: jalankan dari terminal biasa (Ctrl+Alt+T), BUKAN terminal
bawaan VS Code. Terminal VS Code versi Snap mengotori LD_LIBRARY_PATH
dan node-nya akan gagal dengan error __libc_pthread_init.

Hanya memakai pustaka standar Python. Tidak ada yang perlu di-install.
"""

import http.server
import json
import os
import re
import signal
import socketserver
import subprocess
import sys
import threading
import time
from collections import deque

PORT = 8080
MAKS_BARIS_LOG = 250

# ----------------------------------------------------------------------
# Daftar node, beserta jeda sebelum node berikutnya dinyalakan.
#
# Urutannya tidak boleh ditukar. `master_node` wajib paling akhir, saat
# mobil masih diam - titik (0,0) odometry ditentukan dari posisi mobil
# ketika node itu menyala.
# ----------------------------------------------------------------------
DAFTAR_NODE = [
    {
        "nama": "receiver",
        "judul": "UDP Receiver",
        "ket": "Menerima feedback dari simulasi (port 12152)",
        "perintah": ["ros2", "run", "communication", "udp_receiver_node",
                     "--ros-args", "-p", "listen_port:=12152"],
        "jeda": 2.5,
    },
    {
        "nama": "sender",
        "judul": "UDP Sender",
        "ket": "Mengirim perintah kecepatan ke simulasi (port 12151)",
        "perintah": ["ros2", "run", "communication", "udp_sender_node",
                     "--ros-args", "-p", "sim_port:=12151"],
        "jeda": 1.5,
    },
    {
        "nama": "lidar",
        "judul": "LiDAR",
        "ket": "Koridor depan dan samping, mengikuti lengkung jalur",
        "perintah": ["ros2", "run", "vision", "lidar_node",
                     "--ros-args", "-p", "lebar_koridor:=2.2"],
        "jeda": 1.5,
    },
    {
        "nama": "vision",
        "judul": "Vision",
        "ket": "Threshold, Bird-Eye View, titik belok, jenis garis",
        "perintah": ["ros2", "run", "vision", "vision_node"],
        "jeda": 3.0,
    },
    {
        "nama": "master",
        "judul": "Master",
        "ket": "Odometry, kinematika, FSM penghindaran",
        "perintah": None,          # dibangun saat start, ikut pilihan di UI
        "jeda": 0.0,
    },
]

# Container lama kerap tertinggal bila Gazebo ditutup paksa, dan
# run_simulation.sh menolak jalan selama namanya masih terpakai. Jadi
# sisa container dibersihkan dulu, setiap kali.
PERINTAH_SIMULASI = [
    "bash", "-c",
    "podman rm -f fira_simulation_ros2_docker 2>/dev/null; exec ./run_simulation.sh",
]


# ----------------------------------------------------------------------
# Pembacaan baris log master_node menjadi angka yang bisa ditampilkan
# ----------------------------------------------------------------------
RE_UTAMA = re.compile(
    r"\]:\s*(?P<state>\S+)\s*\|\s*offset=(?P<offset>[+-]?\d+)\s*px\s*"
    r"geser=(?P<geser>[+-]?\d+)"
)
RE_PWM = re.compile(r"pwm=(?P<pwm>-?\d+)\s+steer=(?P<steer>-?\d+)")
RE_LIDAR = re.compile(
    r"lidar\s+D(?P<d>[\d.]+)\((?P<sd>[+-][\d.]+)\)\s+"
    r"K(?P<k>[\d.]+)\s+N(?P<n>[\d.]+)"
)
RE_GARIS = re.compile(r"garis\s+K:(?P<gk>\S+)\s+N:(?P<gn>\S+)")

NAMA_KEADAAN = {"IKUTI", "HINDAR", "KEMBALI", "BERHENTI", "KOREKSI"}


class Telemetri:
    """Keadaan mobil terakhir yang terbaca, dijaga satu kunci."""

    def __init__(self):
        self.kunci = threading.Lock()
        self.data = {
            "keadaan": "-", "offset": None, "geser": None,
            "pwm": None, "steer": None,
            "depan": None, "samping_depan": None, "kiri": None, "kanan": None,
            "garis_kiri": "-", "garis_kanan": "-",
            "lajur_seberang": False, "pulang": False, "rem": False,
            "lihat": None, "umur": None,
        }
        self.saat = 0.0

    def baca_baris(self, baris):
        m = RE_UTAMA.search(baris)
        if not m or m.group("state") not in NAMA_KEADAAN:
            return

        with self.kunci:
            d = self.data
            d["keadaan"] = m.group("state")
            d["offset"] = int(m.group("offset"))
            d["geser"] = int(m.group("geser"))

            mp = RE_PWM.search(baris)
            if mp:
                d["pwm"] = int(mp.group("pwm"))
                d["steer"] = int(mp.group("steer"))

            ml = RE_LIDAR.search(baris)
            if ml:
                d["depan"] = float(ml.group("d"))
                d["samping_depan"] = float(ml.group("sd"))
                d["kiri"] = float(ml.group("k"))
                d["kanan"] = float(ml.group("n"))
            elif "lidar: -" in baris:
                d["depan"] = d["kiri"] = d["kanan"] = None

            mg = RE_GARIS.search(baris)
            if mg:
                d["garis_kiri"] = mg.group("gk")
                d["garis_kanan"] = mg.group("gn")

            d["lajur_seberang"] = "[LAJUR SEBERANG]" in baris
            d["pulang"] = "[PULANG]" in baris
            d["rem"] = "[REM]" in baris
            d["lihat"] = ("[lihat]" in baris) if "[lihat]" in baris or "[tebak]" in baris else None

            self.saat = time.time()

    def ambil(self):
        with self.kunci:
            d = dict(self.data)
            d["umur"] = round(time.time() - self.saat, 1) if self.saat else None
            return d


TELEMETRI = Telemetri()


# ----------------------------------------------------------------------
# Satu proses yang diawasi
# ----------------------------------------------------------------------
class Proses:
    def __init__(self, nama, judul, ket):
        self.nama = nama
        self.judul = judul
        self.ket = ket
        self.popen = None
        self.log = deque(maxlen=MAKS_BARIS_LOG)
        self.kunci = threading.Lock()
        self.perintah_terakhir = None

    def jalan(self):
        return self.popen is not None and self.popen.poll() is None

    def kode_keluar(self):
        if self.popen is None:
            return None
        return self.popen.poll()

    def catat(self, baris):
        with self.kunci:
            self.log.append(baris.rstrip("\n"))

    def start(self, perintah, cwd):
        if self.jalan():
            return False, "sudah berjalan"

        self.perintah_terakhir = perintah
        self.catat("$ " + " ".join(perintah))
        try:
            self.popen = subprocess.Popen(
                perintah,
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                # Grup proses sendiri, supaya seluruh anak-anaknya ikut mati
                # saat dihentikan. `ros2 run` memanggil binary lain, jadi
                # membunuh PID-nya saja meninggalkan node tetap hidup.
                start_new_session=True,
            )
        except FileNotFoundError as e:
            self.catat("GAGAL: %s" % e)
            return False, str(e)

        t = threading.Thread(target=self._baca, daemon=True)
        t.start()
        return True, "ok"

    def _baca(self):
        p = self.popen
        try:
            for baris in p.stdout:
                self.catat(baris)
                if self.nama == "master":
                    TELEMETRI.baca_baris(baris)
        except Exception:
            pass
        kode = p.wait()
        self.catat("--- proses berhenti (kode %s) ---" % kode)

    def stop(self):
        if not self.jalan():
            return False, "tidak berjalan"

        pgid = os.getpgid(self.popen.pid)
        try:
            os.killpg(pgid, signal.SIGINT)   # Ctrl+C, agar node menutup rapi
        except ProcessLookupError:
            return True, "sudah mati"

        for _ in range(30):                  # beri waktu 3 detik
            if self.popen.poll() is not None:
                return True, "ok"
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return True, "dipaksa"

    def status(self):
        with self.kunci:
            baris = list(self.log)
        return {
            "nama": self.nama,
            "judul": self.judul,
            "ket": self.ket,
            "jalan": self.jalan(),
            "pid": self.popen.pid if self.jalan() else None,
            "kode": self.kode_keluar() if not self.jalan() else None,
            "log": baris[-60:],
        }


class Orkestra:
    def __init__(self, cwd):
        self.cwd = cwd
        self.proses = {}
        for n in DAFTAR_NODE:
            self.proses[n["nama"]] = Proses(n["nama"], n["judul"], n["ket"])
        self.proses["simulasi"] = Proses(
            "simulasi", "Simulasi Gazebo",
            "Container podman FIRA Autonomous Cars Simulator")
        self.sedang_start = False
        self.pesan = ""
        self.masalah = []        # peringatan lingkungan, ditampilkan di halaman

    def perintah_master(self, opsi):
        c = ["ros2", "run", "master", "master_node",
             "--ros-args", "-p", "mode:=%s" % opsi.get("mode", "follow")]
        if opsi.get("mode") == "point":
            c += ["-p", "target_x:=%s" % opsi.get("target_x", "10.0"),
                  "-p", "target_y:=%s" % opsi.get("target_y", "0.0")]
        c += ["-p", "pulang_lajur:=%s" % ("true" if opsi.get("pulang_lajur", True) else "false")]
        c += ["-p", "hindar_aktif:=%s" % ("true" if opsi.get("hindar_aktif", True) else "false")]
        tambahan = (opsi.get("tambahan") or "").strip()
        if tambahan:
            c += tambahan.split()
        return c

    def perintah(self, nama, opsi):
        if nama == "master":
            return self.perintah_master(opsi)
        if nama == "simulasi":
            return PERINTAH_SIMULASI
        for n in DAFTAR_NODE:
            if n["nama"] == nama:
                return list(n["perintah"])
        return None

    def start_satu(self, nama, opsi):
        p = self.proses.get(nama)
        if p is None:
            return False, "node tidak dikenal"
        return p.start(self.perintah(nama, opsi), self.cwd)

    def stop_satu(self, nama):
        p = self.proses.get(nama)
        if p is None:
            return False, "node tidak dikenal"
        return p.stop()

    def start_semua(self, opsi):
        if self.sedang_start:
            return
        self.sedang_start = True
        threading.Thread(target=self._start_semua, args=(opsi,), daemon=True).start()

    def _start_semua(self, opsi):
        try:
            for n in DAFTAR_NODE:
                p = self.proses[n["nama"]]
                if p.jalan():
                    self.pesan = "%s sudah berjalan, dilewati." % n["judul"]
                else:
                    self.pesan = "Menyalakan %s ..." % n["judul"]
                    p.start(self.perintah(n["nama"], opsi), self.cwd)
                if n["jeda"] > 0:
                    time.sleep(n["jeda"])
            self.pesan = "Semua node berjalan."
        finally:
            self.sedang_start = False

    def stop_semua(self):
        # Urutan dibalik: master lebih dulu, supaya mobil berhenti sebelum
        # jalur komunikasinya ikut tertutup. Watchdog di udp_sender akan
        # mengirim (0,0) begitu perintah berhenti datang.
        for n in reversed(DAFTAR_NODE):
            self.proses[n["nama"]].stop()
        self.pesan = "Semua node dihentikan."

    def status(self):
        return {
            "node": [self.proses[n["nama"]].status() for n in DAFTAR_NODE],
            "simulasi": self.proses["simulasi"].status(),
            "sedang_start": self.sedang_start,
            "pesan": self.pesan,
            "masalah": self.masalah,
            "telemetri": TELEMETRI.ambil(),
        }


ORKESTRA = None


# ----------------------------------------------------------------------
# Halaman
# ----------------------------------------------------------------------
HALAMAN = r"""<!doctype html>
<html lang="id">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IRIS FP1 Demo</title>
<style>
  :root {
    --bg: #0d1117; --panel: #161b22; --garis: #30363d;
    --teks: #e6edf3; --redup: #8b949e;
    --hijau: #3fb950; --kuning: #d29922; --merah: #f85149;
    --biru: #58a6ff; --ungu: #bc8cff;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--teks);
    font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
    font-size: 15px;
  }
  .bungkus { max-width: 1240px; margin: 0 auto; padding: 20px 16px 48px; }

  header { display: flex; align-items: baseline; gap: 14px; flex-wrap: wrap;
           margin-bottom: 20px; }
  h1 { font-size: 20px; margin: 0; letter-spacing: -0.01em; }
  header .sub { color: var(--redup); font-size: 13px; }

  .kartu { background: var(--panel); border: 1px solid var(--garis);
           border-radius: 10px; padding: 16px; }
  .kartu + .kartu { margin-top: 14px; }
  .kartu h2 { font-size: 12px; text-transform: uppercase;
              letter-spacing: 0.08em; color: var(--redup);
              margin: 0 0 12px; font-weight: 600; }

  .kendali { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }
  button {
    font: inherit; font-weight: 600; cursor: pointer;
    border: 1px solid var(--garis); background: #21262d; color: var(--teks);
    padding: 9px 16px; border-radius: 7px;
  }
  button:hover:not(:disabled) { background: #2d333b; }
  button:disabled { opacity: 0.45; cursor: not-allowed; }
  button.utama { background: var(--hijau); border-color: var(--hijau);
                 color: #04260f; font-size: 16px; padding: 11px 26px; }
  button.utama:hover:not(:disabled) { background: #4ac45c; }
  button.bahaya { background: #3d1518; border-color: #6e2328; color: #ffb4ad; }
  button.bahaya:hover:not(:disabled) { background: #4d1a1e; }
  button.kecil { padding: 5px 11px; font-size: 13px; font-weight: 500; }

  label.pilih { display: inline-flex; align-items: center; gap: 7px;
                color: var(--redup); font-size: 13px; }
  select, input[type=text] {
    font: inherit; font-size: 14px; background: #0d1117; color: var(--teks);
    border: 1px solid var(--garis); border-radius: 6px; padding: 7px 9px;
  }
  input[type=text] { width: 230px; font-family: ui-monospace, monospace;
                     font-size: 13px; }

  .pesan { margin-top: 12px; color: var(--redup); font-size: 13px;
           min-height: 18px; }

  /* --- telemetri --- */
  .telemetri { display: grid; grid-template-columns: auto 1fr; gap: 20px;
               align-items: center; }
  @media (max-width: 760px) { .telemetri { grid-template-columns: 1fr; } }

  .keadaan { text-align: center; padding: 18px 26px; border-radius: 10px;
             border: 2px solid var(--garis); min-width: 220px; }
  .keadaan .nama { font-size: 34px; font-weight: 700; letter-spacing: -0.02em;
                   line-height: 1.1; }
  .keadaan .cap { font-size: 11px; text-transform: uppercase;
                  letter-spacing: 0.1em; color: var(--redup); margin-top: 4px; }
  .k-IKUTI    { border-color: var(--hijau); }
  .k-IKUTI    .nama { color: var(--hijau); }
  .k-HINDAR   { border-color: var(--kuning); }
  .k-HINDAR   .nama { color: var(--kuning); }
  .k-KEMBALI  { border-color: var(--biru); }
  .k-KEMBALI  .nama { color: var(--biru); }
  .k-KOREKSI  { border-color: var(--ungu); }
  .k-KOREKSI  .nama { color: var(--ungu); }
  .k-BERHENTI { border-color: var(--merah); }
  .k-BERHENTI .nama { color: var(--merah); }

  .angka { display: grid; grid-template-columns: repeat(auto-fit, minmax(96px, 1fr));
           gap: 10px; }
  .sel { background: #0d1117; border: 1px solid var(--garis);
         border-radius: 8px; padding: 9px 11px; }
  .sel .cap { font-size: 10px; text-transform: uppercase;
              letter-spacing: 0.07em; color: var(--redup); }
  .sel .val { font-size: 21px; font-weight: 650; font-variant-numeric: tabular-nums;
              font-family: ui-monospace, monospace; margin-top: 2px; }
  .sel .val.kecilan { font-size: 15px; }

  .tanda { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 12px;
           min-height: 26px; }
  .pil { font-size: 11.5px; font-weight: 700; letter-spacing: 0.06em;
         padding: 4px 10px; border-radius: 99px; text-transform: uppercase; }
  .pil.seberang { background: #4a2c05; color: #f0b429; }
  .pil.pulang   { background: #0d3049; color: #79c0ff; }
  .pil.rem      { background: #3d1518; color: #ffb4ad; }
  .pil.solid    { background: #3d1518; color: #ffb4ad; }
  .pil.putus    { background: #11301c; color: #7ee787; }
  .pil.kosong   { background: #21262d; color: var(--redup); }

  /* --- node --- */
  .daftar { display: flex; flex-direction: column; gap: 8px; }
  .node { border: 1px solid var(--garis); border-radius: 8px;
          background: #0d1117; overflow: hidden; }
  .node .atas { display: flex; align-items: center; gap: 12px; padding: 11px 13px; }
  .lampu { width: 9px; height: 9px; border-radius: 50%; flex: 0 0 auto;
           background: #484f58; }
  .lampu.hidup { background: var(--hijau); box-shadow: 0 0 8px rgba(63,185,80,.65); }
  .lampu.mati  { background: var(--merah); }
  .node .info { flex: 1; min-width: 0; }
  .node .judul { font-weight: 600; font-size: 14px; }
  .node .ket { font-size: 12px; color: var(--redup); margin-top: 1px;
               white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .node .pid { font-family: ui-monospace, monospace; font-size: 11.5px;
               color: var(--redup); }
  pre.log { margin: 0; border-top: 1px solid var(--garis); padding: 10px 13px;
            max-height: 190px; overflow: auto; font-size: 11.5px; line-height: 1.5;
            font-family: ui-monospace, monospace; color: #adbac7;
            white-space: pre-wrap; word-break: break-word; background: #010409; }
  pre.log:empty { display: none; }

  .spanduk { background: #3d1518; border: 1px solid #6e2328; border-radius: 10px;
             padding: 14px 16px; margin-bottom: 14px; }
  .spanduk h3 { margin: 0 0 8px; font-size: 13px; color: #ffb4ad;
                text-transform: uppercase; letter-spacing: 0.07em; }
  .spanduk ul { margin: 0; padding-left: 20px; color: #ffd7d3;
                font-size: 13.5px; line-height: 1.7; }
  .spanduk code { background: #0d1117; padding: 1px 5px; border-radius: 4px;
                  font-family: ui-monospace, monospace; font-size: 12.5px; }

  ol.cek { margin: 0; padding-left: 20px; color: var(--redup);
           font-size: 13.5px; line-height: 1.75; }
  ol.cek b { color: var(--teks); font-weight: 600; }
</style>
</head>
<body>
<div class="bungkus">

  <header>
    <h1>IRIS FP1 &mdash; Autonomous Car</h1>
    <span class="sub">Dashboard demo &middot; localhost</span>
  </header>

  <div class="spanduk" id="spanduk" style="display:none">
    <h3>Lingkungan belum siap</h3>
    <ul id="spanduk-isi"></ul>
  </div>

  <div class="kartu">
    <h2>Kendali</h2>
    <div class="kendali">
      <button class="utama" id="btn-start">&#9654; START SEMUA</button>
      <button class="bahaya" id="btn-stop">&#9632; Stop semua</button>
      <span style="width:14px"></span>
      <button class="kecil" id="btn-sim">Nyalakan simulasi Gazebo</button>
    </div>
    <div class="kendali" style="margin-top:14px">
      <label class="pilih">Mode
        <select id="mode">
          <option value="follow">follow &mdash; mengikuti lajur</option>
          <option value="point">point &mdash; menuju koordinat</option>
        </select>
      </label>
      <label class="pilih"><input type="checkbox" id="hindar" checked> penghindaran aktif</label>
      <label class="pilih"><input type="checkbox" id="pulang" checked> pulang ke lajur asal</label>
      <label class="pilih">Parameter tambahan
        <input type="text" id="tambahan" placeholder="-p jarak_hindar:=15.0">
      </label>
    </div>
    <div class="pesan" id="pesan"></div>
  </div>

  <div class="kartu">
    <h2>Keadaan mobil</h2>
    <div class="telemetri">
      <div class="keadaan k-IKUTI" id="kotak-keadaan">
        <div class="nama" id="t-keadaan">&mdash;</div>
        <div class="cap" id="t-umur">menunggu master</div>
      </div>
      <div>
        <div class="angka">
          <div class="sel"><div class="cap">Depan</div><div class="val" id="t-depan">&mdash;</div></div>
          <div class="sel"><div class="cap">Kiri</div><div class="val" id="t-kiri">&mdash;</div></div>
          <div class="sel"><div class="cap">Kanan</div><div class="val" id="t-kanan">&mdash;</div></div>
          <div class="sel"><div class="cap">PWM</div><div class="val" id="t-pwm">&mdash;</div></div>
          <div class="sel"><div class="cap">Setir</div><div class="val" id="t-steer">&mdash;</div></div>
          <div class="sel"><div class="cap">Offset</div><div class="val kecilan" id="t-offset">&mdash;</div></div>
          <div class="sel"><div class="cap">Geseran</div><div class="val kecilan" id="t-geser">&mdash;</div></div>
        </div>
        <div class="tanda" id="t-tanda"></div>
      </div>
    </div>
  </div>

  <div class="kartu">
    <h2>Node</h2>
    <div class="daftar" id="daftar-node"></div>
  </div>

  <div class="kartu">
    <h2>Sebelum menekan start</h2>
    <ol class="cek">
      <li>Gazebo sudah terbuka penuh dan <b>Real Time Factor sekitar 1.00</b>. Kalau 0, simulasi sedang pause &mdash; tekan tombol play.</li>
      <li><b>Edit &rarr; Reset Model Poses</b>, supaya mobil kembali ke garis start.</li>
      <li>Kotak halangan ditaruh <b>minimal 25 meter</b> di depan mobil.</li>
      <li>Mobil masih <b>diam</b> saat START ditekan &mdash; titik (0,0) odometry diambil dari posisi saat itu.</li>
    </ol>
  </div>

</div>

<script>
const $ = (id) => document.getElementById(id);
let adaLog = {};

function opsi() {
  return {
    mode: $("mode").value,
    hindar_aktif: $("hindar").checked,
    pulang_lajur: $("pulang").checked,
    tambahan: $("tambahan").value,
  };
}

async function kirim(jalan, muatan) {
  const r = await fetch(jalan, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(muatan || {}),
  });
  return r.json();
}

$("btn-start").onclick = () => kirim("/api/start_semua", opsi());
$("btn-stop").onclick  = () => kirim("/api/stop_semua");
$("btn-sim").onclick   = () => kirim("/api/start/simulasi", opsi());

function fmt(v, satuan, desimal) {
  if (v === null || v === undefined) return "—";
  const n = Number(v);
  if (!isFinite(n)) return "—";
  return n.toFixed(desimal === undefined ? 1 : desimal) + (satuan || "");
}

function warnaJarak(sel, v) {
  sel.style.color = "";
  if (v === null || v === undefined) return;
  if (v < 5)       sel.style.color = "var(--merah)";
  else if (v < 10) sel.style.color = "var(--kuning)";
  else             sel.style.color = "var(--hijau)";
}

function gambarTelemetri(t) {
  const kotak = $("kotak-keadaan");
  kotak.className = "keadaan k-" + (t.keadaan || "IKUTI");
  $("t-keadaan").textContent = t.keadaan || "—";
  $("t-umur").textContent =
    t.umur === null ? "menunggu master"
    : t.umur > 3    ? "tidak ada kabar " + t.umur + " s"
    : "diperbarui " + t.umur + " s lalu";

  $("t-depan").textContent = fmt(t.depan, " m");
  $("t-kiri").textContent  = fmt(t.kiri,  " m");
  $("t-kanan").textContent = fmt(t.kanan, " m");
  warnaJarak($("t-depan"), t.depan);

  $("t-pwm").textContent    = t.pwm === null ? "—" : t.pwm;
  $("t-steer").textContent  = t.steer === null ? "—" : t.steer + "°";
  $("t-offset").textContent = t.offset === null ? "—" : (t.offset > 0 ? "+" : "") + t.offset + " px";
  $("t-geser").textContent  = t.geser === null ? "—" : (t.geser > 0 ? "+" : "") + t.geser + " px";

  const tanda = [];
  const jenis = (g) => g === "solid" ? "solid" : g === "putus" ? "putus" : "kosong";
  tanda.push('<span class="pil ' + jenis(t.garis_kiri) + '">kiri: ' + t.garis_kiri + '</span>');
  tanda.push('<span class="pil ' + jenis(t.garis_kanan) + '">kanan: ' + t.garis_kanan + '</span>');
  if (t.lajur_seberang) tanda.push('<span class="pil seberang">lajur seberang</span>');
  if (t.pulang)         tanda.push('<span class="pil pulang">pulang</span>');
  if (t.rem)            tanda.push('<span class="pil rem">rem</span>');
  $("t-tanda").innerHTML = tanda.join("");
}

function gambarNode(daftar, sim) {
  const wadah = $("daftar-node");
  const semua = [sim].concat(daftar);

  if (wadah.children.length !== semua.length) {
    wadah.innerHTML = semua.map((n) => `
      <div class="node" id="n-${n.nama}">
        <div class="atas">
          <span class="lampu" id="l-${n.nama}"></span>
          <div class="info">
            <div class="judul">${n.judul}</div>
            <div class="ket">${n.ket}</div>
          </div>
          <span class="pid" id="p-${n.nama}"></span>
          <button class="kecil" id="s-${n.nama}"></button>
        </div>
        <pre class="log" id="g-${n.nama}"></pre>
      </div>`).join("");

    semua.forEach((n) => {
      $("s-" + n.nama).onclick = () => {
        const jalan = $("s-" + n.nama).dataset.jalan === "1";
        kirim(jalan ? "/api/stop/" + n.nama : "/api/start/" + n.nama, opsi());
      };
    });
  }

  semua.forEach((n) => {
    const lampu = $("l-" + n.nama);
    lampu.className = "lampu " + (n.jalan ? "hidup" : (n.kode === null ? "" : "mati"));
    $("p-" + n.nama).textContent = n.jalan ? "pid " + n.pid
                                 : (n.kode === null ? "" : "keluar " + n.kode);
    const b = $("s-" + n.nama);
    b.textContent = n.jalan ? "Stop" : "Start";
    b.dataset.jalan = n.jalan ? "1" : "0";

    const g = $("g-" + n.nama);
    const isi = n.log.join("\n");
    if (g.textContent !== isi) {
      const dibawah = g.scrollTop + g.clientHeight >= g.scrollHeight - 24;
      g.textContent = isi;
      if (dibawah) g.scrollTop = g.scrollHeight;
    }
  });
}

async function tarik() {
  try {
    const r = await fetch("/api/status");
    const s = await r.json();
    $("pesan").textContent = s.pesan || "";
    $("btn-start").disabled = s.sedang_start;

    const sp = $("spanduk");
    if (s.masalah && s.masalah.length) {
      sp.style.display = "";
      $("spanduk-isi").innerHTML = s.masalah.map(
        (m) => "<li>" + m.replace(/`([^`]+)`/g, "<code>$1</code>") + "</li>").join("");
    } else {
      sp.style.display = "none";
    }

    gambarTelemetri(s.telemetri);
    gambarNode(s.node, s.simulasi);
  } catch (e) {
    $("pesan").textContent = "Server dashboard tidak merespons.";
  }
}

tarik();
setInterval(tarik, 400);
</script>
</body>
</html>
"""


class Penangan(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass                      # jangan mengotori terminal

    def _balas(self, kode, isi, tipe="application/json"):
        data = isi if isinstance(isi, bytes) else isi.encode("utf-8")
        self.send_response(kode)
        self.send_header("Content-Type", tipe + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._balas(200, HALAMAN, "text/html")
        elif self.path == "/api/status":
            self._balas(200, json.dumps(ORKESTRA.status()))
        else:
            self._balas(404, json.dumps({"salah": "tidak ada"}))

    def do_POST(self):
        panjang = int(self.headers.get("Content-Length") or 0)
        mentah = self.rfile.read(panjang) if panjang else b"{}"
        try:
            opsi = json.loads(mentah or b"{}")
        except json.JSONDecodeError:
            opsi = {}

        p = self.path
        if p == "/api/start_semua":
            ORKESTRA.start_semua(opsi)
            hasil = {"ok": True}
        elif p == "/api/stop_semua":
            ORKESTRA.stop_semua()
            hasil = {"ok": True}
        elif p.startswith("/api/start/"):
            ok, pesan = ORKESTRA.start_satu(p.rsplit("/", 1)[-1], opsi)
            hasil = {"ok": ok, "pesan": pesan}
        elif p.startswith("/api/stop/"):
            ok, pesan = ORKESTRA.stop_satu(p.rsplit("/", 1)[-1])
            hasil = {"ok": ok, "pesan": pesan}
        else:
            self._balas(404, json.dumps({"salah": "tidak ada"}))
            return

        self._balas(200, json.dumps(hasil))


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def periksa_lingkungan(cwd):
    masalah = []

    if subprocess.call(["which", "ros2"],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL) != 0:
        masalah.append("Perintah `ros2` tidak ditemukan. "
                       "Jalankan: source /opt/ros/jazzy/setup.bash")

    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    if "iris_fp1_ws" not in ament:
        masalah.append(
            "Workspace belum di-source, jadi semua node akan gagal dengan "
            "\"Package not found\". Hentikan dashboard (Ctrl+C), lalu dari "
            "terminal biasa: cd ~/iris_fp1_ws && source install/setup.bash "
            "&& python3 demo_dashboard.py")

    if os.environ.get("TERM_PROGRAM") == "vscode" or "VSCODE_PID" in os.environ:
        masalah.append(
            "Dashboard ini dijalankan dari dalam VS Code. Pakai terminal "
            "biasa (Ctrl+Alt+T) - tombol Run di VS Code tidak meneruskan "
            "environment ROS.")

    if not os.path.isdir(os.path.join(cwd, "install")):
        masalah.append("Tidak ada folder install/ di sini. "
                       "Jalankan dashboard dari dalam ~/iris_fp1_ws")

    ld = os.environ.get("LD_LIBRARY_PATH", "")
    if "/snap/" in ld:
        masalah.append("LD_LIBRARY_PATH memuat /snap/ - ini terminal VS Code. "
                       "Node akan gagal. Pakai terminal biasa (Ctrl+Alt+T).")

    return masalah


def main():
    global ORKESTRA
    cwd = os.getcwd()
    ORKESTRA = Orkestra(cwd)

    print("=" * 62)
    print(" Dashboard demo IRIS FP1")
    print("=" * 62)

    masalah = periksa_lingkungan(cwd)
    ORKESTRA.masalah = masalah
    if masalah:
        print("\n PERIKSA DULU:")
        for m in masalah:
            print("   - " + m)
        print()
    else:
        print(" Lingkungan ROS terlihat siap.")

    print(" Folder kerja : %s" % cwd)
    print(" Buka         : http://localhost:%d" % PORT)
    print(" Hentikan     : Ctrl+C di terminal ini")
    print("=" * 62)

    srv = Server(("127.0.0.1", PORT), Penangan)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nMenghentikan semua node ...")
        ORKESTRA.stop_semua()
        srv.shutdown()
        print("Selesai.")


if __name__ == "__main__":
    main()
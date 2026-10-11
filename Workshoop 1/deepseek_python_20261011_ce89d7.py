# =====================================================================
#  GROUND STATION — Serial -> Map (Auto-detect Port + Overlay Koordinat)
#  + Dummy GPS kalau belum fix (biar marker tetap muncul di peta)
# =====================================================================

import re
import sys
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install dulu: pip install pyserial")
    sys.exit(1)

try:
    import tkintermapview
except ImportError:
    print("Install dulu: pip install tkintermapview")
    sys.exit(1)


# =====================================================================
#  KONFIGURASI
# =====================================================================
BAUD_RATE     = 115200
SERIAL_PORT   = None            # None = auto-detect, atau isi "COM5"
MAP_START_LAT = -6.9730
MAP_START_LON = 107.6300
MAP_ZOOM      = 16

# Kalau True: saat GPS belum fix, pakai dummy koordinat biar marker tetap muncul.
# Kalau False: marker tidak muncul sama sekali saat GPS belum fix.
USE_DUMMY_GPS_WHEN_NO_FIX = True

USB_CHIP_KEYWORDS = [
    "cp210", "cp2102", "cp2104",
    "ch340", "ch341", "ch9102",
    "ft232", "ftdi", "ft231",
    "silicon labs", "prolific", "pl2303",
    "usb serial", "usb-serial",
    "uart", "esp32", "espressif",
]

TX_REGEX = re.compile(
    r"\[TX #(?P<pkt>\d+)\]\s+"
    r"t=(?P<t>\d+)s\s+"
    r"P=(?P<p>[-\d.]+)\s+"
    r"Alt=(?P<alt>[-\d.]+)\s+"
    r"V=(?P<v>[-\d.]+)\s+"
    r"I=(?P<i>[-\d.]+)\s+"
    r"(?P<state>\w+)\s+"
    r"lat=(?P<lat>[-.\d]+|nan)\s+"
    r"lon=(?P<lon>[-.\d]+|nan)\s+"
    r"R=(?P<roll>[-\d.]+)\s+"
    r"P=(?P<pitch>[-\d.]+)\s+"
    r"Y=(?P<yaw>[-\d.]+)"
)


# =====================================================================
#  AUTO-DETECT SERIAL PORT
# =====================================================================
def list_serial_ports():
    return list(serial.tools.list_ports.comports())


def score_port(port_info):
    desc = (port_info.description or "").lower()
    hwid = (port_info.hwid or "").lower()
    mfg  = (getattr(port_info, "manufacturer", "") or "").lower()
    name = (port_info.device or "").lower()
    haystack = f"{desc} {hwid} {mfg} {name}"
    score = 0
    for kw in USB_CHIP_KEYWORDS:
        if kw in haystack:
            score += 10
    if "bluetooth" in haystack or "bt" in desc.split():
        score -= 50
    return score


def auto_detect_port(verbose=True):
    ports = list_serial_ports()
    if not ports:
        if verbose:
            print("[AUTO] Tidak ada port serial terdeteksi.")
        return None
    if verbose:
        print(f"[AUTO] Ditemukan {len(ports)} port:")
        for p in ports:
            print(f"   - {p.device:12s} | {p.description} | {p.hwid}")
    scored = sorted(ports, key=score_port, reverse=True)
    best = scored[0]
    if verbose:
        print(f"[AUTO] Kandidat terbaik: {best.device} ({best.description})")
    if verify_port(best.device):
        return best.device
    for p in scored[1:]:
        if verify_port(p.device, verbose=False):
            if verbose:
                print(f"[AUTO] Fallback ke {p.device}")
            return p.device
    if verbose:
        print("[AUTO] Tidak ada port yang bisa dibuka.")
    return None


def verify_port(device, verbose=True):
    try:
        s = serial.Serial(device, BAUD_RATE, timeout=0.1)
        s.close()
        if verbose:
            print(f"[AUTO] Port {device} OK.")
        return True
    except Exception as e:
        if verbose:
            print(f"[AUTO] Port {device} gagal dibuka: {e}")
        return False


def parse_tx_line(line: str):
    m = TX_REGEX.search(line)
    if not m:
        return None
    d = m.groupdict()

    def to_float(s):
        if s is None or s.lower() == "nan":
            return None
        try:
            return float(s)
        except ValueError:
            return None

    return {
        "pkt":   int(d["pkt"]),
        "t":     int(d["t"]),
        "p":     to_float(d["p"]),
        "alt":   to_float(d["alt"]),
        "v":     to_float(d["v"]),
        "i":     to_float(d["i"]),
        "state": d["state"],
        "lat":   to_float(d["lat"]),
        "lon":   to_float(d["lon"]),
        "roll":  to_float(d["roll"]),
        "pitch": to_float(d["pitch"]),
        "yaw":   to_float(d["yaw"]),
    }


# =====================================================================
#  APLIKASI
# =====================================================================
class GroundStation:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Payload Satelit Mini — Ground Station")
        self.root.geometry("1280x800")
        self.root.configure(bg="#111")

        self.ser = None
        self.serial_port_used = None
        self.running = True
        self.marker = None
        self.path = None
        self.trail = []
        self.fix_count = 0          # jumlah paket dengan GPS fix asli
        self.total_count = 0        # jumlah paket TX diterima
        self.locked_view = False

        # state untuk dummy GPS
        self._dummy_phase = 0.0

        self._build_ui()
        self._connect_serial()
        if self.ser is not None:
            self._start_reader_thread()

    # ---------------- UI ----------------
    def _build_ui(self):
        container = tk.Frame(self.root, bg="#111")
        container.pack(fill="both", expand=True)

        # ---- PETA ----
        map_frame = tk.Frame(container, bg="#111")
        map_frame.pack(side="left", fill="both", expand=True)

        self.map = tkintermapview.TkinterMapView(map_frame, corner_radius=0)
        self.map.pack(fill="both", expand=True)
        self.map.set_position(MAP_START_LAT, MAP_START_LON)
        self.map.set_zoom(MAP_ZOOM)

        # ---- OVERLAY KOORDINAT (kanan atas peta) ----
        self._build_coord_overlay(map_frame)

        # ---- TOMBOL KONTROL (kiri atas peta) ----
        ctrl = tk.Frame(map_frame, bg="#111")
        ctrl.place(relx=0.01, rely=0.01, anchor="nw")

        self.btn_lock = tk.Button(
            ctrl, text="🔓 Auto-follow: ON", bg="#2a2a2a", fg="#0f0",
            font=("Consolas", 10), relief="flat", padx=10, pady=5,
            command=self._toggle_lock
        )
        self.btn_lock.pack(anchor="w")

        tk.Button(
            ctrl, text="🧹 Hapus trail", bg="#2a2a2a", fg="#fff",
            font=("Consolas", 10), relief="flat", padx=10, pady=5,
            command=self._clear_trail
        ).pack(anchor="w", pady=(5, 0))

        # ---- SIDEBAR ----
        side = tk.Frame(container, width=320, bg="#1a1a1a")
        side.pack(side="right", fill="y")
        side.pack_propagate(False)

        tk.Label(side, text="TELEMETRI", fg="#0ff", bg="#1a1a1a",
                 font=("Consolas", 14, "bold")).pack(pady=(15, 10))

        self.labels = {}
        fields = [
            ("pkt",   "Pkt #"),
            ("t",     "Waktu"),
            ("state", "State"),
            ("p",     "Tekanan"),
            ("alt",   "Ketinggian"),
            ("v",     "Tegangan"),
            ("i",     "Arus"),
            ("lat",   "Latitude"),
            ("lon",   "Longitude"),
            ("roll",  "Roll"),
            ("pitch", "Pitch"),
            ("yaw",   "Yaw"),
        ]
        for key, name in fields:
            row = tk.Frame(side, bg="#1a1a1a")
            row.pack(fill="x", padx=15, pady=3)
            tk.Label(row, text=name, fg="#888", bg="#1a1a1a",
                     width=11, anchor="w", font=("Consolas", 10)).pack(side="left")
            lbl = tk.Label(row, text="—", fg="#0f0", bg="#1a1a1a",
                           anchor="w", font=("Consolas", 11, "bold"))
            lbl.pack(side="left")
            self.labels[key] = lbl

        tk.Frame(side, height=1, bg="#333").pack(fill="x", padx=15, pady=12)

        tk.Label(side, text="STATISTIK", fg="#0ff", bg="#1a1a1a",
                 font=("Consolas", 12, "bold")).pack(pady=(0, 5))

        self.lbl_stats = tk.Label(side, text="Paket: 0  |  Fix: 0",
                                  fg="#aaa", bg="#1a1a1a", font=("Consolas", 10))
        self.lbl_stats.pack(pady=3)

        self.lbl_serial = tk.Label(side, text="Serial: mencari...",
                                   fg="#f80", bg="#1a1a1a", font=("Consolas", 9),
                                   wraplength=290, justify="left")
        self.lbl_serial.pack(pady=(15, 3), padx=10)

        tk.Button(
            side, text="🔄 Cari port ulang", bg="#2a2a2a", fg="#0ff",
            font=("Consolas", 10), relief="flat", padx=10, pady=5,
            command=self._reconnect
        ).pack(pady=5)

        self.status = tk.Label(self.root, text="Menunggu data...",
                               bg="#000", fg="#0f0", anchor="w",
                               font=("Consolas", 10))
        self.status.pack(side="bottom", fill="x")

    # ---------------- OVERLAY KOORDINAT ----------------
    def _build_coord_overlay(self, parent):
        wrap = tk.Frame(parent, bg="#000")
        wrap.place(relx=0.99, rely=0.01, anchor="ne")

        border = tk.Frame(wrap, bg="#0f0", padx=2, pady=2)
        border.pack()

        inner = tk.Frame(border, bg="#0a0a0a", padx=14, pady=10)
        inner.pack()

        tk.Label(inner, text="◉ GPS COORDINATE", fg="#0f0", bg="#0a0a0a",
                 font=("Consolas", 9, "bold")).pack(anchor="w")

        # ---- LATITUDE ----
        row_lat = tk.Frame(inner, bg="#0a0a0a")
        row_lat.pack(anchor="w", pady=(6, 0))
        tk.Label(row_lat, text="LAT", fg="#0aa", bg="#0a0a0a",
                 font=("Consolas", 10, "bold"), width=4, anchor="w").pack(side="left")
        self.lbl_lat_big = tk.Label(
            row_lat, text="—", fg="#0f0", bg="#0a0a0a",
            font=("Consolas", 18, "bold"), anchor="w"
        )
        self.lbl_lat_big.pack(side="left")

        # ---- LONGITUDE ----
        row_lon = tk.Frame(inner, bg="#0a0a0a")
        row_lon.pack(anchor="w", pady=(2, 0))
        tk.Label(row_lon, text="LON", fg="#0aa", bg="#0a0a0a",
                 font=("Consolas", 10, "bold"), width=4, anchor="w").pack(side="left")
        self.lbl_lon_big = tk.Label(
            row_lon, text="—", fg="#0f0", bg="#0a0a0a",
            font=("Consolas", 18, "bold"), anchor="w"
        )
        self.lbl_lon_big.pack(side="left")

        tk.Frame(inner, height=1, bg="#0a5").pack(fill="x", pady=(8, 6))

        info = tk.Frame(inner, bg="#0a0a0a")
        info.pack(anchor="w", fill="x")

        def info_row(parent, tag):
            r = tk.Frame(parent, bg="#0a0a0a")
            r.pack(anchor="w", fill="x", pady=1)
            tk.Label(r, text=tag, fg="#888", bg="#0a0a0a",
                     font=("Consolas", 9), width=6, anchor="w").pack(side="left")
            lbl = tk.Label(r, text="—", fg="#0ff", bg="#0a0a0a",
                           font=("Consolas", 9, "bold"), anchor="w")
            lbl.pack(side="left")
            return lbl

        self.lbl_ov_alt   = info_row(info, "ALT")
        self.lbl_ov_state = info_row(info, "STATE")
        self.lbl_ov_pkt   = info_row(info, "PKT")
        self.lbl_ov_fix   = info_row(info, "FIX")

    # ---------------- DUMMY GPS ----------------
    def _dummy_gps(self):
        """
        GPS dummy: muter-muter pelan di sekitar MAP_START_LAT/LON.
        Radius ~50 m, periode ~30 detik.
        Return: (lat, lon)
        """
        self._dummy_phase += 0.05
        r = 0.0005
        lat = MAP_START_LAT + r * math.sin(self._dummy_phase)
        lon = MAP_START_LON + r * math.cos(self._dummy_phase)
        return lat, lon

    # ---------------- SERIAL ----------------
    def _connect_serial(self):
        port = SERIAL_PORT
        if port is None:
            self.status.config(text="Auto-detect port...")
            self.root.update_idletasks()
            port = auto_detect_port(verbose=True)
        if port is None:
            self.lbl_serial.config(text="Serial: tidak ditemukan port", fg="#f00")
            self.status.config(text="Tidak ada port serial. Klik 'Cari port ulang'.")
            return
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            self.serial_port_used = port
            self.lbl_serial.config(text=f"Serial: {port} @ {BAUD_RATE}", fg="#0f0")
            self.status.config(text=f"Terhubung ke {port}")
        except Exception as e:
            self.lbl_serial.config(text=f"Serial ERROR: {e}", fg="#f00")
            self.status.config(text=f"Gagal buka {port}: {e}")

    def _reconnect(self):
        self.running = False
        time.sleep(0.3)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.running = True
        self.lbl_serial.config(text="Serial: mencari...", fg="#f80")
        self._connect_serial()
        if self.ser is not None:
            self._start_reader_thread()

    def _start_reader_thread(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()

    def _reader_loop(self):
        while self.running:
            if self.ser is None:
                break
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            data = parse_tx_line(line)
            if data:
                self.root.after(0, self._update_ui, data)
            else:
                self.root.after(0, lambda l=line: self.status.config(text=l))

    # ---------------- UPDATE UI ----------------
    def _update_ui(self, d):
        self.total_count += 1

        # ---- Tentukan apakah pakai GPS asli atau dummy ----
        gps_asli = (d["lat"] is not None and d["lon"] is not None)
        gps_dummy = False

        if gps_asli:
            lat, lon = d["lat"], d["lon"]
        else:
            if USE_DUMMY_GPS_WHEN_NO_FIX:
                lat, lon = self._dummy_gps()
                gps_dummy = True
            else:
                lat, lon = None, None

        def fmt(val, unit="", prec=2):
            if val is None:
                return "no fix"
            if isinstance(val, float):
                return f"{val:.{prec}f}{unit}"
            return f"{val}{unit}"

        # ---- SIDEBAR ----
        self.labels["pkt"].config(text=str(d["pkt"]))
        self.labels["t"].config(text=f"{d['t']} s")
        self.labels["state"].config(text=d["state"])
        self.labels["p"].config(text=fmt(d["p"], " hPa"))
        self.labels["alt"].config(text=fmt(d["alt"], " m"))
        self.labels["v"].config(text=fmt(d["v"], " V"))
        self.labels["i"].config(text=fmt(d["i"], " A"))
        self.labels["lat"].config(text=fmt(lat, "", 6))
        self.labels["lon"].config(text=fmt(lon, "", 6))
        self.labels["roll"].config(text=fmt(d["roll"], "°"))
        self.labels["pitch"].config(text=fmt(d["pitch"], "°"))
        self.labels["yaw"].config(text=fmt(d["yaw"], "°"))

        # ---- OVERLAY KOORDINAT ----
        if lat is None or lon is None:
            self.lbl_lat_big.config(text="— no fix —", fg="#f80")
            self.lbl_lon_big.config(text="— no fix —", fg="#f80")
            self.lbl_ov_fix.config(text="NO FIX", fg="#f80")
        elif gps_dummy:
            self.lbl_lat_big.config(text=f"{lat:+.6f}", fg="#f80")
            self.lbl_lon_big.config(text=f"{lon:+.6f}", fg="#f80")
            self.lbl_ov_fix.config(text="DUMMY", fg="#f80")
        else:
            self.lbl_lat_big.config(text=f"{lat:+.6f}", fg="#0f0")
            self.lbl_lon_big.config(text=f"{lon:+.6f}", fg="#0f0")
            self.lbl_ov_fix.config(text="FIXED", fg="#0f0")

        self.lbl_ov_alt.config(text=fmt(d["alt"], " m"))
        self.lbl_ov_state.config(text=d["state"])
        self.lbl_ov_pkt.config(text=f"#{d['pkt']}  t={d['t']}s")

        # ---- Marker & Trail ----
        if lat is not None and lon is not None:
            # Marker
            if self.marker is None:
                self.marker = self.map.set_marker(lat, lon, text="Payload")
            else:
                self.marker.set_position(lat, lon)

            # Trail hanya dari fix asli
            if gps_asli:
                self.fix_count += 1
                self.trail.append((lat, lon))
                if len(self.trail) > 1:
                    if self.path is not None:
                        self.path.delete()
                    self.path = self.map.set_path(
                        self.trail, color="#ff3030", width=3
                    )

            # Auto-center
            if not self.locked_view:
                self.map.set_position(lat, lon)

            if gps_dummy:
                self.status.config(
                    text=f"DUMMY  lat={lat:.6f}  lon={lon:.6f}  (GPS belum fix)"
                )
            else:
                self.status.config(text=f"FIX  lat={lat:.6f}  lon={lon:.6f}")
        else:
            self.status.config(text="Menunggu GPS fix...")

        self.lbl_stats.config(
            text=f"Paket: {self.total_count}  |  Fix: {self.fix_count}"
        )

    # ---------------- AKSI ----------------
    def _toggle_lock(self):
        self.locked_view = not self.locked_view
        if self.locked_view:
            self.btn_lock.config(text="🔒 Auto-follow: OFF", fg="#f80")
        else:
            self.btn_lock.config(text="🔓 Auto-follow: ON", fg="#0f0")

    def _clear_trail(self):
        self.trail.clear()
        if self.path is not None:
            self.path.delete()
            self.path = None

    def on_close(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.root.destroy()


# =====================================================================
#  MAIN
# =====================================================================
def main():
    if "--list" in sys.argv:
        print("Port tersedia:")
        for p in list_serial_ports():
            print(f"  {p.device:12s} | {p.description} | {p.hwid}")
        return

    global SERIAL_PORT
    if "--port" in sys.argv:
        i = sys.argv.index("--port")
        if i + 1 < len(sys.argv):
            SERIAL_PORT = sys.argv[i + 1]
            print(f"[CLI] Pakai port manual: {SERIAL_PORT}")

    root = tk.Tk()
    app = GroundStation(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
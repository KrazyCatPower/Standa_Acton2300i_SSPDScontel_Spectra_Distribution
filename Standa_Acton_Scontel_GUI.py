import os, sys, time, math, socket, re, threading, queue, webbrowser
import ctypes as ct
from ctypes import byref
from datetime import datetime
from itertools import cycle

import serial, serial.tools.list_ports
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# ------------------------------------------------------------
# Параметры подвижки
fullstep_1 = 1.25
ustep_8 = 0.15625
USTEPS_PER_STEP = 8

# TimeHarp — общие константы
TH_LIB_VERSION = "3.2"
TH_MAXDEVNUM = 4
TH_MODE_HIST = 0
TH_MAXLENCODE = 5
TH_MAXINPCHAN = 2
TH_MAXHISTLEN = 32768
TH_FLAG_OVERFLOW = 0x0001

TH_INPUT_CFD_LEVEL = -50
TH_SYNC_CFD_ZERO_CROSS = 0
TH_INPUT_CFD_ZERO_CROSS = -10
TH_OFFSET = 0

TH_SYNC_LEVEL_DEFAULT = -30
TH_SYNC_DIVIDER_DEFAULT = 4
TH_OFFSET1_DEFAULT = 0
TH_RESOLUTION_DEFAULT = 25.0

AUTO_OFFSET_TARGET_NS = 1.0

ORIGIN_COLORS = [
    '#0000FF', '#FF0000', '#008000', '#FF8C00', '#800080',
    '#00CED1', '#FF69B4', '#8B4513', '#808000', '#4682B4'
]


def fmt_sci(x):
    try:
        s = f"{float(x):.16E}"
    except Exception:
        return "0.0000000000000000E+000"
    if "E" not in s:
        return s
    mant, exp = s.split("E")
    sign = exp[0]
    val = int(exp[1:])
    return f"{mant}E{sign}{val:03d}"


# ------------------------------------------------------------
class ActonPy:
    def __init__(self, port='COM3'):
        self.ser = serial.Serial(port, baudrate=9600, timeout=1)
        time.sleep(0.1)
        if self.ser.is_open:
            self.model = self.query('MODEL')
            if not self.model:
                raise ConnectionError(f'Нет ответа на MODEL, порт {port}')
        else:
            raise ConnectionError(f'Could not open serial port {port}')
        self.tolerance_nm = 0.1

    def _extract_float(self, text):
        cleaned = re.sub(r'\bok\b', '', text, flags=re.IGNORECASE)
        m = re.search(r'[-+]?\d*\.?\d+', cleaned)
        if m:
            return float(m.group())
        raise ValueError(f"No number found in '{text}'")

    def query(self, cmd):
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.2)
                self.ser.reset_input_buffer()
            self.ser.write((cmd + '\r').encode())
            self.ser.flush()
            time.sleep(0.05)
            raw = self.ser.read_all().decode(errors='ignore')
            if raw.strip():
                for suffix in ('ok\r\n', 'ok\n'):
                    if raw.endswith(suffix):
                        raw = raw[:-len(suffix)].strip()
                        break
                return raw.strip()
        raise RuntimeError(f"Команда {cmd} не вернула ответ")

    def write(self, cmd):
        self.ser.write((cmd + '\r').encode())
        self.ser.flush()

    def closeConnection(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def get_wavelength(self):
        for _ in range(3):
            try:
                resp = self.query('?NM')
            except RuntimeError:
                time.sleep(0.2)
                continue
            if resp and resp.lower() != 'ok':
                try:
                    return self._extract_float(resp)
                except ValueError:
                    continue
            time.sleep(0.2)
            self.ser.reset_input_buffer()
        raise RuntimeError("Не удалось получить длину волны")

    def is_moving(self):
        for cmd in ('?STATUS', 'STATUS', 'MONO-EE STATUS'):
            try:
                resp = self.query(cmd)
                if resp and '?' not in resp:
                    return 'MOVING' in resp.upper()
            except Exception:
                pass
        return False

    def goto(self, wavelength, max_wait=30):
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.2)
                self.ser.reset_input_buffer()
            self.write(f'{wavelength:.3f} GOTO')
            ack = self.ser.readline().decode(errors='ignore')
            if 'ok' in ack.lower():
                break
        else:
            raise RuntimeError("GOTO не подтверждён монохроматором")
        for _ in range(max_wait):
            try:
                current = self.get_wavelength()
            except RuntimeError:
                time.sleep(1)
                continue
            if abs(current - wavelength) <= self.tolerance_nm:
                if not self.is_moving():
                    return
            time.sleep(1)
        raise TimeoutError(f"Монохроматор не достиг {wavelength:.3f} нм")

    def get_scanrate(self):
        return self._extract_float(self.query('?NM/MIN'))

    def set_scanrate(self, rate):
        self.write(f'{rate:.2f} NM/MIN')
        ack = self.ser.readline().decode(errors='ignore')
        if 'ok' not in ack.lower():
            return 'failed'
        return self.get_scanrate()


def find_acton_port():
    for p in serial.tools.list_ports.comports():
        if 'standa' in p.description.lower() or 'ximc' in p.description.lower():
            continue
        try:
            ser = serial.Serial(p.device, baudrate=9600, timeout=0.5)
            time.sleep(0.1)
            ser.write(b'MODEL\r')
            ser.flush()
            time.sleep(0.1)
            resp = ser.read_all().decode(errors='ignore')
            ser.close()
            if 'SP-' in resp:
                return p.device
        except Exception:
            continue
    return None


# ------------------------------------------------------------
def get_cps(host, port, dev_num, timeout=2.0):
    cmd = f"SSPD:DEV{dev_num}:COUN?\n".encode()
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.sendall(cmd)
        data = sock.recv(256)
        sock.close()
        resp = data.decode('utf-8', errors='replace')
        m = re.search(r'[-+]?\d*\.?\d+', resp)
        return float(m.group()) if m else None
    except Exception:
        return None


def measure_cps_average(host, port, dev_num, accum_time_sec):
    if accum_time_sec < 1.0:
        if accum_time_sec < 0.01:
            return get_cps(host, port, dev_num)
        time.sleep(accum_time_sec)
        return get_cps(host, port, dev_num)
    total, valid = 0.0, 0
    for _ in range(int(accum_time_sec)):
        cps = get_cps(host, port, dev_num)
        if cps is not None:
            total += cps
            valid += 1
        time.sleep(1)
    return total / valid if valid else None


# ------------------------------------------------------------
try:
    import libximc.highlevel as ximc
except ImportError:
    cur_dir = os.path.abspath(os.path.dirname(__file__))
    ximc_dir = os.path.join(cur_dir, "ximc")
    ximc_package_dir = os.path.join(ximc_dir, "crossplatform", "wrappers", "python")
    sys.path.append(ximc_package_dir)
    import libximc.highlevel as ximc


def um_to_ximc(target_um):
    total_usteps = int(round(target_um / ustep_8))
    steps = total_usteps // USTEPS_PER_STEP
    usteps = total_usteps % USTEPS_PER_STEP
    return steps, usteps


def ximc_to_um(steps, usteps):
    return (steps * USTEPS_PER_STEP + usteps) * ustep_8


def set_microstep_mode_8(axis):
    s = axis.get_engine_settings()
    s.MicrostepMode = ximc.MicrostepMode.MICROSTEP_MODE_FRAC_8
    axis.set_engine_settings(s)


def move(axis, distance, udistance, wait_ms=100, stabilize_s=0.2,
         tolerance_usteps=2):
    axis.command_move(distance, udistance)
    time.sleep(0.05)
    try:
        axis.command_wait_for_stop(wait_ms)
    except Exception as e:
        print(f"[move] command_wait_for_stop: {e}")
        time.sleep(1.0)
    time.sleep(stabilize_s)
    try:
        pos = axis.get_position()
        actual_um = ximc_to_um(pos.Position, pos.uPosition)
        target_total = distance * USTEPS_PER_STEP + udistance
        actual_total = pos.Position * USTEPS_PER_STEP + pos.uPosition
        if abs(actual_total - target_total) > tolerance_usteps:
            print(f"[move] Позиция не совпала: цель="
                  f"{ximc_to_um(distance, udistance):.4f} мкм, "
                  f"факт={actual_um:.4f} мкм")
        return actual_um
    except Exception as e:
        print(f"[move] Не удалось прочитать позицию: {e}")
        return ximc_to_um(distance, udistance)


# ------------------------------------------------------------
class TimeHarpPy:
    def __init__(self, sync_divider=TH_SYNC_DIVIDER_DEFAULT,
                 sync_level=TH_SYNC_LEVEL_DEFAULT,
                 resolution_ps=TH_RESOLUTION_DEFAULT,
                 log_func=None):
        self.log = log_func or (lambda m: print(m))
        self.sync_divider = int(sync_divider)
        self.sync_level = int(sync_level)
        self.resolution_override_ps = float(resolution_ps)
        self.opened = False
        self.dev = []
        self.model = ""
        self._hw_resolution_ps = 25.0

        if os.name == "nt":
            self.lib = ct.WinDLL("th260lib64.dll")
        else:
            self.lib = ct.CDLL("libth260.so")
        self._open()

    @property
    def resolution_ps(self):
        if self.resolution_override_ps and self.resolution_override_ps > 0:
            return float(self.resolution_override_ps)
        return float(self._hw_resolution_ps)

    def _tryfunc(self, retcode, funcName):
        if retcode < 0:
            err = ct.create_string_buffer(b"", 40)
            try:
                self.lib.TH260_GetErrorString(err, ct.c_int(retcode))
            except Exception:
                pass
            msg = f"TH260_{funcName} error {retcode} ({err.value.decode('utf-8')})"
            self.log(msg)
            raise RuntimeError(msg)

    def _open(self):
        libVersion = ct.create_string_buffer(b"", 8)
        self.lib.TH260_GetLibraryVersion(libVersion)
        self.log(f"TimeHarp lib ver: {libVersion.value.decode('utf-8')}")

        hwSerial = ct.create_string_buffer(b"", 8)
        for i in range(TH_MAXDEVNUM):
            if self.lib.TH260_OpenDevice(ct.c_int(i), hwSerial) == 0:
                self.dev.append(i)
                self.log(f"TimeHarp #{i} S/N {hwSerial.value.decode('utf-8')}")

        if not self.dev:
            raise RuntimeError("TimeHarp: нет доступных устройств")

        self._tryfunc(self.lib.TH260_Initialize(
            ct.c_int(self.dev[0]), ct.c_int(TH_MODE_HIST)), "Initialize")

        hwModel = ct.create_string_buffer(b"", 16)
        hwPartno = ct.create_string_buffer(b"", 8)
        hwVersion = ct.create_string_buffer(b"", 16)
        self._tryfunc(self.lib.TH260_GetHardwareInfo(
            self.dev[0], hwModel, hwPartno, hwVersion), "GetHardwareInfo")
        self.model = hwModel.value.decode("utf-8")
        self.log(f"TimeHarp: {self.model}")

        numChannels = ct.c_int()
        self._tryfunc(self.lib.TH260_GetNumOfInputChannels(
            ct.c_int(self.dev[0]), byref(numChannels)), "GetNumOfInputChannels")
        self.num_channels = numChannels.value

        self._tryfunc(self.lib.TH260_SetSyncDiv(
            ct.c_int(self.dev[0]), ct.c_int(self.sync_divider)), "SetSyncDiv")

        if self.model == "TimeHarp 260 P":
            self._tryfunc(self.lib.TH260_SetSyncCFD(
                ct.c_int(self.dev[0]), ct.c_int(self.sync_level),
                ct.c_int(TH_SYNC_CFD_ZERO_CROSS)), "SetSyncCFD")
            for ch in range(self.num_channels):
                self._tryfunc(self.lib.TH260_SetInputCFD(
                    ct.c_int(self.dev[0]), ct.c_int(ch),
                    ct.c_int(TH_INPUT_CFD_LEVEL),
                    ct.c_int(TH_INPUT_CFD_ZERO_CROSS)), "SetInputCFD")

        self._tryfunc(self.lib.TH260_SetSyncChannelOffset(
            ct.c_int(self.dev[0]), ct.c_int(0)), "SetSyncChannelOffset")
        for ch in range(self.num_channels):
            self._tryfunc(self.lib.TH260_SetInputChannelOffset(
                ct.c_int(self.dev[0]), ct.c_int(ch), ct.c_int(0)),
                "SetInputChannelOffset")

        histLen = ct.c_int()
        self._tryfunc(self.lib.TH260_SetHistoLen(
            ct.c_int(self.dev[0]), ct.c_int(TH_MAXLENCODE), byref(histLen)),
            "SetHistoLen")
        self.hist_len = histLen.value

        self._tryfunc(self.lib.TH260_SetBinning(
            ct.c_int(self.dev[0]), ct.c_int(0)), "SetBinning")
        self._tryfunc(self.lib.TH260_SetOffset(
            ct.c_int(self.dev[0]), ct.c_int(TH_OFFSET)), "SetOffset")

        resolution = ct.c_double()
        self._tryfunc(self.lib.TH260_GetResolution(
            ct.c_int(self.dev[0]), byref(resolution)), "GetResolution")
        self._hw_resolution_ps = float(resolution.value)
        self.log(f"Hardware resolution: {self._hw_resolution_ps:.4f} ps")

        time.sleep(0.05)
        self._tryfunc(self.lib.TH260_SetStopOverflow(
            ct.c_int(self.dev[0]), ct.c_int(0), ct.c_int(10000)),
            "SetStopOverflow")

        self.counts = [(ct.c_uint * TH_MAXHISTLEN)() for _ in range(TH_MAXINPCHAN)]
        self.opened = True

    def apply_sync_divider(self, value):
        self.sync_divider = int(value)
        if self.opened:
            self._tryfunc(self.lib.TH260_SetSyncDiv(
                ct.c_int(self.dev[0]), ct.c_int(self.sync_divider)), "SetSyncDiv")

    def apply_sync_level(self, value):
        self.sync_level = int(value)
        if self.opened and self.model == "TimeHarp 260 P":
            self._tryfunc(self.lib.TH260_SetSyncCFD(
                ct.c_int(self.dev[0]), ct.c_int(self.sync_level),
                ct.c_int(TH_SYNC_CFD_ZERO_CROSS)), "SetSyncCFD")

    def apply_resolution(self, value):
        try:
            v = float(value)
            if v > 0:
                self.resolution_override_ps = v
        except Exception:
            pass

    def get_sync_rate(self):
        sr = ct.c_int()
        self._tryfunc(self.lib.TH260_GetSyncRate(
            ct.c_int(self.dev[0]), byref(sr)), "GetSyncRate")
        return sr.value

    def get_count_rate(self, ch=0):
        cr = ct.c_int()
        self._tryfunc(self.lib.TH260_GetCountRate(
            ct.c_int(self.dev[0]), ct.c_int(ch), byref(cr)), "GetCountRate")
        return cr.value

    def _read_histogram(self, elapsed_ms):
        for ch in range(self.num_channels):
            self._tryfunc(self.lib.TH260_GetHistogram(
                ct.c_int(self.dev[0]), byref(self.counts[ch]),
                ct.c_int(ch), ct.c_int(0)), "GetHistogram")
        dt_s = max(elapsed_ms, 1) / 1000.0
        kin = [0.0] * self.hist_len
        for j in range(self.hist_len):
            total = 0
            for ch in range(self.num_channels):
                total += self.counts[ch][j]
            kin[j] = total / dt_s
        return kin

    @staticmethod
    def _apply_offset(kin, offset):
        if not kin:
            return list(kin)
        offset = int(offset) % len(kin)
        if offset == 0:
            return list(kin)
        return list(kin[offset:]) + list(kin[:offset])

    def measure_kinetics(self, tacq_ms, offset1=0,
                          progress_cb=None, chunk_ms=200,
                          cancel_event=None, kinetics_interval_ms=0):
        self._tryfunc(self.lib.TH260_ClearHistMem(
            ct.c_int(self.dev[0])), "ClearHistMem")
        sync_rate = self.get_sync_rate()

        dt_s = self.resolution_ps * 1e-12
        if sync_rate > 0:
            period_s = 1.0 / sync_rate
            period_bins = max(2, int(math.floor(period_s / dt_s)))
        else:
            period_bins = self.hist_len

        keep_bins = period_bins - 1

        tacq_ms = int(tacq_ms)
        done = 0
        last_kin = [0.0]
        last_kin_ms = 0

        while done < tacq_ms:
            if cancel_event is not None and cancel_event.is_set():
                break
            chunk = min(chunk_ms, tacq_ms - done)
            if chunk <= 0:
                break
            self._tryfunc(self.lib.TH260_StartMeas(
                ct.c_int(self.dev[0]), ct.c_int(chunk)), "StartMeas")
            ctcstatus = ct.c_int(0)
            while ctcstatus.value == 0:
                if cancel_event is not None and cancel_event.is_set():
                    break
                self._tryfunc(self.lib.TH260_CTCStatus(
                    ct.c_int(self.dev[0]), byref(ctcstatus)), "CTCStatus")
                time.sleep(0.005)
            self._tryfunc(self.lib.TH260_StopMeas(
                ct.c_int(self.dev[0])), "StopMeas")
            done += chunk

            raw = self._read_histogram(done)
            # Обрезка: убираем последнюю точку (по syncrate) и первую точку.
            end_idx = min(keep_bins, len(raw))
            if end_idx >= 2:
                trimmed = raw[1:end_idx]
            elif end_idx >= 1:
                trimmed = raw[:end_idx]
            else:
                trimmed = raw
            kin = self._apply_offset(trimmed, offset1)
            last_kin = kin

            if progress_cb is not None:
                if kinetics_interval_ms <= 0:
                    is_kin_time = True
                else:
                    is_kin_time = ((done - last_kin_ms) >= kinetics_interval_ms
                                   or done >= tacq_ms)
                if is_kin_time:
                    last_kin_ms = done
                try:
                    progress_cb(list(kin), sync_rate, done, tacq_ms, is_kin_time)
                except TypeError:
                    try:
                        progress_cb(list(kin), sync_rate, done, tacq_ms)
                    except Exception as e:
                        self.log(f"progress_cb error: {e}")
                except Exception as e:
                    self.log(f"progress_cb error: {e}")

            if cancel_event is not None and cancel_event.is_set():
                break

        cr = [self.get_count_rate(ch) for ch in range(self.num_channels)]
        return last_kin, sync_rate, cr

    def close(self):
        if not self.opened:
            return
        try:
            for i in range(TH_MAXDEVNUM):
                try:
                    self.lib.TH260_CloseDevice(ct.c_int(i))
                except Exception:
                    pass
        finally:
            self.opened = False


# ------------------------------------------------------------
class Plot1DTab:
    def __init__(self, parent, dtype, graph_win):
        self.parent = parent
        self.graph_win = graph_win
        self.dtype = dtype
        self.app = graph_win.master

        self.x_unit_mode = 0
        self.x_unit_labels = ["Длина волны, нм", "Энергия фотонов, мэВ",
                              "Волновое число, см⁻¹"]
        self.peak_annots_per_plot = []
        self.cursor_annotation = None

        top_frame = ttk.Frame(parent)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=top_frame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.canvas.mpl_connect('pick_event', self._on_pick)

        self.cursor_annotation = self.ax.annotate(
            "", xy=(0, 0), xytext=(10, 10), textcoords="offset points",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            fontsize=9, ha='left', va='top')
        self.cursor_annotation.set_visible(False)

        right_panel = ttk.Frame(top_frame, width=200)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        right_panel.pack_propagate(False)
        ttk.Label(right_panel, text="Графики:").pack(anchor="w")
        self.check_canvas = tk.Canvas(right_panel, width=180, height=120)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(right_panel, orient="vertical",
                           command=self.check_canvas.yview)
        sb.pack(side=tk.RIGHT, fill="y")
        self.check_frame = ttk.Frame(self.check_canvas)
        self.check_canvas.create_window((0, 0), window=self.check_frame, anchor="nw")
        self.check_frame.bind("<Configure>",
                              lambda e: self.check_canvas.configure(
                                  scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.configure(yscrollcommand=sb.set)

        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        actions_frame = ttk.LabelFrame(bottom_frame, text="Действия", padding=5)
        actions_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        btn_text = "Загрузить спектры" if dtype == 'spectra' else "Загрузить распред."
        ttk.Button(actions_frame, text=btn_text,
                   command=self.load_from_files).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Удалить выбранные",
                   command=self.delete_selected).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Поиск пиков",
                   command=self.find_peaks).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Сохранить в файл",
                   command=self.save_to_file).pack(anchor="w", fill="x", pady=1)

        if dtype == 'spectra':
            x_frame = ttk.LabelFrame(bottom_frame, text="Ось X", padding=5)
            x_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
            self.x_mode_label = ttk.Label(x_frame, text=self.x_unit_labels[0])
            self.x_mode_label.pack(anchor="w")
            ttk.Button(x_frame, text="Сменить единицы",
                       command=self.switch_x_unit).pack(anchor="w", pady=2)
        else:
            d_frame = ttk.LabelFrame(bottom_frame, text="Ось X", padding=5)
            d_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
            ttk.Label(d_frame, text="Позиция, мкм").pack(anchor="w")

        axes_frame = ttk.LabelFrame(bottom_frame, text="Оси", padding=5)
        axes_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(axes_frame, text="X min:").grid(row=0, column=0, sticky="e")
        self.xmin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmin_var, width=7).grid(row=0, column=1)
        ttk.Label(axes_frame, text="X max:").grid(row=0, column=2, sticky="e")
        self.xmax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmax_var, width=7).grid(row=0, column=3)
        ttk.Label(axes_frame, text="Y min:").grid(row=1, column=0, sticky="e")
        self.ymin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymin_var, width=7).grid(row=1, column=1)
        ttk.Label(axes_frame, text="Y max:").grid(row=1, column=2, sticky="e")
        self.ymax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymax_var, width=7).grid(row=1, column=3)
        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(axes_frame, text="Автомасштаб",
                        variable=self.autoscale_var).grid(
            row=2, column=0, columnspan=2, sticky="w")
        self.logy_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(axes_frame, text="Log Y",
                        variable=self.logy_var,
                        command=self.apply_axes).grid(
            row=2, column=2, sticky="w")
        ttk.Button(axes_frame, text="Применить",
                   command=self.apply_axes).grid(
            row=2, column=3, pady=5)

        self.plots = []
        self.plot_vars = []
        self.color_cycle = cycle(ORIGIN_COLORS)

        self.ax.set_ylabel("CPS")
        self.ax.set_title("Спектр" if dtype == 'spectra' else "Распределение")
        self.ax.set_xlabel(self.x_unit_labels[0] if dtype == 'spectra'
                           else "Позиция, мкм")
        self.apply_axes()

    def add_plot(self, x, y, label):
        color = next(self.color_cycle)
        x_arr = np.array(x, dtype=float)
        y_arr = np.array(y, dtype=float)
        line, = self.ax.plot(x_arr, y_arr, color=color, label=label)
        var = tk.BooleanVar(value=True)
        cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                             command=lambda idx=len(self.plots):
                             self.toggle_visibility(idx))
        cb.pack(anchor="w")
        self.plots.append({
            'x_orig': x_arr.copy(), 'y_orig': y_arr.copy(),
            'x': x_arr.copy(), 'y': y_arr.copy(),
            'label': label, 'line': line, 'color': color, 'var': var})
        self.plot_vars.append(var)
        self.peak_annots_per_plot.append([])
        self.canvas.draw_idle()

    def toggle_visibility(self, idx):
        if idx >= len(self.plots):
            return
        plot = self.plots[idx]
        vis = plot['var'].get()
        plot['line'].set_visible(vis)
        for ann in self.peak_annots_per_plot[idx]:
            ann.set_visible(vis)
        self.canvas.draw_idle()

    def delete_selected(self):
        to_remove = [i for i, p in enumerate(self.plots) if p['var'].get()]
        if not to_remove:
            messagebox.showinfo("Удаление", "Нет выбранных графиков.")
            return
        for i in sorted(to_remove, reverse=True):
            self.plots[i]['line'].remove()
            for ann in self.peak_annots_per_plot[i]:
                ann.remove()
            del self.peak_annots_per_plot[i]
            del self.plots[i]
            del self.plot_vars[i]
        for w in self.check_frame.winfo_children():
            w.destroy()
        for idx, p in enumerate(self.plots):
            cb = ttk.Checkbutton(self.check_frame, text=p['label'],
                                 variable=p['var'],
                                 command=lambda i=idx: self.toggle_visibility(i))
            cb.pack(anchor="w")
        self.canvas.draw_idle()

    def apply_axes(self):
        try:
            if self.logy_var.get():
                self.ax.set_yscale('log')
            else:
                self.ax.set_yscale('linear')
        except Exception:
            pass
        if not self.autoscale_var.get():
            try:
                xmin = float(self.xmin_var.get()) if self.xmin_var.get() else None
                xmax = float(self.xmax_var.get()) if self.xmax_var.get() else None
                ymin = float(self.ymin_var.get()) if self.ymin_var.get() else None
                ymax = float(self.ymax_var.get()) if self.ymax_var.get() else None
                if xmin is not None:
                    self.ax.set_xlim(left=xmin)
                if xmax is not None:
                    self.ax.set_xlim(right=xmax)
                if ymin is not None:
                    self.ax.set_ylim(bottom=ymin)
                if ymax is not None:
                    self.ax.set_ylim(top=ymax)
            except ValueError:
                pass
        else:
            self.ax.autoscale()
        self.canvas.draw_idle()

    def load_from_files(self):
        files = filedialog.askopenfilenames(
            title="Выберите файлы данных",
            filetypes=[("Text files", "*.txt"), ("Data files", "*.dat"),
                       ("All files", "*.*")])
        for fpath in files:
            try:
                x, y = self._parse_data_file(fpath)
                label = os.path.splitext(os.path.basename(fpath))[0]
                self.add_plot(x, y, label)
            except Exception as e:
                messagebox.showwarning("Ошибка", f"{fpath}:\n{e}")

    def _parse_data_file(self, filepath):
        x, y = [], []
        with open(filepath, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
        if not x:
            raise ValueError("Файл не содержит 2 колонок")
        return x, y

    def _on_mouse_move(self, event):
        if event.inaxes == self.ax:
            self.cursor_annotation.set_visible(True)
            self.cursor_annotation.xy = (event.xdata, event.ydata)
            self.cursor_annotation.set_text(
                f"X={event.xdata:.4f}\nY={event.ydata:.4f}")
        else:
            self.cursor_annotation.set_visible(False)
        self.canvas.draw_idle()

    def _on_pick(self, event):
        if not hasattr(event, 'artist'):
            return
        ann = event.artist
        for idx, annots in enumerate(self.peak_annots_per_plot):
            if ann in annots:
                ann.remove()
                annots.remove(ann)
                self.canvas.draw_idle()
                break

    def find_peaks(self):
        for annots in self.peak_annots_per_plot:
            for ann in annots:
                ann.remove()
            annots.clear()

        for idx, p in enumerate(self.plots):
            if not p['var'].get():
                continue
            x = p['x']
            y = p['y']
            peaks_idx = self._simple_find_peaks(x, y)
            for i in peaks_idx:
                ann = self.ax.annotate(
                    f'{x[i]:.2f}', (x[i], y[i]),
                    textcoords="offset points", xytext=(0, 12),
                    ha='center', fontsize=8, color=p['color'],
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                              alpha=0.7, edgecolor='none'),
                    picker=5)
                self.peak_annots_per_plot[idx].append(ann)
        self.canvas.draw_idle()

    def _simple_find_peaks(self, x, y, height_frac=0.03, distance_idx=5):
        if len(y) < 3:
            return []
        window = max(3, len(y) // 50)
        if window % 2 == 0:
            window += 1
        y_sm = np.convolve(y, np.ones(window) / window, mode='same')
        peaks = []
        for i in range(1, len(y_sm) - 1):
            if y_sm[i] > y_sm[i - 1] and y_sm[i] > y_sm[i + 1]:
                peaks.append(i)
        if not peaks:
            return []
        max_y = np.max(y_sm)
        peaks = [i for i in peaks if y_sm[i] >= height_frac * max_y]
        filtered = []
        for i in peaks:
            left = max(0, i - distance_idx)
            right = min(len(y_sm) - 1, i + distance_idx)
            if y_sm[i] == np.max(y_sm[left:right + 1]):
                filtered.append(i)
        return sorted(list(set(filtered)))

    def save_to_file(self):
        visible = [(p['x'], p['y'], p['label'])
                   for p in self.plots if p['var'].get()]
        if not visible:
            messagebox.showwarning("Сохранение", "Нет видимых графиков.")
            return
        min_x = max([np.min(x) for x, y, _ in visible])
        max_x = min([np.max(x) for x, y, _ in visible])
        if min_x >= max_x:
            messagebox.showerror("Ошибка", "Диапазоны X не пересекаются.")
            return
        dx = None
        for x, y, _ in visible:
            if len(x) > 1:
                d = np.min(np.diff(x))
                if d > 0 and (dx is None or d < dx):
                    dx = d
        if dx is None or dx <= 0:
            dx = (max_x - min_x) / 1000
        n = int((max_x - min_x) / dx) + 1
        common_x = np.linspace(min_x, max_x, n)

        interp = []
        labels = []
        for x, y, label in visible:
            interp.append(np.interp(common_x, x, y))
            labels.append(label)

        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Сохранить объединённые данные")
        if not filepath:
            return
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("X\t" + "\t".join(labels) + "\n")
                for i in range(len(common_x)):
                    line = f"{common_x[i]:.6f}"
                    for y_arr in interp:
                        line += f"\t{y_arr[i]:.6f}"
                    f.write(line + "\n")
            self.app.data_queue.put(("log", f"Сохранено: {filepath}"))
        except Exception as e:
            self.app.data_queue.put(("error", f"Ошибка сохранения: {e}"))

    def switch_x_unit(self):
        if self.dtype != 'spectra':
            return
        self.x_unit_mode = (self.x_unit_mode + 1) % 3
        label = self.x_unit_labels[self.x_unit_mode]
        self.x_mode_label.config(text=label)
        self.ax.set_xlabel(label)
        for p in self.plots:
            orig_x = p['x_orig']
            if self.x_unit_mode == 0:
                new_x = orig_x
            elif self.x_unit_mode == 1:
                new_x = 1240.0 / orig_x * 1000.0
            else:
                new_x = 1e7 / orig_x
            p['x'] = new_x
            p['line'].set_data(new_x, p['y'])
        for annots in self.peak_annots_per_plot:
            for ann in annots:
                ann.remove()
            annots.clear()
        self.ax.relim()
        self.apply_axes()
        self.canvas.draw_idle()


# ------------------------------------------------------------
class PlotKineticsTab:
    def __init__(self, parent, graph_win):
        self.parent = parent
        self.graph_win = graph_win
        self.app = graph_win.master

        self.cursor_annotation = None
        self.plots = []
        self.color_cycle = cycle(ORIGIN_COLORS)

        top_frame = ttk.Frame(parent)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=top_frame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)

        self.cursor_annotation = self.ax.annotate(
            "", xy=(0, 0), xytext=(10, 10), textcoords="offset points",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            fontsize=9, ha='left', va='top')
        self.cursor_annotation.set_visible(False)

        right_panel = ttk.Frame(top_frame, width=200)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        right_panel.pack_propagate(False)
        ttk.Label(right_panel, text="Кинетики:").pack(anchor="w")
        self.check_canvas = tk.Canvas(right_panel, width=180, height=120)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(right_panel, orient="vertical",
                           command=self.check_canvas.yview)
        sb.pack(side=tk.RIGHT, fill="y")
        self.check_frame = ttk.Frame(self.check_canvas)
        self.check_canvas.create_window((0, 0), window=self.check_frame, anchor="nw")
        self.check_frame.bind("<Configure>",
                              lambda e: self.check_canvas.configure(
                                  scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.configure(yscrollcommand=sb.set)

        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        actions_frame = ttk.LabelFrame(bottom_frame, text="Действия", padding=5)
        actions_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Button(actions_frame, text="Загрузить кинетики",
                   command=self.load_from_files).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Удалить выбранные",
                   command=self.delete_selected).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Сохранить в файл",
                   command=self.save_to_file).pack(anchor="w", fill="x", pady=1)

        axes_frame = ttk.LabelFrame(bottom_frame, text="Оси", padding=5)
        axes_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(axes_frame, text="X min:").grid(row=0, column=0, sticky="e")
        self.xmin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmin_var, width=7).grid(row=0, column=1)
        ttk.Label(axes_frame, text="X max:").grid(row=0, column=2, sticky="e")
        self.xmax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmax_var, width=7).grid(row=0, column=3)
        ttk.Label(axes_frame, text="Y min:").grid(row=1, column=0, sticky="e")
        self.ymin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymin_var, width=7).grid(row=1, column=1)
        ttk.Label(axes_frame, text="Y max:").grid(row=1, column=2, sticky="e")
        self.ymax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymax_var, width=7).grid(row=1, column=3)
        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(axes_frame, text="Автомасштаб",
                        variable=self.autoscale_var).grid(
            row=2, column=0, columnspan=2, sticky="w")
        self.logy_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(axes_frame, text="Log Y",
                        variable=self.logy_var,
                        command=self.apply_axes).grid(
            row=2, column=2, sticky="w")
        ttk.Button(axes_frame, text="Применить",
                   command=self.apply_axes).grid(
            row=2, column=3, pady=5)

        self.ax.set_xlabel("Время, нс")
        self.ax.set_ylabel("Интенсивность, CPS")
        self.ax.set_title("Кинетики")
        self.apply_axes()

    def add_plot(self, times_ns, kinetics, label):
        color = next(self.color_cycle)
        x_arr = np.array(times_ns, dtype=float)
        y_arr = np.array(kinetics, dtype=float)
        line, = self.ax.plot(x_arr, y_arr, color=color, label=label, linewidth=1.2)
        var = tk.BooleanVar(value=True)
        cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                             command=lambda idx=len(self.plots):
                             self.toggle_visibility(idx))
        cb.pack(anchor="w")
        self.plots.append({
            'x': x_arr.copy(), 'y': y_arr.copy(),
            'label': label, 'line': line, 'color': color, 'var': var})
        self.ax.relim()
        self.apply_axes()
        self.canvas.draw_idle()

    def toggle_visibility(self, idx):
        if idx >= len(self.plots):
            return
        plot = self.plots[idx]
        plot['line'].set_visible(plot['var'].get())
        self.canvas.draw_idle()

    def delete_selected(self):
        to_remove = [i for i, p in enumerate(self.plots) if p['var'].get()]
        if not to_remove:
            messagebox.showinfo("Удаление", "Нет выбранных кинетик.")
            return
        for i in sorted(to_remove, reverse=True):
            self.plots[i]['line'].remove()
            del self.plots[i]
        for w in self.check_frame.winfo_children():
            w.destroy()
        for idx, p in enumerate(self.plots):
            cb = ttk.Checkbutton(self.check_frame, text=p['label'],
                                 variable=p['var'],
                                 command=lambda i=idx: self.toggle_visibility(i))
            cb.pack(anchor="w")
        self.ax.relim()
        self.apply_axes()
        self.canvas.draw_idle()

    def apply_axes(self):
        try:
            if self.logy_var.get():
                self.ax.set_yscale('log')
            else:
                self.ax.set_yscale('linear')
        except Exception:
            pass
        if not self.autoscale_var.get():
            try:
                xmin = float(self.xmin_var.get()) if self.xmin_var.get() else None
                xmax = float(self.xmax_var.get()) if self.xmax_var.get() else None
                ymin = float(self.ymin_var.get()) if self.ymin_var.get() else None
                ymax = float(self.ymax_var.get()) if self.ymax_var.get() else None
                if xmin is not None:
                    self.ax.set_xlim(left=xmin)
                if xmax is not None:
                    self.ax.set_xlim(right=xmax)
                if ymin is not None:
                    self.ax.set_ylim(bottom=ymin)
                if ymax is not None:
                    self.ax.set_ylim(top=ymax)
            except ValueError:
                pass
        else:
            self.ax.autoscale()
        self.canvas.draw_idle()

    def load_from_files(self):
        files = filedialog.askopenfilenames(
            title="Выберите файлы кинетик",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        for fpath in files:
            try:
                x, y = self._parse_data_file(fpath)
                x_arr = np.array(x, dtype=float)
                if x_arr.size > 0 and x_arr.max() < 1e-3:
                    x_arr = x_arr * 1e9
                label = os.path.splitext(os.path.basename(fpath))[0]
                self.add_plot(list(x_arr), y, label)
            except Exception as e:
                messagebox.showwarning("Ошибка", f"{fpath}:\n{e}")

    def _parse_data_file(self, filepath):
        x, y = [], []
        with open(filepath, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
        if not x:
            raise ValueError("Файл не содержит 2 колонок")
        return x, y

    def _on_mouse_move(self, event):
        if event.inaxes == self.ax:
            self.cursor_annotation.set_visible(True)
            self.cursor_annotation.xy = (event.xdata, event.ydata)
            self.cursor_annotation.set_text(
                f"t={event.xdata:.3f} нс\nY={event.ydata:.3f}")
        else:
            self.cursor_annotation.set_visible(False)
        self.canvas.draw_idle()

    def save_to_file(self):
        visible = [(p['x'], p['y'], p['label'])
                   for p in self.plots if p['var'].get()]
        if not visible:
            messagebox.showwarning("Сохранение", "Нет видимых кинетик.")
            return
        min_x = max([np.min(x) for x, y, _ in visible])
        max_x = min([np.max(x) for x, y, _ in visible])
        if min_x >= max_x:
            messagebox.showerror("Ошибка", "Диапазоны X не пересекаются.")
            return
        dx = None
        for x, y, _ in visible:
            if len(x) > 1:
                d = np.min(np.diff(x))
                if d > 0 and (dx is None or d < dx):
                    dx = d
        if dx is None or dx <= 0:
            dx = (max_x - min_x) / 1000
        n = int((max_x - min_x) / dx) + 1
        common_x = np.linspace(min_x, max_x, n)
        interp = []
        labels = []
        for x, y, label in visible:
            interp.append(np.interp(common_x, x, y))
            labels.append(label)
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Сохранить объединённые кинетики")
        if not filepath:
            return
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("Time(ns)\t" + "\t".join(labels) + "\n")
                for i in range(len(common_x)):
                    line = f"{common_x[i]:.6f}"
                    for y_arr in interp:
                        line += f"\t{y_arr[i]:.6f}"
                    f.write(line + "\n")
            self.app.data_queue.put(("log", f"Сохранено: {filepath}"))
        except Exception as e:
            self.app.data_queue.put(("error", f"Ошибка сохранения: {e}"))


# ------------------------------------------------------------
class Plot3DTab:
    def __init__(self, parent, graph_win):
        self.parent = parent
        self.graph_win = graph_win
        self.app = graph_win.master

        top_frame = ttk.Frame(parent)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=top_frame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(top_frame, width=200)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        right_panel.pack_propagate(False)
        ttk.Label(right_panel, text="3D карты:").pack(anchor="w")
        self.check_canvas = tk.Canvas(right_panel, width=180, height=120)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(right_panel, orient="vertical",
                           command=self.check_canvas.yview)
        sb.pack(side=tk.RIGHT, fill="y")
        self.check_frame = ttk.Frame(self.check_canvas)
        self.check_canvas.create_window((0, 0), window=self.check_frame, anchor="nw")
        self.check_frame.bind("<Configure>",
                              lambda e: self.check_canvas.configure(
                                  scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.configure(yscrollcommand=sb.set)

        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        actions_frame = ttk.LabelFrame(bottom_frame, text="Действия", padding=5)
        actions_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Button(actions_frame, text="Загрузить 3D карту",
                   command=self.load_from_files).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Удалить выбранные",
                   command=self.delete_selected).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Сохранить в Origin",
                   command=self.save_to_origin).pack(anchor="w", fill="x", pady=1)

        labels_frame = ttk.LabelFrame(bottom_frame, text="Подписи", padding=5)
        labels_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(labels_frame, text="Подпись X:").grid(row=0, column=0, sticky="w")
        self.xlabel_var = tk.StringVar(value="Длина волны, нм")
        ttk.Entry(labels_frame, textvariable=self.xlabel_var,
                  width=12).grid(row=0, column=1, padx=5)
        ttk.Label(labels_frame, text="Подпись Y:").grid(row=1, column=0, sticky="w")
        self.ylabel_var = tk.StringVar(value="Позиция, мкм")
        ttk.Entry(labels_frame, textvariable=self.ylabel_var,
                  width=12).grid(row=1, column=1, padx=5)
        ttk.Label(labels_frame, text="Подпись Z:").grid(row=2, column=0, sticky="w")
        self.zlabel_var = tk.StringVar(value="CPS")
        ttk.Entry(labels_frame, textvariable=self.zlabel_var,
                  width=12).grid(row=2, column=1, padx=5)
        ttk.Label(labels_frame, text="Заголовок:").grid(row=3, column=0, sticky="w")
        self.title_var = tk.StringVar(value="")
        ttk.Entry(labels_frame, textvariable=self.title_var,
                  width=12).grid(row=3, column=1, padx=5)
        ttk.Button(labels_frame, text="Применить",
                   command=self.apply_labels).grid(row=4, column=0,
                                                   columnspan=2, pady=2)

        axes_frame = ttk.LabelFrame(bottom_frame, text="Оси", padding=5)
        axes_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(axes_frame, text="X min:").grid(row=0, column=0, sticky="e")
        self.xmin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmin_var, width=7).grid(row=0, column=1)
        ttk.Label(axes_frame, text="X max:").grid(row=0, column=2, sticky="e")
        self.xmax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.xmax_var, width=7).grid(row=0, column=3)
        ttk.Label(axes_frame, text="Y min:").grid(row=1, column=0, sticky="e")
        self.ymin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymin_var, width=7).grid(row=1, column=1)
        ttk.Label(axes_frame, text="Y max:").grid(row=1, column=2, sticky="e")
        self.ymax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.ymax_var, width=7).grid(row=1, column=3)
        ttk.Label(axes_frame, text="Z min:").grid(row=2, column=0, sticky="e")
        self.zmin_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.zmin_var, width=7).grid(row=2, column=1)
        ttk.Label(axes_frame, text="Z max:").grid(row=2, column=2, sticky="e")
        self.zmax_var = tk.StringVar(value="")
        ttk.Entry(axes_frame, textvariable=self.zmax_var, width=7).grid(row=2, column=3)
        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(axes_frame, text="Автомасштаб",
                        variable=self.autoscale_var).grid(
            row=3, column=0, columnspan=2, sticky="w")
        ttk.Button(axes_frame, text="Применить",
                   command=self.apply_axes).grid(
            row=3, column=2, columnspan=2, pady=5)

        self.maps = []
        self.map_vars = []
        self.current_im = None
        self.cbar = None

    def add_plot(self, matrix, x, y, label):
        var = tk.BooleanVar(value=True)
        cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                             command=lambda idx=len(self.maps):
                             self.toggle_visibility(idx))
        cb.pack(anchor="w")
        self.maps.append((matrix, x, y, label, None, var))
        self.map_vars.append(var)
        self.redraw()

    def toggle_visibility(self, idx):
        self.redraw()

    def delete_selected(self):
        to_remove = [i for i, (_, _, _, _, _, var)
                     in enumerate(self.maps) if var.get()]
        if not to_remove:
            messagebox.showinfo("Удаление", "Нет выбранных карт.")
            return
        for i in sorted(to_remove, reverse=True):
            del self.maps[i]
            del self.map_vars[i]
        for w in self.check_frame.winfo_children():
            w.destroy()
        for idx, (_, _, _, label, _, var) in enumerate(self.maps):
            cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                                 command=lambda i=idx: self.toggle_visibility(i))
            cb.pack(anchor="w")
        self.redraw()

    def redraw(self):
        self.ax.clear()
        if self.cbar:
            self.cbar.remove()
            self.cbar = None
        vis_maps = [(m, x, y, label)
                    for (m, x, y, label, im, var) in self.maps if var.get()]
        if vis_maps:
            matrix, x, y, label = vis_maps[-1]
            im = self.ax.imshow(matrix, aspect='auto', origin='lower',
                                extent=[x[0], x[-1], y[0], y[-1]])
            self.cbar = self.figure.colorbar(im, ax=self.ax,
                                             label=self.zlabel_var.get())
            self.current_im = im
        self.apply_labels()
        self.apply_axes()
        self.canvas.draw_idle()

    def apply_labels(self):
        self.ax.set_xlabel(self.xlabel_var.get())
        self.ax.set_ylabel(self.ylabel_var.get())
        if self.cbar:
            self.cbar.set_label(self.zlabel_var.get())
        self.ax.set_title(self.title_var.get())
        self.canvas.draw_idle()

    def apply_axes(self):
        if not self.autoscale_var.get():
            try:
                xmin = float(self.xmin_var.get()) if self.xmin_var.get() else None
                xmax = float(self.xmax_var.get()) if self.xmax_var.get() else None
                ymin = float(self.ymin_var.get()) if self.ymin_var.get() else None
                ymax = float(self.ymax_var.get()) if self.ymax_var.get() else None
                if xmin is not None:
                    self.ax.set_xlim(left=xmin)
                if xmax is not None:
                    self.ax.set_xlim(right=xmax)
                if ymin is not None:
                    self.ax.set_ylim(bottom=ymin)
                if ymax is not None:
                    self.ax.set_ylim(top=ymax)
                if self.current_im:
                    zmin = float(self.zmin_var.get()) if self.zmin_var.get() else None
                    zmax = float(self.zmax_var.get()) if self.zmax_var.get() else None
                    if zmin is not None:
                        self.current_im.set_clim(vmin=zmin)
                    if zmax is not None:
                        self.current_im.set_clim(vmax=zmax)
            except ValueError:
                pass
        else:
            self.ax.autoscale()
            if self.current_im:
                self.current_im.autoscale()
        self.canvas.draw_idle()

    def load_from_files(self):
        mat_file = filedialog.askopenfilename(
            title="Выберите файл матрицы (matrix.txt)",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not mat_file:
            return
        folder = os.path.dirname(mat_file)
        pos_file = os.path.join(folder, "positions.txt")
        wl_file = os.path.join(folder, "wavelengths.txt")
        if not os.path.exists(pos_file) or not os.path.exists(wl_file):
            messagebox.showerror("Ошибка",
                                 "Рядом с matrix.txt должны лежать "
                                 "positions.txt и wavelengths.txt")
            return
        try:
            matrix = np.loadtxt(mat_file)
            pos = np.loadtxt(pos_file)
            wls = np.loadtxt(wl_file)
            label = os.path.splitext(os.path.basename(mat_file))[0]
            self.add_plot(matrix, wls, pos, label)
        except Exception as e:
            messagebox.showerror("Ошибка загрузки 3D", str(e))

    def save_to_origin(self):
        vis_maps = [(m, x, y, label)
                    for (m, x, y, label, im, var) in self.maps if var.get()]
        if not vis_maps:
            messagebox.showwarning("Предупреждение", "Нет видимых 3D карт.")
            return
        filepath = filedialog.asksaveasfilename(
            defaultextension=".opju",
            filetypes=[("Origin Project", "*.opju")],
            title="Сохранить проект Origin")
        if not filepath:
            return
        xlabel = self.xlabel_var.get()
        ylabel = self.ylabel_var.get()
        zlabel = self.zlabel_var.get()
        title = self.title_var.get()
        self.app.data_queue.put(("log", "Сохранение 3D в Origin..."))
        threading.Thread(target=self._save_to_origin_thread,
                         args=(vis_maps, filepath, xlabel, ylabel, zlabel, title),
                         daemon=True).start()

    def _save_to_origin_thread(self, vis_maps, filepath, xlabel, ylabel,
                               zlabel, title):
        try:
            import originpro as op
        except ImportError:
            self.app.data_queue.put(("error", "originpro не установлена."))
            return
        try:
            op.new()
            for i, (matrix, x, y, label) in enumerate(vis_maps, start=1):
                mat_sheet = op.new_sheet('m', label[:30].replace(' ', '_'))
                mat_sheet.from_np(matrix)
                mat_sheet.activate()
                mat_sheet.lt_exec("worksheet -p 240 contour;")
                ag = op.find_graph()
                if ag:
                    gl = ag[0]
                    gl.lt_exec(f'label -x "{xlabel}";')
                    gl.lt_exec(f'label -y "{ylabel}";')
                    gl.lt_exec(f'label -z "{zlabel}";')
                    gl.lt_exec(f'title.text$ = "{title}";')
            save_dir = os.path.dirname(filepath)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            ok = op.save(filepath)
            self.app.data_queue.put(("log", "Проект сохранён." if ok
                                     else "Origin API вернул False."))
        except Exception as e:
            self.app.data_queue.put(("error", f"Ошибка Origin: {e}"))


# ------------------------------------------------------------
class GraphWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.title("Графики")
        self.geometry("820x780")
        self.protocol("WM_DELETE_WINDOW", self.hide)
        self.withdraw()

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        self.tab_spec = ttk.Frame(self.notebook)
        self.tab_dist = ttk.Frame(self.notebook)
        self.tab_3d = ttk.Frame(self.notebook)
        self.tab_kin = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_spec, text="Спектры")
        self.notebook.add(self.tab_dist, text="Распределения")
        self.notebook.add(self.tab_kin, text="Кинетики")
        self.notebook.add(self.tab_3d, text="3D спектры")

        self.spec_tab = Plot1DTab(self.tab_spec, 'spectra', self)
        self.dist_tab = Plot1DTab(self.tab_dist, 'distributions', self)
        self.kin_tab = PlotKineticsTab(self.tab_kin, self)
        self.tab3d = Plot3DTab(self.tab_3d, self)

    def show(self):
        self.deiconify()
        self.position()

    def hide(self):
        self.withdraw()

    def position(self):
        self.update_idletasks()
        master = self.master
        x = master.winfo_rootx() + master.winfo_width()
        y = master.winfo_rooty()
        self.geometry(f"+{x}+{y}")

    def add_spectrum(self, x, y, label):
        self.spec_tab.add_plot(x, y, label)

    def add_distribution(self, x, y, label):
        self.dist_tab.add_plot(x, y, label)

    def add_kinetics(self, times_ns, kinetics, label):
        self.kin_tab.add_plot(times_ns, kinetics, label)

    def add_3d_map(self, matrix, x, y, label):
        self.tab3d.add_plot(matrix, x, y, label)


# ------------------------------------------------------------
class Application(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Спектры, распределения и кинетики ФЛ "
                   "(Scontel/TimeHarp + Standa + Acton)")
        self.geometry("1080x960")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.mono = None
        self.axis = None
        self.pos_axis = None
        self.th = None
        self.measurement_thread = None
        self.stop_requested = False
        self.data_queue = queue.Queue()
        self.closing = False

        self._th_poll_active = False
        self.last_kinetics_data = None

        self.osc_cancel_event = threading.Event()

        self.detector_var = tk.StringVar(value="Scontel")

        self.th_sync_divider = tk.IntVar(value=TH_SYNC_DIVIDER_DEFAULT)
        self.th_sync_level = tk.IntVar(value=TH_SYNC_LEVEL_DEFAULT)
        self.th_offset1 = tk.IntVar(value=TH_OFFSET1_DEFAULT)
        self.th_resolution = tk.DoubleVar(value=TH_RESOLUTION_DEFAULT)
        self.th_cps_var = tk.StringVar(value="--")
        self.th_sync_var = tk.StringVar(value="--")
        self.save_kinetics_var = tk.BooleanVar(value=False)
        self.oscilloscope_var = tk.BooleanVar(value=False)
        self.auto_offset_var = tk.BooleanVar(value=False)
        self.kin_logy_var = tk.BooleanVar(value=False)

        self.folder_path = tk.StringVar(value=os.path.expanduser("~"))
        self.filename = tk.StringVar(value="01_spectrum")
        self.scontel_ip = tk.StringVar(value="169.254.149.79")
        self.scontel_dev = tk.IntVar(value=2)
        self.accum_time = tk.DoubleVar(value=1.0)

        self.mode_var = tk.IntVar(value=0)

        self.wl_start = tk.DoubleVar(value=1000.0)
        self.wl_end = tk.DoubleVar(value=1900.0)
        self.wl_step = tk.DoubleVar(value=5.0)
        self.com_port = tk.StringVar(value="")

        self.just_wl = tk.DoubleVar(value=0.0)

        self.step_um = tk.DoubleVar(value=10.0)
        self.start_pos_um = tk.DoubleVar(value=0.0)
        self.end_pos_um = tk.DoubleVar(value=100.0)

        self.target_um = tk.DoubleVar(value=0.0)
        self.pos_current_var = tk.StringVar(value="--")

        self.view_3d = tk.BooleanVar(value=False)
        self.vmin = tk.DoubleVar(value=0.0)
        self.vmax = tk.DoubleVar(value=1000.0)
        self.show_grid = tk.BooleanVar(value=True)
        self.last_2d_data = None

        self.offset_entry = None

        self.kinetics_aux_visible = False
        self.kin_aux_axes = None

        self.graph_window = GraphWindow(self)

        self.create_widgets()

        self.detector_var.trace_add("write",
                                    lambda *a: self.update_detector_ui())
        self.update_detector_ui()
        self.update_kinetics_aux_visibility()

        # Разделитель основного/вспомогательного окна — по умолчанию 50/50
        self.after(300, self._set_paned_50_50)

        self.after(100, self.process_queue)

    def create_widgets(self):
        left_frame = ttk.Frame(self, width=460)
        left_frame.pack(side="left", fill="y", padx=5, pady=5)
        left_frame.pack_propagate(False)

        right_frame = ttk.Frame(self, width=580)
        right_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        right_frame.pack_propagate(False)

        common_frame = ttk.LabelFrame(left_frame, text="Общие настройки",
                                      padding=8)
        common_frame.pack(fill="x", pady=4)
        ttk.Label(common_frame, text="Папка:").grid(row=0, column=0, sticky="w")
        ttk.Entry(common_frame, textvariable=self.folder_path,
                  width=32).grid(row=0, column=1, padx=5, columnspan=2, sticky="we")
        ttk.Button(common_frame, text="Обзор",
                   command=self.browse_folder).grid(row=0, column=3)
        ttk.Label(common_frame, text="Имя файла:").grid(row=1, column=0, sticky="w", pady=2)
        ttk.Entry(common_frame, textvariable=self.filename,
                  width=32).grid(row=1, column=1, padx=5, columnspan=2, sticky="we")

        self.scontel_ip_label = ttk.Label(common_frame, text="IP Scontel:")
        self.scontel_ip_label.grid(row=2, column=0, sticky="w", pady=2)
        self.scontel_ip_frame = ttk.Frame(common_frame)
        self.scontel_ip_frame.grid(row=2, column=1, columnspan=3, sticky="w")
        ttk.Entry(self.scontel_ip_frame, textvariable=self.scontel_ip,
                  width=15).pack(side="left", padx=5)
        self.scontel_dev_label = ttk.Label(self.scontel_ip_frame, text="Устр:")
        self.scontel_dev_label.pack(side="left")
        self.scontel_dev_entry = ttk.Entry(self.scontel_ip_frame,
                                           textvariable=self.scontel_dev, width=5)
        self.scontel_dev_entry.pack(side="left")

        com_frame = ttk.Frame(common_frame)
        com_frame.grid(row=3, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(com_frame, text="COM Acton:").pack(side="left")
        self.com_combo = ttk.Combobox(com_frame, textvariable=self.com_port, width=10)
        self.com_combo.pack(side="left", padx=5)
        self.com_combo.bind("<Button-1>", lambda e: self.refresh_com_ports())
        ttk.Button(com_frame, text="Найти",
                   command=self.detect_acton_port).pack(side="left", padx=2)

        # --- Детектор ---
        self.det_frame = ttk.LabelFrame(left_frame, text="Детектор", padding=8)
        self.det_frame.pack(fill="x", pady=4)
        det_row = ttk.Frame(self.det_frame)
        det_row.pack(fill="x")
        ttk.Radiobutton(det_row, text="Scontel (CPS)",
                        variable=self.detector_var,
                        value="Scontel").pack(side="left", padx=(0, 15))
        ttk.Radiobutton(det_row, text="TimeHarp (кинетика)",
                        variable=self.detector_var,
                        value="TimeHarp").pack(side="left")

        # --- Параметры TimeHarp ---
        self.th_frame = ttk.LabelFrame(left_frame, text="Параметры TimeHarp",
                                       padding=8)

        row1 = ttk.Frame(self.th_frame)
        row1.pack(fill="x")
        ttk.Label(row1, text="Freq.div:").pack(side="left")
        ttk.Entry(row1, textvariable=self.th_sync_divider, width=5).pack(
            side="left", padx=(3, 10))
        ttk.Label(row1, text="syncLevel:").pack(side="left")
        ttk.Entry(row1, textvariable=self.th_sync_level, width=5).pack(
            side="left", padx=(3, 10))
        ttk.Label(row1, text="offset:").pack(side="left")
        self.offset_entry = ttk.Entry(row1, textvariable=self.th_offset1, width=5)
        self.offset_entry.pack(side="left", padx=(3, 10))
        ttk.Label(row1, text="Resolution(ps):").pack(side="left")
        ttk.Entry(row1, textvariable=self.th_resolution, width=7).pack(
            side="left", padx=(3, 10))

        row2 = ttk.Frame(self.th_frame)
        row2.pack(fill="x", pady=(4, 0))
        info = ttk.Frame(row2)
        info.pack(side="left", fill="x", expand=True)
        ttk.Label(info, text="CPS:").pack(side="left")
        ttk.Label(info, textvariable=self.th_cps_var,
                  font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(3, 15))
        ttk.Label(info, text="Sync rate:").pack(side="left")
        ttk.Label(info, textvariable=self.th_sync_var,
                  font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=3)
        ttk.Button(row2, text="Применить",
                   command=self._apply_timeharp_params).pack(side="right")

        row3 = ttk.Frame(self.th_frame)
        row3.pack(fill="x", pady=(4, 0))
        ttk.Checkbutton(row3, text="auto offset",
                        variable=self.auto_offset_var,
                        command=self._on_auto_offset_toggle).pack(side="left")

        # --- Статус устройств ---
        status_frame = ttk.LabelFrame(left_frame, text="Статус устройств",
                                      padding=8)
        status_frame.pack(fill="x", pady=4)
        ind_row = ttk.Frame(status_frame)
        ind_row.pack(anchor="center")

        self.scontel_ind = ttk.Frame(ind_row)
        self.scontel_canvas = tk.Canvas(self.scontel_ind, width=18, height=18,
                                        highlightthickness=0)
        self.scontel_canvas.pack(side="left")
        self.scontel_circle = self.scontel_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(self.scontel_ind, text="Scontel").pack(side="left", padx=(2, 0))
        self.scontel_ind.pack(side="left", padx=(0, 12))

        self.th_ind = ttk.Frame(ind_row)
        self.th_canvas = tk.Canvas(self.th_ind, width=18, height=18,
                                   highlightthickness=0)
        self.th_canvas.pack(side="left")
        self.th_circle = self.th_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(self.th_ind, text="TimeHarp").pack(side="left", padx=(2, 0))

        self.standa_ind = ttk.Frame(ind_row)
        self.standa_canvas = tk.Canvas(self.standa_ind, width=18, height=18,
                                       highlightthickness=0)
        self.standa_canvas.pack(side="left")
        self.standa_circle = self.standa_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(self.standa_ind, text="Standa").pack(side="left", padx=(2, 0))
        self.standa_ind.pack(side="left", padx=(0, 12))

        self.acton_ind = ttk.Frame(ind_row)
        self.acton_canvas = tk.Canvas(self.acton_ind, width=18, height=18,
                                      highlightthickness=0)
        self.acton_canvas.pack(side="left")
        self.acton_circle = self.acton_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(self.acton_ind, text="Acton").pack(side="left", padx=(2, 0))
        self.acton_ind.pack(side="left")

        ttk.Button(status_frame, text="Проверить подключения",
                   command=self.check_connections).pack(pady=(6, 0))

        # --- Режимы ---
        mode_frame = ttk.LabelFrame(left_frame, text="Режим", padding=8)
        mode_frame.pack(fill="x", pady=4)

        self.mode0_radio = ttk.Radiobutton(
            mode_frame, text="0 – Юстировка", variable=self.mode_var,
            value=0, command=self.update_mode)
        self.mode0_radio.pack(anchor="w")
        self.mode1_radio = ttk.Radiobutton(
            mode_frame, text="1 – Измерение кинетики", variable=self.mode_var,
            value=1, command=self.update_mode)
        self.mode2_radio = ttk.Radiobutton(
            mode_frame, text="2 – Позиционирование", variable=self.mode_var,
            value=2, command=self.update_mode)
        self.mode2_radio.pack(anchor="w")
        self.mode3_radio = ttk.Radiobutton(
            mode_frame, text="3 – Спектр в точке", variable=self.mode_var,
            value=3, command=self.update_mode)
        self.mode3_radio.pack(anchor="w")
        self.mode4_radio = ttk.Radiobutton(
            mode_frame, text="4 – Пространственное распределение",
            variable=self.mode_var, value=4, command=self.update_mode)
        self.mode4_radio.pack(anchor="w")
        self.mode5_radio = ttk.Radiobutton(
            mode_frame, text="5 – Спектры в разных точках",
            variable=self.mode_var, value=5, command=self.update_mode)
        self.mode5_radio.pack(anchor="w")

        self.param_frame = ttk.Frame(left_frame)
        self.param_frame.pack(fill="x", pady=4)

        self.time_label = ttk.Label(left_frame, text="Расчётное время: --:--",
                                    font=("TkDefaultFont", 10, "bold"),
                                    relief="sunken", anchor="center")
        self.time_label.pack(fill="x", pady=4)

        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(pady=4)
        self.start_btn = ttk.Button(btn_frame, text="Старт",
                                    command=self.start_measurement)
        self.start_btn.pack(side="left", padx=5)
        self.stop_btn = ttk.Button(btn_frame, text="Стоп",
                                   command=self.stop_measurement,
                                   state="disabled")
        self.stop_btn.pack(side="left", padx=5)
        self.graph_btn = ttk.Button(btn_frame, text="Графики",
                                    command=self.toggle_graph_window)
        self.graph_btn.pack(side="left", padx=5)

        link_frame = ttk.Frame(right_frame)
        link_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(2, 2))
        link_label = tk.Label(link_frame, text="GitHub Repository",
                              fg="blue", cursor="hand2",
                              font=("TkDefaultFont", 9, "underline"))
        link_label.pack()
        link_label.bind(
            "<Button-1>",
            lambda e: webbrowser.open(
                "https://github.com/KrazyCatPower/"
                "Standa_Acton2300i_SSPDScontel_Spectra_Distribution.git"))

        self.progress = ttk.Progressbar(left_frame, length=200,
                                         mode="determinate")
        self.progress.pack(side="bottom", fill="x", pady=4)

        # ---------- Правая панель ----------
        # Журнал — фиксирован внизу
        self.log_frame = ttk.LabelFrame(right_frame, text="Журнал", padding=5)
        self.log_frame.pack(side="bottom", fill="x", pady=5)
        self.log_text = scrolledtext.ScrolledText(self.log_frame, height=8,
                                                  state="disabled")
        self.log_text.pack(fill="both", expand=True)

        # Панель с изменяемым размером между основным графиком и
        # вспомогательным окном накопления кинетики.
        self.right_paned = ttk.PanedWindow(right_frame, orient=tk.VERTICAL)
        self.right_paned.pack(side="top", fill="both", expand=True)

        # Основной график
        self.graph_frame = ttk.Frame(self.right_paned)
        self.right_paned.add(self.graph_frame, weight=4)

        self.view_3d_frame = ttk.LabelFrame(self.graph_frame,
                                            text="Визуализация карты",
                                            padding=5)

        self.figure = Figure(figsize=(5, 3), dpi=100)
        self.ax = None
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.cbar = None

        # Вспомогательное окно (кинетика) — создаём, но пока не добавляем
        # в PanedWindow. Добавим при необходимости в update_kinetics_aux_visibility.
        self.kinetics_aux_frame = ttk.LabelFrame(
            self.right_paned,
            text="Накопление текущей кинетики",
            padding=2)

        self.kin_aux_figure = Figure(figsize=(5, 2.0), dpi=100)
        self.kin_aux_axes = self.kin_aux_figure.add_subplot(111)
        self.kin_aux_canvas = FigureCanvasTkAgg(
            self.kin_aux_figure, master=self.kinetics_aux_frame)
        self.kin_aux_canvas.get_tk_widget().pack(
            fill="both", expand=True, padx=2, pady=2)
        self.kin_aux_axes.set_xlabel("Время, нс", fontsize=9)
        self.kin_aux_axes.set_ylabel("CPS", fontsize=9)
        self.kin_aux_axes.tick_params(labelsize=8)
        self.kin_aux_axes.grid(True, alpha=0.3)

        self.update_mode()
        self.update_kinetics_aux_visibility()

    # ---------- PanedWindow 50/50 ----------
    def _set_paned_50_50(self):
        """Установить разделитель PanedWindow по центру (50/50)."""
        if not self.kinetics_aux_visible:
            return
        try:
            self.right_paned.update_idletasks()
            total_h = self.right_paned.winfo_height()
            if total_h > 10:
                self.right_paned.sashpos(0, total_h // 2)
        except tk.TclError:
            pass

    # ---------- Вспомогательное окно кинетики ----------
    def update_kinetics_aux_visibility(self):
        """
        Показывать окно накопления текущей кинетики только когда выбран
        TimeHarp и режим из (3, 4, 5). При Scontel — скрыто.
        Панель можно тянуть за разделитель PanedWindow.
        """
        det = self.detector_var.get()
        try:
            mode = self.mode_var.get()
        except Exception:
            mode = 0
        should_show = (det == "TimeHarp" and mode in (3, 4, 5))

        if should_show and not self.kinetics_aux_visible:
            try:
                self.right_paned.add(self.kinetics_aux_frame, weight=1)
            except tk.TclError:
                pass
            self.kinetics_aux_visible = True
            try:
                self.kin_aux_axes.clear()
                self.kin_aux_axes.set_xlabel("Время, нс", fontsize=9)
                self.kin_aux_axes.set_ylabel("CPS", fontsize=9)
                self.kin_aux_axes.tick_params(labelsize=8)
                self.kin_aux_axes.grid(True, alpha=0.3)
                self.kin_aux_axes.set_title(
                    "Ожидание начала накопления...", fontsize=9)
                self.kin_aux_figure.tight_layout()
                self.kin_aux_canvas.draw_idle()
            except Exception:
                pass
            # После добавления панели дать Tk пересчитать layout
            self.after(150, self._set_paned_50_50)
        elif (not should_show) and self.kinetics_aux_visible:
            try:
                self.right_paned.forget(self.kinetics_aux_frame)
            except tk.TclError:
                pass
            self.kinetics_aux_visible = False

    def _plot_kinetics_aux(self, times_ns, kin, title):
        if not self.kinetics_aux_visible or self.kin_aux_axes is None:
            return
        try:
            self.kin_aux_axes.clear()
            self.kin_aux_axes.plot(times_ns, kin, 'b-', linewidth=1.0)
            self.kin_aux_axes.set_xlabel("Время, нс", fontsize=9)
            self.kin_aux_axes.set_ylabel("CPS", fontsize=9)
            self.kin_aux_axes.tick_params(labelsize=8)
            self.kin_aux_axes.grid(True, alpha=0.3)
            if title:
                self.kin_aux_axes.set_title(title, fontsize=9)
            if times_ns:
                self.kin_aux_axes.set_xlim(times_ns[0], times_ns[-1])
            self.kin_aux_figure.tight_layout()
            self.kin_aux_canvas.draw_idle()
        except Exception:
            pass

    # ---------- Auto offset ----------
    def _on_auto_offset_toggle(self):
        if self.auto_offset_var.get():
            if self.offset_entry is not None:
                self.offset_entry.config(state="disabled")
            self.log("auto offset включён: максимум кинетики будет "
                     "установлен строго на 1.0 нс")
        else:
            if self.offset_entry is not None:
                self.offset_entry.config(state="normal")
            self.log("auto offset выключен: offset снова управляется вручную")

    # ---------- TimeHarp params ----------
    def _apply_timeharp_params(self):
        if self.th is None or not self.th.opened:
            messagebox.showerror("TimeHarp",
                                 "TimeHarp не подключён. Нажмите "
                                 "«Проверить подключения».")
            return
        try:
            self.th.apply_sync_divider(self.th_sync_divider.get())
            self.th.apply_sync_level(self.th_sync_level.get())
            self.th.apply_resolution(self.th_resolution.get())
            self.log("Параметры TimeHarp применены (без переинициализации)")
        except Exception as e:
            messagebox.showerror("TimeHarp", f"Ошибка применения параметров: {e}")

    # ---------- Детектор ----------
    def update_detector_ui(self):
        det = self.detector_var.get()
        if det == "TimeHarp":
            if not self.th_frame.winfo_ismapped():
                self.th_frame.pack(fill="x", pady=4, after=self.det_frame)
            self.scontel_ip_label.grid_remove()
            self.scontel_ip_frame.grid_remove()
            self.scontel_ind.pack_forget()
            self.th_ind.pack(side="left", padx=(0, 12),
                             before=self.standa_ind)
            if not self.mode1_radio.winfo_ismapped():
                self.mode1_radio.pack(anchor="w", before=self.mode2_radio)
        else:
            self.th_frame.pack_forget()
            self.scontel_ip_label.grid()
            self.scontel_ip_frame.grid()
            self.th_ind.pack_forget()
            self.scontel_ind.pack(side="left", padx=(0, 12),
                                  before=self.standa_ind)
            self.mode1_radio.pack_forget()
            if self.mode_var.get() == 1:
                self.mode_var.set(0)
                self.update_mode()
            self._stop_th_poll()
            self._release_timeharp()
            self.th_cps_var.set("--")
            self.th_sync_var.set("--")
        self.update_kinetics_aux_visibility()

    def _release_timeharp(self):
        if self.th is not None:
            try:
                self.th.close()
            except Exception:
                pass
            self.th = None

    def _start_th_poll(self):
        if self._th_poll_active:
            return
        self._th_poll_active = True
        self._poll_th_rates()

    def _stop_th_poll(self):
        self._th_poll_active = False

    def _poll_th_rates(self):
        if not self._th_poll_active:
            return
        measuring = (self.measurement_thread is not None
                     and self.measurement_thread.is_alive())
        if not measuring and self.th is not None and self.th.opened:
            try:
                cps = self.th.get_count_rate(0)
                sr = self.th.get_sync_rate()
                self.th_cps_var.set(f"{cps} /s")
                self.th_sync_var.set(f"{sr} Hz")
            except Exception:
                pass
        self.after(1000, self._poll_th_rates)

    # ---------- Вспомогательные ----------
    def toggle_graph_window(self):
        if self.graph_window.winfo_viewable():
            self.graph_window.withdraw()
        else:
            self.graph_window.show()

    def refresh_com_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_combo['values'] = ports
        if ports and not self.com_port.get():
            self.com_port.set(ports[0])

    def do_move(self):
        if self.pos_axis is None:
            messagebox.showerror("Standa",
                                 "Standa не подключена. Нажмите "
                                 "«Проверить подключения».")
            return
        try:
            target = float(self.target_um.get())
        except Exception:
            messagebox.showerror("Ошибка", "Некорректная позиция")
            return
        try:
            steps, usteps = um_to_ximc(target)
            self.log(f"Перемещение: цель {target:.4f} мкм")
            actual_um = move(self.pos_axis, steps, usteps)
            self.pos_current_var.set(f"{actual_um:.4f}")
            self.log(f"Фактическая позиция: {actual_um:.4f} мкм")
        except Exception as e:
            messagebox.showerror("Ошибка перемещения", str(e))

    # ---------- Интерфейс по режимам ----------
    def update_mode(self):
        mode = self.mode_var.get()
        for w in self.param_frame.winfo_children():
            w.destroy()

        if mode == 0:
            ttk.Label(self.param_frame, text="Длина волны (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.just_wl,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Накопление (с):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.accum_time,
                      width=5).pack(anchor="w", pady=2)
        elif mode == 1:
            ttk.Label(self.param_frame, text="Длина волны (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.just_wl,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Позиция Standa (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.target_um,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Накопление (с):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.accum_time,
                      width=5).pack(anchor="w", pady=2)
            ttk.Checkbutton(self.param_frame,
                            text="oscilloscope (циклический просмотр, без сохранения)",
                            variable=self.oscilloscope_var).pack(anchor="w", pady=(6, 0))
            ttk.Checkbutton(self.param_frame, text="Log Y (ось Y лог.)",
                            variable=self.kin_logy_var,
                            command=self._update_main_plot_yscale).pack(anchor="w")
        elif mode == 2:
            ttk.Label(self.param_frame,
                      text="Целевая позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.target_um,
                      width=10).pack(anchor="w", pady=2)
            ttk.Button(self.param_frame, text="Перейти",
                       command=self.do_move).pack(anchor="w", fill="x", pady=2)
            ttk.Label(self.param_frame,
                      text="Текущая позиция (мкм):").pack(anchor="w", pady=(8, 0))
            ttk.Label(self.param_frame, textvariable=self.pos_current_var,
                      font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        else:
            ttk.Label(self.param_frame, text="Накопление (с):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.accum_time,
                      width=5).pack(anchor="w", pady=2)

        if mode == 3:
            ttk.Label(self.param_frame, text="λ нач (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_start,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="λ кон (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_end,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_step,
                      width=5).pack(anchor="w", pady=2)
        elif mode == 4:
            ttk.Label(self.param_frame, text="Нач. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.start_pos_um,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Кон. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.end_pos_um,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.step_um,
                      width=5).pack(anchor="w", pady=2)
            if self.detector_var.get() == "TimeHarp":
                ttk.Checkbutton(self.param_frame,
                                text="Сохранять кинетики в каждой точке",
                                variable=self.save_kinetics_var).pack(
                    anchor="w", pady=(8, 0))
        elif mode == 5:
            cols = ttk.Frame(self.param_frame)
            cols.pack(anchor="w", fill="x")
            left_sub = ttk.Frame(cols)
            left_sub.pack(side="left", anchor="nw")
            right_sub = ttk.Frame(cols)
            right_sub.pack(side="left", anchor="nw", padx=(20, 0))

            ttk.Label(left_sub, text="--- Спектр ---").pack(anchor="w")
            ttk.Label(left_sub, text="λ нач (нм):").pack(anchor="w")
            ttk.Entry(left_sub, textvariable=self.wl_start,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(left_sub, text="λ кон (нм):").pack(anchor="w")
            ttk.Entry(left_sub, textvariable=self.wl_end,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(left_sub, text="Шаг (нм):").pack(anchor="w")
            ttk.Entry(left_sub, textvariable=self.wl_step,
                      width=5).pack(anchor="w", pady=2)

            ttk.Label(right_sub, text="--- Подвижка ---").pack(anchor="w")
            ttk.Label(right_sub, text="Нач. поз. (мкм):").pack(anchor="w")
            ttk.Entry(right_sub, textvariable=self.start_pos_um,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(right_sub, text="Кон. поз. (мкм):").pack(anchor="w")
            ttk.Entry(right_sub, textvariable=self.end_pos_um,
                      width=8).pack(anchor="w", pady=2)
            ttk.Label(right_sub, text="Шаг (мкм):").pack(anchor="w")
            ttk.Entry(right_sub, textvariable=self.step_um,
                      width=5).pack(anchor="w", pady=2)

            if self.detector_var.get() == "TimeHarp":
                ttk.Checkbutton(self.param_frame,
                                text="Сохранять кинетики в каждой точке",
                                variable=self.save_kinetics_var).pack(
                    anchor="w", pady=(8, 0))

        if mode == 5:
            for w in self.view_3d_frame.winfo_children():
                w.destroy()
            ttk.Checkbutton(self.view_3d_frame, text="3D вид",
                            variable=self.view_3d,
                            command=self.refresh_plot2d).grid(row=0, column=0, padx=5)
            ttk.Checkbutton(self.view_3d_frame, text="Сетка",
                            variable=self.show_grid,
                            command=self.refresh_plot2d).grid(row=0, column=1, padx=5)
            ttk.Label(self.view_3d_frame,
                      text="Мин CPS:").grid(row=0, column=2, padx=5)
            ttk.Entry(self.view_3d_frame, textvariable=self.vmin,
                      width=6).grid(row=0, column=3)
            ttk.Label(self.view_3d_frame,
                      text="Макс CPS:").grid(row=0, column=4, padx=5)
            ttk.Entry(self.view_3d_frame, textvariable=self.vmax,
                      width=6).grid(row=0, column=5)
            ttk.Button(self.view_3d_frame, text="Обновить",
                       command=self.refresh_plot2d).grid(row=0, column=6, padx=10)
            self.view_3d_frame.pack(side="bottom", fill="x", pady=2, anchor="w")
        else:
            self.view_3d_frame.pack_forget()

        self.update_time_estimate()
        for var in (self.accum_time, self.wl_start, self.wl_end, self.wl_step,
                    self.step_um, self.start_pos_um, self.end_pos_um):
            var.trace_add("write", lambda *a: self.update_time_estimate())

        self.update_kinetics_aux_visibility()

    def refresh_plot2d(self):
        if self.last_2d_data:
            self._plot_2d(self.last_2d_data)

    def update_time_estimate(self):
        try:
            mode = self.mode_var.get()
            accum = self.accum_time.get()
            if mode == 0:
                self.time_label.config(text=f"Интервал опроса: {accum:.1f} с")
                return
            if mode == 1:
                self.time_label.config(text=f"Кинетика: {accum:.1f} с")
                return
            if mode == 2:
                self.time_label.config(text="Режим позиционирования")
                return
            if accum <= 0:
                self.time_label.config(text="Расчётное время: --:--")
                return
            if mode == 3:
                n = int(abs(self.wl_end.get() - self.wl_start.get()) /
                        self.wl_step.get()) + 1
                t = n * (accum + 1)
            elif mode == 4:
                n = int(abs(self.end_pos_um.get() - self.start_pos_um.get()) /
                        self.step_um.get()) + 1
                t = n * (accum + 1)
            elif mode == 5:
                n_wl = int(abs(self.wl_end.get() - self.wl_start.get()) /
                           self.wl_step.get()) + 1
                n_pos = int(abs(self.end_pos_um.get() - self.start_pos_um.get()) /
                            self.step_um.get()) + 1
                t = n_pos * n_wl * (accum + 1)
            else:
                t = 0
            mins, secs = divmod(int(t), 60)
            self.time_label.config(
                text=f"Расчётное время: {mins:02d} мин {secs:02d} с")
        except Exception:
            self.time_label.config(text="Расчётное время: --:--")

    def browse_folder(self):
        path = filedialog.askdirectory(initialdir=self.folder_path.get())
        if path:
            self.folder_path.set(path)

    def detect_acton_port(self):
        self.log("Поиск монохроматора...")
        port = find_acton_port()
        if port:
            self.com_port.set(port)
            self.log(f"Найден на {port}")
            messagebox.showinfo("OK", f"Порт {port}")
        else:
            self.log("Не найден")
            messagebox.showwarning("Поиск", "Не найден")

    def increment_filename(self):
        name = self.filename.get()
        match = re.match(r'^(\d+)_?(.*)', name)
        if match:
            num = int(match.group(1))
            rest = match.group(2)
            new_name = f"{num + 1:02d}_{rest}" if rest else f"{num + 1:02d}"
        else:
            new_name = f"01_{name}"
        self.filename.set(new_name)

    # ---------- Проверка подключений ----------
    def check_connections(self):
        det = self.detector_var.get()
        self.log(f"Проверка подключений ({det})...")
        if det == "TimeHarp":
            self.set_indicator("timeharp", "gray")
        else:
            self.set_indicator("scontel", "gray")
        self.set_indicator("standa", "gray")
        self.set_indicator("acton", "gray")
        threading.Thread(target=self._check_devices,
                         args=(det,), daemon=True).start()

    def _check_devices(self, detector):
        ip = self.scontel_ip.get()
        dev = self.scontel_dev.get()
        port = self.com_port.get()

        if detector == "TimeHarp":
            try:
                if self.th is None or not self.th.opened:
                    th_new = TimeHarpPy(
                        sync_divider=self.th_sync_divider.get(),
                        sync_level=self.th_sync_level.get(),
                        resolution_ps=self.th_resolution.get(),
                        log_func=lambda m: self.data_queue.put(("log", m)))
                    self.th = th_new
                cps = self.th.get_count_rate(0)
                sr = self.th.get_sync_rate()
                self.data_queue.put(("indicator", ("timeharp", "green")))
                self.data_queue.put(
                    ("log", f"TimeHarp: CPS={cps}/s, Sync={sr} Hz"))
                self.data_queue.put(("th_rates",
                                     (f"{cps} /s", f"{sr} Hz")))
                self._start_th_poll()
            except Exception as e:
                self.data_queue.put(("indicator", ("timeharp", "red")))
                self.data_queue.put(("log", f"TimeHarp: {e}"))
                self._release_timeharp()
        else:
            try:
                if get_cps(ip, 9876, dev, timeout=2.0) is not None:
                    self.data_queue.put(("indicator", ("scontel", "green")))
                    self.data_queue.put(("log", "Scontel: OK"))
                else:
                    self.data_queue.put(("indicator", ("scontel", "red")))
                    self.data_queue.put(("log", "Scontel: нет данных"))
            except Exception as e:
                self.data_queue.put(("indicator", ("scontel", "red")))
                self.data_queue.put(("log", f"Scontel: {e}"))

        try:
            if self.pos_axis is not None:
                self.data_queue.put(("indicator", ("standa", "green")))
                self.data_queue.put(("log", "Standa: OK (переиспользуется)"))
            else:
                devenum = ximc.enumerate_devices(
                    ximc.EnumerateFlags.ENUMERATE_PROBE |
                    ximc.EnumerateFlags.ENUMERATE_NETWORK, "addr=")
                open_name = (devenum[0]["uri"] if devenum else
                             "xi-emu:///" + os.path.join(
                                 os.path.expanduser('~'), "testdevice.bin"))
                ax = ximc.Axis(open_name)
                ax.open_device()
                set_microstep_mode_8(ax)
                self.pos_axis = ax
                self.data_queue.put(("indicator", ("standa", "green")))
                self.data_queue.put(("log", "Standa: OK"))
        except Exception as e:
            self.data_queue.put(("indicator", ("standa", "red")))
            self.data_queue.put(("log", f"Standa: {e}"))
            self.pos_axis = None

        if port:
            try:
                if self.mono is None:
                    self.mono = ActonPy(port)
                wl = self.mono.get_wavelength()
                self.data_queue.put(("indicator", ("acton", "green")))
                self.data_queue.put(("log",
                                     f"Acton ({port}): {self.mono.model}, "
                                     f"λ={wl:.2f} нм"))
            except Exception as e:
                self.data_queue.put(("indicator", ("acton", "red")))
                self.data_queue.put(("log", f"Acton: {e}"))
                try:
                    if self.mono is not None:
                        self.mono.closeConnection()
                except Exception:
                    pass
                self.mono = None
        else:
            self.data_queue.put(("indicator", ("acton", "red")))
            self.data_queue.put(("log", "Acton: не выбран порт"))

    def set_indicator(self, dev, color):
        canvases = {
            "scontel": (self.scontel_canvas, self.scontel_circle),
            "timeharp": (self.th_canvas, self.th_circle),
            "standa": (self.standa_canvas, self.standa_circle),
            "acton": (self.acton_canvas, self.acton_circle)}
        if dev in canvases:
            canvases[dev][0].itemconfig(canvases[dev][1], fill=color)

    def log(self, msg):
        t = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{t}] {msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def set_ui_state(self, running):
        s = "disabled" if running else "normal"
        self.start_btn.config(state=s)
        self.stop_btn.config(state="normal" if running else "disabled")

    # ---------- Старт измерения ----------
    def start_measurement(self):
        mode = self.mode_var.get()
        detector = self.detector_var.get()

        if mode == 2:
            messagebox.showinfo("Информация",
                                "Введите целевую позицию и нажмите «Перейти».")
            return

        if mode == 1 and detector != "TimeHarp":
            messagebox.showerror("Ошибка",
                                 "Режим «Измерение кинетики» доступен только "
                                 "с детектором TimeHarp.")
            return

        need_th = detector == "TimeHarp"
        need_mono = (mode in (0, 1, 3, 5)) and bool(self.com_port.get())
        need_standa = mode in (4, 5) or (mode == 1)

        if need_th and (self.th is None or not self.th.opened):
            messagebox.showerror(
                "TimeHarp не подключён",
                "Нажмите «Проверить подключения», чтобы инициализировать "
                "TimeHarp, затем запустите измерение.")
            return
        if mode in (0, 3, 5) and need_mono and self.mono is None:
            messagebox.showerror(
                "Acton не подключён",
                "Нажмите «Проверить подключения», чтобы инициализировать "
                "монохроматор, затем запустите измерение.")
            return
        if need_standa and self.pos_axis is None:
            messagebox.showerror(
                "Standa не подключена",
                "Нажмите «Проверить подключения», чтобы инициализировать "
                "подвижку, затем запустите измерение.")
            return

        if mode != 0 and mode != 1:
            if self.accum_time.get() <= 0:
                messagebox.showerror("Ошибка", "Накопление > 0")
                return

        if mode in (3, 5) and not self.com_port.get():
            messagebox.showerror("Ошибка", "Укажите COM-порт")
            return

        if mode in (3, 5):
            try:
                if self.wl_step.get() <= 0 or self.wl_start.get() == self.wl_end.get():
                    raise ValueError
            except Exception:
                messagebox.showerror("Ошибка", "Параметры длин волн")
                return

        if mode in (4, 5):
            try:
                if self.step_um.get() <= 0 or self.start_pos_um.get() == self.end_pos_um.get():
                    raise ValueError
            except Exception:
                messagebox.showerror("Ошибка", "Параметры подвижки")
                return

        self.osc_cancel_event.clear()
        self.stop_requested = False
        self.set_ui_state(True)
        self.log("=" * 30 + " СТАРТ " + "=" * 30)
        self.progress["value"] = 0
        self.measurement_thread = threading.Thread(
            target=self.run_measurement, args=(mode, detector), daemon=True)
        self.measurement_thread.start()

    def stop_measurement(self):
        self.stop_requested = True
        self.osc_cancel_event.set()
        self.log("Остановка...")

    # ---------- info.txt ----------
    def _write_info_txt(self, folder, mode, detector, extra_lines=None):
        try:
            lines = []
            lines.append("Experiment info")
            lines.append("=" * 40)
            lines.append(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            lines.append(f"Mode: {mode}")
            lines.append(f"Detector: {detector}")
            lines.append(f"Accum time (s): {self.accum_time.get()}")
            if detector == "TimeHarp" and self.th is not None and self.th.opened:
                try:
                    lines.append(f"SyncRate (Hz): {self.th.get_sync_rate()}")
                except Exception:
                    pass
                lines.append(f"Resolution (ps): {self.th.resolution_ps}")
                lines.append(f"Sync divider: {self.th_sync_divider.get()}")
                lines.append(f"Sync level: {self.th_sync_level.get()}")
                lines.append(f"Offset (bins): {self.th_offset1.get()}")
                lines.append(f"Auto offset: {self.auto_offset_var.get()}")
            if extra_lines:
                lines.append("")
                lines.extend(extra_lines)
            with open(os.path.join(folder, "info.txt"), "w",
                      encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        except Exception as e:
            self.data_queue.put(("log", f"info.txt: {e}"))

    # ---------- основной цикл измерения ----------
    def run_measurement(self, mode, detector):
        use_th = (detector == "TimeHarp")
        try:
            ip = self.scontel_ip.get()
            dev = self.scontel_dev.get()
            accum = self.accum_time.get()
            folder = self.folder_path.get()
            base_name = self.filename.get()

            if use_th and (self.th is None or not self.th.opened):
                raise RuntimeError("TimeHarp не подключён. Нажмите «Проверить подключения».")
            if mode in (0, 3, 5) and self.com_port.get() and self.mono is None:
                raise RuntimeError("Acton не подключён. Нажмите «Проверить подключения».")
            if mode in (4, 5) and self.pos_axis is None:
                raise RuntimeError("Standa не подключена. Нажмите «Проверить подключения».")

            axis = self.pos_axis

            positions_um = None
            numstep = 0
            if mode in (4, 5):
                start_um = self.start_pos_um.get()
                end_um = self.end_pos_um.get()
                step_um = self.step_um.get()
                numstep = int(math.ceil(abs(end_um - start_um) / step_um)) + 1
                positions_um = [start_um + i * step_um if end_um >= start_um
                                else start_um - i * step_um
                                for i in range(numstep)]
                move(axis, 0, 0)
                self.data_queue.put(("log", f"Standa готова, точек: {numstep}"))
            elif mode == 1:
                target_um = self.target_um.get()
                steps, usteps = um_to_ximc(target_um)
                move(axis, steps, usteps)
                self.data_queue.put(("log", f"Позиция {target_um:.3f} мкм"))

            aux_cb = None
            if use_th:
                dt_ns_aux = self.th.resolution_ps * 0.001

                def aux_cb(kin, sr, done_ms, total_ms, is_kin_time=True):
                    times_ns_local = [i * dt_ns_aux for i in range(len(kin))]
                    pct = (done_ms / max(total_ms, 1)) * 100.0
                    title = f"Накопление: {done_ms} / {total_ms} мс ({pct:.0f}%)"
                    self.data_queue.put(
                        ("plot_kinetics_aux",
                         (times_ns_local, list(kin), title)))

            def measure_value(progress_cb=None, cancel_event=None,
                              kinetics_interval_ms=0):
                if use_th:
                    tacq_ms = int(accum * 1000)
                    kin, sr, cr = self.th.measure_kinetics(
                        tacq_ms, offset1=self.th_offset1.get(),
                        progress_cb=progress_cb,
                        cancel_event=cancel_event,
                        kinetics_interval_ms=kinetics_interval_ms)
                    value = sum(kin)
                    self.data_queue.put(("th_rates",
                                         (f"{cr[0]} /s", f"{sr} Hz")))
                    return value, kin, sr, cr[0]
                else:
                    val = measure_cps_average(ip, 9876, dev, accum)
                    return (val if val is not None else 0.0), None, 0, 0

            # ---------- Режимы ----------
            if mode == 0:
                wl = self.just_wl.get()
                if self.mono:
                    self.mono.goto(wl)
                self.data_queue.put(("log", f"Юстировка на {wl} нм"))
                times, values = [], []
                start_time = time.time()
                while not self.stop_requested:
                    v, _, _, _ = measure_value()
                    t = time.time() - start_time
                    times.append(t)
                    values.append(v)
                    while times and times[0] < t - 10:
                        times.pop(0)
                        values.pop(0)
                    self.data_queue.put(("plot_just",
                                         (times.copy(), values.copy(), v)))
                self.data_queue.put(("log", "Юстировка завершена"))

            elif mode == 1:
                wl = self.just_wl.get()
                if self.mono:
                    self.mono.goto(wl)
                tacq_ms = int(accum * 1000)
                dt_ns = self.th.resolution_ps * 0.001
                pos = self.target_um.get()

                osc_mode = self.oscilloscope_var.get()

                if osc_mode:
                    self.data_queue.put(
                        ("log", "oscilloscope: циклическое накопление. "
                                "Остановка — кнопка Стоп."))
                    cycle_idx = 0
                    while not self.stop_requested and not self.osc_cancel_event.is_set():
                        cycle_idx += 1
                        self.data_queue.put(("progress_reset", (0, tacq_ms)))

                        def live_cb(kin, sr, done_ms, total_ms, is_kin_time):
                            self.data_queue.put(("progress", (done_ms, total_ms)))
                            remaining = max(0.0, (total_ms - done_ms) / 1000.0)
                            self.data_queue.put(("time_label",
                                                 f"Осталось: {remaining:.1f} с"))
                            if is_kin_time:
                                times_ns_local = [i * dt_ns
                                                  for i in range(len(kin))]
                                title = (f"Oscilloscope #{cycle_idx}: "
                                         f"λ={wl} нм, pos={pos} мкм, "
                                         f"sync={sr / 1e6:.2f} MHz")
                                self.data_queue.put(("plot_kinetics",
                                                     (times_ns_local,
                                                      list(kin), title)))

                        kin, sr, cr = self.th.measure_kinetics(
                            tacq_ms, offset1=self.th_offset1.get(),
                            progress_cb=live_cb,
                            cancel_event=self.osc_cancel_event,
                            kinetics_interval_ms=1000)
                        self.data_queue.put(("th_rates",
                                             (f"{cr[0]} /s", f"{sr} Hz")))
                        times_ns = [i * dt_ns for i in range(len(kin))]
                        title = (f"Oscilloscope #{cycle_idx}: λ={wl} нм, "
                                 f"pos={pos} мкм, sync={sr / 1e6:.2f} MHz")
                        self.data_queue.put(("plot_kinetics",
                                             (times_ns, kin, title)))
                        if self.auto_offset_var.get():
                            new_off = self._compute_auto_offset(kin, dt_ns)
                            if new_off is not None:
                                self.data_queue.put(("auto_offset", new_off))
                        self.data_queue.put(("progress", (tacq_ms, tacq_ms)))

                else:
                    def live_cb(kin, sr, done_ms, total_ms, is_kin_time):
                        self.data_queue.put(("progress", (done_ms, total_ms)))
                        remaining = max(0.0, (total_ms - done_ms) / 1000.0)
                        self.data_queue.put(("time_label",
                                             f"Осталось: {remaining:.1f} с"))
                        if is_kin_time:
                            times_ns_local = [i * dt_ns
                                              for i in range(len(kin))]
                            title = (f"Кинетика (live): λ={wl} нм, pos={pos} мкм, "
                                     f"sync={sr / 1e6:.2f} MHz, "
                                     f"offset={self.th_offset1.get()}")
                            self.data_queue.put(("plot_kinetics",
                                                 (times_ns_local,
                                                  list(kin), title)))

                    kin, sr, cr = self.th.measure_kinetics(
                        tacq_ms, offset1=self.th_offset1.get(),
                        progress_cb=live_cb,
                        cancel_event=self.osc_cancel_event,
                        kinetics_interval_ms=0)
                    self.data_queue.put(("th_rates", (f"{cr[0]} /s", f"{sr} Hz")))

                    if self.auto_offset_var.get():
                        new_off = self._compute_auto_offset(kin, dt_ns)
                        if new_off is not None:
                            self.data_queue.put(("auto_offset", new_off))

                    times_ns = [i * dt_ns for i in range(len(kin))]
                    title = (f"Кинетика: λ={wl} нм, pos={pos} мкм, "
                             f"sync={sr / 1e6:.2f} MHz, offset={self.th_offset1.get()}")
                    self.data_queue.put(("plot_kinetics",
                                         (times_ns, kin, title)))

                    filepath = os.path.join(folder, f"{base_name}.txt")
                    with open(filepath, "w") as f:
                        for i, k in enumerate(kin):
                            t_s = i * self.th.resolution_ps * 1e-12
                            f.write(f"{fmt_sci(t_s)}  {fmt_sci(k)}\n")
                    self.increment_filename()
                    self.data_queue.put(("log", f"Кинетика сохранена: {filepath}"))
                    label = f"{base_name} (λ={wl:.1f} нм, pos={pos:.2f} мкм)"
                    self.data_queue.put(("add_kinetics", (times_ns, kin, label)))

            elif mode == 3:
                wl_start = self.wl_start.get()
                wl_end = self.wl_end.get()
                wl_step = self.wl_step.get()
                n_wl = int(abs(wl_end - wl_start) / wl_step) + 1
                self.data_queue.put(("progress", (0, n_wl)))

                exp_folder = os.path.join(folder, base_name)
                os.makedirs(exp_folder, exist_ok=True)

                wl_array, val_array = [], []
                spectrum_file = os.path.join(exp_folder, f"{base_name}.txt")

                kin_folder = None
                if use_th:
                    kin_folder = os.path.join(exp_folder, "kinetics")
                    os.makedirs(kin_folder, exist_ok=True)
                    self.data_queue.put(("log", f"Папка кинетик: {kin_folder}"))

                with open(spectrum_file, "w") as f:
                    self.mono.goto(wl_start)
                    for i in range(n_wl):
                        if self.stop_requested:
                            break
                        target = (wl_start + i * wl_step if wl_end > wl_start
                                  else wl_start - i * wl_step)
                        self.mono.goto(target)
                        v, kin, sr, cps = measure_value(progress_cb=aux_cb)
                        wl_array.append(target)
                        val_array.append(v)
                        f.write(f"{target:.2f}\t{v:.6f}\n")
                        f.flush()

                        if use_th and kin is not None and kin_folder:
                            dt_ns = self.th.resolution_ps * 0.001
                            kfile = os.path.join(kin_folder,
                                                 f"{target:.1f}.txt")
                            with open(kfile, "w") as kf:
                                for j, k in enumerate(kin):
                                    t_s = j * self.th.resolution_ps * 1e-12
                                    kf.write(f"{fmt_sci(t_s)}  {fmt_sci(k)}\n")
                            if self.auto_offset_var.get():
                                new_off = self._compute_auto_offset(kin, dt_ns)
                                if new_off is not None:
                                    self.data_queue.put(("auto_offset", new_off))

                        self.data_queue.put(("plot_1d",
                                             (wl_array.copy(),
                                              val_array.copy(), target, v)))
                        self.data_queue.put(("progress", (i + 1, n_wl)))

                info_extra = [
                    f"Wavelength start (nm): {wl_start}",
                    f"Wavelength end (nm): {wl_end}",
                    f"Wavelength step (nm): {wl_step}",
                    f"Number of points: {len(wl_array)}",
                    f"Data file: {os.path.basename(spectrum_file)}",
                ]
                if kin_folder:
                    info_extra.append("Kinetics folder: kinetics/")
                self._write_info_txt(exp_folder, mode, detector, info_extra)

                self.increment_filename()
                self.data_queue.put(("log", f"Спектр записан: {spectrum_file}"))
                self.data_queue.put(("add_spectrum",
                                     (wl_array, val_array, base_name)))

            elif mode == 4:
                exp_folder = os.path.join(folder, base_name)
                os.makedirs(exp_folder, exist_ok=True)
                data_file = os.path.join(exp_folder, f"{base_name}.txt")

                save_kin = self.save_kinetics_var.get() if use_th else False
                kin_folder = os.path.join(exp_folder, "kinetics")
                if save_kin:
                    os.makedirs(kin_folder, exist_ok=True)

                X, Y = [], []
                self.data_queue.put(("progress", (0, numstep)))
                with open(data_file, "w") as f:
                    for idx, target_um in enumerate(positions_um, start=1):
                        if self.stop_requested:
                            break
                        steps, usteps = um_to_ximc(target_um)
                        actual_um = move(axis, steps, usteps)
                        v, kin, sr, cps = measure_value(progress_cb=aux_cb)
                        X.append(actual_um)
                        Y.append(v)
                        f.write(f"{actual_um:.6f}\t{v:.6f}\n")
                        f.flush()
                        if save_kin and kin is not None:
                            # Имя: <имя файла>_<позиция>mkm.txt
                            kinfile = os.path.join(
                                kin_folder,
                                f"{base_name}_{actual_um:.2f}mkm.txt")
                            dt_ns = self.th.resolution_ps * 0.001
                            times_ns = [j * dt_ns for j in range(len(kin))]
                            with open(kinfile, "w") as kf:
                                for j, k in enumerate(kin):
                                    t_s = j * self.th.resolution_ps * 1e-12
                                    kf.write(f"{fmt_sci(t_s)}  {fmt_sci(k)}\n")
                            label = f"{base_name} pos={actual_um:.2f} мкм"
                            self.data_queue.put(
                                ("add_kinetics", (times_ns, kin, label)))
                            if self.auto_offset_var.get():
                                new_off = self._compute_auto_offset(kin, dt_ns)
                                if new_off is not None:
                                    self.data_queue.put(("auto_offset", new_off))
                        self.data_queue.put(("plot_1d",
                                             (X.copy(), Y.copy(), actual_um, v)))
                        self.data_queue.put(("progress", (idx, numstep)))
                move(axis, 0, 0)

                info_extra = [
                    f"Position start (um): {self.start_pos_um.get()}",
                    f"Position end (um): {self.end_pos_um.get()}",
                    f"Position step (um): {self.step_um.get()}",
                    f"Number of points: {len(X)}",
                    f"Data file: {os.path.basename(data_file)}",
                ]
                if save_kin:
                    info_extra.append("Kinetics folder: kinetics/")
                self._write_info_txt(exp_folder, mode, detector, info_extra)

                self.increment_filename()
                self.data_queue.put(("log",
                                     f"Сканирование завершено: {data_file}"))
                self.data_queue.put(("add_distribution", (X, Y, base_name)))

            elif mode == 5:
                wl_start = self.wl_start.get()
                wl_end = self.wl_end.get()
                wl_step = self.wl_step.get()
                n_wl = int(abs(wl_end - wl_start) / wl_step) + 1
                exp_folder = os.path.join(folder, base_name)
                os.makedirs(exp_folder, exist_ok=True)
                save_kin = self.save_kinetics_var.get() if use_th else False
                kin_folder = os.path.join(exp_folder, "kinetics")
                if save_kin:
                    os.makedirs(kin_folder, exist_ok=True)

                all_spectra = []
                for idx, target_um in enumerate(positions_um, start=1):
                    if self.stop_requested:
                        break
                    steps, usteps = um_to_ximc(target_um)
                    actual_um = move(axis, steps, usteps)
                    wl_list, val_list = [], []
                    spec_file = os.path.join(
                        exp_folder, f"spectrum_pos{target_um:.1f}um.txt")
                    with open(spec_file, "w") as f:
                        self.mono.goto(wl_start)
                        for i in range(n_wl):
                            if self.stop_requested:
                                break
                            target_wl = (wl_start + i * wl_step
                                         if wl_end > wl_start
                                         else wl_start - i * wl_step)
                            self.mono.goto(target_wl)
                            v, kin, sr, cps = measure_value(progress_cb=aux_cb)
                            wl_list.append(target_wl)
                            val_list.append(v)
                            f.write(f"{target_wl:.2f}\t{v:.6f}\n")
                            if save_kin and kin is not None:
                                kinfile = os.path.join(
                                    kin_folder,
                                    f"pos{idx:03d}_{actual_um:.2f}um_"
                                    f"wl{target_wl:.1f}nm.txt")
                                dt_ns = self.th.resolution_ps * 0.001
                                times_ns = [j * dt_ns
                                            for j in range(len(kin))]
                                with open(kinfile, "w") as kf:
                                    for j, k in enumerate(kin):
                                        t_s = j * self.th.resolution_ps * 1e-12
                                        kf.write(f"{fmt_sci(t_s)}  {fmt_sci(k)}\n")
                                label = (f"{base_name} pos={actual_um:.2f}мкм "
                                         f"λ={target_wl:.1f}нм")
                                self.data_queue.put(
                                    ("add_kinetics", (times_ns, kin, label)))
                                if self.auto_offset_var.get():
                                    new_off = self._compute_auto_offset(kin, dt_ns)
                                    if new_off is not None:
                                        self.data_queue.put(("auto_offset", new_off))
                            self.data_queue.put(("plot_1d",
                                                 (wl_list.copy(),
                                                  val_list.copy(),
                                                  actual_um, None)))
                        f.flush()
                    if wl_list:
                        all_spectra.append((actual_um, wl_list, val_list))
                    self.data_queue.put(("progress", (idx, len(positions_um))))

                if all_spectra and not self.stop_requested:
                    wl_common = all_spectra[0][1]
                    matrix = np.zeros((len(all_spectra), len(wl_common)))
                    positions_list = [p[0] for p in all_spectra]
                    for i, (p, wls, vals) in enumerate(all_spectra):
                        if wls == wl_common:
                            matrix[i, :] = vals
                        else:
                            matrix[i, :] = np.interp(wl_common, wls, vals)
                    np.savetxt(os.path.join(exp_folder, "matrix.txt"),
                               matrix, delimiter="\t")
                    with open(os.path.join(exp_folder, "positions.txt"), "w") as f:
                        f.write("\n".join(map(str, positions_list)))
                    with open(os.path.join(exp_folder, "wavelengths.txt"), "w") as f:
                        f.write("\n".join(map(str, wl_common)))
                    self.last_2d_data = (matrix, wl_common, positions_list)
                    self.data_queue.put(("plot_2d", self.last_2d_data))
                    self.data_queue.put(("add_3d",
                                         (matrix, wl_common,
                                          positions_list, base_name)))
                move(axis, 0, 0)

                info_extra = [
                    f"Wavelength start (nm): {wl_start}",
                    f"Wavelength end (nm): {wl_end}",
                    f"Wavelength step (nm): {wl_step}",
                    f"Position start (um): {self.start_pos_um.get()}",
                    f"Position end (um): {self.end_pos_um.get()}",
                    f"Position step (um): {self.step_um.get()}",
                    f"Number of spectra: {len(all_spectra)}",
                    f"Data folder: {os.path.basename(exp_folder)}/",
                ]
                if save_kin:
                    info_extra.append("Kinetics folder: kinetics/")
                self._write_info_txt(exp_folder, mode, detector, info_extra)

                self.increment_filename()
                self.data_queue.put(("log", f"Измерение завершено: {exp_folder}"))

        except Exception as e:
            self.data_queue.put(("log", f"ОШИБКА: {e}"))
            self.data_queue.put(("error", str(e)))
        finally:
            self.data_queue.put(("finish", None))

    # ---------- auto-offset ----------
    def _compute_auto_offset(self, kin, dt_ns):
        if kin is None or len(kin) == 0:
            return None
        try:
            arr = np.asarray(kin, dtype=float)
            if arr.size == 0:
                return None
            idx_max = int(np.argmax(arr))
        except Exception:
            return None
        n = len(kin)
        target_idx = int(round(AUTO_OFFSET_TARGET_NS / dt_ns)) % n
        new_off = (idx_max - target_idx) % n
        if new_off == int(self.th_offset1.get()) % n:
            return None
        return int(new_off)

    def process_queue(self):
        try:
            while True:
                msg = self.data_queue.get_nowait()
                if msg[0] == "log":
                    self.log(msg[1])
                elif msg[0] == "progress":
                    cur, tot = msg[1]
                    self.progress["maximum"] = max(1, tot)
                    self.progress["value"] = cur
                elif msg[0] == "progress_reset":
                    cur, tot = msg[1]
                    self.progress["maximum"] = max(1, tot)
                    self.progress["value"] = cur
                elif msg[0] == "time_label":
                    self.time_label.config(text=msg[1])
                elif msg[0] == "indicator":
                    self.set_indicator(*msg[1])
                elif msg[0] == "th_rates":
                    self.th_cps_var.set(msg[1][0])
                    self.th_sync_var.set(msg[1][1])
                elif msg[0] == "auto_offset":
                    try:
                        self.th_offset1.set(int(msg[1]))
                    except Exception:
                        pass
                elif msg[0] == "plot_kinetics_aux":
                    times_ns, kin, title = msg[1]
                    self._plot_kinetics_aux(times_ns, kin, title)
                elif msg[0] == "plot_1d":
                    x, y, extra, cps = msg[1]
                    self._plot_1d(x, y, extra, cps, is_just=False)
                elif msg[0] == "plot_just":
                    x, y, cps = msg[1]
                    self._plot_1d(x, y, None, cps, is_just=True)
                elif msg[0] == "plot_kinetics":
                    times_ns, kin, title = msg[1]
                    self._plot_kinetics(times_ns, kin, title)
                elif msg[0] == "plot_2d":
                    self._plot_2d(msg[1])
                elif msg[0] == "add_spectrum":
                    x, y, label = msg[1]
                    self.graph_window.add_spectrum(x, y, label)
                elif msg[0] == "add_distribution":
                    x, y, label = msg[1]
                    self.graph_window.add_distribution(x, y, label)
                elif msg[0] == "add_kinetics":
                    times_ns, kin, label = msg[1]
                    self.graph_window.add_kinetics(times_ns, kin, label)
                elif msg[0] == "add_3d":
                    matrix, x, y, label = msg[1]
                    self.graph_window.add_3d_map(matrix, x, y, label)
                elif msg[0] == "error":
                    messagebox.showerror("Ошибка", msg[1])
                elif msg[0] == "finish":
                    self.set_ui_state(False)
                    self.measurement_thread = None
                    self.update_time_estimate()
        except queue.Empty:
            pass
        if self.closing and self.measurement_thread is None:
            self.destroy()
            return
        self.after(100, self.process_queue)

    def _plot_1d(self, x, y, extra, cps, is_just=False):
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        self.ax.plot(x, y, 'b-')
        mode = self.mode_var.get()
        if is_just or mode == 0:
            self.ax.set_xlabel('Время, с')
            self.ax.set_ylabel('CPS')
            self.ax.set_title(f'Юстировка, значение: {cps:.2f}')
            if x:
                t_max = max(x)
                self.ax.set_xlim(t_max - 10, t_max + 1)
                max_y = max(y) if y else 1
                self.ax.set_ylim(0, max_y * 1.1 if max_y > 0 else 1)
        elif mode == 4:
            self.ax.set_xlabel('Позиция, мкм')
            self.ax.set_title(f'Поз: {extra:.2f} мкм, знач: {cps:.2f}')
        elif mode == 3:
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_title(f'λ = {extra:.2f} нм, знач = {cps:.2f}')
        else:
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_title(f'Спектр на {extra:.1f} мкм')
        self.ax.grid(True, alpha=0.3)
        self.figure.tight_layout()
        self.canvas.draw()

    def _plot_kinetics(self, times_ns, kin, title):
        self.last_kinetics_data = (times_ns, kin, title)
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        try:
            if self.kin_logy_var.get():
                self.ax.set_yscale('log')
            else:
                self.ax.set_yscale('linear')
        except Exception:
            pass
        self.ax.plot(times_ns, kin, 'b-', linewidth=1.2)
        self.ax.set_xlabel('Время, нс')
        self.ax.set_ylabel('Интенсивность, CPS')
        self.ax.set_title(title)
        self.ax.grid(True, alpha=0.3)
        if times_ns:
            self.ax.set_xlim(times_ns[0], times_ns[-1])
        self.figure.tight_layout()
        self.canvas.draw()

    def _update_main_plot_yscale(self):
        if self.last_kinetics_data is not None:
            self._plot_kinetics(*self.last_kinetics_data)

    def _plot_2d(self, data):
        matrix, wls, pos = data
        self.figure.clear()
        if self.view_3d.get():
            self.ax = self.figure.add_subplot(111, projection='3d')
            X, Y = np.meshgrid(wls, pos)
            self.ax.plot_surface(X, Y, matrix, cmap='viridis',
                                 rstride=1, cstride=1, alpha=0.9)
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_ylabel('Позиция, мкм')
            self.ax.set_zlabel('CPS')
            if not self.show_grid.get():
                self.ax.grid(False)
            self.ax.set_zlim(self.vmin.get(), self.vmax.get())
        else:
            self.ax = self.figure.add_subplot(111)
            im = self.ax.imshow(matrix, aspect='auto', origin='lower',
                                extent=[wls[0], wls[-1], pos[0], pos[-1]],
                                vmin=self.vmin.get(), vmax=self.vmax.get())
            if self.cbar is not None:
                self.cbar.remove()
                self.cbar = None
            self.cbar = self.figure.colorbar(im, ax=self.ax, label='CPS')
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_ylabel('Позиция, мкм')
            self.ax.set_title('Карта интенсивности')
        self.figure.tight_layout()
        self.canvas.draw()

    def on_closing(self):
        if self.measurement_thread and self.measurement_thread.is_alive():
            if not self.closing:
                self.closing = True
                self.stop_requested = True
                self.osc_cancel_event.set()
                self.log("Завершение, ожидание остановки измерения...")
        else:
            self._stop_th_poll()
            self.release_pos_axis()
            self._release_timeharp()
            if self.mono is not None:
                try:
                    self.mono.closeConnection()
                except Exception:
                    pass
                self.mono = None
            self.graph_window.destroy()
            self.destroy()

    def release_pos_axis(self):
        if self.pos_axis is not None:
            try:
                self.pos_axis.close_device()
            except Exception:
                pass
            self.pos_axis = None


if __name__ == "__main__":
    app = Application()
    app.mainloop()

import os, sys, time, math, socket, re, threading, queue, webbrowser
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

# Палитра Origin
ORIGIN_COLORS = [
    '#0000FF', '#FF0000', '#008000', '#FF8C00', '#800080',
    '#00CED1', '#FF69B4', '#8B4513', '#808000', '#4682B4'
]

# ------------------------------------------------------------
# Класс монохроматора Acton
class ActonPy:
    def __init__(self, port='COM3'):
        self.ser = serial.Serial(port, baudrate=9600, timeout=1)
        time.sleep(0.1)
        if self.ser.is_open:
            self.model = self.query('MODEL')
            if not self.model:
                raise ConnectionError(f'Нет ответа на MODEL, порт {port}')
            print(f'Spectrometer {self.model} connected on {port}')
        else:
            raise ConnectionError(f'Could not open serial port {port}')
        self.tolerance_nm = 0.1

    def _extract_float(self, text):
        cleaned = re.sub(r'\bok\b', '', text, flags=re.IGNORECASE)
        match = re.search(r'[-+]?\d*\.?\d+', cleaned)
        if match:
            return float(match.group())
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
        raise RuntimeError(f"Команда {cmd} не вернула ответ после 3 попыток")

    def write(self, cmd):
        self.ser.write((cmd + '\r').encode())
        self.ser.flush()

    def closeConnection(self):
        try: self.ser.close()
        except: pass
        print('Connection closed')

    def get_wavelength(self):
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.2)
                self.ser.reset_input_buffer()
            resp = self.query('?NM')
            if resp and resp.lower() != 'ok':
                try:
                    return self._extract_float(resp)
                except ValueError:
                    continue
        raise RuntimeError("Не удалось получить длину волны: ответ содержит только 'ok' или пуст")

    def is_moving(self):
        for cmd in ('?STATUS', 'STATUS', 'MONO-EE STATUS'):
            try:
                resp = self.query(cmd)
                if resp and '?' not in resp: return 'MOVING' in resp.upper()
            except: pass
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
            raise RuntimeError("GOTO не подтверждён монохроматором после 3 попыток")

        for _ in range(max_wait):
            try:
                current = self.get_wavelength()
            except RuntimeError:
                time.sleep(1)
                continue
            if abs(current - wavelength) <= self.tolerance_nm:
                if not self.is_moving(): return
            time.sleep(1)
        raise TimeoutError(f"Монохроматор не достиг {wavelength:.3f} нм за {max_wait} с")

    def get_scanrate(self):
        resp = self.query('?NM/MIN')
        return self._extract_float(resp)

    def set_scanrate(self, rate):
        self.write(f'{rate:.2f} NM/MIN')
        ack = self.ser.readline().decode(errors='ignore')
        if 'ok' not in ack.lower(): return 'failed'
        return self.get_scanrate()

    def get_info(self):
        for cmd in ('?STATUS', 'STATUS', 'MONO-EE STATUS'):
            resp = self.query(cmd)
            if resp and '?' not in resp: return resp
        return 'No extended status available'

# ------------------------------------------------------------
# Автопоиск порта Acton
def find_acton_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if 'standa' in p.description.lower() or 'ximc' in p.description.lower(): continue
        try:
            ser = serial.Serial(p.device, baudrate=9600, timeout=0.5)
            time.sleep(0.1)
            ser.write(b'MODEL\r'); ser.flush()
            time.sleep(0.1)
            resp = ser.read_all().decode(errors='ignore')
            ser.close()
            if 'SP-' in resp: return p.device
        except: continue
    return None

# ------------------------------------------------------------
# Scontel
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
        match = re.search(r'[-+]?\d*\.?\d+', resp)
        return float(match.group()) if match else None
    except: return None

def measure_cps_average(host, port, dev_num, accum_time_sec):
    if accum_time_sec < 1.0:
        if accum_time_sec < 0.01:
            return get_cps(host, port, dev_num)
        time.sleep(accum_time_sec)
        return get_cps(host, port, dev_num)
    total, valid = 0.0, 0
    for _ in range(int(accum_time_sec)):
        cps = get_cps(host, port, dev_num)
        if cps is not None: total += cps; valid += 1
        time.sleep(1)
    return total / valid if valid else None

# ------------------------------------------------------------
# Ximc (Standa)
try:
    import libximc.highlevel as ximc
except ImportError:
    cur_dir = os.path.abspath(os.path.dirname(__file__))
    ximc_dir = os.path.join(cur_dir, "ximc")
    ximc_package_dir = os.path.join(ximc_dir, "crossplatform", "wrappers", "python")
    sys.path.append(ximc_package_dir)
    import libximc.highlevel as ximc

def move(axis, distance, udistance):
    axis.command_move(distance, udistance)
    time.sleep(1)

def set_microstep_mode_8(axis):
    settings = axis.get_engine_settings()
    settings.MicrostepMode = ximc.MicrostepMode.MICROSTEP_MODE_FRAC_8
    axis.set_engine_settings(settings)

# ------------------------------------------------------------
# Вкладка для 1D графиков (спектры / распределения)
class Plot1DTab:
    def __init__(self, parent, dtype, graph_win):
        self.parent = parent
        self.graph_win = graph_win
        self.dtype = dtype
        self.app = graph_win.master

        # Для переключения оси X (только для спектров)
        self.x_unit_mode = 0  # 0: нм, 1: эВ, 2: см⁻¹
        self.x_unit_labels = ["Длина волны, нм", "Энергия фотонов, мэВ", "Волновое число, см⁻¹"]

        self.peak_annots_per_plot = []   # список списков Annotation
        self.cursor_annotation = None

        top_frame = ttk.Frame(parent)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(6,4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=top_frame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # События мыши
        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.canvas.mpl_connect('pick_event', self._on_pick)

        # Невидимая аннотация для координат курсора
        self.cursor_annotation = self.ax.annotate(
            "", xy=(0, 0), xytext=(10, 10), textcoords="offset points",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            fontsize=9, ha='left', va='top'
        )
        self.cursor_annotation.set_visible(False)

        right_panel = ttk.Frame(top_frame, width=200)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        right_panel.pack_propagate(False)
        ttk.Label(right_panel, text="Графики:").pack(anchor="w")
        self.check_canvas = tk.Canvas(right_panel, width=180, height=120)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(right_panel, orient="vertical", command=self.check_canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        self.check_frame = ttk.Frame(self.check_canvas)
        self.check_canvas.create_window((0,0), window=self.check_frame, anchor="nw")
        self.check_frame.bind("<Configure>", lambda e: self.check_canvas.configure(scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.configure(yscrollcommand=scrollbar.set)

        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        actions_frame = ttk.LabelFrame(bottom_frame, text="Действия", padding=5)
        actions_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        btn_text = "Загрузить спектры" if dtype == 'spectra' else "Загрузить распред."
        ttk.Button(actions_frame, text=btn_text, command=self.load_from_files).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Удалить выбранные", command=self.delete_selected).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Поиск пиков", command=self.find_peaks).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Сохранить в файл", command=self.save_to_file).pack(anchor="w", fill="x", pady=1)

        # Блок управления осью X
        if dtype == 'spectra':
            x_switch_frame = ttk.LabelFrame(bottom_frame, text="Ось X", padding=5)
            x_switch_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
            self.x_mode_label = ttk.Label(x_switch_frame, text=self.x_unit_labels[0])
            self.x_mode_label.pack(anchor="w")
            ttk.Button(x_switch_frame, text="Сменить единицы", command=self.switch_x_unit).pack(anchor="w", pady=2)
        else:
            # Для распределений статичная подпись
            dist_label_frame = ttk.LabelFrame(bottom_frame, text="Ось X", padding=5)
            dist_label_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
            ttk.Label(dist_label_frame, text="Позиция, мкм").pack(anchor="w")

        # Блок управления осями
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
        ttk.Checkbutton(axes_frame, text="Автомасштаб", variable=self.autoscale_var).grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Button(axes_frame, text="Применить", command=self.apply_axes).grid(row=2, column=2, columnspan=2, pady=5)

        self.plots = []        # каждый элемент: словарь
        self.plot_vars = []
        self.color_cycle = cycle(ORIGIN_COLORS)

        # Начальные подписи
        self.ax.set_ylabel("CPS")
        self.ax.set_title("Спектр" if dtype == 'spectra' else "Распределение")
        self.ax.set_xlabel(self.x_unit_labels[0] if dtype == 'spectra' else "Позиция, мкм")
        self.apply_axes()

    # ------------------ Работа с графиками ------------------
    def add_plot(self, x, y, label):
        color = next(self.color_cycle)
        x_arr = np.array(x, dtype=float)
        y_arr = np.array(y, dtype=float)
        line, = self.ax.plot(x_arr, y_arr, color=color, label=label)
        var = tk.BooleanVar(value=True)
        cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                             command=lambda idx=len(self.plots): self.toggle_visibility(idx))
        cb.pack(anchor="w")
        self.plots.append({
            'x_orig': x_arr.copy(),
            'y_orig': y_arr.copy(),
            'x': x_arr.copy(),
            'y': y_arr.copy(),
            'label': label,
            'line': line,
            'color': color,
            'var': var
        })
        self.plot_vars.append(var)
        self.peak_annots_per_plot.append([])
        self.canvas.draw_idle()

    def toggle_visibility(self, idx):
        if idx >= len(self.plots):
            return
        plot = self.plots[idx]
        visible = plot['var'].get()
        plot['line'].set_visible(visible)
        for ann in self.peak_annots_per_plot[idx]:
            ann.set_visible(visible)
        self.canvas.draw_idle()

    def delete_selected(self):
        to_remove = [i for i, p in enumerate(self.plots) if p['var'].get()]
        if not to_remove:
            messagebox.showinfo("Удаление", "Нет выбранных графиков для удаления.")
            return
        for i in sorted(to_remove, reverse=True):
            self.plots[i]['line'].remove()
            for ann in self.peak_annots_per_plot[i]:
                ann.remove()
            del self.peak_annots_per_plot[i]
            del self.plots[i]
            del self.plot_vars[i]
        for widget in self.check_frame.winfo_children():
            widget.destroy()
        for idx, p in enumerate(self.plots):
            cb = ttk.Checkbutton(self.check_frame, text=p['label'], variable=p['var'],
                                 command=lambda i=idx: self.toggle_visibility(i))
            cb.pack(anchor="w")
        self.canvas.draw_idle()

    def apply_axes(self):
        if not self.autoscale_var.get():
            try:
                xmin = float(self.xmin_var.get()) if self.xmin_var.get() else None
                xmax = float(self.xmax_var.get()) if self.xmax_var.get() else None
                ymin = float(self.ymin_var.get()) if self.ymin_var.get() else None
                ymax = float(self.ymax_var.get()) if self.ymax_var.get() else None
                if xmin is not None: self.ax.set_xlim(left=xmin)
                if xmax is not None: self.ax.set_xlim(right=xmax)
                if ymin is not None: self.ax.set_ylim(bottom=ymin)
                if ymax is not None: self.ax.set_ylim(top=ymax)
            except ValueError:
                pass
        else:
            self.ax.autoscale()
        self.canvas.draw_idle()

    def load_from_files(self):
        files = filedialog.askopenfilenames(
            title="Выберите файлы данных",
            filetypes=[("Text files", "*.txt"), ("Data files", "*.dat"), ("All files", "*.*")]
        )
        for fpath in files:
            try:
                x, y = self._parse_data_file(fpath)
                label = os.path.splitext(os.path.basename(fpath))[0]
                self.add_plot(x, y, label)
            except Exception as e:
                messagebox.showwarning("Ошибка загрузки", f"Не удалось загрузить {fpath}:\n{e}")

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
            raise ValueError("Файл не содержит данных в двух колонках")
        return x, y

    # ------------------ Курсор и пики ------------------
    def _on_mouse_move(self, event):
        if event.inaxes == self.ax:
            self.cursor_annotation.set_visible(True)
            self.cursor_annotation.xy = (event.xdata, event.ydata)
            self.cursor_annotation.set_text(f"X={event.xdata:.4f}\nY={event.ydata:.4f}")
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
                    f'{x[i]:.2f}',
                    (x[i], y[i]),
                    textcoords="offset points",
                    xytext=(0, 12),
                    ha='center',
                    fontsize=8,
                    color=p['color'],
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'),
                    picker=5
                )
                self.peak_annots_per_plot[idx].append(ann)
        self.canvas.draw_idle()

    def _simple_find_peaks(self, x, y, height_frac=0.03, distance_idx=5):
        if len(y) < 3:
            return []
        window = max(3, len(y)//50)
        if window % 2 == 0:
            window += 1
        y_sm = np.convolve(y, np.ones(window)/window, mode='same')
        peaks = []
        for i in range(1, len(y_sm)-1):
            if y_sm[i] > y_sm[i-1] and y_sm[i] > y_sm[i+1]:
                peaks.append(i)
        if not peaks:
            return []
        max_y = np.max(y_sm)
        peaks = [i for i in peaks if y_sm[i] >= height_frac * max_y]
        filtered = []
        for i in peaks:
            left = max(0, i - distance_idx)
            right = min(len(y_sm)-1, i + distance_idx)
            if y_sm[i] == np.max(y_sm[left:right+1]):
                filtered.append(i)
        return sorted(list(set(filtered)))

    def save_to_file(self):
        visible = [(p['x'], p['y'], p['label']) for p in self.plots if p['var'].get()]
        if not visible:
            messagebox.showwarning("Сохранение", "Нет видимых графиков для сохранения.")
            return

        min_x = max([np.min(x) for x, y, _ in visible])
        max_x = min([np.max(x) for x, y, _ in visible])
        if min_x >= max_x:
            messagebox.showerror("Ошибка", "Диапазоны X графиков не пересекаются.")
            return
        dx = None
        for x, y, _ in visible:
            if len(x) > 1:
                d = np.min(np.diff(x))
                if d > 0 and (dx is None or d < dx):
                    dx = d
        if dx is None or dx <= 0:
            dx = (max_x - min_x) / 1000
        num_points = int((max_x - min_x) / dx) + 1
        common_x = np.linspace(min_x, max_x, num_points)

        interpolated = []
        labels = []
        for x, y, label in visible:
            y_interp = np.interp(common_x, x, y)
            interpolated.append(y_interp)
            labels.append(label)

        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Сохранить объединённые данные"
        )
        if not filepath:
            return

        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                header = "X\t" + "\t".join(labels)
                f.write(header + "\n")
                for i in range(len(common_x)):
                    line = f"{common_x[i]:.6f}"
                    for y_arr in interpolated:
                        line += f"\t{y_arr[i]:.6f}"
                    f.write(line + "\n")
            self.app.data_queue.put(("log", f"Данные сохранены в {filepath}"))
        except Exception as e:
            self.app.data_queue.put(("error", f"Ошибка сохранения файла: {e}"))

    # ------------------ Переключение оси X ------------------
    def switch_x_unit(self):
        """Циклически переключает единицы оси X для спектров."""
        if self.dtype != 'spectra':
            return
        self.x_unit_mode = (self.x_unit_mode + 1) % 3
        label = self.x_unit_labels[self.x_unit_mode]
        self.x_mode_label.config(text=label)
        self.ax.set_xlabel(label)

        for p in self.plots:
            orig_x = p['x_orig']
            if self.x_unit_mode == 0:  # нм
                new_x = orig_x
            elif self.x_unit_mode == 1:  # мэВ
                new_x = 1240.0 / orig_x * 1000.0
            else:  # см⁻¹
                new_x = 1e7 / orig_x
            p['x'] = new_x
            p['line'].set_data(new_x, p['y'])
        # Удаляем аннотации пиков
        for annots in self.peak_annots_per_plot:
            for ann in annots:
                ann.remove()
            annots.clear()
        self.ax.relim()
        self.apply_axes()
        self.canvas.draw_idle()

# ------------------------------------------------------------
# Вкладка для 3D карт (с кнопкой "Сохранить в Origin")
class Plot3DTab:
    def __init__(self, parent, graph_win):
        self.parent = parent
        self.graph_win = graph_win
        self.app = graph_win.master

        top_frame = ttk.Frame(parent)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(6,4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=top_frame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(top_frame, width=200)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        right_panel.pack_propagate(False)
        ttk.Label(right_panel, text="3D карты:").pack(anchor="w")
        self.check_canvas = tk.Canvas(right_panel, width=180, height=120)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(right_panel, orient="vertical", command=self.check_canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        self.check_frame = ttk.Frame(self.check_canvas)
        self.check_canvas.create_window((0,0), window=self.check_frame, anchor="nw")
        self.check_frame.bind("<Configure>", lambda e: self.check_canvas.configure(scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.configure(yscrollcommand=scrollbar.set)

        bottom_frame = ttk.Frame(parent)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        actions_frame = ttk.LabelFrame(bottom_frame, text="Действия", padding=5)
        actions_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Button(actions_frame, text="Загрузить 3D карту", command=self.load_from_files).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Удалить выбранные", command=self.delete_selected).pack(anchor="w", fill="x", pady=1)
        ttk.Button(actions_frame, text="Сохранить в Origin", command=self.save_to_origin).pack(anchor="w", fill="x", pady=1)

        labels_frame = ttk.LabelFrame(bottom_frame, text="Подписи", padding=5)
        labels_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        default_xlabel = "Длина волны, нм"
        default_ylabel = "Позиция, мкм"
        default_zlabel = "CPS"
        default_title = ""

        ttk.Label(labels_frame, text="Подпись X:").grid(row=0, column=0, sticky="w")
        self.xlabel_var = tk.StringVar(value=default_xlabel)
        ttk.Entry(labels_frame, textvariable=self.xlabel_var, width=12).grid(row=0, column=1, padx=5)
        ttk.Label(labels_frame, text="Подпись Y:").grid(row=1, column=0, sticky="w")
        self.ylabel_var = tk.StringVar(value=default_ylabel)
        ttk.Entry(labels_frame, textvariable=self.ylabel_var, width=12).grid(row=1, column=1, padx=5)
        ttk.Label(labels_frame, text="Подпись Z:").grid(row=2, column=0, sticky="w")
        self.zlabel_var = tk.StringVar(value=default_zlabel)
        ttk.Entry(labels_frame, textvariable=self.zlabel_var, width=12).grid(row=2, column=1, padx=5)
        ttk.Label(labels_frame, text="Заголовок:").grid(row=3, column=0, sticky="w")
        self.title_var = tk.StringVar(value=default_title)
        ttk.Entry(labels_frame, textvariable=self.title_var, width=12).grid(row=3, column=1, padx=5)
        ttk.Button(labels_frame, text="Применить", command=self.apply_labels).grid(row=4, column=0, columnspan=2, pady=2)

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
        ttk.Checkbutton(axes_frame, text="Автомасштаб", variable=self.autoscale_var).grid(row=3, column=0, columnspan=2, sticky="w")
        ttk.Button(axes_frame, text="Применить", command=self.apply_axes).grid(row=3, column=2, columnspan=2, pady=5)

        self.maps = []
        self.map_vars = []
        self.current_im = None
        self.cbar = None

    def add_plot(self, matrix, x, y, label):
        var = tk.BooleanVar(value=True)
        cb = ttk.Checkbutton(self.check_frame, text=label, variable=var,
                             command=lambda idx=len(self.maps): self.toggle_visibility(idx))
        cb.pack(anchor="w")
        self.maps.append((matrix, x, y, label, None, var))
        self.map_vars.append(var)
        self.redraw()

    def toggle_visibility(self, idx):
        self.redraw()

    def delete_selected(self):
        to_remove = [i for i, (_, _, _, _, _, var) in enumerate(self.maps) if var.get()]
        if not to_remove:
            messagebox.showinfo("Удаление", "Нет выбранных карт для удаления.")
            return
        for i in sorted(to_remove, reverse=True):
            del self.maps[i]
            del self.map_vars[i]
        for widget in self.check_frame.winfo_children():
            widget.destroy()
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
        vis_maps = [(m, x, y, label) for (m, x, y, label, im, var) in self.maps if var.get()]
        if vis_maps:
            matrix, x, y, label = vis_maps[-1]
            im = self.ax.imshow(matrix, aspect='auto', origin='lower',
                                extent=[x[0], x[-1], y[0], y[-1]])
            self.cbar = self.figure.colorbar(im, ax=self.ax, label=self.zlabel_var.get())
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
                if xmin is not None: self.ax.set_xlim(left=xmin)
                if xmax is not None: self.ax.set_xlim(right=xmax)
                if ymin is not None: self.ax.set_ylim(bottom=ymin)
                if ymax is not None: self.ax.set_ylim(top=ymax)
                if self.current_im:
                    zmin = float(self.zmin_var.get()) if self.zmin_var.get() else None
                    zmax = float(self.zmax_var.get()) if self.zmax_var.get() else None
                    if zmin is not None: self.current_im.set_clim(vmin=zmin)
                    if zmax is not None: self.current_im.set_clim(vmax=zmax)
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
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not mat_file: return
        folder = os.path.dirname(mat_file)
        pos_file = os.path.join(folder, "positions.txt")
        wl_file = os.path.join(folder, "wavelengths.txt")
        if not os.path.exists(pos_file) or not os.path.exists(wl_file):
            messagebox.showerror("Ошибка", "Рядом с matrix.txt должны лежать positions.txt и wavelengths.txt")
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
        vis_maps = [(m, x, y, label) for (m, x, y, label, im, var) in self.maps if var.get()]
        if not vis_maps:
            messagebox.showwarning("Предупреждение", "Нет видимых 3D карт.")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".opju",
            filetypes=[("Origin Project", "*.opju")],
            title="Сохранить проект Origin"
        )
        if not filepath:
            return

        xlabel = self.xlabel_var.get()
        ylabel = self.ylabel_var.get()
        zlabel = self.zlabel_var.get()
        title = self.title_var.get()
        self.app.data_queue.put(("log", "Запуск сохранения 3D карты в Origin..."))
        threading.Thread(target=self._save_to_origin_thread,
                         args=(vis_maps, filepath, xlabel, ylabel, zlabel, title),
                         daemon=True).start()

    def _save_to_origin_thread(self, vis_maps, filepath, xlabel, ylabel, zlabel, title):
        try:
            import originpro as op
        except ImportError:
            self.app.data_queue.put(("error", "Библиотека originpro не установлена."))
            return

        try:
            self.app.data_queue.put(("log", "Подключение к Origin..."))
            op.new()
            self.app.data_queue.put(("log", "Создание матричного листа..."))
            for i, (matrix, x, y, label) in enumerate(vis_maps, start=1):
                self.app.data_queue.put(("log", f"Добавление матрицы {i}/{len(vis_maps)}: {label}"))
                mat_sheet = op.new_sheet('m', label[:30].replace(' ', '_'))
                mat_sheet.from_np(matrix)
                mat_sheet.activate()
                mat_sheet.lt_exec("worksheet -p 240 contour;")
                active_graph = op.find_graph()
                if active_graph:
                    gl = active_graph[0]
                    gl.lt_exec(f'label -x "{xlabel}";')
                    gl.lt_exec(f'label -y "{ylabel}";')
                    gl.lt_exec(f'label -z "{zlabel}";')
                    gl.lt_exec(f'title.text$ = "{title}";')

            save_dir = os.path.dirname(filepath)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
                self.app.data_queue.put(("log", f"Создана папка: {save_dir}"))

            self.app.data_queue.put(("log", f"Сохранение проекта в {filepath}..."))
            success = op.save(filepath)
            if success:
                self.app.data_queue.put(("log", "Проект успешно сохранён."))
            else:
                self.app.data_queue.put(("log", "Origin API вернул False при сохранении. Проверьте путь и права доступа."))
        except Exception as e:
            self.app.data_queue.put(("error", f"Ошибка Origin: {e}"))

# ------------------------------------------------------------
# Окно графиков
class GraphWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.title("Графики")
        self.geometry("800x750")
        self.protocol("WM_DELETE_WINDOW", self.hide)
        self.withdraw()

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        self.tab_spec = ttk.Frame(self.notebook)
        self.tab_dist = ttk.Frame(self.notebook)
        self.tab_3d = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_spec, text="Спектры")
        self.notebook.add(self.tab_dist, text="Распределения")
        self.notebook.add(self.tab_3d, text="3D спектры")

        self.spec_tab = Plot1DTab(self.tab_spec, 'spectra', self)
        self.dist_tab = Plot1DTab(self.tab_dist, 'distributions', self)
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

    def add_3d_map(self, matrix, x, y, label):
        self.tab3d.add_plot(matrix, x, y, label)

# ------------------------------------------------------------
# Главное приложение (полный код)
class Application(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Спектры и распределения ФЛ(ЭЛ) (Scontel + Standa + Acton)")
        self.geometry("900x850")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.mono = None
        self.axis = None
        self.measurement_thread = None
        self.stop_requested = False
        self.data_queue = queue.Queue()
        self.closing = False

        self.folder_path = tk.StringVar(value=os.path.expanduser("~"))
        self.filename = tk.StringVar(value="01_spectrum")
        self.scontel_ip = tk.StringVar(value="169.254.149.79")
        self.scontel_dev = tk.IntVar(value=2)
        self.accum_time = tk.DoubleVar(value=1.0)
        self.mode_var = tk.IntVar(value=1)

        self.wl_start = tk.DoubleVar(value=1000.0)
        self.wl_end = tk.DoubleVar(value=1900.0)
        self.wl_step = tk.DoubleVar(value=5.0)
        self.com_port = tk.StringVar(value="")

        self.just_wl = tk.DoubleVar(value=0.0)

        self.step_um = tk.DoubleVar(value=10.0)
        self.start_pos_um = tk.DoubleVar(value=0.0)
        self.end_pos_um = tk.DoubleVar(value=100.0)

        self.view_3d = tk.BooleanVar(value=False)
        self.vmin = tk.DoubleVar(value=0.0)
        self.vmax = tk.DoubleVar(value=1000.0)
        self.show_grid = tk.BooleanVar(value=True)
        self.last_2d_data = None

        self.graph_window = GraphWindow(self)

        self.create_widgets()
        self.after(100, self.process_queue)

    def create_widgets(self):
        left_frame = ttk.Frame(self, width=350)
        left_frame.pack(side="left", fill="y", padx=5, pady=5)
        left_frame.pack_propagate(False)

        right_frame = ttk.Frame(self, width=400)
        right_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        right_frame.pack_propagate(False)

        # Левая панель
        common_frame = ttk.LabelFrame(left_frame, text="Общие настройки", padding=10)
        common_frame.pack(fill="x", pady=5)
        ttk.Label(common_frame, text="Папка:").grid(row=0, column=0, sticky="w")
        ttk.Entry(common_frame, textvariable=self.folder_path, width=25).grid(row=0, column=1, padx=5)
        ttk.Button(common_frame, text="Обзор", command=self.browse_folder).grid(row=0, column=2)
        ttk.Label(common_frame, text="Имя файла:").grid(row=1, column=0, sticky="w", pady=2)
        ttk.Entry(common_frame, textvariable=self.filename, width=25).grid(row=1, column=1, padx=5)
        ttk.Label(common_frame, text="IP Scontel:").grid(row=2, column=0, sticky="w", pady=2)
        ip_frame = ttk.Frame(common_frame)
        ip_frame.grid(row=2, column=1, columnspan=2, sticky="w")
        ttk.Entry(ip_frame, textvariable=self.scontel_ip, width=15).pack(side="left", padx=5)
        ttk.Label(ip_frame, text="Устр:").pack(side="left")
        ttk.Entry(ip_frame, textvariable=self.scontel_dev, width=5).pack(side="left")
        com_frame = ttk.Frame(common_frame)
        com_frame.grid(row=3, column=0, columnspan=3, sticky="w", pady=2)
        ttk.Label(com_frame, text="COM Acton:").pack(side="left")
        self.com_combo = ttk.Combobox(com_frame, textvariable=self.com_port, width=10)
        self.com_combo.pack(side="left", padx=5)
        self.com_combo.bind("<Button-1>", lambda e: self.refresh_com_ports())
        ttk.Button(com_frame, text="Найти", command=self.detect_acton_port).pack(side="left", padx=2)

        status_frame = ttk.LabelFrame(left_frame, text="Статус устройств", padding=10)
        status_frame.pack(fill="x", pady=5)
        ind_frame = ttk.Frame(status_frame)
        ind_frame.pack(fill="x")
        self.scontel_canvas = tk.Canvas(ind_frame, width=20, height=20, highlightthickness=0)
        self.scontel_canvas.grid(row=0, column=0, padx=5)
        self.scontel_circle = self.scontel_canvas.create_oval(2,2,18,18, fill="gray", outline="black")
        ttk.Label(ind_frame, text="Scontel").grid(row=0, column=1, padx=5)
        self.standa_canvas = tk.Canvas(ind_frame, width=20, height=20, highlightthickness=0)
        self.standa_canvas.grid(row=0, column=2, padx=15)
        self.standa_circle = self.standa_canvas.create_oval(2,2,18,18, fill="gray", outline="black")
        ttk.Label(ind_frame, text="Standa").grid(row=0, column=3, padx=5)
        self.acton_canvas = tk.Canvas(ind_frame, width=20, height=20, highlightthickness=0)
        self.acton_canvas.grid(row=0, column=4, padx=15)
        self.acton_circle = self.acton_canvas.create_oval(2,2,18,18, fill="gray", outline="black")
        ttk.Label(ind_frame, text="Acton").grid(row=0, column=5, padx=5)
        ttk.Button(status_frame, text="Проверить подключения", command=self.check_connections).pack(pady=5)

        mode_frame = ttk.LabelFrame(left_frame, text="Режим", padding=10)
        mode_frame.pack(fill="x", pady=5)
        ttk.Radiobutton(mode_frame, text="0 – Юстировка", variable=self.mode_var, value=0, command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="1 – Спектр в точке", variable=self.mode_var, value=1, command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="2 – Пространственное распределение", variable=self.mode_var, value=2, command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="3 – Спектры в разных точках", variable=self.mode_var, value=3, command=self.update_mode).pack(anchor="w")

        self.param_frame = ttk.Frame(left_frame)
        self.param_frame.pack(fill="x", pady=5)

        self.time_label = ttk.Label(left_frame, text="Расчётное время: --:--",
                                    font=("TkDefaultFont", 10, "bold"), relief="sunken", anchor="center")
        self.time_label.pack(fill="x", pady=5)

        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(pady=5)
        self.start_btn = ttk.Button(btn_frame, text="Старт", command=self.start_measurement)
        self.start_btn.pack(side="left", padx=5)
        self.stop_btn = ttk.Button(btn_frame, text="Стоп", command=self.stop_measurement, state="disabled")
        self.stop_btn.pack(side="left", padx=5)
        self.graph_btn = ttk.Button(btn_frame, text="Графики", command=self.toggle_graph_window)
        self.graph_btn.pack(side="left", padx=5)

        link_frame = ttk.Frame(right_frame)
        link_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(2, 2))
        link_label = tk.Label(
            link_frame,
            text="GitHub Repository",
            fg="blue",
            cursor="hand2",
            font=("TkDefaultFont", 9, "underline")
        )
        link_label.pack()
        link_label.bind(
            "<Button-1>",
            lambda e: webbrowser.open("https://github.com/KrazyCatPower/Standa_Acton2300i_SSPDScontel_Spectra_Distribution.git")
        )

        self.progress = ttk.Progressbar(left_frame, length=200, mode="determinate")
        self.progress.pack(side="bottom", fill="x", pady=5)

        # Правая панель
        self.graph_frame = ttk.Frame(right_frame)
        self.graph_frame.pack(side="top", fill="both", expand=True)

        self.view_3d_frame = ttk.LabelFrame(self.graph_frame, text="Визуализация карты", padding=5)

        self.figure = Figure(figsize=(5,3), dpi=100)
        self.ax = None
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.cbar = None

        log_frame = ttk.LabelFrame(right_frame, text="Журнал", padding=5)
        log_frame.pack(side="bottom", fill="x", pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state="disabled")
        self.log_text.pack(fill="both", expand=True)

        self.update_mode()

    # ---------- Вспомогательные методы ----------
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

    def update_mode(self):
        for w in self.param_frame.winfo_children():
            w.destroy()
        mode = self.mode_var.get()

        if mode == 0:
            ttk.Label(self.param_frame, text="Длина волны (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.just_wl, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Накопление (с):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.accum_time, width=5).pack(anchor="w", pady=2)
        else:
            ttk.Label(self.param_frame, text="Накопление (с):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.accum_time, width=5).pack(anchor="w", pady=2)

        if mode == 1:
            ttk.Label(self.param_frame, text="λ нач (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_start, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="λ кон (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_end, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_step, width=5).pack(anchor="w", pady=2)
        elif mode == 2:
            ttk.Label(self.param_frame, text="Нач. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.start_pos_um, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Кон. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.end_pos_um, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.step_um, width=5).pack(anchor="w", pady=2)
        elif mode == 3:
            ttk.Label(self.param_frame, text="--- Спектр ---").pack(anchor="w")
            ttk.Label(self.param_frame, text="λ нач (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_start, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="λ кон (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_end, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (нм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.wl_step, width=5).pack(anchor="w", pady=2)

            ttk.Label(self.param_frame, text="--- Подвижка ---").pack(anchor="w", pady=(10,0))
            ttk.Label(self.param_frame, text="Нач. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.start_pos_um, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Кон. позиция (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.end_pos_um, width=8).pack(anchor="w", pady=2)
            ttk.Label(self.param_frame, text="Шаг (мкм):").pack(anchor="w")
            ttk.Entry(self.param_frame, textvariable=self.step_um, width=5).pack(anchor="w", pady=2)

        if mode == 3:
            for w in self.view_3d_frame.winfo_children():
                w.destroy()
            ttk.Checkbutton(self.view_3d_frame, text="3D вид", variable=self.view_3d, command=self.refresh_plot2d).grid(row=0, column=0, padx=5)
            ttk.Checkbutton(self.view_3d_frame, text="Сетка", variable=self.show_grid, command=self.refresh_plot2d).grid(row=0, column=1, padx=5)
            ttk.Label(self.view_3d_frame, text="Мин CPS:").grid(row=0, column=2, padx=5)
            ttk.Entry(self.view_3d_frame, textvariable=self.vmin, width=6).grid(row=0, column=3)
            ttk.Label(self.view_3d_frame, text="Макс CPS:").grid(row=0, column=4, padx=5)
            ttk.Entry(self.view_3d_frame, textvariable=self.vmax, width=6).grid(row=0, column=5)
            ttk.Button(self.view_3d_frame, text="Обновить", command=self.refresh_plot2d).grid(row=0, column=6, padx=10)
            self.view_3d_frame.pack(side="bottom", fill="x", pady=2, anchor="w")
        else:
            self.view_3d_frame.pack_forget()

        self.update_time_estimate()
        for var in (self.accum_time, self.wl_start, self.wl_end, self.wl_step,
                     self.step_um, self.start_pos_um, self.end_pos_um):
            var.trace_add("write", lambda *a: self.update_time_estimate())

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
            if accum <= 0:
                self.time_label.config(text="Расчётное время: --:--")
                return
            if mode == 1:
                n = int(abs(self.wl_end.get() - self.wl_start.get()) / self.wl_step.get()) + 1
                t = n * (accum + 1)
            elif mode == 2:
                n = int(abs(self.end_pos_um.get() - self.start_pos_um.get()) / self.step_um.get()) + 1
                t = n * (accum + 1)
            elif mode == 3:
                n_wl = int(abs(self.wl_end.get() - self.wl_start.get()) / self.wl_step.get()) + 1
                n_pos = int(abs(self.end_pos_um.get() - self.start_pos_um.get()) / self.step_um.get()) + 1
                t = n_pos * n_wl * (accum + 1)
            else:
                t = 0
            mins, secs = divmod(int(t), 60)
            self.time_label.config(text=f"Расчётное время: {mins:02d} мин {secs:02d} с")
        except:
            self.time_label.config(text="Расчётное время: --:--")

    def browse_folder(self):
        path = filedialog.askdirectory(initialdir=self.folder_path.get())
        if path: self.folder_path.set(path)

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
            new_name = f"{num+1:02d}_{rest}" if rest else f"{num+1:02d}"
        else:
            new_name = f"01_{name}"
        self.filename.set(new_name)

    def check_connections(self):
        self.log("Проверка подключений...")
        self.set_indicator("scontel", "gray")
        self.set_indicator("standa", "gray")
        self.set_indicator("acton", "gray")
        threading.Thread(target=self._check_devices, daemon=True).start()

    def _check_devices(self):
        ip = self.scontel_ip.get()
        dev = self.scontel_dev.get()
        port = self.com_port.get()

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
            devenum = ximc.enumerate_devices(ximc.EnumerateFlags.ENUMERATE_PROBE | ximc.EnumerateFlags.ENUMERATE_NETWORK, "addr=")
            if devenum:
                axis = ximc.Axis(devenum[0]["uri"])
                axis.open_device()
                axis.get_status()
                axis.close_device()
                self.data_queue.put(("indicator", ("standa", "green")))
                self.data_queue.put(("log", "Standa: OK"))
            else:
                self.data_queue.put(("indicator", ("standa", "red")))
                self.data_queue.put(("log", "Standa: не найдена"))
        except Exception as e:
            self.data_queue.put(("indicator", ("standa", "red")))
            self.data_queue.put(("log", f"Standa: {e}"))

        if port:
            try:
                mono_test = ActonPy(port)
                wl = mono_test.get_wavelength()
                self.data_queue.put(("indicator", ("acton", "green")))
                self.data_queue.put(("log", f"Acton ({port}): {mono_test.model}, тек. λ={wl:.2f} нм"))
                mono_test.closeConnection()
            except Exception as e:
                self.data_queue.put(("indicator", ("acton", "red")))
                self.data_queue.put(("log", f"Acton: {e}"))
        else:
            self.data_queue.put(("indicator", ("acton", "red")))
            self.data_queue.put(("log", "Acton: не выбран порт"))

    def set_indicator(self, dev, color):
        canvases = {"scontel": (self.scontel_canvas, self.scontel_circle),
                    "standa": (self.standa_canvas, self.standa_circle),
                    "acton": (self.acton_canvas, self.acton_circle)}
        if dev in canvases: canvases[dev][0].itemconfig(canvases[dev][1], fill=color)

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

    def start_measurement(self):
        mode = self.mode_var.get()
        if mode != 0:
            if self.accum_time.get() <= 0:
                messagebox.showerror("Ошибка", "Время накопления должно быть > 0")
                return
        if mode in (1,3) and not self.com_port.get():
            messagebox.showerror("Ошибка", "Укажите COM-порт"); return
        if mode in (1,3):
            try:
                if self.wl_step.get() <= 0 or self.wl_start.get() == self.wl_end.get(): raise ValueError
            except:
                messagebox.showerror("Ошибка", "Параметры длин волн"); return
        if mode in (2,3):
            try:
                if self.step_um.get() <= 0 or self.start_pos_um.get() == self.end_pos_um.get(): raise ValueError
            except:
                messagebox.showerror("Ошибка", "Параметры подвижки"); return

        if self.mono is not None:
            try: self.mono.closeConnection()
            except: pass
            self.mono = None
            time.sleep(1.0)

        self.stop_requested = False
        self.set_ui_state(True)
        self.log("="*30 + " СТАРТ " + "="*30)
        self.progress["value"] = 0
        self.measurement_thread = threading.Thread(target=self.run_measurement, args=(mode,), daemon=True)
        self.measurement_thread.start()

    def stop_measurement(self):
        self.stop_requested = True
        self.log("Остановка...")

    def run_measurement(self, mode):
        try:
            ip = self.scontel_ip.get()
            dev = self.scontel_dev.get()
            accum = self.accum_time.get()
            folder = self.folder_path.get()
            base_name = self.filename.get()

            if mode in (0, 1, 3):
                self.mono = ActonPy(self.com_port.get())
                self.data_queue.put(("log", "Acton подключён"))

            if mode == 2 and self.com_port.get():
                try:
                    mono_temp = ActonPy(self.com_port.get())
                    wl = mono_temp.get_wavelength()
                    if abs(wl) > mono_temp.tolerance_nm:
                        mono_temp.goto(0.0)
                    mono_temp.closeConnection()
                    self.data_queue.put(("log", f"Монохроматор установлен на 0 нм"))
                except Exception as e:
                    self.data_queue.put(("log", f"Не удалось установить монохроматор в 0: {e}"))

            if mode in (2,3):
                devenum = ximc.enumerate_devices(ximc.EnumerateFlags.ENUMERATE_PROBE | ximc.EnumerateFlags.ENUMERATE_NETWORK, "addr=")
                if devenum: open_name = devenum[0]["uri"]
                else: open_name = "xi-emu:///" + os.path.join(os.path.expanduser('~'), "testdevice.bin")
                self.axis = ximc.Axis(open_name)
                self.axis.open_device()
                set_microstep_mode_8(self.axis)
                move(self.axis, 0, 0)
                start_um = self.start_pos_um.get()
                end_um = self.end_pos_um.get()
                step_um = self.step_um.get()
                numstep = int(math.ceil(abs(end_um - start_um) / step_um)) + 1
                positions_um = [start_um + i * step_um if end_um >= start_um else start_um - i * step_um for i in range(numstep)]
                self.data_queue.put(("log", f"Standa готова, точек: {numstep}"))

            # ---------- Режимы измерений ----------
            if mode == 0:
                wl = self.just_wl.get()
                self.mono.goto(wl)
                self.data_queue.put(("log", f"Монохроматор на {wl} нм, запись до Стоп (файл не сохраняется)"))
                times, cpss = [], []
                start_time = time.time()
                while not self.stop_requested:
                    cps = measure_cps_average(ip, 9876, dev, accum)
                    t = time.time() - start_time
                    if cps is None: cps = 0.0
                    times.append(t); cpss.append(cps)
                    while times and times[0] < t - 10:
                        times.pop(0); cpss.pop(0)
                    self.data_queue.put(("plot_just", (times.copy(), cpss.copy(), cps)))
                self.mono.closeConnection(); self.mono = None
                self.log("Юстировка завершена")

            elif mode == 1:
                wl_start = self.wl_start.get(); wl_end = self.wl_end.get(); wl_step = self.wl_step.get()
                n_wl = int(abs(wl_end - wl_start) / wl_step) + 1
                self.data_queue.put(("progress", (0, n_wl)))
                wl_array, cps_array = [], []
                filepath = os.path.join(folder, f"{base_name}.txt")
                with open(filepath, "w") as f:
                    f.write(f"# Wavelength(nm)\tCPS\tAccum_time={accum}s\n")
                    self.mono.goto(wl_start)
                    for i in range(n_wl):
                        if self.stop_requested: break
                        target = wl_start + i * wl_step if wl_end > wl_start else wl_start - i * wl_step
                        self.mono.goto(target)
                        cps = measure_cps_average(ip, 9876, dev, accum)
                        if cps is None: continue
                        wl_array.append(target); cps_array.append(cps)
                        f.write(f"{target:.2f}\t{cps:.6f}\n"); f.flush()
                        self.data_queue.put(("plot_1d", (wl_array.copy(), cps_array.copy(), target, cps)))
                        self.data_queue.put(("progress", (i+1, n_wl)))
                self.mono.closeConnection(); self.mono = None
                self.increment_filename()
                self.data_queue.put(("log", "Спектр записан"))
                self.data_queue.put(("add_spectrum", (wl_array, cps_array, base_name)))

            elif mode == 2:
                filepath = os.path.join(folder, f"{base_name}.txt")
                X, Y = [], []
                self.data_queue.put(("progress", (0, numstep)))
                with open(filepath, "w") as f:
                    f.write(f"# Position(um)\tCPS\tAccum={accum}s\n")
                    for idx, target_um in enumerate(positions_um, start=1):
                        if self.stop_requested: break
                        pos = int(math.ceil(target_um / fullstep_1))
                        move(self.axis, pos, 0)
                        avg = measure_cps_average(ip, 9876, dev, accum)
                        if avg is None:
                            time.sleep(1); avg = measure_cps_average(ip, 9876, dev, accum)
                            if avg is None: continue
                        X.append(target_um); Y.append(avg)
                        f.write(f"{target_um:.6f}\t{avg:.6f}\n"); f.flush()
                        self.data_queue.put(("plot_1d", (X.copy(), Y.copy(), target_um, avg)))
                        self.data_queue.put(("progress", (idx, numstep)))
                move(self.axis, 0, 0); self.axis.close_device(); self.axis = None
                self.increment_filename()
                self.data_queue.put(("log", "Сканирование завершено"))
                self.data_queue.put(("add_distribution", (X, Y, base_name)))

            elif mode == 3:
                wl_start = self.wl_start.get(); wl_end = self.wl_end.get(); wl_step = self.wl_step.get()
                n_wl = int(abs(wl_end - wl_start) / wl_step) + 1
                data_folder = os.path.join(folder, base_name)
                os.makedirs(data_folder, exist_ok=True)
                all_spectra = []
                for idx, target_um in enumerate(positions_um, start=1):
                    if self.stop_requested: break
                    pos = int(math.ceil(target_um / fullstep_1))
                    move(self.axis, pos, 0)
                    wl_list, cps_list = [], []
                    filepath = os.path.join(data_folder, f"spectrum_pos{target_um:.1f}um.txt")
                    with open(filepath, "w") as f:
                        f.write(f"# Position={target_um:.3f}um\n# WL(nm)\tCPS\n")
                        self.mono.goto(wl_start)
                        for i in range(n_wl):
                            if self.stop_requested: break
                            target_wl = wl_start + i * wl_step if wl_end > wl_start else wl_start - i * wl_step
                            self.mono.goto(target_wl)
                            cps = measure_cps_average(ip, 9876, dev, accum)
                            if cps is None: continue
                            wl_list.append(target_wl); cps_list.append(cps)
                            f.write(f"{target_wl:.2f}\t{cps:.6f}\n")
                            self.data_queue.put(("plot_1d", (wl_list.copy(), cps_list.copy(), target_um, None)))
                        f.flush()
                    if wl_list:
                        all_spectra.append((target_um, wl_list, cps_list))
                    self.data_queue.put(("progress", (idx, len(positions_um))))
                if all_spectra and not self.stop_requested:
                    wl_common = all_spectra[0][1]
                    matrix = np.zeros((len(all_spectra), len(wl_common)))
                    positions_list = [p[0] for p in all_spectra]
                    for idx, (p, wls, cpses) in enumerate(all_spectra):
                        if wls == wl_common: matrix[idx,:] = cpses
                        else: matrix[idx,:] = np.interp(wl_common, wls, cpses)
                    np.savetxt(os.path.join(data_folder, "matrix.txt"), matrix, delimiter="\t")
                    with open(os.path.join(data_folder, "positions.txt"), "w") as f: f.write("\n".join(map(str, positions_list)))
                    with open(os.path.join(data_folder, "wavelengths.txt"), "w") as f: f.write("\n".join(map(str, wl_common)))
                    self.last_2d_data = (matrix, wl_common, positions_list)
                    self.data_queue.put(("plot_2d", self.last_2d_data))
                    self.data_queue.put(("add_3d", (matrix, wl_common, positions_list, base_name)))
                self.mono.closeConnection(); self.mono = None
                move(self.axis, 0, 0); self.axis.close_device(); self.axis = None
                self.increment_filename()
                self.data_queue.put(("log", "Измерение завершено"))

        except Exception as e:
            self.data_queue.put(("log", f"ОШИБКА: {e}"))
            self.data_queue.put(("error", str(e)))
        finally:
            if self.mono:
                try: self.mono.closeConnection()
                except: pass
                self.mono = None
            if self.axis:
                try: move(self.axis, 0, 0); self.axis.close_device()
                except: pass
                self.axis = None
            self.data_queue.put(("finish", None))

    def process_queue(self):
        try:
            while True:
                msg = self.data_queue.get_nowait()
                if msg[0] == "log": self.log(msg[1])
                elif msg[0] == "progress":
                    cur, tot = msg[1]
                    self.progress["maximum"] = tot
                    self.progress["value"] = cur
                elif msg[0] == "indicator": self.set_indicator(*msg[1])
                elif msg[0] == "plot_1d":
                    x, y, extra, cps = msg[1]
                    self._plot_1d(x, y, extra, cps, is_just=False)
                elif msg[0] == "plot_just":
                    x, y, cps = msg[1]
                    self._plot_1d(x, y, None, cps, is_just=True)
                elif msg[0] == "plot_2d":
                    self._plot_2d(msg[1])
                elif msg[0] == "add_spectrum":
                    x, y, label = msg[1]
                    self.graph_window.add_spectrum(x, y, label)
                elif msg[0] == "add_distribution":
                    x, y, label = msg[1]
                    self.graph_window.add_distribution(x, y, label)
                elif msg[0] == "add_3d":
                    matrix, x, y, label = msg[1]
                    self.graph_window.add_3d_map(matrix, x, y, label)
                elif msg[0] == "error":
                    messagebox.showerror("Ошибка", msg[1])
                elif msg[0] == "finish":
                    self.set_ui_state(False)
                    self.measurement_thread = None
        except queue.Empty:
            pass
        if self.closing and self.measurement_thread is None:
            self.destroy()
            return
        self.after(100, self.process_queue)

    def _plot_1d(self, x, y, extra, cps, is_just=False):
        if not hasattr(self, 'ax') or self.ax is None:
            self.ax = self.figure.add_subplot(111)
        self.ax.clear()
        self.ax.plot(x, y, 'b-')
        mode = self.mode_var.get()
        if is_just or mode == 0:
            self.ax.set_xlabel('Время, с')
            self.ax.set_ylabel('CPS')
            self.ax.set_title(f'Юстировка, CPS: {cps:.2f}')
            if x:
                t_max = max(x)
                self.ax.set_xlim(t_max - 10, t_max + 1)
                max_y = max(y) if y else 1
                self.ax.set_ylim(0, max_y * 1.1 if max_y > 0 else 1)
        elif mode == 2:
            self.ax.set_xlabel('Позиция, мкм')
            self.ax.set_title(f'Поз: {extra:.2f} мкм, CPS: {cps:.2f}')
        elif mode == 1:
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_title(f'λ = {extra:.2f} нм, CPS = {cps:.2f}')
        else:
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_title(f'Спектр на {extra:.1f} мкм')
        self.ax.grid(True, alpha=0.3)
        self.figure.tight_layout()
        self.canvas.draw()

    def _plot_2d(self, data):
        matrix, wls, pos = data
        self.figure.clear()
        if self.view_3d.get():
            self.ax = self.figure.add_subplot(111, projection='3d')
            X, Y = np.meshgrid(wls, pos)
            self.ax.plot_surface(X, Y, matrix, cmap='viridis', rstride=1, cstride=1, alpha=0.9)
            self.ax.set_xlabel('Длина волны, нм')
            self.ax.set_ylabel('Позиция, мкм')
            self.ax.set_zlabel('CPS')
            if not self.show_grid.get(): self.ax.grid(False)
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
                self.log("Завершение, ожидание остановки измерения...")
        else:
            self.graph_window.destroy()
            self.destroy()

if __name__ == "__main__":
    app = Application()
    app.mainloop()
import tkinter as tk
from tkinter import ttk, messagebox
import cv2
from PIL import Image, ImageTk
import serial
import pygame
import threading
import time
import numpy as np


# KONFIGURACJA 
SERIAL_PORT = "COM5"
BAUD_RATE = 9600
CAMERA_INDEX = 0

SPEED_M1_US = 12000
SPEED_M2_US = 8000
SPEED_M3_US = 8000

# KONFIGURACJA PADA (XBOX)
DEADZONE = 0.25 # Ustawienie martwej strefy
LOOP_DELAY = 0.05  # 20 Hz

STEP_PACKET_M1 = 6 # Obrót 1 osi przyciskami LB/RB
STEP_PACKET_M2 = 6 # Obrót 2 osi lewą gałką 
STEP_PACKET_M3 = 6 # Obrót 3 osi prawą gałką

AXIS_LEFT_Y = 3 
AXIS_RIGHT_Y = 1 
# Odwrócenie kierunku ruchu
INVERT_LEFT_Y = True
INVERT_RIGHT_Y = False

BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5

SERVO_OPEN = 15 # Szczęki otwarete
SERVO_CLOSE = 70 # Szczęki zamknięte

AUTO_CONF_TH = 0.85
AUTO_STABLE_FRAMES = 6
AUTO_COOLDOWN_SEC = 1.0

# Odtwarzanie trajektorii
PLAYBACK_DT = LOOP_DELAY


# MODEL KERAS
try:
    from keras.models import load_model
    KERAS_AVAILABLE = True
except ImportError:
    print("UWAGA: Brak TensorFlow/Keras — tryb symulacji wizyjnej.")
    KERAS_AVAILABLE = False


class RobotController:
    def __init__(self):
        self.connected = False
        self.ser = None

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.connected = True
            print(f"[OK] Arduino połączone na {SERIAL_PORT}.")
        except Exception as e:
            print(f"[UWAGA] BRAK ARDUINO — tryb symulacji ({e})")

        self.target = {'1': 0, '2': 0, '3': 0}
        self.servo_pos = SERVO_OPEN

        self.saved_positions = {
            "POBRANIE": {'1': 0, '2': 0, '3': 0},
        }

        self.motion_lock = threading.Lock()

    def send_command(self, motor_char, value):
        if not self.connected:
            return
        cmd = f"{motor_char}:{int(value)}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            print(f"[BŁĄD] Wysyłanie: {e}")

    def send_all_targets(self, target_dict):
        if not self.connected:
            return
        try:
            self.ser.write(f"1:{int(target_dict['1'])}\n".encode())
            self.ser.write(f"2:{int(target_dict['2'])}\n".encode())
            self.ser.write(f"3:{int(target_dict['3'])}\n".encode())
        except Exception as e:
            print(f"[BŁĄD] Pakiet osi: {e}")

    def go_to_position(self, pos_dict):
        self.send_all_targets(pos_dict)
        self.target = pos_dict.copy()

    def go_home(self):
        self.go_to_position({'1': 0, '2': 0, '3': 0})

    def set_zero_here(self):
        # zerowanie softowe w Arduino (AccelStepper setCurrentPosition)
        self.target = {'1': 0, '2': 0, '3': 0}
        self.send_command('H', 0)
        print("[OK] Ustawiono ZERO.")

    def move_servo(self, angle):
        self.servo_pos = int(angle)
        self.send_command('V', self.servo_pos)
        
# Obliczenie czasu potrzebnego na dojazd z uwzględnieniem rampy przyspieszewń
    def calculate_wait_time(self, current_target_dict, next_target_dict):
       
        speeds = {'1': SPEED_M1_US, '2': SPEED_M2_US, '3': SPEED_M3_US}
        max_time = 0.0
        for m in ['1', '2', '3']:
            dist = abs(int(next_target_dict[m]) - int(current_target_dict[m]))
            t_sec = (dist * speeds[m]) / 1_000_000.0
            max_time = max(max_time, t_sec)
        return max_time + 0.50  

# Zwracanie obiektów
class VisionSystem:
    def __init__(self):
        self.model = None
        self.class_names = []

        if KERAS_AVAILABLE:
            try:
                self.model = load_model("keras_model.h5", compile=False)
                with open("labels.txt", "r", encoding="utf-8") as f:
                    self.class_names = [line.strip() for line in f.readlines()]
                print("[OK] Model załadowany.")
            except Exception as e:
                print(f"[UWAGA] Błąd modelu: {e}")
                self.model = None

    def predict(self, frame):
        if self.model:
            img = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
            img_array = np.asarray(img, dtype=np.float32).reshape(1, 224, 224, 3)
            img_array = (img_array / 127.5) - 1.0

            prediction = self.model.predict(img_array, verbose=0)
            index = int(np.argmax(prediction))
            confidence = float(prediction[0][index])
            raw_label = self.class_names[index].lower()

            if "puszka" in raw_label:
                return "PUSZKA", confidence
            if "butelka" in raw_label:
                return "BUTELKA", confidence
            return "BRAK", confidence

        # fallback testowy
        h, w, _ = frame.shape
        center = frame[h // 2, w // 2]
        b, g, r = center
        if r > 160 and g < 100 and b < 100:
            return "PUSZKA", 0.95
        if b > 160 and r < 100 and g < 100:
            return "BUTELKA", 0.95
        return "BRAK", 0.0

# Panel operatora z 3 trybami 
class App:
    def __init__(self, root, robot: RobotController):
        self.root = root
        self.robot = robot
        self.vision = VisionSystem()

        self.root.title("PANEL OPERATORA ")
        self.root.geometry("1150x820")
        self.root.bind('<q>', lambda event: self.on_close())

        self.current_mode = "FREE"
        self.running = True

        self.detected_obj = "BRAK"
        self.confidence = 0.0

        self._stable_label = "BRAK"
        self._stable_count = 0
        self._auto_busy = False

        self.joystick = None
        self.cap = cv2.VideoCapture(CAMERA_INDEX)

        # trajektorie: lista stanów 
        self.traj = {"PUSZKA": [], "BUTELKA": []}

        # nagrywanie
        self.is_recording = False
        self.record_name = None
        self.record_buffer = []
        self.record_lock = threading.Lock()

        self.setup_gui()
        self.setup_gamepad()

        # kalibracja gałek do sterowania osiami
        self.axis_zero = {"ly": 0.0, "ry": 0.0}
        if self.joystick:
            pygame.event.pump()
            try:
                self.axis_zero["ly"] = float(self.joystick.get_axis(AXIS_LEFT_Y))
                self.axis_zero["ry"] = float(self.joystick.get_axis(AXIS_RIGHT_Y))
                print("[OK] Kalibracja osi:", self.axis_zero)
            except Exception:
                pass

        self.t_camera = threading.Thread(target=self.camera_loop, daemon=True)
        self.t_camera.start()

        self.t_gamepad = threading.Thread(target=self.gamepad_loop, daemon=True)
        self.t_gamepad.start()

        self.t_auto = threading.Thread(target=self.auto_logic_loop, daemon=True)
        self.t_auto.start()

    # GUI
    def setup_gui(self):
        main_frame = tk.Frame(self.root, bg="#f0f0f0")
        main_frame.pack(fill="both", expand=True)

        video_panel = tk.Frame(main_frame, bg="black", width=720, height=540)
        video_panel.grid(row=0, column=0, padx=10, pady=10)
        self.lbl_video = tk.Label(video_panel, bg="black")
        self.lbl_video.pack()

        ctrl_panel = tk.Frame(main_frame, bg="#f0f0f0")
        ctrl_panel.grid(row=0, column=1, sticky="ns", padx=10, pady=10)

        tk.Label(ctrl_panel, text="PANEL STEROWANIA", font=("Arial", 16, "bold"),
                 bg="#f0f0f0").pack(pady=6)

        mode_frame = tk.LabelFrame(ctrl_panel, text="TRYB PRACY", bg="#f0f0f0",
                                   font=("Arial", 10, "bold"))
        mode_frame.pack(fill="x", pady=6)

        tk.Button(mode_frame, text="SWOBODNY", bg="lightblue",
                  command=lambda: self.set_mode("FREE")).pack(fill="x", pady=2)
        tk.Button(mode_frame, text="UCZENIE (NAGRYWANIE)", bg="lightyellow",
                  command=lambda: self.set_mode("TEACH")).pack(fill="x", pady=2)
        tk.Button(mode_frame, text="AUTOMAT (ODTWARZANIE)", bg="lightgreen",
                  command=lambda: self.set_mode("AUTO")).pack(fill="x", pady=2)

        self.lbl_current_mode = tk.Label(ctrl_panel, text="TRYB: SWOBODNY",
                                         font=("Arial", 16, "bold"),
                                         fg="blue", bg="#f0f0f0")
        self.lbl_current_mode.pack(pady=10)

        pos_frame = tk.LabelFrame(ctrl_panel, text="POZYCJE (CELE)", bg="#f0f0f0",
                                  font=("Arial", 10, "bold"))
        pos_frame.pack(fill="x", pady=6)

        self.lbl_coords = tk.Label(pos_frame, text="M1:0  M2:0  M3:0",
                                   font=("Consolas", 13, "bold"),
                                   relief="sunken", bg="white")
        self.lbl_coords.pack(fill="x", padx=6, pady=6)

        tk.Button(ctrl_panel, text="POWRÓT DO POZYCJI ZERO", bg="orange",
                  command=self.robot.go_home).pack(fill="x", pady=2)

        tk.Button(ctrl_panel, text="KALIBRACJA PUNKTU ZERO", bg="red", fg="white",
                  command=self.robot.set_zero_here).pack(fill="x", pady=2)

        # UCZENIE
        self.teach_frame = tk.LabelFrame(ctrl_panel, text="UCZENIE — POBRANIE + TRAJEKTORIE", bg="#f0f0f0",
                                         font=("Arial", 10, "bold"))

        tk.Button(self.teach_frame, text="ZAPISZ PUNKT: POBRANIE", bg="yellow",
                  command=self.save_pickup).pack(fill="x", pady=2, padx=6)

        ttk.Separator(self.teach_frame, orient="horizontal").pack(fill="x", padx=6, pady=6)

        tk.Button(self.teach_frame, text="START NAGRYWANIA: PUSZKA (od POBRANIA)",
                  bg="#ffbcbc", command=lambda: self.start_record("PUSZKA")).pack(fill="x", pady=2, padx=6)

        tk.Button(self.teach_frame, text="START NAGRYWANIA: BUTELKA (od POBRANIA)",
                  bg="#c3cbff", command=lambda: self.start_record("BUTELKA")).pack(fill="x", pady=2, padx=6)

        tk.Button(self.teach_frame, text="STOP NAGRYWANIA",
                  bg="#ff7777", fg="white", command=self.stop_record).pack(fill="x", pady=6, padx=6)

        self.lbl_traj_info = tk.Label(self.teach_frame,
                                      text=self._traj_info_text(),
                                      font=("Consolas", 10, "bold"),
                                      bg="#f0f0f0", justify="left")
        self.lbl_traj_info.pack(fill="x", padx=6, pady=2)

        # STATUS
        det_frame = tk.LabelFrame(ctrl_panel, text="WYKRYCIE I STATUS", bg="#f0f0f0",
                                  font=("Arial", 10, "bold"))
        det_frame.pack(fill="x", pady=10)

        self.lbl_detection = tk.Label(det_frame,
                                      text="WYKRYCIE: BRAK\nPEWNOSC: 0%\nSTATUS: GOTOWY",
                                      font=("Consolas", 12, "bold"),
                                      bg="#f0f0f0", justify="left")
        self.lbl_detection.pack(fill="x", padx=6, pady=6)

        # Instrukcja
        help_frame = tk.LabelFrame(ctrl_panel, text="STEROWANIE PADEM ", bg="#f0f0f0",
                                   font=("Arial", 10, "bold"))
        help_frame.pack(fill="x", pady=6)

        txt = (
            "M2: lewa gałka (góra/dół)\n"
            "M3: prawa gałka (góra/dół)\n"
            "M1 baza: LB/RB\n"
            "Serwo: X=OTWÓRZ, Y=ZAMKNIJ\n"
            "Wyjście: Q"
        )
        tk.Label(help_frame, text=txt, bg="#f0f0f0", justify="left",
                 font=("Consolas", 10)).pack(anchor="w", padx=6, pady=6)

    def setup_gamepad(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[OK] Pad: {self.joystick.get_name()}")
        else:
            self.joystick = None
            print("[UWAGA] Brak pada.")

    def set_mode(self, mode):
        self.current_mode = mode
        self.teach_frame.pack_forget()
        if mode == "FREE":
            self.lbl_current_mode.config(text="TRYB: SWOBODNY", fg="blue")
        elif mode == "TEACH":
            self.lbl_current_mode.config(text="TRYB: UCZENIE (NAGRYWANIE)", fg="orange")
            self.teach_frame.pack(fill="x", pady=10)
        elif mode == "AUTO":
            self.lbl_current_mode.config(text="TRYB: AUTOMAT (ODTWARZANIE)", fg="green")

    # ---------------- UCZENIE ----------------
    def _traj_info_text(self):
        return f"TRASY: PUSZKA={len(self.traj['PUSZKA'])} kroków, BUTELKA={len(self.traj['BUTELKA'])} kroków"

    def save_pickup(self):
        if self.current_mode != "TEACH":
            messagebox.showwarning("Błąd", "Zapis działa tylko w trybie UCZENIE.")
            return
        self.robot.saved_positions["POBRANIE"] = self.robot.target.copy()
        messagebox.showinfo("OK", "Zapisano punkt POBRANIE.")

    def start_record(self, name):
        if self.current_mode != "TEACH":
            messagebox.showwarning("Błąd", "Nagrywanie działa tylko w trybie UCZENIE.")
            return

        pickup = self.robot.saved_positions.get("POBRANIE")
        if not pickup:
            messagebox.showwarning("Błąd", "Najpierw zapisz punkt POBRANIE.")
            return

        with self.record_lock:
            if self.is_recording:
                messagebox.showwarning("Błąd", "Nagrywanie już trwa.")
                return
            self.is_recording = True
            self.record_name = name
            self.record_buffer = []

        # Wymuszenie startu od POBRANIA
        with self.robot.motion_lock:
            cur = self.robot.target.copy()
            wait = self.robot.calculate_wait_time(cur, pickup)
            self.robot.go_to_position(pickup)
            time.sleep(wait)

        # pierwsza próbka
        with self.record_lock:
            self.record_buffer.append(self._snapshot_state())

        messagebox.showinfo(
            "Nagrywanie",
            f"START: {name}\nNagrywanie zaczęte od POBRANIA.\nPoruszaj padem i naciśnij STOP."
        )

    def stop_record(self):
        with self.record_lock:
            if not self.is_recording:
                return
            name = self.record_name
            data = list(self.record_buffer)
            self.is_recording = False
            self.record_name = None
            self.record_buffer = []

        
        simplified = []
        last = None
        for s in data:
            if last is None or s != last:
                simplified.append(s)
            last = s

        self.traj[name] = simplified
        self.lbl_traj_info.config(text=self._traj_info_text())
        messagebox.showinfo("OK", f"Zapisano trajektorię {name}\nKroki: {len(simplified)}")

    def _snapshot_state(self):
        return {
            "1": int(self.robot.target["1"]),
            "2": int(self.robot.target["2"]),
            "3": int(self.robot.target["3"]),
            "V": int(self.robot.servo_pos),
        }

    # ---------------- Kamera ----------------
    def camera_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            obj, conf = self.vision.predict(frame)
            self.detected_obj = obj
            self.confidence = conf
            pewnosc = int(conf * 100)

            if obj != "BRAK" and conf >= 0.70:
                overlay = f"WYKRYCIE: {obj}   PEWNOSC: {pewnosc}%"
                cv2.putText(frame, overlay, (10, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "WYKRYCIE: BRAK", (10, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 180, 255), 2)

            status = "GOTOWY"
            if self.current_mode == "AUTO":
                status = "AUTO: OCZEKIWANIE" if (not self._auto_busy) else "AUTO: ODTWARZANIE"
            if self.current_mode == "TEACH" and self.is_recording:
                status = f"UCZENIE: NAGRYWANIE ({self.record_name})"

            panel_txt = f"WYKRYCIE: {obj}\nPEWNOSC: {pewnosc}%\nSTATUS: {status}"
            self.root.after(0, lambda t=panel_txt: self.lbl_detection.config(text=t))

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            imgtk = ImageTk.PhotoImage(image=Image.fromarray(img))
            self.root.after(0, lambda i=imgtk: self.lbl_video.configure(image=i))
            self.root.after(0, lambda i=imgtk: setattr(self.lbl_video, 'imgtk', i))

            time.sleep(0.03)

    # ---------------- Pad (Swobodny/Uczenie) ----------------
    def gamepad_loop(self):
        while self.running:
            if (self.robot.connected and self.joystick
                    and self.current_mode in ["FREE", "TEACH"]
                    and not self._auto_busy):

                pygame.event.pump()
                try:
                    ly = float(self.joystick.get_axis(AXIS_LEFT_Y)) - self.axis_zero["ly"]
                    if INVERT_LEFT_Y:
                        ly = -ly
                    if abs(ly) < DEADZONE:
                        ly = 0.0

                    ry = float(self.joystick.get_axis(AXIS_RIGHT_Y)) - self.axis_zero["ry"]
                    if INVERT_RIGHT_Y:
                        ry = -ry
                    if abs(ry) < DEADZONE:
                        ry = 0.0

                    d2 = int(ly * STEP_PACKET_M2)
                    d3 = int(ry * STEP_PACKET_M3)

                    d1 = 0
                    if self.joystick.get_button(BTN_LB):
                        d1 -= STEP_PACKET_M1
                    if self.joystick.get_button(BTN_RB):
                        d1 += STEP_PACKET_M1

                    # Serwo
                    if self.joystick.get_button(BTN_X):
                        self.robot.move_servo(SERVO_OPEN)
                    if self.joystick.get_button(BTN_Y):
                        self.robot.move_servo(SERVO_CLOSE)

                    # Ruch osi
                    if d1 or d2 or d3:
                        new_target = self.robot.target.copy()
                        new_target["1"] += d1
                        new_target["2"] += d2
                        new_target["3"] += d3
                        self.robot.go_to_position(new_target)

                    info = f"M1:{self.robot.target['1']}  M2:{self.robot.target['2']}  M3:{self.robot.target['3']}"
                    self.root.after(0, lambda t=info: self.lbl_coords.config(text=t))

                    # Nagrywanie
                    if self.current_mode == "TEACH":
                        with self.record_lock:
                            if self.is_recording:
                                self.record_buffer.append(self._snapshot_state())

                except Exception:
                    pass

            time.sleep(LOOP_DELAY)

    # ---------------- Odtwarzanie ----------------
    def play_trajectory(self, name):
        traj = self.traj.get(name, [])
        if not traj:
            print(f"[AUTO] Brak trajektorii dla {name}.")
            return False

        pickup = self.robot.saved_positions.get("POBRANIE")
        if not pickup:
            print("[AUTO] Brak punktu POBRANIE.")
            return False

        # Zawsze start od POBRANIA
        cur = self.robot.target.copy()
        wait = self.robot.calculate_wait_time(cur, pickup)
        self.robot.go_to_position(pickup)
        time.sleep(wait)

        # Odtwarzanie próbek
        for s in traj:
            if int(s["V"]) != int(self.robot.servo_pos):
                self.robot.move_servo(int(s["V"]))

            next_pos = {"1": int(s["1"]), "2": int(s["2"]), "3": int(s["3"])}
            self.robot.go_to_position(next_pos)
            time.sleep(PLAYBACK_DT)

        return True

    # ---------------- AUTO ----------------
    def auto_logic_loop(self):
        while self.running:
            if self.current_mode != "AUTO":
                self._stable_label = "BRAK"
                self._stable_count = 0
                time.sleep(0.1)
                continue

            if self._auto_busy:
                time.sleep(0.1)
                continue

            if self.detected_obj == "BRAK" or self.confidence < AUTO_CONF_TH:
                self._stable_label = "BRAK"
                self._stable_count = 0
                time.sleep(0.1)
                continue

            if self.detected_obj == self._stable_label:
                self._stable_count += 1
            else:
                self._stable_label = self.detected_obj
                self._stable_count = 1

            if self._stable_count < AUTO_STABLE_FRAMES:
                time.sleep(0.05)
                continue

            if self._stable_label not in ["PUSZKA", "BUTELKA"]:
                self._stable_label = "BRAK"
                self._stable_count = 0
                time.sleep(0.1)
                continue

            if len(self.traj.get(self._stable_label, [])) == 0:
                print("[AUTO] Brak nagranej trajektorii! Nagraj w UCZENIU.")
                self._stable_label = "BRAK"
                self._stable_count = 0
                time.sleep(0.5)
                continue

            self._auto_busy = True
            try:
                with self.robot.motion_lock:
                    print(f"[AUTO] Odtwarzanie: {self._stable_label}")
                    self.play_trajectory(self._stable_label)
                    print("[AUTO] Koniec cyklu.")
            finally:
                self._stable_label = "BRAK"
                self._stable_count = 0
                self.detected_obj = "BRAK"
                self.confidence = 0.0
                self._auto_busy = False
                time.sleep(AUTO_COOLDOWN_SEC)

    def on_close(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if hasattr(self.robot, "ser") and self.robot.ser and self.robot.ser.is_open:
            self.robot.ser.close()
        pygame.quit()
        self.root.destroy()


if __name__ == "__main__":
    robot = RobotController()
    root = tk.Tk()
    app = App(root, robot)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

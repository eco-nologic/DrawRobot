import math
import asyncio
import sys
import subprocess
import json
import threading
import queue
import csv
import os
import time

# --- Détection et installation automatique des dépendances ---
try:
    import bleak
except ImportError:
    print("[Python] Module 'bleak' manquant. Installation...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak"])
    import bleak

from bleak import BleakClient, BleakScanner

import tkinter as tk
from tkinter import ttk, messagebox

class RobotRemoteGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("DrawRobot Remote Control (Python)")
        self.root.geometry("1100x750")
        
        # Configuration BLE (doit correspondre à BluetoothManager.h/cpp)
        self.device_name = "GiRobot_BLE"
        self.service_uuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
        self.char_uuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

        self.client = None
        self.is_connected = False
        self.cmd_queue = queue.Queue()
        self.telemetry = {}
        self.path_history = []

        # Logging state
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None

        self.setup_ui()
        # Démarrage de la boucle BLE dans un thread séparé
        self.start_ble_thread()

    def setup_ui(self):
        # Style
        style = ttk.Style()
        style.configure("TButton", padding=5)
        style.configure("Warning.TButton", foreground="red")

        # --- Layout Containers ---
        left_panel = ttk.Frame(self.root)
        left_panel.pack(side="left", fill="both", expand=True, padx=5)

        right_panel = ttk.Frame(self.root)
        right_panel.pack(side="right", fill="both", expand=True, padx=5)

        # --- Panel 1: Connexion Status (RIGHT) ---
        status_frame = ttk.LabelFrame(right_panel, text="System Status", padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)
        self.lbl_status = ttk.Label(status_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack()

        # --- Panel 2: Manual Control (D-Pad) (LEFT) ---
        manual_frame = ttk.LabelFrame(left_panel, text="Manual Control (Hold to move)", padding=10)
        manual_frame.pack(fill="x", padx=10, pady=5)

        btn_fwd = ttk.Button(manual_frame, text="FORWARD")
        btn_fwd.grid(row=0, column=1, pady=2)
        btn_fwd.bind('<ButtonPress-1>', lambda e: self.send_cmd("FORWARD"))
        btn_fwd.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_lft = ttk.Button(manual_frame, text="LEFT")
        btn_lft.grid(row=1, column=0, pady=2)
        btn_lft.bind('<ButtonPress-1>', lambda e: self.send_cmd("TURN_LEFT"))
        btn_lft.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_stop = ttk.Button(manual_frame, text="STOP", style="Warning.TButton")
        btn_stop.grid(row=1, column=1, pady=2)
        btn_stop.configure(command=lambda: self.send_cmd("STOP"))

        btn_rgt = ttk.Button(manual_frame, text="RIGHT")
        btn_rgt.grid(row=1, column=2, pady=2)
        btn_rgt.bind('<ButtonPress-1>', lambda e: self.send_cmd("TURN_RIGHT"))
        btn_rgt.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_bwd = ttk.Button(manual_frame, text="BACKWARD")
        btn_bwd.grid(row=2, column=1, pady=2)
        btn_bwd.bind('<ButtonPress-1>', lambda e: self.send_cmd("BACKWARD"))
        btn_bwd.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        # --- Panel 3: Sequences (LEFT) ---
        seq_frame = ttk.LabelFrame(left_panel, text="Drawing Sequences", padding=10)
        seq_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(seq_frame, text="Sequence 1: Stairs", command=lambda: self.send_cmd("stairs")).pack(fill="x", pady=2)
        
        # Squares with inputs
        sq_subframe = ttk.Frame(seq_frame)
        sq_subframe.pack(fill="x", pady=2)
        self.ent_sq_size = ttk.Entry(sq_subframe, width=8)
        self.ent_sq_size.insert(0, "50")
        self.ent_sq_size.pack(side="left", padx=2)
        ttk.Button(sq_subframe, text="Draw Squares", command=self.gui_draw_squares).pack(side="left", padx=2)

        # Circle with input
        ci_subframe = ttk.Frame(seq_frame)
        ci_subframe.pack(fill="x", pady=2)
        self.ent_radius = ttk.Entry(ci_subframe, width=8)
        self.ent_radius.insert(0, "50")
        self.ent_radius.pack(side="left", padx=2)
        ttk.Button(ci_subframe, text="Draw Circle", command=self.gui_draw_circle).pack(side="left", padx=2)

        ttk.Button(seq_frame, text="Sequence 3: North", command=lambda: self.send_cmd("north")).pack(fill="x", pady=2)
        
        # --- Panel 4: Special Functions (LEFT) ---
        special_frame = ttk.LabelFrame(left_panel, text="Special Operations", padding=10)
        special_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(special_frame, text="Calibrate Magnetometer", command=lambda: self.send_cmd("calibrateMag")).pack(fill="x", pady=2)
        ttk.Button(special_frame, text="🛹 Segway Mode (Secret)", command=self.gui_balance).pack(fill="x", pady=2)

        # --- Panel 5: PID Config (LEFT) ---
        pid_frame = ttk.LabelFrame(left_panel, text="PID Tuning", padding=10)
        pid_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(pid_frame, text="Lin P:").grid(row=0, column=0)
        self.ent_lkp = ttk.Entry(pid_frame, width=8)
        self.ent_lkp.insert(0, "0.5")
        self.ent_lkp.grid(row=0, column=1)

        ttk.Label(pid_frame, text="Ang P:").grid(row=0, column=2)
        self.ent_akp = ttk.Entry(pid_frame, width=8)
        self.ent_akp.insert(0, "0.8")
        self.ent_akp.grid(row=0, column=3)

        ttk.Button(pid_frame, text="Apply PID", command=self.gui_send_config).grid(row=1, column=0, columnspan=4, pady=5)

        # --- Panel 6: Live Telemetry (RIGHT) ---
        telem_frame = ttk.LabelFrame(right_panel, text="Live Telemetry", padding=10)
        telem_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_heading = ttk.Label(telem_frame, text="Heading: --°")
        self.lbl_heading.pack(anchor="w")
        self.lbl_battery = ttk.Label(telem_frame, text="Battery: -- V")
        self.lbl_battery.pack(anchor="w")
        self.lbl_targets = ttk.Label(telem_frame, text="Targets: L=--, θ=--")
        self.lbl_targets.pack(anchor="w")

        # Compass Drawing Area
        self.compass_canvas = tk.Canvas(telem_frame, width=100, height=100, bg="#f0f0f0", highlightthickness=1)
        self.compass_canvas.pack(pady=5)

        # --- Panel 7: XY Map (RIGHT) ---
        map_frame = ttk.LabelFrame(right_panel, text="Pen Path (XY Map)", padding=10)
        map_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.map_canvas = tk.Canvas(map_frame, width=400, height=300, bg="white", highlightthickness=1)
        self.map_canvas.pack(fill="both", expand=True)
        ttk.Button(map_frame, text="Clear Map", command=self.clear_map).pack(pady=2)

        # --- Panel 8: Data Logging (RIGHT) ---
        log_frame = ttk.LabelFrame(right_panel, text="Data Logging", padding=10)
        log_frame.pack(fill="x", padx=10, pady=5)

        self.btn_record = ttk.Button(log_frame, text="🔴 Start Recording Telemetry", command=self.toggle_recording)
        self.btn_record.pack(fill="x")
        self.lbl_file = ttk.Label(log_frame, text="Not recording", font=("Arial", 8))
        self.lbl_file.pack(pady=2)

        # Start UI update loop (polling internal state)
        self.update_ui_loop()

    def toggle_recording(self):
        """Start or stop saving telemetry to CSV."""
        if not self.is_recording:
            # Start
            if not os.path.exists("Output"):
                os.makedirs("Output")
            
            filename = f"robot_data_{int(time.time())}.csv"
            filepath = os.path.join("Output", filename)
            
            try:
                self.csv_file = open(filepath, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                # Headers matching log_telemetry.py
                headers = ["timestamp_ms", "x_mm", "y_mm", "heading_deg", "battery_v",
                           "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
                           "mag_x", "mag_y", "mag_z", "target_l", "target_theta", "target_r"]
                self.csv_writer.writerow(headers)
                self.is_recording = True
                self.btn_record.config(text="⏹️ Stop Recording")
                self.lbl_file.config(text=f"Recording to: {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Could not create file: {e}")
        else:
            # Stop
            self.is_recording = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            self.btn_record.config(text="🔴 Start Recording Telemetry")
            self.lbl_file.config(text="File saved.")

    def clear_map(self):
        """Clear the path history and reset the visual point to center (0,0)."""
        # On réinitialise avec l'origine pour que draw_map ait un point à dessiner au centre
        self.path_history = [(0.0, 0.0)]
        self.telemetry['x'] = 0.0
        self.telemetry['y'] = 0.0
        self.map_canvas.delete("all")
        self.draw_map()

    def update_ui_loop(self):
        """Refresh the GUI labels from the telemetry data received via BLE."""
        if self.telemetry:
            h = self.telemetry.get('h', 0.0)
            b = self.telemetry.get('b', 0.0)
            x = self.telemetry.get('x', 0.0)
            y = self.telemetry.get('y', 0.0)
            lth = self.telemetry.get('lth', 0.0)
            ath = self.telemetry.get('ath', 0.0)
            self.lbl_heading.config(text=f"Heading: {h:.1f}°")
            self.lbl_battery.config(text=f"Battery: {b:.2f} V")
            self.lbl_targets.config(text=f"Targets: L={lth:.0f}mm, θ={ath:.1f}°")
            self.draw_compass(h)
            self.draw_map()
        self.root.after(200, self.update_ui_loop)

    def draw_compass(self, heading):
        """Draw a simple compass needle representing the robot's heading."""
        self.compass_canvas.delete("all")
        cx, cy, r = 50, 50, 40
        
        # Draw Compass Face
        self.compass_canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="black", width=2)
        
        # Directional Marks
        self.compass_canvas.create_text(cx, cy-r+8, text="N", font=("Arial", 8, "bold"), fill="red")
        self.compass_canvas.create_text(cx, cy+r-8, text="S", font=("Arial", 7))
        self.compass_canvas.create_text(cx+r-8, cy, text="E", font=("Arial", 7))
        self.compass_canvas.create_text(cx-r+8, cy, text="W", font=("Arial", 7))
        
        # Calculate needle tip using CW robotics convention (0 is North)
        # MATH: y is subtracted because canvas coordinates start from top-left
        rad = math.radians(heading)
        nx = cx + (r-5) * math.sin(rad)
        ny = cy - (r-5) * math.cos(rad)
        
        self.compass_canvas.create_line(cx, cy, nx, ny, fill="red", arrow=tk.LAST, width=2)
        self.compass_canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="black") # Pivot point

    def draw_map(self):
        """Draw the robot's pen path history on the XY map canvas with autofocus."""
        if not self.path_history:
            return
            
        self.map_canvas.delete("all")
        w = self.map_canvas.winfo_width()
        h = self.map_canvas.winfo_height()

        # Handle unitialized canvas size
        if w < 10 or h < 10:
            w, h = 400, 300

        cx, cy = w // 2, h // 2

        # Calculate Bounding Box for Autofocus
        all_x = [p[0] for p in self.path_history]
        all_y = [p[1] for p in self.path_history]
        min_x, max_x = min(min(all_x), 0), max(max(all_x), 0)
        min_y, max_y = min(min(all_y), 0), max(max(all_y), 0)

        # Determine Scale (with 15% margin)
        range_x = max(max_x - min_x, 50.0) 
        range_y = max(max_y - min_y, 50.0)
        scale = min((w * 0.85) / range_x, (h * 0.85) / range_y)

        # Offset to center the bounding box in the canvas
        offset_x = cx - ((min_x + max_x) / 2.0) * scale
        offset_y = cy - ((min_y + max_y) / 2.0) * scale

        # Draw coordinate axes (Light Gray)
        ax_x, ax_y = offset_x + 0 * scale, offset_y + 0 * scale
        self.map_canvas.create_line(0, ax_y, w, ax_y, fill="#eeeeee")
        self.map_canvas.create_line(ax_x, 0, ax_x, h, fill="#eeeeee")

        # MATH: Determine dynamic tick step to avoid label overlap on large scales
        max_range = max(range_x, range_y)
        if max_range <= 120: tick_step = 20
        elif max_range <= 300: tick_step = 50
        elif max_range <= 750: tick_step = 100
        elif max_range <= 1500: tick_step = 200
        elif max_range <= 4000: tick_step = 500
        else: tick_step = 1000

        # Horizontal ticks (X axis values)
        start_tick_x = int(min_x // tick_step) * tick_step
        for val in range(start_tick_x, int(max_x) + tick_step, tick_step):
            tx = offset_x + val * scale
            if 0 <= tx <= w:
                self.map_canvas.create_line(tx, ax_y - 3, tx, ax_y + 3, fill="#cccccc")
                if val != 0: # Avoid cluttering the origin
                    self.map_canvas.create_text(tx, ax_y + 12, text=str(val), fill="#bbbbbb", font=("Arial", 7))

        # Vertical ticks (Y axis values)
        start_tick_y = int(min_y // tick_step) * tick_step
        for val in range(start_tick_y, int(max_y) + tick_step, tick_step):
            ty = offset_y + val * scale
            if 0 <= ty <= h:
                self.map_canvas.create_line(ax_x - 3, ty, ax_x + 3, ty, fill="#cccccc")
                if val != 0:
                    self.map_canvas.create_text(ax_x - 15, ty, text=str(val), fill="#bbbbbb", font=("Arial", 7))

        # Draw path (simplified to a single polyline)
        points = []
        for p in self.path_history:
            px = offset_x + p[0] * scale
            py = offset_y + p[1] * scale
            points.extend([px, py])
            
        if len(points) >= 4:
            self.map_canvas.create_line(points, fill="#3498db", width=2)
            
        # Draw current pen position (red dot)
        curr_x = offset_x + self.path_history[-1][0] * scale
        curr_y = offset_y + self.path_history[-1][1] * scale
        self.map_canvas.create_oval(curr_x-4, curr_y-4, curr_x+4, curr_y+4, fill="#e74c3c", outline="white")

    def start_ble_thread(self):
        """Lance l'event loop asyncio pour BLE dans un thread dédié."""
        def run_async_loop():
            asyncio.run(self.ble_manager_task())

        self.ble_thread = threading.Thread(target=run_async_loop, daemon=True)
        self.ble_thread.start()

    async def ble_manager_task(self):
        """Gère la découverte, la connexion et l'envoi de commandes BLE."""
        while True:
            if not self.is_connected:
                self.lbl_status.config(text=f"Searching for {self.device_name}...", foreground="orange")
                device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)
                
                if device:
                    self.lbl_status.config(text="Connecting...", foreground="blue")
                    try:
                        async with BleakClient(device) as client:
                            self.client = client
                            self.is_connected = True
                            self.lbl_status.config(text=f"Connected to {self.device_name}", foreground="green")
                            
                            # Subscription for Telemetry Notifications
                            def notification_handler(sender, data):
                                try:
                                    decoded = data.decode('utf-8')
                                    # Format: A:...|G:...|M:...|H:...|B:...|T:...|X:...|Y:...|Lth:...|Ath:...|Rth:...
                                    parts = decoded.split('|')
                                    if len(parts) >= 11:
                                        # Update UI dict
                                        self.telemetry['h'] = float(parts[3].split(':')[1])
                                        self.telemetry['b'] = float(parts[4].split(':')[1])
                                        self.telemetry['x'] = float(parts[6].split(':')[1])
                                        self.telemetry['y'] = float(parts[7].split(':')[1])
                                        self.telemetry['lth'] = float(parts[8].split(':')[1])
                                        self.telemetry['ath'] = float(parts[9].split(':')[1])
                                        
                                        # Append to path history for map drawing
                                        self.path_history.append((self.telemetry['x'], self.telemetry['y']))
                                        if len(self.path_history) > 1500: # Limit history to avoid slow UI
                                            self.path_history.pop(0)

                                        # Save to CSV if recording
                                        if self.is_recording and self.csv_writer:
                                            accel = [float(x) for x in parts[0].split(':')[1].split(',')]
                                            gyro = [float(x) for x in parts[1].split(':')[1].split(',')]
                                            mag = [float(x) for x in parts[2].split(':')[1].split(',')]
                                            row = [
                                                int(parts[5].split(':')[1]), # T
                                                self.telemetry['x'], self.telemetry['y'],
                                                self.telemetry['h'],
                                                self.telemetry['b']
                                            ] + accel + gyro + mag + [
                                                self.telemetry['lth'],
                                                self.telemetry['ath'],
                                                float(parts[10].split(':')[1]) # Rth
                                            ]
                                            self.csv_writer.writerow(row)
                                            # Flush periodically
                                            if int(time.time()) % 2 == 0:
                                                self.csv_file.flush()
                                except Exception:
                                    # Silent fail to avoid flooding console during motion jitter
                                    pass

                            await client.start_notify(self.char_uuid, notification_handler)

                            # Boucle de traitement des commandes tant que connecté
                            while client.is_connected:
                                try:
                                    # On récupère la commande sans bloquer l'event loop
                                    cmd_json = self.cmd_queue.get_nowait()
                                    await client.write_gatt_char(self.char_uuid, cmd_json.encode('utf-8'))
                                    print(f"[BLE] Sent: {cmd_json}")
                                except queue.Empty:
                                    await asyncio.sleep(0.1)
                                    
                            self.is_connected = False
                    except Exception as e:
                        print(f"[BLE] Connection error: {e}")
                        self.is_connected = False
                else:
                    self.lbl_status.config(text="Robot not found", foreground="red")
            
            await asyncio.sleep(2.0)

    def send_cmd(self, cmd, params=None):
        if not self.is_connected:
            print(f"[Remote] Cannot send '{cmd}': BLE not connected")
            return

        payload = {"cmd": cmd}
        if params:
            payload.update(params)
        
        # Ajout à la queue pour le thread BLE
        self.cmd_queue.put(json.dumps(payload))

    # --- Logic Helpers ---
    def gui_draw_squares(self):
        size = float(self.ent_sq_size.get())
        self.send_cmd("squares", {"size": size, "count": 3})

    def gui_draw_circle(self):
        radius = float(self.ent_radius.get())
        self.send_cmd("circle", {"radius": radius})

    def gui_balance(self):
        if messagebox.askokcancel("Segway Mode", "Ensure robot is standing vertical before starting!"):
            self.send_cmd("balance")

    def gui_send_config(self):
        lkp = float(self.ent_lkp.get())
        akp = float(self.ent_akp.get())
        self.send_cmd("config", {
            "lkp": lkp, "lki": 0.1, "lkd": 0.01,
            "akp": akp, "aki": 0.2, "akd": 0.1
        })

if __name__ == "__main__":
    root = tk.Tk()
    # MATH/DEFENSE: "Comment ce script remplace-t-il la page web ?"
    # ANSWER: Il utilise le même protocole WebSocket et le même format d'objet JSON 
    # que le JavaScript du dashboard, garantissant une compatibilité totale avec CommandHandler.cpp.
    gui = RobotRemoteGUI(root)
    root.mainloop()
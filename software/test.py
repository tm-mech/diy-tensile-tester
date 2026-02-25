"""
Tensile Test Control

Python host application for the tensile testing machine.
Communicates with the Arduino firmware over serial, handles real-time
data logging, and provides an interactive command interface.

Commands:
  start       - Start tensile test
  stop        - Emergency stop
  speed X     - Set speed [mm/min] (e.g. "speed 5")
  dir X       - Set direction (1 = pull, -1 = return)
  up / down   - Manual jog for specimen positioning
  tare        - Zero force measurement
  reset       - Reset machine to IDLE state
  status      - Show current status
  force       - Show current force reading
  save        - Save data as CSV
  clear       - Clear recorded data
  plot        - Show force-displacement plot
  help        - Show commands
  quit        - Exit (auto-saves data)
"""

import serial
import threading
import queue
import time
import sys
import csv
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: matplotlib not available, plotting disabled")

# =============================================================================
# SETTINGS
# =============================================================================

COM_PORT  = "COM7"
BAUD_RATE = 115200

# Calibration (must match firmware values)
FORCE_CALIBRATION_FACTOR = 2.217E-04  # raw_value × factor = Newton
STEPS_PER_MM = (200 * 4) / 4.0       # 200 steps × 4 microsteps / 4 mm lead


# =============================================================================
# DATA STORE
# =============================================================================

class DataStore:
    """Thread-safe storage for all measurement data."""

    def __init__(self):
        self.lock = threading.Lock()
        self.timestamps = []
        self.steps_list = []
        self.displacement_mm = []
        self.force_raw = []
        self.force_N = []
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []
        self.endstop_states = []
        self.step_loss = []
        self.event_queue = queue.Queue()
        self.running = True

    def add_point(self, t, steps, raw_f, ax, ay, az, endstop, step_loss=0):
        """Add a data point from serial stream."""
        with self.lock:
            self.timestamps.append(t)
            self.steps_list.append(steps)
            self.displacement_mm.append(steps / STEPS_PER_MM)
            self.force_raw.append(raw_f)
            self.force_N.append(raw_f * FORCE_CALIBRATION_FACTOR)
            self.accel_x.append(ax)
            self.accel_y.append(ay)
            self.accel_z.append(az)
            self.endstop_states.append(endstop)
            self.step_loss.append(step_loss)

    def clear(self):
        """Clear all recorded data."""
        with self.lock:
            self.timestamps.clear()
            self.steps_list.clear()
            self.displacement_mm.clear()
            self.force_raw.clear()
            self.force_N.clear()
            self.accel_x.clear()
            self.accel_y.clear()
            self.accel_z.clear()
            self.endstop_states.clear()
            self.step_loss.clear()
        print("Data cleared.")

    def get_plot_data(self):
        """Return a snapshot of data for plotting."""
        with self.lock:
            return (
                list(self.displacement_mm),
                list(self.force_N),
                list(self.timestamps),
                list(self.accel_z)
            )

    def count(self):
        """Return number of recorded data points."""
        with self.lock:
            return len(self.timestamps)

    def save_csv(self):
        """Save all data to a timestamped CSV file."""
        with self.lock:
            if len(self.timestamps) == 0:
                print("No data to save.")
                return None

            filename = f"tensile_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f, delimiter=';')
                writer.writerow([
                    'time_s', 'steps', 'displacement_mm', 'force_raw', 'force_N',
                    'accel_x', 'accel_y', 'accel_z', 'endstop', 'step_loss'
                ])

                for i in range(len(self.timestamps)):
                    writer.writerow([
                        f"{self.timestamps[i]:.3f}",
                        self.steps_list[i],
                        f"{self.displacement_mm[i]:.4f}",
                        self.force_raw[i],
                        f"{self.force_N[i]:.3f}",
                        self.accel_x[i],
                        self.accel_y[i],
                        self.accel_z[i],
                        self.endstop_states[i],
                        self.step_loss[i]
                    ])

            print(f"Saved: {filename} ({len(self.timestamps)} data points)")
            return filename


# =============================================================================
# SERIAL HANDLER
# =============================================================================

class SerialHandler(threading.Thread):
    """Background thread for serial communication with Arduino."""

    def __init__(self, port, baudrate, data_store):
        super().__init__(daemon=True)
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.data = data_store
        self.step_loss_reported = False

    def connect(self):
        """Establish serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.ser.reset_input_buffer()
            self.connected = True
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error: {e}")
            return False

    def send(self, cmd):
        """Send command string to Arduino."""
        if self.connected and self.ser:
            self.ser.write(f"{cmd}\n".encode())
            self.ser.flush()

    def run(self):
        """Main thread loop — continuously reads serial data."""
        while self.data.running:
            if not self.connected:
                time.sleep(0.1)
                continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.process_line(line)
            except Exception as e:
                print(f"Serial error: {e}")

    def process_line(self, line):
        """Parse incoming serial data or events."""
        parts = line.split(';')

        if parts[0] == "DATA" and len(parts) >= 9:
            try:
                t         = int(parts[1]) / 1000.0
                steps     = int(parts[2])
                raw_f     = int(parts[3])
                ax        = int(parts[4])
                ay        = int(parts[5])
                az        = int(parts[6])
                endstop   = int(parts[7])
                step_loss = int(parts[8])

                self.data.add_point(t, steps, raw_f, ax, ay, az, endstop, step_loss)

                if step_loss and not self.step_loss_reported:
                    self.step_loss_reported = True
                    self.data.event_queue.put(("WARNING", "Step loss detected!"))

            except (ValueError, IndexError):
                pass

        elif parts[0] == "EVENT":
            event = parts[1] if len(parts) > 1 else "UNKNOWN"
            self.data.event_queue.put(("EVENT", event))

        elif parts[0] == "STATUS":
            if len(parts) >= 4:
                states = ["IDLE", "RUNNING", "STOPPED", "ERROR", "JOG"]
                state_idx = int(parts[1])
                state_name = states[state_idx] if state_idx < len(states) else "?"
                self.data.event_queue.put((
                    "STATUS",
                    f"{state_name}, Speed: {parts[2]} mm/min, Dir: {parts[3]}"
                ))

    def close(self):
        """Close serial connection."""
        if self.ser:
            self.ser.close()


# =============================================================================
# PLOTTER
# =============================================================================

class Plotter:
    """On-demand plotting of test data."""

    def __init__(self, data_store):
        self.data = data_store

    def refresh(self):
        """Generate force-displacement and vibration plots."""
        if not PLOTTING_AVAILABLE:
            print("matplotlib not available")
            return

        disp, force, times, accel = self.data.get_plot_data()
        n = len(disp)

        if n == 0:
            print("No data yet.")
            return

        print(f"Plotting {n} points...")
        print(f"  Displacement: {min(disp):.3f} – {max(disp):.3f} mm")
        print(f"  Force: {min(force):.1f} – {max(force):.1f} N")

        plt.close('all')
        fig, axes = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle('Tensile Test')

        axes[0].plot(disp, force, 'b-', linewidth=1.5)
        axes[0].set_xlabel('Displacement [mm]')
        axes[0].set_ylabel('Force [N]')
        axes[0].set_title('Force–Displacement')
        axes[0].grid(True)

        axes[1].plot(times, accel, 'r-', linewidth=1)
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Acceleration Z')
        axes[1].set_title('Vibration (Z-axis)')
        axes[1].grid(True)

        plt.tight_layout()
        plt.show()


# =============================================================================
# COMMAND LOOP
# =============================================================================

def print_help():
    print("""
Commands:
  start       - Start tensile test
  stop        - Emergency stop
  speed X     - Set speed [mm/min] (e.g. "speed 5")
  dir X       - Set direction (1 = pull, -1 = return)
  up / down   - Manual jog for positioning
  tare        - Zero force measurement
  reset       - Reset to IDLE state
  status      - Show current status
  force       - Show current force reading
  save        - Save data as CSV
  clear       - Clear recorded data
  plot        - Show force-displacement plot
  help        - Show this help
  quit        - Exit (auto-saves data)
""")


def command_loop(serial_handler, data, plotter):
    """Interactive command loop."""

    print("\nTensile Test Control")
    print("=" * 40)
    print("Type 'help' for commands\n")

    while data.running:
        try:
            # Display any buffered events
            while not data.event_queue.empty():
                try:
                    msg = data.event_queue.get_nowait()
                    if isinstance(msg, tuple):
                        msg_type, content = msg
                        print(f"  [{msg_type}] {content}")
                    else:
                        print(f"  {msg}")
                except queue.Empty:
                    break

            cmd = input("> ").strip().lower()

            if cmd in ("quit", "exit"):
                print("Exiting...")
                data.save_csv()
                data.running = False
                break

            elif cmd == "start":
                data.clear()
                serial_handler.step_loss_reported = False
                serial_handler.send("START")

            elif cmd == "stop":
                serial_handler.send("STOP")

            elif cmd == "up":
                serial_handler.send("UP")

            elif cmd == "down":
                serial_handler.send("DOWN")

            elif cmd.startswith("speed "):
                try:
                    speed = float(cmd.split()[1])
                    serial_handler.send(f"SET_SPEED:{speed}")
                except (IndexError, ValueError):
                    print("Usage: speed X (e.g. speed 5)")

            elif cmd.startswith("dir "):
                try:
                    direction = int(cmd.split()[1])
                    if direction in [1, -1]:
                        serial_handler.send(f"SET_DIR:{direction}")
                    else:
                        print("Direction must be 1 or -1")
                except (IndexError, ValueError):
                    print("Usage: dir 1 (pull) or dir -1 (return)")

            elif cmd == "tare":
                serial_handler.send("TARE")

            elif cmd == "force":
                serial_handler.send("FORCE")

            elif cmd == "reset":
                serial_handler.send("RESET")

            elif cmd == "status":
                serial_handler.send("STATUS")
                print(f"Data points: {data.count()}")

            elif cmd == "save":
                data.save_csv()

            elif cmd == "clear":
                data.clear()

            elif cmd == "plot":
                plotter.refresh()

            elif cmd == "help":
                print_help()

            elif cmd == "":
                pass

            else:
                print(f"Unknown command: {cmd}")

        except KeyboardInterrupt:
            print("\nCtrl+C — exiting...")
            data.save_csv()
            data.running = False
            break
        except EOFError:
            data.running = False
            break


# =============================================================================
# MAIN
# =============================================================================

def main():
    data = DataStore()
    plotter = Plotter(data)
    serial_handler = SerialHandler(COM_PORT, BAUD_RATE, data)

    if not serial_handler.connect():
        print("Could not connect. Check COM port!")
        sys.exit(1)

    serial_handler.start()

    # Wait for Arduino READY signal
    print("Waiting for Arduino...")
    timeout = time.time() + 5
    while time.time() < timeout:
        try:
            msg = data.event_queue.get(timeout=0.5)
            if isinstance(msg, tuple) and "READY" in msg[1]:
                print("Arduino ready!")
                break
        except queue.Empty:
            pass

    try:
        command_loop(serial_handler, data, plotter)
    finally:
        data.running = False
        serial_handler.close()
        print("Connection closed.")


if __name__ == "__main__":
    main()

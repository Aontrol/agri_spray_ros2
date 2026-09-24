import queue
import threading
import tkinter as tk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String


class CorrectedTelemetryDetectorNode(Node):
    """ROS 2 Node subscribing to live bag topics and evaluating Task 2 Detector."""

    def __init__(self, data_queue):
        super().__init__("corrected_telemetry_detector")
        self.data_queue = data_queue

        # Drone State Variables
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_stamp_sec = 0.0
        self.prev_stamp_sec = None

        self.latest_flow = 0.0
        self.latest_rpm = 0.0
        self.latest_cc_status = "NOMINAL"
        self.latest_time_ist = "LIVE"

        # Task 2 Detector Thresholds
        self.nominal_capacity_ml = 10000.0  # 10 L initial tank capacity
        self.motor_rpm_thresh = 1200.0  # Operational pump RPM cutoff
        self.low_flow_thresh_ml_min = 50.0  # Low-flow evaluation boundary
        self.slosh_probe_window_sec = 5.0  # Transient slosh probe duration
        self.empty_volume_limit_ml = 300.0  # 0.3 L Reserve volume limit
        self.empty_confirm_window_sec = 2.0  # Depletion confirmation window

        # State Estimator Tracking Variables
        self.detector_remaining_ml = self.nominal_capacity_ml
        self.low_flow_probe_timer = 0.0

        # ROS 2 Subscriptions
        self.create_subscription(
            PoseStamped, "/drone/telemetry", self.pose_cb, 10
        )
        self.create_subscription(
            Float32, "/spray_system/flow_rate", self.flow_cb, 10
        )
        self.create_subscription(
            Float32, "/spray_system/motor_rpm", self.rpm_cb, 10
        )
        self.create_subscription(
            String, "/spray_system/cc_status", self.status_cb, 10
        )
        self.create_subscription(
            String, "/spray_system/time_ist", self.time_cb, 10
        )

        # ROS 2 Output Publishers
        self.pub_detector_status = self.create_publisher(
            String, "/spray_system/corrected_status", 10
        )
        self.pub_est_volume = self.create_publisher(
            Float32, "/spray_system/est_volume_liters", 10
        )

    def flow_cb(self, msg: Float32):
        self.latest_flow = float(msg.data)

    def rpm_cb(self, msg: Float32):
        self.latest_rpm = float(msg.data)

    def status_cb(self, msg: String):
        self.latest_cc_status = str(msg.data)
        if any(
            kw in self.latest_cc_status.upper()
            for kw in ["REFILL", "RESET", "GROUND", "FULL"]
        ):
            self.detector_remaining_ml = self.nominal_capacity_ml

    def time_cb(self, msg: String):
        self.latest_time_ist = str(msg.data)

    def pose_cb(self, msg: PoseStamped):
        self.latest_x = msg.pose.position.x
        self.latest_y = msg.pose.position.y
        self.latest_stamp_sec = msg.header.stamp.sec + (
            msg.header.stamp.nanosec * 1e-9
        )

        # Dynamic Delta-T calculation
        if self.prev_stamp_sec is not None:
            dt_sec = max(
                0.1, min(5.0, self.latest_stamp_sec - self.prev_stamp_sec)
            )
        else:
            dt_sec = 1.0
        self.prev_stamp_sec = self.latest_stamp_sec

        # 1. Mass Balance Volume Estimator
        vol_step_ml = (self.latest_flow / 60.0) * dt_sec
        self.detector_remaining_ml = max(
            0.0, self.detector_remaining_ml - vol_step_ml
        )

        # 2. Adaptive Detector Logic
        is_pump_running = self.latest_rpm > self.motor_rpm_thresh
        is_flow_low = self.latest_flow < self.low_flow_thresh_ml_min
        has_sufficient_payload = (
            self.detector_remaining_ml > self.empty_volume_limit_ml
        )

        if is_pump_running and is_flow_low:
            self.low_flow_probe_timer += dt_sec
        else:
            self.low_flow_probe_timer = 0.0

        if is_pump_running and is_flow_low:
            if has_sufficient_payload:
                if self.low_flow_probe_timer <= self.slosh_probe_window_sec:
                    corrected_status = (
                        f"PROBING_SLOSH ({int(self.low_flow_probe_timer)}s)"
                    )
                else:
                    corrected_status = "FAULT_CLOG_OR_LINE_ISSUE"
            else:
                if self.low_flow_probe_timer >= self.empty_confirm_window_sec:
                    corrected_status = "DETECTOR_TANK_EMPTY"
                else:
                    corrected_status = "PROBING_TANK_EMPTY"
        else:
            corrected_status = "DETECTOR_NOMINAL_OK"

        # Publish Corrected Output Topics
        self.pub_detector_status.publish(String(data=corrected_status))
        self.pub_est_volume.publish(
            Float32(data=self.detector_remaining_ml / 1000.0)
        )

        # Push frame to UI Thread Queue
        self.data_queue.put(
            {
                "t": self.latest_stamp_sec,
                "time_ist": self.latest_time_ist,
                "x": self.latest_x,
                "y": self.latest_y,
                "flow": self.latest_flow,
                "rpm": self.latest_rpm,
                "vol": self.detector_remaining_ml / 1000.0,
                "cc_status": self.latest_cc_status,
                "detector_status": corrected_status,
            }
        )


class DashboardApp(tk.Tk):
    """High-contrast Dashboard UI comparing CC Logged vs Corrected Detector state."""

    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.title("ROS 2 Telemetry Monitor — CC Logged vs Corrected Detector")
        self.geometry("1300x780")
        self.configure(bg="#121212")

        self.history = []

        # Top Header Bar
        top_bar = tk.Frame(self, bg="#1e1e1e", bd=1, relief="solid")
        top_bar.pack(fill="x", padx=8, pady=8)

        self.lbl_time = tk.Label(
            top_bar,
            text="TIME IST: --:--:--",
            font=("Consolas", 11, "bold"),
            fg="#00E676",
            bg="#1e1e1e",
        )
        self.lbl_time.pack(side="left", padx=12, pady=8)

        self.lbl_cc_status = tk.Label(
            top_bar,
            text="CC LOGGED: OFFLINE",
            font=("Consolas", 11, "bold"),
            fg="#A0A0A0",
            bg="#262626",
            padx=10,
            pady=4,
        )
        self.lbl_cc_status.pack(side="left", padx=10, pady=8)

        self.lbl_det_status = tk.Label(
            top_bar,
            text="CORRECTED DETECTOR: AWAITING_STREAM",
            font=("Consolas", 11, "bold"),
            fg="#29B6F6",
            bg="#262626",
            padx=10,
            pady=4,
        )
        self.lbl_det_status.pack(side="left", padx=10, pady=8)

        self.lbl_volume = tk.Label(
            top_bar,
            text="EST PAYLOAD: 10.00 L",
            font=("Consolas", 11, "bold"),
            fg="#00E676",
            bg="#262626",
            padx=10,
            pady=4,
        )
        self.lbl_volume.pack(side="right", padx=12, pady=8)

        # Plot Canvas Frame
        plot_frame = tk.Frame(self, bg="#121212")
        plot_frame.pack(fill="both", expand=True, padx=8, pady=4)

        self.fig = Figure(figsize=(10, 6), dpi=95, facecolor="#181818")

        self.ax_map = self.fig.add_subplot(221)
        self.ax_vol = self.fig.add_subplot(223)
        self.ax_flow = self.fig.add_subplot(222)
        self.ax_rpm = self.fig.add_subplot(224)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.after(100, self.update_gui_loop)

    def update_gui_loop(self):
        new_data = False
        while not self.data_queue.empty():
            self.history.append(self.data_queue.get_nowait())
            new_data = True

        if new_data and self.history:
            latest = self.history[-1]

            self.lbl_time.config(text=f"TIME IST: {latest['time_ist']}")

            cc_st = latest["cc_status"]
            self.lbl_cc_status.config(text=f"CC LOGGED: {cc_st}")
            if "EMPTY" in cc_st.upper() or "RTL" in cc_st.upper():
                self.lbl_cc_status.config(fg="#FF5252", bg="#3A0D0D")
            else:
                self.lbl_cc_status.config(fg="#A0A0A0", bg="#262626")

            det_st = latest["detector_status"]
            self.lbl_det_status.config(text=f"CORRECTED DETECTOR: {det_st}")
            if "EMPTY" in det_st or "FAULT" in det_st:
                self.lbl_det_status.config(fg="#FF5252", bg="#3A0D0D")
            elif "PROBING" in det_st:
                self.lbl_det_status.config(fg="#FFD54F", bg="#3A2E0D")
            else:
                self.lbl_det_status.config(fg="#00E676", bg="#0D3A1A")

            self.lbl_volume.config(
                text=f"EST PAYLOAD: {latest['vol']:.2f} L / 10.0L"
            )

            recent = self.history[-180:]
            t_axis = [r["t"] for r in recent]
            all_xs = [r["x"] for r in self.history]
            all_ys = [r["y"] for r in self.history]

            # Trajectory Map Plot
            self.ax_map.clear()
            self.ax_map.set_facecolor("#101010")
            self.ax_map.set_title(
                "Drone Flight Trajectory (ENU Meters)",
                color="#E0E0E0",
                fontsize=9,
                fontweight="bold",
            )
            self.ax_map.tick_params(colors="#CCCCCC", labelsize=7)
            self.ax_map.grid(True, linestyle="--", alpha=0.25, color="#777777")
            self.ax_map.plot(
                all_xs,
                all_ys,
                color="#29B6F6",
                linewidth=1.3,
                label="Flight Path",
            )
            self.ax_map.plot(
                latest["x"],
                latest["y"],
                marker="o",
                markersize=7,
                color="#00E676",
                label="Current Pos",
            )
            self.ax_map.legend(
                loc="upper right",
                fontsize=7,
                facecolor="#222222",
                labelcolor="#EEEEEE",
            )

            # Payload Volume Plot
            self.ax_vol.clear()
            self.ax_vol.set_facecolor("#101010")
            self.ax_vol.set_title(
                "Estimated Payload Volume (L)",
                color="#E0E0E0",
                fontsize=9,
                fontweight="bold",
            )
            self.ax_vol.tick_params(colors="#CCCCCC", labelsize=7)
            self.ax_vol.grid(True, linestyle="--", alpha=0.25, color="#777777")
            self.ax_vol.plot(
                t_axis,
                [r["vol"] for r in recent],
                color="#00E676",
                linewidth=1.4,
            )
            self.ax_vol.axhline(
                y=0.3,
                color="#FF5252",
                linestyle=":",
                alpha=0.8,
                label="0.3L Limit",
            )
            self.ax_vol.legend(
                loc="upper right",
                fontsize=7,
                facecolor="#222222",
                labelcolor="#EEEEEE",
            )

            # Flow Rate Plot
            self.ax_flow.clear()
            self.ax_flow.set_facecolor("#101010")
            self.ax_flow.set_title(
                "Spray Flow Rate (mL/min)",
                color="#E0E0E0",
                fontsize=9,
                fontweight="bold",
            )
            self.ax_flow.tick_params(colors="#CCCCCC", labelsize=7)
            self.ax_flow.grid(True, linestyle="--", alpha=0.25, color="#777777")
            self.ax_flow.plot(
                t_axis,
                [r["flow"] for r in recent],
                color="#7C4DFF",
                linewidth=1.4,
            )
            self.ax_flow.axhline(
                y=50.0, color="#FFD54F", linestyle="--", alpha=0.6
            )

            # Motor Speed Plot
            self.ax_rpm.clear()
            self.ax_rpm.set_facecolor("#101010")
            self.ax_rpm.set_title(
                "Spray Motor Speed (RPM)",
                color="#E0E0E0",
                fontsize=9,
                fontweight="bold",
            )
            self.ax_rpm.tick_params(colors="#CCCCCC", labelsize=7)
            self.ax_rpm.grid(True, linestyle="--", alpha=0.25, color="#777777")
            self.ax_rpm.plot(
                t_axis,
                [r["rpm"] for r in recent],
                color="#FFD54F",
                linewidth=1.4,
            )
            self.ax_rpm.axhline(
                y=1200.0, color="#FF5252", linestyle="--", alpha=0.6
            )

            self.fig.tight_layout()
            self.canvas.draw_idle()

        self.after(100, self.update_gui_loop)


def main():
    rclpy.init()
    data_queue = queue.Queue()

    node = CorrectedTelemetryDetectorNode(data_queue)
    executor_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    executor_thread.start()

    app = DashboardApp(data_queue)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

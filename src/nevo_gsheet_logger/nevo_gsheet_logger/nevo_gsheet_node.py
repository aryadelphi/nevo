#!/usr/bin/env python3
"""
NEVO Google Sheets Logger Node
================================
Extends nevo_logging_node.cpp logic ke Python dengan output ke Google Sheets.

Topics yang di-subscribe (sama dengan node C++ lama):
  /plan                              → nav_msgs/Path       → replan count
  /odometry/filtered                 → nav_msgs/Odometry   → EKF distance, real velocity
  /cmd_vel                           → geometry_msgs/Twist → cmd velocity
  /navigate_to_pose/_action/status   → action_msgs/GoalStatusArray → start/end goal

Kolom yang dikirim ke Google Sheets (per leg/goal):
  No Run, Tanggal, Kode Uji, Planner, Load (kg), Obstacle Count,
  Battery Start (%), Koneksi, Goal Sequence,
  EKF Distance (m), Waktu Sistem (s), Replanning Count,
  Avg Velocity Sistem (m/s), Max Velocity (m/s),
  Waktu Stopwatch (s) → dikosongkan, diisi manual
  Error Waktu, Kecepatan Aktual, Error Kecepatan → formula di Sheets
  Success, Status, Notes → diisi manual setelah run

Setup:
  1. Taruh file credentials JSON di ~/nevo_ws/src/nevo_gsheet_logger/credentials.json
  2. pip install gspread google-auth --break-system-packages
  3. ros2 run nevo_gsheet_logger nevo_gsheet_node \
       --ros-args -p uji_kode:=V1 -p planner:=DWB \
                  -p load_kg:=0.0 -p obstacle_count:=0 \
                  -p battery_start:=100 -p koneksi:=Online \
                  -p goal_sequence:="G1-G2-G3-G4-G5"
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray

import gspread
from google.oauth2.service_account import Credentials

import math
import os
import json
from datetime import datetime

# ── Sheet name mapping per kode uji ──────────────────────────
SHEET_MAP = {
    "V1": "V1_Kepadatan",
    "V2": "V2_Load",
    "V3": "V3_Planner",
    "V4": "V4_Environment",
    "V5": "V5_Velocity",
    "V6": "V6_Battery",
    "V7": "V7_Recovery",
}

# Kolom data (kolom A=1 di spreadsheet, baris data mulai row 5)
# Sesuai struktur spreadsheet yang sudah dibuat
COL_NO_RUN      = 1   # A - Manual/node
COL_TANGGAL     = 2   # B - node
COL_KODE_UJI    = 3   # C - node (dari parameter)
COL_PLANNER     = 4   # D - node (dari parameter)
COL_LOAD        = 5   # E - node (dari parameter)
COL_OBSTACLE    = 6   # F - node (dari parameter)
COL_BATTERY     = 7   # G - node (dari parameter)
COL_KONEKSI     = 8   # H - node (dari parameter)
COL_GOAL_SEQ    = 9   # I - node (dari parameter)
COL_EKF         = 10  # J - node (dari /odometry/filtered)
COL_WAKTU_SYS   = 11  # K - node (duration)
COL_REPLAN      = 12  # L - node (dari /plan)
COL_AVG_VEL     = 13  # M - node
COL_MAX_VEL     = 14  # N - node
COL_STOPWATCH   = 15  # O - MANUAL (dikosongkan oleh node)
COL_ERR_WAKTU   = 16  # P - FORMULA di Sheets
COL_V_AKTUAL    = 17  # Q - FORMULA di Sheets
COL_ERR_VEL     = 18  # R - FORMULA di Sheets
COL_SUCCESS     = 19  # S - MANUAL
COL_STATUS      = 20  # T - node isi VALID default, operator ganti INVALID jika perlu
COL_NOTES       = 21  # U - MANUAL

DATA_START_ROW  = 5   # Baris pertama data di setiap sheet


class NevoGSheetLogger(Node):

    def __init__(self):
        super().__init__("nevo_gsheet_logger")

        # ── Parameters (diisi saat ros2 run) ──────────────────
        self.declare_parameter("uji_kode",       "V1")
        self.declare_parameter("planner",        "DWB")
        self.declare_parameter("load_kg",        0.0)
        self.declare_parameter("obstacle_count", 0)
        self.declare_parameter("battery_start",  100)
        self.declare_parameter("koneksi",        "Online")
        self.declare_parameter("goal_sequence",  "G1-G2-G3-G4-G5")
        self.declare_parameter("spreadsheet_id",
            "1UXD4VQjxTXkYz-Tk_n6YmpVkaPC2Bte_jy2t2IE1UCo")
        self.declare_parameter("credentials_path",
            os.path.expanduser("~/nevo_ws/src/nevo_gsheet_logger/credentials.json"))

        self.uji_kode       = self.get_parameter("uji_kode").value
        self.planner        = self.get_parameter("planner").value
        self.load_kg        = self.get_parameter("load_kg").value
        self.obstacle_count = self.get_parameter("obstacle_count").value
        self.battery_start  = self.get_parameter("battery_start").value
        self.koneksi        = self.get_parameter("koneksi").value
        self.goal_sequence  = self.get_parameter("goal_sequence").value
        self.spreadsheet_id = self.get_parameter("spreadsheet_id").value
        self.creds_path     = self.get_parameter("credentials_path").value

        # ── Google Sheets setup ───────────────────────────────
        self.worksheet = None
        self._connect_sheets()

        # ── State variables (sama dengan node C++ lama) ───────
        self.goal_active         = False
        self.first_plan_received = False
        self.start_pose_captured = False
        self.goal_id             = 0        # leg counter dalam satu sesi
        self.run_counter         = 0        # run counter (tiap 5 leg = 1 run)

        self.start_time          = None
        self.ekf_distance        = 0.0
        self.replan_count        = 0
        self.prev_x              = 0.0
        self.prev_y              = 0.0
        self.first_odom          = True
        self.last_cmd_vel        = 0.0
        self.cmd_vel_list        = []
        self.real_vel_list       = []

        # ── ROS2 Subscribers ──────────────────────────────────
        self.plan_sub = self.create_subscription(
            Path, "/plan", self.plan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 50)

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 50)

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.status_callback, 10)

        self.get_logger().info(
            f"NEVO GSheet Logger aktif | Kode: {self.uji_kode} | "
            f"Planner: {self.planner} | Load: {self.load_kg}kg | "
            f"Obstacle: {self.obstacle_count} | Battery: {self.battery_start}%"
        )

    # ══════════════════════════════════════════════════════════
    # Google Sheets Connection
    # ══════════════════════════════════════════════════════════

    def _connect_sheets(self):
        """Konek ke Google Sheets via service account credentials."""
        try:
            scopes = [
                "https://www.googleapis.com/auth/spreadsheets",
                "https://www.googleapis.com/auth/drive",
            ]
            creds = Credentials.from_service_account_file(
                self.creds_path, scopes=scopes)
            client = gspread.authorize(creds)
            spreadsheet = client.open_by_key(self.spreadsheet_id)

            sheet_name = SHEET_MAP.get(self.uji_kode)
            if sheet_name is None:
                self.get_logger().error(
                    f"Kode uji '{self.uji_kode}' tidak dikenal. "
                    f"Pilih dari: {list(SHEET_MAP.keys())}")
                return

            self.worksheet = spreadsheet.worksheet(sheet_name)
            self.get_logger().info(
                f"Terhubung ke Google Sheets → sheet: {sheet_name}")

            # Ambil nomor run terakhir untuk auto-increment
            self._sync_run_counter()

        except FileNotFoundError:
            self.get_logger().error(
                f"Credentials tidak ditemukan di: {self.creds_path}\n"
                f"Pastikan file JSON ada di path tersebut.")
        except Exception as e:
            self.get_logger().error(f"Gagal konek ke Google Sheets: {e}")

    def _sync_run_counter(self):
        """Cek baris terakhir di sheet untuk lanjutkan nomor run."""
        try:
            col_a = self.worksheet.col_values(COL_NO_RUN)
            data_rows = [
                v for v in col_a[DATA_START_ROW - 1:]
                if v != "" and str(v).startswith("R") and "-" in str(v)
            ]
            if data_rows:
                last_val = data_rows[-1]
                # Format no_run: "R001-L5" → ambil run number
                if "-" in str(last_val):
                    self.run_counter = int(str(last_val).split("-")[0][1:])
                else:
                    self.run_counter = int(str(last_val))
            self.get_logger().info(
                f"Run counter dilanjutkan dari: {self.run_counter}")
        except Exception as e:
            self.get_logger().warn(f"Tidak bisa sync run counter: {e}")

    # ══════════════════════════════════════════════════════════
    # ROS2 Callbacks (logic sama dengan node C++ lama)
    # ══════════════════════════════════════════════════════════

    def plan_callback(self, msg: Path):
        if not self.goal_active:
            return
        self.replan_count += 1
        if not self.first_plan_received and msg.poses:
            self.first_plan_received = True

    def odom_callback(self, msg: Odometry):
        if not self.goal_active:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        real_vel = msg.twist.twist.linear.x

        if not self.start_pose_captured:
            self.prev_x = x
            self.prev_y = y
            self.start_pose_captured = True

        if not self.first_odom:
            self.ekf_distance += math.hypot(x - self.prev_x, y - self.prev_y)

        self.prev_x = x
        self.prev_y = y
        self.first_odom = False
        self.real_vel_list.append(real_vel)

    def cmd_callback(self, msg: Twist):
        if not self.goal_active:
            return
        self.last_cmd_vel = msg.linear.x
        self.cmd_vel_list.append(self.last_cmd_vel)

    def status_callback(self, msg: GoalStatusArray):
        for status in msg.status_list:
            # Status 2 = EXECUTING → goal mulai
            if status.status == 2 and not self.goal_active:
                self._start_goal()
            # Status 4 = SUCCEEDED → goal selesai
            elif status.status == 4 and self.goal_active:
                self._end_goal(success=True)
            # Status 6 = ABORTED → goal gagal
            elif status.status == 6 and self.goal_active:
                self._end_goal(success=False)

    # ══════════════════════════════════════════════════════════
    # Goal Logic
    # ══════════════════════════════════════════════════════════

    def _start_goal(self):
        self.goal_active         = True
        self.goal_id            += 1
        self.start_time          = self.get_clock().now()
        self.ekf_distance        = 0.0
        self.replan_count        = 0
        self.first_odom          = True
        self.first_plan_received = False
        self.start_pose_captured = False
        self.cmd_vel_list        = []
        self.real_vel_list       = []

        # Setiap 5 leg = 1 run baru
        leg_in_run = ((self.goal_id - 1) % 5) + 1
        if leg_in_run == 1:
            self.run_counter += 1

        self.get_logger().info(
            f"Goal {self.goal_id} dimulai "
            f"(Run {self.run_counter}, Leg {leg_in_run}/5)")

    def _end_goal(self, success: bool):
        duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        avg_vel = (sum(self.real_vel_list) / len(self.real_vel_list)
                   if self.real_vel_list else 0.0)
        max_vel = max(self.real_vel_list) if self.real_vel_list else 0.0

        leg_in_run = ((self.goal_id - 1) % 5) + 1
        no_run_label = f"R{self.run_counter:03d}-L{leg_in_run}"

        row_data = self._build_row(
            no_run_label, duration, avg_vel, max_vel, success)

        self._write_to_sheets(row_data)

        self.get_logger().info(
            f"Goal {self.goal_id} selesai | "
            f"EKF: {self.ekf_distance:.3f}m | "
            f"Durasi: {duration:.2f}s | "
            f"Replan: {self.replan_count} | "
            f"Success: {success} | "
            f"→ Sheets: {no_run_label}"
        )

        self.goal_active = False

    def _build_row(self, no_run_label, duration, avg_vel, max_vel, success):
        now_str = datetime.now().strftime("%d/%m/%Y %H:%M")
        return [
            no_run_label,                    # A - No Run
            now_str,                         # B - Tanggal
            self.uji_kode,                   # C - Kode Uji
            self.planner,                    # D - Planner
            self.load_kg,                    # E - Load (kg)
            self.obstacle_count,             # F - Obstacle Count
            self.battery_start,              # G - Battery Start (%)
            self.koneksi,                    # H - Koneksi
            self.goal_sequence,              # I - Goal Sequence
            round(self.ekf_distance, 4),     # J - EKF Distance (m)
            round(duration, 3),              # K - Waktu Sistem (s)
            self.replan_count,               # L - Replanning Count
            round(avg_vel, 4),               # M - Avg Velocity Sistem (m/s)
            round(max_vel, 4),               # N - Max Velocity (m/s)
            "",                              # O - Stopwatch → MANUAL
        ]

    # ══════════════════════════════════════════════════════════
    # Google Sheets Write
    # ══════════════════════════════════════════════════════════

    def _write_to_sheets(self, row_data: list):
        """Tulis satu baris data ke sheet yang sesuai."""
        if self.worksheet is None:
            self.get_logger().warn(
                "Worksheet tidak tersambung. Data tidak terkirim ke Sheets.")
            self._fallback_csv(row_data)
            return

        try:
            # Cari baris kosong berikutnya mulai dari DATA_START_ROW
            col_a = self.worksheet.col_values(COL_NO_RUN)
            next_row = DATA_START_ROW
            for i, val in enumerate(col_a[DATA_START_ROW - 1:], start=DATA_START_ROW):
                if val != "":
                    next_row = i + 1
                else:
                    break

            self.worksheet.update(
                f"A{next_row}:O{next_row}",
                [row_data],
                value_input_option="USER_ENTERED"
            )
            self.worksheet.update(
                f"T{next_row}",
                [["VALID"]],
                value_input_option="USER_ENTERED"
            )
            self.get_logger().info(
                f"Data terkirim ke Sheets baris {next_row}")

        except gspread.exceptions.APIError as e:
            self.get_logger().error(f"Google Sheets API error: {e}")
            self._fallback_csv(row_data)
        except Exception as e:
            self.get_logger().error(f"Gagal tulis ke Sheets: {e}")
            self._fallback_csv(row_data)

    def _fallback_csv(self, row_data: list):
        """Fallback: simpan ke CSV lokal jika Sheets tidak tersedia."""
        csv_path = os.path.expanduser(
            f"~/nevo_ws/logs/nevo_{self.uji_kode}_fallback.csv")
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)

        write_header = not os.path.exists(csv_path)
        with open(csv_path, "a") as f:
            if write_header:
                f.write("no_run,tanggal,kode_uji,planner,load_kg,obstacle_count,"
                        "battery_start,koneksi,goal_sequence,ekf_distance_m,"
                        "waktu_sistem_s,replan_count,avg_vel_ms,max_vel_ms,"
                        "stopwatch_s,err_waktu,v_aktual,err_vel,success,status,notes\n")
            f.write(",".join(str(v) for v in row_data) + "\n")

        self.get_logger().warn(
            f"Data disimpan ke fallback CSV: {csv_path}")


# ══════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NevoGSheetLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node dihentikan.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

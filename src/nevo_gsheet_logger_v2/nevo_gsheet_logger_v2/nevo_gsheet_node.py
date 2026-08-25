#!/usr/bin/env python3
"""
NEVO Google Sheets Logger Node — v2.1
=====================================
Changelog dari v2:
  - FIX KRITIS: Status callback sekarang track goal_uid spesifik
                supaya SUCCESS (4) dan ABORTED (6) tidak salah mapping
  - FIX: Loop status_list tidak lagi trigger end_goal untuk goal lama
  - FIX: Guard double-call _end_goal dengan flag self.goal_ending
  - NEW: Debug log status Nav2 masuk (level DEBUG)
  - NEW: Retry write ke Sheets maksimal 3x sebelum fallback CSV

Topics yang di-subscribe:
  /plan                            → replan count
  /odometry/filtered               → EKF distance, real velocity
  /cmd_vel                         → cmd velocity (reserved)
  /navigate_to_pose/_action/status → start/end goal, success/fail
  /nevo/sensor/battery             → battery_percent otomatis

Struktur kolom spreadsheet (v2):
  A  No Run          node
  B  Tanggal         node
  C  Kode Uji        parameter
  D  Planner         parameter
  E  Load (kg)       parameter
  F  Obstacle Count  parameter
  G  Battery Start   node (otomatis dari /nevo/sensor/battery)
  H  Battery End     node (otomatis dari /nevo/sensor/battery)
  I  Battery Drop    node (G - H)
  J  Koneksi         parameter
  K  Goal Sequence   parameter
  L  EKF Distance    node
  M  Waktu Sistem    node
  N  Replanning      node
  O  Avg Velocity    node
  P  Max Velocity    node
  Q  Waktu Stopwatch MANUAL
  R  Error Waktu     FORMULA =M-Q
  S  V Aktual        FORMULA =L/Q
  T  Error Velocity  FORMULA =O-S
  U  Success         node (otomatis Nav2: TRUE/FALSE)
  V  Status          node (otomatis Nav2: VALID/INVALID)
  W  Notes           MANUAL
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String

import gspread
from google.oauth2.service_account import Credentials

import math
import os
import time
import json as pyjson
from datetime import datetime

# ── Sheet mapping ──────────────────────────────────────────────
SHEET_MAP = {
    "V1": "V1_Kepadatan",
    "V2": "V2_Load",
    "V3": "V3_Planner",
    "V4": "V4_Environment",
    "V5": "V5_Velocity",
    "V6": "V6_Battery",
    "V7": "V7_Recovery",
}

# ── Kolom indices (1-based) ────────────────────────────────────
COL_NO_RUN    = 1   # A
COL_TANGGAL   = 2   # B
COL_KODE      = 3   # C
COL_PLANNER   = 4   # D
COL_LOAD      = 5   # E
COL_OBSTACLE  = 6   # F
COL_BAT_START = 7   # G
COL_BAT_END   = 8   # H
COL_BAT_DROP  = 9   # I
COL_KONEKSI   = 10  # J
COL_GOAL_SEQ  = 11  # K
COL_EKF       = 12  # L
COL_WSYS      = 13  # M
COL_REPLAN    = 14  # N
COL_AVG_VEL   = 15  # O
COL_MAX_VEL   = 16  # P
COL_STOPWATCH = 17  # Q — MANUAL
# R, S, T     = 18,19,20 — FORMULA, tidak dikirim node
COL_SUCCESS   = 21  # U
COL_STATUS    = 22  # V
COL_NOTES     = 23  # W — MANUAL

DATA_START_ROW = 5
MAX_WRITE_RETRY = 3


class NevoGSheetLogger(Node):

    def __init__(self):
        super().__init__("nevo_gsheet_logger_v2")

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter("uji_kode",       "V1")
        self.declare_parameter("planner",        "DWB")
        self.declare_parameter("load_kg",        0.0)
        self.declare_parameter("obstacle_count", 0)
        self.declare_parameter("koneksi",        "Online")
        self.declare_parameter("goal_sequence",  "G1-G2-G3-G4-G5")
        self.declare_parameter("spreadsheet_id",
            "1UXD4VQjxTXkYz-Tk_n6YmpVkaPC2Bte_jy2t2IE1UCo")
        self.declare_parameter("credentials_path",
            os.path.expanduser(
                "~/nevo_ws/src/nevo_gsheet_logger_v2/credentials.json"))

        self.uji_kode       = self.get_parameter("uji_kode").value
        self.planner        = self.get_parameter("planner").value
        self.load_kg        = self.get_parameter("load_kg").value
        self.obstacle_count = self.get_parameter("obstacle_count").value
        self.koneksi        = self.get_parameter("koneksi").value
        self.goal_sequence  = self.get_parameter("goal_sequence").value
        self.spreadsheet_id = self.get_parameter("spreadsheet_id").value
        self.creds_path     = self.get_parameter("credentials_path").value

        # ── Navigation state ────────────────────────────────────
        self.goal_active         = False
        self.goal_ending         = False   # ← guard double-call _end_goal
        self.first_plan_received = False
        self.start_pose_captured = False
        self.goal_id             = 0
        self.run_counter         = 0
        self.start_time          = None
        self.ekf_distance        = 0.0
        self.replan_count        = 0
        self.prev_x              = 0.0
        self.prev_y              = 0.0
        self.first_odom          = True
        self.real_vel_list       = []

        # ── FIX: Track goal UID spesifik dari Nav2 ─────────────
        self.current_goal_uid    = None

        # ── Battery state ───────────────────────────────────────
        self.battery_current   = -1.0
        self.battery_start_val = -1.0
        self.battery_end_val   = -1.0

        # ── Google Sheets ───────────────────────────────────────
        self.worksheet = None
        self._connect_sheets()

        # ── Subscribers ─────────────────────────────────────────
        self.create_subscription(Path, "/plan",
            self.plan_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
            self.odom_callback, 50)
        self.create_subscription(Twist, "/cmd_vel",
            self.cmd_callback, 50)
        self.create_subscription(GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.status_callback, 10)
        self.create_subscription(String,
            "/nevo/sensor/battery",
            self.battery_callback, 10)

        self.get_logger().info(
            f"NEVO GSheet Logger v2.1 | Kode: {self.uji_kode} | "
            f"Planner: {self.planner} | Load: {self.load_kg}kg | "
            f"Obstacle: {self.obstacle_count} | Koneksi: {self.koneksi}"
        )

    # ══════════════════════════════════════════════════════════
    # Sheets
    # ══════════════════════════════════════════════════════════

    def _connect_sheets(self):
        try:
            scopes = [
                "https://www.googleapis.com/auth/spreadsheets",
                "https://www.googleapis.com/auth/drive",
            ]
            creds = Credentials.from_service_account_file(
                self.creds_path, scopes=scopes)
            client = gspread.authorize(creds)
            sheet_name = SHEET_MAP.get(self.uji_kode)
            if not sheet_name:
                self.get_logger().error(
                    f"Kode '{self.uji_kode}' tidak dikenal. "
                    f"Pilih dari: {list(SHEET_MAP.keys())}")
                return
            sp = client.open_by_key(self.spreadsheet_id)
            self.worksheet = sp.worksheet(sheet_name)
            self.get_logger().info(
                f"Terhubung ke Sheets → {sheet_name}")
            self._sync_run_counter()
        except FileNotFoundError:
            self.get_logger().error(
                f"Credentials tidak ditemukan: {self.creds_path}")
        except Exception as e:
            self.get_logger().error(f"Gagal konek ke Sheets: {e}")

    def _sync_run_counter(self):
        try:
            col_a = self.worksheet.col_values(COL_NO_RUN)
            data_rows = [
                v for v in col_a[DATA_START_ROW - 1:]
                if v and str(v).startswith("R") and "-" in str(v)
            ]
            if data_rows:
                self.run_counter = int(
                    str(data_rows[-1]).split("-")[0][1:])
            self.get_logger().info(
                f"Run counter dilanjutkan dari: {self.run_counter}")
        except Exception as e:
            self.get_logger().warn(f"Sync run counter gagal: {e}")

    # ══════════════════════════════════════════════════════════
    # Battery
    # ══════════════════════════════════════════════════════════

    def battery_callback(self, msg: String):
        try:
            outer = pyjson.loads(msg.data)
            if outer.get("code") != "BATTERY":
                return
            inner = outer.get("data", {})
            # FIX: topic kirim data sebagai double-encoded JSON string
            # sehingga inner bisa berupa str → perlu di-parse lagi
            if isinstance(inner, str):
                inner = pyjson.loads(inner)
            pct = inner.get("battery_percent", -1.0)
            if pct >= 0:
                self.battery_current = float(pct)
                self.get_logger().debug(
                    f"Battery update: {self.battery_current:.1f}%")
        except Exception as e:
            self.get_logger().debug(f"battery_callback parse error: {e}")

    # ══════════════════════════════════════════════════════════
    # Navigation callbacks
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
        if not self.start_pose_captured:
            self.prev_x, self.prev_y = x, y
            self.start_pose_captured = True
        if not self.first_odom:
            self.ekf_distance += math.hypot(
                x - self.prev_x, y - self.prev_y)
        self.prev_x, self.prev_y = x, y
        self.first_odom = False
        self.real_vel_list.append(msg.twist.twist.linear.x)

    def cmd_callback(self, msg: Twist):
        pass  # reserved

    def status_callback(self, msg: GoalStatusArray):
        """
        FIX v2.1:
        - Track goal_uid spesifik agar tidak salah tangkap status goal lain
        - Guard goal_ending mencegah _end_goal dipanggil dua kali
        - Debug log status masuk (aktifkan dengan --log-level debug)

        Nav2 GoalStatus codes:
          1 = ACCEPTED
          2 = EXECUTING   ← trigger _start_goal
          3 = CANCELING
          4 = SUCCEEDED   ← trigger _end_goal(success=True)
          5 = CANCELED
          6 = ABORTED     ← trigger _end_goal(success=False)
        """
        for s in msg.status_list:
            uid = bytes(s.goal_info.goal_id.uuid).hex()

            # Debug: log semua status masuk
            self.get_logger().debug(
                f"Nav2 status: {s.status} | uid: {uid[:8]}... | "
                f"goal_active: {self.goal_active} | "
                f"current_uid: {str(self.current_goal_uid)[:8] if self.current_goal_uid else 'None'}..."
            )

            # ── Start goal baru ─────────────────────────────────
            if s.status == 2 and not self.goal_active:
                self.current_goal_uid = uid
                self._start_goal()

            # ── Goal selesai SUKSES ─────────────────────────────
            elif (s.status == 4
                  and self.goal_active
                  and not self.goal_ending
                  and uid == self.current_goal_uid):
                self.goal_ending = True
                self._end_goal(success=True)

            # ── Goal ABORTED / gagal ────────────────────────────
            elif (s.status == 6
                  and self.goal_active
                  and not self.goal_ending
                  and uid == self.current_goal_uid):
                self.goal_ending = True
                self._end_goal(success=False)

            # ── Goal CANCELED ───────────────────────────────────
            elif (s.status == 5
                  and self.goal_active
                  and not self.goal_ending
                  and uid == self.current_goal_uid):
                self.goal_ending = True
                self.get_logger().warn(
                    f"Goal {self.goal_id} dibatalkan (CANCELED) → tidak ditulis ke Sheets")
                self._reset_goal_state()

    # ══════════════════════════════════════════════════════════
    # Goal logic
    # ══════════════════════════════════════════════════════════

    def _start_goal(self):
        self.goal_active         = True
        self.goal_ending         = False
        self.goal_id            += 1
        self.start_time          = self.get_clock().now()
        self.ekf_distance        = 0.0
        self.replan_count        = 0
        self.first_odom          = True
        self.first_plan_received = False
        self.start_pose_captured = False
        self.real_vel_list       = []
        self.battery_start_val   = self.battery_current

        leg = ((self.goal_id - 1) % 5) + 1
        if leg == 1:
            self.run_counter += 1

        bat = (f"{self.battery_start_val:.1f}%"
               if self.battery_start_val >= 0 else "N/A")
        self.get_logger().info(
            f"Goal {self.goal_id} mulai "
            f"(Run {self.run_counter}, Leg {leg}/5) | Bat: {bat}")

    def _end_goal(self, success: bool):
        duration = (
            self.get_clock().now() - self.start_time).nanoseconds / 1e9
        avg_vel = (sum(self.real_vel_list) / len(self.real_vel_list)
                   if self.real_vel_list else 0.0)
        max_vel = max(self.real_vel_list) if self.real_vel_list else 0.0

        self.battery_end_val = self.battery_current
        bat_drop = (
            round(self.battery_start_val - self.battery_end_val, 2)
            if self.battery_start_val >= 0 and self.battery_end_val >= 0
            else -1.0)

        leg = ((self.goal_id - 1) % 5) + 1
        label = f"R{self.run_counter:03d}-L{leg}"
        status = "VALID" if success else "INVALID"

        row = self._build_row(
            label, duration, avg_vel, max_vel,
            self.battery_start_val, self.battery_end_val, bat_drop)
        self._write_to_sheets(row, success, status)

        self.get_logger().info(
            f"Goal {self.goal_id} selesai | "
            f"EKF: {self.ekf_distance:.3f}m | t: {duration:.2f}s | "
            f"Replan: {self.replan_count} | "
            f"Bat: {self.battery_start_val:.1f}%→{self.battery_end_val:.1f}% "
            f"(Δ{bat_drop}%) | "
            f"{'✓' if success else '✗'} {status} | → {label}")

        self._reset_goal_state()

    def _reset_goal_state(self):
        """Reset semua flag goal ke kondisi idle."""
        self.goal_active      = False
        self.goal_ending      = False
        self.current_goal_uid = None

    def _build_row(self, label, duration, avg_vel, max_vel,
                   bat_s, bat_e, bat_d):
        def fb(v):
            return round(v, 1) if v >= 0 else "N/A"
        return [
            label,                        # A
            datetime.now().strftime("%d/%m/%Y %H:%M"),  # B
            self.uji_kode,                # C
            self.planner,                 # D
            self.load_kg,                 # E
            self.obstacle_count,          # F
            fb(bat_s),                    # G Battery Start
            fb(bat_e),                    # H Battery End
            fb(bat_d),                    # I Battery Drop
            self.koneksi,                 # J
            self.goal_sequence,           # K
            round(self.ekf_distance, 4),  # L EKF Distance
            round(duration, 3),           # M Waktu Sistem
            self.replan_count,            # N Replanning
            round(avg_vel, 4),            # O Avg Velocity
            round(max_vel, 4),            # P Max Velocity
            "",                           # Q Stopwatch → MANUAL
            # R, S, T → FORMULA di Sheets, tidak dikirim node
        ]

    # ══════════════════════════════════════════════════════════
    # Write to Sheets (dengan retry)
    # ══════════════════════════════════════════════════════════

    def _write_to_sheets(self, row: list, success: bool, status: str):
        if self.worksheet is None:
            self.get_logger().warn("Sheets tidak tersambung → CSV fallback")
            self._fallback_csv(row, success, status)
            return

        for attempt in range(1, MAX_WRITE_RETRY + 1):
            try:
                # Cari baris kosong berikutnya
                col_a = self.worksheet.col_values(COL_NO_RUN)
                next_row = DATA_START_ROW
                for i, val in enumerate(
                        col_a[DATA_START_ROW - 1:], start=DATA_START_ROW):
                    if val and str(val).startswith("R") and "-" in str(val):
                        next_row = i + 1
                    else:
                        break

                # Kirim A:Q (data + stopwatch kosong)
                self.worksheet.update(
                    f"A{next_row}:Q{next_row}",
                    [row],
                    value_input_option="USER_ENTERED"
                )
                # Kirim U (Success) dan V (Status) terpisah
                # supaya formula R/S/T tidak ter-overwrite
                self.worksheet.update(
                    f"U{next_row}", [[str(success).upper()]],
                    value_input_option="USER_ENTERED")
                self.worksheet.update(
                    f"V{next_row}", [[status]],
                    value_input_option="USER_ENTERED")

                self.get_logger().info(
                    f"Data terkirim → baris {next_row} "
                    f"(attempt {attempt}/{MAX_WRITE_RETRY})")
                return  # sukses, keluar dari retry loop

            except gspread.exceptions.APIError as e:
                self.get_logger().error(
                    f"Sheets API error (attempt {attempt}): {e}")
                if attempt < MAX_WRITE_RETRY:
                    time.sleep(2 ** attempt)  # exponential backoff
            except Exception as e:
                self.get_logger().error(
                    f"Gagal tulis Sheets (attempt {attempt}): {e}")
                if attempt < MAX_WRITE_RETRY:
                    time.sleep(2 ** attempt)

        # Semua retry habis → fallback CSV
        self.get_logger().error(
            f"Semua {MAX_WRITE_RETRY} attempt gagal → CSV fallback")
        self._fallback_csv(row, success, status)

    def _fallback_csv(self, row: list, success: bool, status: str):
        path = os.path.expanduser(
            f"~/nevo_ws/logs/nevo_{self.uji_kode}_fallback.csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        header = not os.path.exists(path)
        with open(path, "a") as f:
            if header:
                f.write("no_run,tanggal,kode,planner,load,obstacle,"
                        "bat_start,bat_end,bat_drop,koneksi,goal_seq,"
                        "ekf,waktu_sys,replan,avg_vel,max_vel,"
                        "stopwatch,success,status\n")
            f.write(",".join(str(v) for v in row) +
                    f",{success},{status}\n")
        self.get_logger().warn(f"Fallback CSV: {path}")


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
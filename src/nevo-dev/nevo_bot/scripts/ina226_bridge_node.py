#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
import serial

# Your custom mega-topic
from nevo_msgs.msg import NevoTelemetry
# The standard message for the debug trigger and battery JSON
from std_msgs.msg import Float32, String

class INA226Bridge(Node):
    def __init__(self):
        super().__init__('ina226_bridge')

        self.declare_parameter('serial_port','/dev/ttyBATERAI')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('robot_id', 'TFRB2')
        self.declare_parameter('ui_id', 'TFUI2')

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.robot_id = self.get_parameter('robot_id').value
        self.ui_id = self.get_parameter('ui_id').value
        self.ser = None
        
        # --- DEBUG OVERRIDE VARIABLES ---
        self.soc_override = None

        self.connect_serial()

        # Publisher
        self.telemetry_pub_ = self.create_publisher(NevoTelemetry, 'nevo_telemetry', 10)
        self.battery_pub_ = self.create_publisher(String, 'nevo/sensor/battery', 10)
        
        # Subscriber (The Debug Hook)
        self.override_sub_ = self.create_subscription(
            Float32,
            '/debug/force_soc',
            self.override_callback,
            10)

        self.timer = self.create_timer(0.01, self.read_serial_data)

    def override_callback(self, msg):
        """
        Listens to the terminal. If a number is sent, it forces the SoC.
        If -1.0 is sent, it returns to reading the real hardware data.
        """
        if msg.data < 0.0:
            self.soc_override = None
            self.get_logger().info("Debug Override REMOVED. Returning to true hardware SoC.")
        else:
            self.soc_override = msg.data
            self.get_logger().warn(f"DEBUG OVERRIDE ACTIVE: Forcing SoC to {self.soc_override}%")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Serial: {e}")

    def destroy_node(self):
        self.cleanup_serial()
        super().destroy_node()

    def cleanup_serial(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed")
            except Exception as e:
                self.get_logger().warn(f"Error closing serial port: {e}")

    def read_serial_data(self):
        # 1. Check if port is dead. If so, attempt to reconnect and exit this loop.
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return
            
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('<') and line.endswith('>'):
                    clean_line = line[1:-1]
                    data = clean_line.split(',')
                     
                    if len(data) == 13:  # 10 original + SoH, R_int, CycleCount
                        msg = NevoTelemetry()

                        msg.bus_voltage = float(data[0])                 # V
                        msg.battery_current = float(data[1]) / 1000.0    # mA -> A
                        msg.dock_current = float(data[3]) / 1000.0       # mA -> A
                        
                        msg.bat_shunt_voltage = float(data[5]) / 1000.0  # mV -> V
                        msg.bat_power = float(data[6]) / 1000.0          # mW -> W
                        msg.dock_voltage = float(data[7])                # V
                        msg.dock_shunt_voltage = float(data[8]) / 1000.0 # mV -> V
                        msg.dock_power = float(data[9]) / 1000.0         # mW -> W

                        msg.soh = float(data[10])                        # %
                        msg.internal_resistance_ohm = float(data[11]) / 1000.0  # mOhm -> Ohm
                        msg.cycle_count = int(data[12])
                        
                        # --- THE OVERRIDE INJECTION (Indexes 2 and 4 stay the same!) ---
                        real_soc = float(data[2])
                        real_fsm_state = int(data[4])
                        
                        if self.soc_override is not None and real_fsm_state >= 2:
                            self.soc_override = None
                            self.get_logger().warn("Hardware dock detected! Auto-releasing debug override. Returning to REAL battery data.")
                        
                        if self.soc_override is not None:
                            msg.soc = self.soc_override
                            if msg.soc <= 20.0 and real_fsm_state == 0:
                                msg.fsm_state = 1
                            else:
                                msg.fsm_state = real_fsm_state
                        else:
                            msg.soc = real_soc
                            msg.fsm_state = real_fsm_state
                        
                        msg.is_docked = (msg.fsm_state >= 2)
                        self.telemetry_pub_.publish(msg)

                        # Publish battery JSON for cloud_upload
                        battery_json = json.dumps({
                            "code": "BATTERY",
                            "data": {
                                "robot_id": self.robot_id,
                                "ui_id": self.ui_id,
                                "bus_voltage": round(msg.bus_voltage, 3),
                                "battery_current": round(msg.battery_current, 3),
                                "battery_percent": round(msg.soc, 1),
                                "dock_current": round(msg.dock_current, 3),
                                "state_code": msg.fsm_state,
                                "bat_shunt_voltage": round(msg.bat_shunt_voltage, 3),
                                "bat_power": round(msg.bat_power, 3),
                                "dock_voltage": round(msg.dock_voltage, 3),
                                "dock_shunt_voltage": round(msg.dock_shunt_voltage, 3),
                                "dock_power": round(msg.dock_power, 3),
                                "soh": round(msg.soh, 1),
                                "internal_resistance_mohm": round(msg.internal_resistance_ohm * 1000.0, 1),
                                "cycle_count": msg.cycle_count,
                            }
                        })
                        battery_msg = String()
                        battery_msg.data = battery_json
                        self.battery_pub_.publish(battery_msg)
                        
        # 2. Catch the exact PySerial disconnect exception
        except serial.SerialException as e:
            self.get_logger().error(f"Hardware power loss or disconnect! Forcing software reconnect... ({e})")
            self.cleanup_serial()
            self.ser = None  # This forces the loop above to reconnect on the next tick
        except Exception as e:
            self.get_logger().warn(f"Serial read parsing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = INA226Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down bridge node...")

    node.destroy_node()
    
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()

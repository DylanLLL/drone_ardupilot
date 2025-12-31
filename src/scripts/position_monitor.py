#!/usr/bin/env python3
"""
Position Hold Monitor
Real-time visualization of position error for diagnostics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import os
from typing import Optional


class PositionMonitor(Node):
    """Monitor and display position hold error in real-time"""

    def __init__(self):
        super().__init__('position_monitor')

        # State variables
        self.current_position: Optional[PoseStamped] = None
        self.target_position: Optional[PoseStamped] = None
        self.position_tolerance = 0.15  # Match flight controller default

        # Subscribers
        self.current_sub = self.create_subscription(
            PoseStamped,
            '/drone/local_position',
            self.current_position_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.target_position_callback,
            10
        )

        # Update display timer (10 Hz)
        self.display_timer = self.create_timer(0.1, self.update_display)

        self.get_logger().info('Position Monitor started')
        self.get_logger().info('Press Ctrl+C to exit')

    def current_position_callback(self, msg: PoseStamped):
        """Callback for current drone position"""
        self.current_position = msg

    def target_position_callback(self, msg: PoseStamped):
        """Callback for target position setpoint"""
        self.target_position = msg

    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')

    def get_arrow(self, value: float) -> str:
        """Get direction arrow for error value"""
        if abs(value) < 0.01:
            return "—"
        return "→" if value < 0 else "←"

    def get_status(self, distance: float) -> tuple:
        """Get status message and color based on distance"""
        if distance < self.position_tolerance:
            return "✓ POSITION LOCKED", "\033[92m"  # Green
        elif distance < self.position_tolerance * 2:
            return "⚠ CORRECTING", "\033[93m"  # Yellow
        else:
            return "✗ LARGE ERROR", "\033[91m"  # Red

    def update_display(self):
        """Update terminal display with current position data"""
        self.clear_screen()

        # Header
        print("═" * 70)
        print("  POSITION HOLD MONITOR".center(70))
        print("═" * 70)
        print()

        if self.target_position is None and self.current_position is None:
            print("  Waiting for position data...")
            print()
            print("  Make sure:")
            print("    - Navigation system is running")
            print("    - ArUco markers are visible")
            print("    - Drone is in GUIDED mode (for target position)")
            return

        if self.current_position is not None:
            cur_x = self.current_position.pose.position.x
            cur_y = self.current_position.pose.position.y
            cur_z = self.current_position.pose.position.z
            print(f"  Current Position:    X: {cur_x:+7.3f}m  Y: {cur_y:+7.3f}m  Z: {cur_z:+7.3f}m")
        else:
            print("  Current Position:    [NO DATA - Check vision system]")
            cur_x = cur_y = cur_z = 0.0

        if self.target_position is not None:
            tgt_x = self.target_position.pose.position.x
            tgt_y = self.target_position.pose.position.y
            tgt_z = self.target_position.pose.position.z
            print(f"  Target Position:     X: {tgt_x:+7.3f}m  Y: {tgt_y:+7.3f}m  Z: {tgt_z:+7.3f}m")
        else:
            print("  Target Position:     [NO DATA - Switch to GUIDED mode]")
            print()
            return

        print("  " + "─" * 66)

        # Calculate errors
        error_x = tgt_x - cur_x
        error_y = tgt_y - cur_y
        error_z = tgt_z - cur_z
        distance = (error_x**2 + error_y**2 + error_z**2) ** 0.5

        # Position error with direction indicators
        arrow_x = self.get_arrow(error_x)
        arrow_y = self.get_arrow(error_y)
        arrow_z = self.get_arrow(error_z)

        print(f"  Position Error:      X: {error_x:+7.3f}m {arrow_x}  Y: {error_y:+7.3f}m {arrow_y}  Z: {error_z:+7.3f}m {arrow_z}")
        print()

        # Status with color
        status_msg, color = self.get_status(distance)
        reset_color = "\033[0m"
        print(f"  Distance from target: {color}{distance:.3f}m{reset_color}")
        print(f"  Status: {color}{status_msg}{reset_color}")
        print()

        # Correction guidance
        if distance > 0.01:  # Only show if there's meaningful error
            print("  Corrections needed:")
            if abs(error_x) > 0.01:
                direction = "LEFT " if error_x > 0 else "RIGHT"
                print(f"    {'←' if error_x > 0 else '→'} Move {direction}  {abs(error_x):.3f}m")
            if abs(error_y) > 0.01:
                direction = "BACK " if error_y > 0 else "FORWARD"
                print(f"    {'↓' if error_y > 0 else '↑'} Move {direction}  {abs(error_y):.3f}m")
            if abs(error_z) > 0.01:
                direction = "DOWN " if error_z > 0 else "UP"
                print(f"    {'↓' if error_z > 0 else '↑'} Move {direction}  {abs(error_z):.3f}m")
        else:
            print("  ✓ Position is within tolerance")

        print()
        print("═" * 70)
        print()
        print("  Frame coordinate system:")
        print("    X+: Right    Y+: Forward    Z+: Up")
        print()
        print("  Error interpretation:")
        print("    Negative X error = Drone too far RIGHT (should move LEFT)")
        print("    Positive X error = Drone too far LEFT (should move RIGHT)")
        print()
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    monitor = PositionMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Position Monitor stopped')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

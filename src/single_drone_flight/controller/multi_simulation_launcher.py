#!/usr/bin/env python3

"""
Multi-Drone Simulation Launcher for PX4 v1.15 + Gazebo Classic

- Launches Gazebo Classic world once
- Spawns N PX4 SITL instances (iris model)
- Each PX4 instance has unique SYSID and INSTANCE
- Calls spawn_model.py to insert each drone at different positions

Author: ROS Assistance
"""

import subprocess
import time
import os
import rclpy
from rclpy.node import Node


class MultiDroneLauncher(Node):
    def __init__(self, num_drones=3, spacing=2.0):
        super().__init__('multi_drone_launcher')
        self.num_drones = num_drones
        self.spacing = spacing
        self.px4_path = os.path.expanduser("~/PX4-Autopilot")
        self.spawn_script = os.path.join(
            self.px4_path,
            "Tools/simulation/gazebo-classic/sitl_gazebo/scripts/spawn_model.py"
        )

        self.get_logger().info(f"🚀 Launching {self.num_drones} drones in Gazebo Classic...")
        self.launch()

    def launch(self):
        # --- Start Gazebo Classic world ---
        try:
            subprocess.Popen([
                "gnome-terminal", "--tab", "--title", "Gazebo-Classic", "--",
                "bash", "-c",
                f"cd {self.px4_path} && make px4_sitl_default gazebo-classic empty; exec bash"
            ])
            self.get_logger().info("✅ Gazebo Classic world started")
            time.sleep(10)  # allow Gazebo to load
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start Gazebo Classic: {e}")
            return

        # --- Start PX4 + spawn each drone ---
        for i in range(self.num_drones):
            sysid = i + 1
            instance = i
            name = f"drone{sysid}"

            # position offset
            x = i * self.spacing
            y = 0.0
            z = 0.0

            # PX4 SITL command
            px4_cmd = (
                f"cd {self.px4_path} && "
                f"PX4_SYS_AUTOSTART=10016 "
                f"PX4_SYSID={sysid} "
                f"PX4_INSTANCE={instance} "
                f"./build/px4_sitl_default/bin/px4 -i {instance}"
            )

            # spawn command
            spawn_cmd = (
                f"python3 {self.spawn_script} -m iris -n {name} -x {x} -y {y} -z {z}"
            )

            # start PX4 SITL
            try:
                subprocess.Popen([
                    "gnome-terminal", "--tab", "--title", f"{name}-PX4", "--",
                    "bash", "-c", px4_cmd + "; exec bash"
                ])
                self.get_logger().info(f"✅ PX4 instance launched: {name}")
                time.sleep(3)
            except Exception as e:
                self.get_logger().error(f"❌ Failed to launch PX4 for {name}: {e}")

            # spawn into gazebo
            try:
                subprocess.Popen([
                    "gnome-terminal", "--tab", "--title", f"{name}-Spawn", "--",
                    "bash", "-c", spawn_cmd + "; exec bash"
                ])
                self.get_logger().info(f"✅ Spawned {name} at ({x}, {y}, {z})")
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"❌ Failed to spawn {name}: {e}")

        self.get_logger().info("🎯 All drones launched in Gazebo Classic world!")


def main(args=None):
    rclpy.init(args=args)
    launcher = MultiDroneLauncher(num_drones=3, spacing=2.0)
    try:
        rclpy.spin(launcher)
    except KeyboardInterrupt:
        pass
    finally:
        launcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

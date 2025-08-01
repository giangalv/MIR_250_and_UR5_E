#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import ast
import time

class MiRController(Node):
    def __init__(self):
        super().__init__('mir_control_node')
        
        self.create_api_clients()

        self.restAPI_getStatus.wait_for_service()
        #time.sleep(2)
        #self.set_ready_control()

        # Initial robot info and footprint image
        self.update_robot_status(initial=True)

        # Periodic status updates every 2 minutes
        self._status_timer = self.create_timer(120.0, self.update_robot_status)
        #self.create_api_clients()

    def create_api_clients(self):
        self.restAPI_setPause = self.create_client(Trigger, 'mir_250_set_pause')
        self.restAPI_setReady = self.create_client(Trigger, 'mir_250_set_ready')
        self.restAPI_getStatus = self.create_client(Trigger, 'mir_250_get_status')
    
    def set_ready_control(self):
        self.call_trigger_service(self.restAPI_setReady)
    
    def set_pause_control(self):
        self.call_trigger_service(self.restAPI_setPause)

    def get_status_control(self):
        req = Trigger.Request()
        future = self.restAPI_getStatus.call_async(req)
        future.add_done_callback(self.status_response_callback)

    def status_response_callback(self, future):
        try:
            result = future.result()
            if result.success:
                status = ast.literal_eval(result.message)
                #self.get_logger().info(f"Robot status: {status}")
                
                # INITIAL: only once
                if not hasattr(self, '_status_initialized'):
                    self._status_initialized = True
                    self.log_full_status(status)
                    if 'footprint' in status:
                        self.create_footprint_image(status['footprint'])
                else:
                    # PERIODIC updates
                    self.log_velocity_and_battery(status)

            else:
                self.get_logger().warn(f"Service call failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")

    def call_trigger_service(self, client):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60)
        if not future.done():
            self.get_logger().error("timeout")
        else:
            # service done
            self.get_logger().info("service executed")
            res = future.result()
            if res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(res.message)


    def update_robot_status(self, initial=False):
        status = self.get_status_control()
        if not status:
            #self.get_logger().warn("Status is None — skipping update.")
            return

        if initial:
            self.log_full_status(status)
            if 'footprint' in status:
                self.create_footprint_image(status.get('footprint'))
        else:
            self.log_velocity_and_battery(status)

    
    def log_velocity_and_battery(self, status_data):
        vel = status_data.get('velocity', {})
        battery = status_data.get('battery_percentage', 0.0)
        time_left = status_data.get('battery_time_remaining', 0)
        hours = time_left // 3600
        minutes = (time_left % 3600) // 60

        log_msg = (
            f"\nVELOCITY: Linear={vel.get('linear', 0.0):.2f} m/s, "
            f"Angular={vel.get('angular', 0.0):.2f} rad/s\n"
            f"BATTERY: {battery:.1f}% ({hours}h {minutes}m remaining)\n"
        )
        self.get_logger().info(log_msg)

    def create_footprint_image(self, footprint_coords):
        try:            
            if isinstance(footprint_coords, str):
                try:
                    footprint_coords = ast.literal_eval(footprint_coords)
                except Exception as e:
                    self.get_logger().warn(f"Coordinate parse error: {e}")
                    return

            if not footprint_coords or not all(len(p)==2 for p in footprint_coords):
                self.get_logger().warn("Invalid coordinates format")
                return

            fig, ax = plt.subplots(figsize=(12, 12))
            ax.set_aspect('equal')
            ax.set_title('MiR250 FOOTPRINT\nClose the window to control the Battery status')
            ax.set_xlim(-1.0, 1.0)
            ax.set_ylim(-1.0, 1.0)
            
            robot_width = 0.8
            robot_height = 0.58
            robot_corners = [
                (-robot_width/2, -robot_height/2), 
                (-robot_width/2, robot_height/2),   
                (robot_width/2, robot_height/2),    
                (robot_width/2, -robot_height/2)    
            ]
            
            robot = patches.Rectangle(
                robot_corners[0], robot_width, robot_height,
                fill=True, color='blue', alpha=0.3,
                label=f'Robot Body ({robot_width*1000:.0f}×{robot_height*1000:.0f}mm)'
            )
            ax.add_patch(robot)
            
            robot_labels = ['BR', 'BL', 'FL', 'FR']
            for (x,y), label in zip(robot_corners, robot_labels):
                ax.text(x, y, f' {label}\n({x:.2f},{y:.2f})', 
                    color='blue', fontsize=10,
                    ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue'))

            poly = patches.Polygon(
                footprint_coords,
                closed=True,
                fill=False,
                edgecolor='red',
                linewidth=3,
                label='Safety Footprint'
            )
            ax.add_patch(poly)

            for i, (x,y) in enumerate(footprint_coords):
                ax.plot(x, y, 'ro', markersize=8)
                ax.text(x, y, f'({x:.2f},{y:.2f})',
                    color='red', fontsize=10,
                    ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='red'))

            ax.arrow(0, 0, 0.25, 0, head_width=0.05, color='green', label='Front')
            ax.grid(True, linestyle=':')
            ax.axhline(0, color='black', linewidth=0.5)
            ax.axvline(0, color='black', linewidth=0.5)
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.legend(loc='upper right', fontsize=10)

            try:
                plt.tight_layout()
                plt.show(block=True)
                plt.pause(0.001)
                self.get_logger().info("\nMIR 250 visualization displayed.\nControlling the BATTERY status every 2 minutes.")
            except Exception as e:
                self.get_logger().info(f"Visualization error or exit: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")
        finally:
            plt.close('all')

    def log_full_status(self, status_data):
        if not status_data:
            self.get_logger().warn("No status data available")
            return

        output = [
            "\n\n===== ROBOT STATUS =====",
            f"Robot Name: {status_data.get('robot_name', 'Unknown')}",
            f"Model: {status_data.get('robot_model', 'Unknown')}",
            f"Serial: {status_data.get('serial_number', 'Unknown')}",
        ]

        vel = status_data.get('velocity', {})
        output.append(f"Velocity: Linear={vel.get('linear', 0.0):.2f} m/s, Angular={vel.get('angular', 0.0):.2f} rad/s")

        battery = status_data.get('battery_percentage', 0.0)
        time_left = status_data.get('battery_time_remaining', 0)
        hours = time_left // 3600
        minutes = (time_left % 3600) // 60
        output.append(f"Battery: {battery:.1f}% ({hours}h {minutes}m remaining)")
        output.append("===========================\n")

        self.get_logger().info('\n'.join(output))
    
    def destroy_node(self):
        #self.set_pause_control()
        time.sleep(3) # Allow time for the pause command to take effect
        super().destroy_node()

def main():
    rclpy.init()
    node = MiRController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
    except Exception as e:
        node.get_logger().error(f"Fatal error: {str(e)}")
        node.destroy_node()
    finally:
        plt.close('all')  # Ensure all matplotlib figures are closed
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
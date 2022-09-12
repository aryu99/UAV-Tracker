import airsim
import math
import time
import matplotlib.pyplot as plt
from datetime import datetime
from airsim import YawMode
import asyncio
from mavsdk import System

async def run():
    
    drone = System()
    
    await drone.connect(system_address="udp://:14550")
    print("test")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    await drone.action.set_takeoff_altitude(20.0)

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()



async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())



# class Tracker:
#     def __init__(self, tracker: str, target: str, tracking_distance=5, time_thresh=60, plot=True):
#         self.tracker = tracker
#         self.target = target
#         self.distance = tracking_distance
#         self.time_thresh = time_thresh
#         self.init_time = time.time()
#         self.curr_time = time.time()
#         self.takeoff = False
#         self.plot = plot

#         # plotting variables         
#         self.vx_store = []
#         self.vy_store = []
#         self.vz_store = []
#         self.yaw_rate_store = []

#         self.drone = System()            

#     def start(self):
#         self.client.confirmConnection()
#         self.client.enableApiControl(True, vehicle_name=self.tracker)

#         print("arming the chaser...")
#         self.client.armDisarm(True, vehicle_name=self.tracker)

#         landed = self.client.getMultirotorState(self.tracker).landed_state
#         if not self.takeoff and landed == airsim.LandedState.Landed:
#             self.takeoff = True
#             print("Chaser taking off...")
#             self.client.takeoffAsync(vehicle_name=self.tracker).join()

#         self.controller()        
            
#         self.land()

#         if self.plot == True and airsim.LandedState.Landed == 0:
#             print("hello")
#             self.plotter()        

#     def controller(self):
#         prev_time = time.time()
#         Ix = 0
#         Iy = 0
#         Iz = 0
#         Ih = 0
#         ex_prev = 0
#         ey_prev = 0
#         ez_prev = 0
#         eh_prev = 0
#         while True:
#             # PID Calculations

#             ex = self.dist_calc()[1] - self.distance
#             ey = self.dist_calc()[2] - self.distance
#             ez = self.dist_calc()[3]

#             # yaw rate (deg/sec) calculations
#             target_heading = self.heading_calc()
#             current_heading = self.chaser_heading()
#             eh = target_heading - current_heading

#             # time handling for PID calculation
#             current_time = time.time()
#             delta_t = current_time - prev_time

#             # control Equations, constants are adjusted as needed
#             Px = 0.5 * ex
#             Py = 0.5 * ey
#             Pz = 0.5 * ez
#             Ph = 0.9 * eh
#             Ix = Ix + -0.001 * ex * delta_t
#             Iy = Iy + -0.001 * ey * delta_t
#             Iz = Iz + -0.001 * ez * delta_t
#             Ih = Ih + -0.001 * eh * delta_t
#             Dx = 0.01 * (ex - ex_prev) / delta_t
#             Dy = 0.01 * (ey - ey_prev) / delta_t
#             Dz = 0.01 * (ez - ez_prev) / delta_t
#             Dh = 0.01 * (eh - eh_prev) / delta_t

#             Vx = Px + Ix + Dx
#             Vy = Py + Iy + Dy
#             Vz = Pz - Iz + Dz
#             yaw_rate = Ph + Ih + Dh

#             # handling direction of yaw rotation
#             if abs(eh) > 180:
#                 yaw_rate = -yaw_rate

#             self.client.moveByVelocityAsync(Vx, Vy, Vz, 0.1, yaw_mode=YawMode(is_rate=True, yaw_or_rate=yaw_rate),
#                                             vehicle_name=self.tracker).join()

#             # update the stored data for the next iteration
#             ex_prev = ex
#             ey_prev = ey
#             ez_prev = ez
#             eh_prev = eh

#             # update plotting variables
#             self.vx_store.append(Vx)
#             self.vy_store.append(Vy)
#             self.vz_store.append(Vz)
#             self.yaw_rate_store.append(yaw_rate)

#             # break after a certain time threshold
#             prev_time = current_time
#             self.curr_time = time.time()
#             if self.curr_time - self.init_time > self.time_thresh:
#                 print("Breaking PID State controller")
#                 # self.client.reset()
#                 break

#     def chaser_heading(self) -> float:
#         q = self.client.simGetObjectPose(self.tracker).orientation
#         heading = math.degrees(self.q_euler(q)[2])
#         return heading

#     @staticmethod
#     def q_euler(q) -> tuple:
#         '''
#         Converts Quarternion values to Euler angles (Roll, Pitch, Yaw)        
#         '''
#         z = q.z_val
#         y = q.y_val
#         x = q.x_val
#         w = q.w_val
#         ysqr = y * y

#         # roll (x-axis rotation)
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + ysqr)
#         roll = math.atan2(t0, t1)

#         # pitch (y-axis rotation)
#         t2 = +2.0 * (w * y - z * x)
#         if t2 > 1.0:
#             t2 = 1
#         if t2 < -1.0:
#             t2 = -1.0
#         pitch = math.asin(t2)

#         # yaw (z-axis rotation)
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (ysqr + z * z)
#         yaw = math.atan2(t3, t4)

#         return pitch, roll, yaw

#     def heading_calc(self) -> float:
#         '''
#         Calculates yaw required to look towards the runner (Global frame) in degrees 
#         '''
#         magnitude, x_offset, y_offset, z_offset = self.dist_calc()
#         angle = math.degrees(math.atan2(x_offset, y_offset))

#         if angle > 0 and y_offset > 0:
#             yaw = -(90 - angle)
#         elif angle > 0 and y_offset < 0:
#             yaw = angle - 90

#         elif angle < 0 and y_offset < 0:
#             yaw = 270 + angle

#         elif angle < 0 and y_offset > 0:
#             yaw = -90 + angle

#         else:
#             yaw = 0

#         return -yaw

#     def dist_calc(self) -> tuple:
#         '''
#         Get the absolute distance, and the difference between position 
#         of the tracker and the target in the GLOBAL X, Y, Z frames
#         '''
#         x_1, y_1, z_1 = self.state_extractor(self.tracker)
#         x_2, y_2, z_2 = self.state_extractor(self.target)
#         dist = math.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2 + (z_1 - z_2) ** 2)
#         ex = x_2 - x_1
#         ey = y_2 - y_1
#         ez = z_2 - z_1
#         return dist, ex, ey, ez

#     def state_extractor(self, vech) -> tuple:
#         '''
#         Extracts the global position
#         '''
#         pos_x = self.client.simGetObjectPose(vech).position.x_val
#         pos_y = self.client.simGetObjectPose(vech).position.y_val
#         pos_z = self.client.simGetObjectPose(vech).position.z_val
#         return pos_x, pos_y, pos_z


#     def plotter(self):
#         '''
#         Plots the input Vx, Vy, Vz and yaw_rate given to the drone by the controllers and saves the graphs as an image
#         '''
#         plt.subplot(2, 2, 1)
#         plt.plot(self.vx_store, label = 'Vx')
#         plt.xlabel("Epoch")
#         plt.ylabel("Velocity")
#         plt.legend(loc='best')               

#         plt.subplot(2, 2, 2)
#         plt.plot(self.vy_store, label = 'Vy')
#         plt.xlabel("Epoch")
#         plt.ylabel("Velocity")
#         plt.legend(loc='best')

#         plt.subplot(2, 2, 3)
#         plt.plot(self.vz_store, label = 'Vz')
#         plt.xlabel("Epoch")
#         plt.ylabel("Velocity")
#         plt.legend(loc='best')

#         plt.subplot(2, 2, 4)
#         plt.plot(self.yaw_rate_store, label = 'yaw_rate')
#         plt.xlabel("Epoch")
#         plt.ylabel("Velocity")
#         plt.legend(loc='best')      
        
        
        
#         dt_string = datetime.now().strftime("%d_%m_%Y_%H:%M:%S")

#         plt.savefig("Output_" + dt_string, bbox_inches='tight', dpi=100)
    
#     def land(self):
#         '''
#         Begins the landing sequence of the UAV immediately
#         '''
#         if self.takeoff:

#             print("landing MAV...")
#             self.client.landAsync(vehicle_name=self.tracker).join()

#             print("disarming MAV.")
#             self.client.armDisarm(False, self.tracker)

#             print("disabling API control for MAV...")
#             self.client.enableApiControl(False, "MAV")

#             print("resetting client...")
#             self.client.reset()

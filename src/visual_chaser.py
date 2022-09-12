import airsim
import math
import time
import cv2
import chaser
import multiprocessing
from airsim import YawMode


class IntegratedTracker(chaser.Tracker):
    def __init__(self, tracker: str, target: str, tracking_distance, time_thresh, visual_tracking_distance_area=20,
                 switch_time=10, plot=True):
        super().__init__(tracker, target, tracking_distance, time_thresh, plot)

        self.bb_distance_area = visual_tracking_distance_area

        # handle the controller switch
        self.switch_time = switch_time
        self.switch_threads = []

        # Initialize the camera and image input
        self.camera_name = "0"
        self.width = 0
        self.height = 0

        # CV segmentation method
        self.image_type = airsim.ImageType.Segmentation
        self.client.simSetSegmentationObjectID("[\w]*", 0, True)
        self.client.simSetSegmentationObjectID("halo", 255, True)

    def tracker_switcher(self, tracker_id):
        '''
        Handles switching between the state-based and vision-based controller.
        Switches the controller after specified amount of time (seconds)
        as given by switch_time        
        '''
        if tracker_id == 0:
            print("thread running for state-based tracker")
            print("Initializing state-based tracker")
            super(IntegratedTracker, self).start()
        elif tracker_id == 1:
            time.sleep(self.switch_time)
            print("breaking state-based tracker")
            self.switch_threads[0].terminate()

    def start(self):
        p1 = multiprocessing.Process(target=self.tracker_switcher, args=(0,))
        self.switch_threads.append(p1)
        p2 = multiprocessing.Process(target=self.tracker_switcher, args=(1,))
        self.switch_threads.append(p2)

        p1.start()
        p2.start()

        p2.join()

        self.visionController()
            
        self.land()

        if self.plot == True and airsim.LandedState.Landed == 0:
            self.plotter()

    def visionController(self):
        print("Initializing vision-based tracker")
        self.takeoff = True
        prev_time = time.time()
        Ix = 0
        Iy = 0
        Iz = 0
        Ih = 0
        ex_prev = 0
        ey_prev = 0
        ez_prev = 0
        eh_prev = 0
        bb_prev = 0.0
        Vx_prev = 0
        flag = 0
        self.client.simSetCameraFov(self.camera_name, 120, "MAV")

        while True:
            # Time handling
            current_time = time.time()
            if flag == 0:
                delta_t = 0.3
                flag = 1
            else:
                delta_t = current_time - prev_time

            rawImage = self.client.simGetImage(self.camera_name, self.image_type, vehicle_name=self.tracker)
            if not rawImage:
                continue

            bb_center_x, bb_center_y, bb_area, png = self.bb_calculator(rawImage)

            # smoothen the area calculation to reduce jitters in the x-direction
            bb_area = (bb_area + bb_prev) / 2
            bb_prev = bb_area            

            # PID Calculations
            # ex = self.bb_distance_area - bb_area #vision
            ex = self.dist_calc()[0] - self.distance
            ey = bb_center_y - self.width / 2
            ez = bb_center_x - self.height / 2

            try:
                if bb_center_x < self.height/2:
                    eh = math.degrees(math.atan((bb_center_y - self.width / 2) / bb_center_x))
                else:
                    eh = math.degrees(math.atan((bb_center_y - self.width / 2) / (self.height - bb_center_x)))                
            except:
                eh = eh_prev

            # Px = 0.04 * ex #vision
            Px = 0.8 * ex #state
            Py = 0.06 * ey
            Pz = 0.06 * ez
            Ph = 2.35 * eh
            Ix = Ix + 0.0005 * ex * delta_t
            Iy = Iy + 0.001 * ey * delta_t
            Iz = Iz + 0.004 * ez * delta_t
            Ih = Ih + 0.001 * eh * delta_t
            Dx = 0.00000005 * (ex - ex_prev) / delta_t
            Dy = 0.04 * (ey - ey_prev) / delta_t
            Dz = 0.0001 * (ez - ez_prev) / delta_t
            Dh = 0.00002 * (ey - eh_prev) / delta_t

            # Ix = 0
            # Iy = 0
            # Iz = 0
            # Ih = 0
            # Dx = 0
            # Dy = 0
            # Dz = 0
            # Dh = 0           

            Vx = Px + Ix + Dx
            Vy = Py + Iy + Dy
            Vz = Pz + Iz + Dz
            yaw_rate = Ph - Ih + Dh
            Vx = (Vx + Vx_prev) / 2
            # Vy = Vy/2

            cv2.imshow("AirSim", png)            

            self.client.moveByVelocityBodyFrameAsync(Vx, Vy, Vz, 0.1,
                                                     yaw_mode=YawMode(is_rate=True, yaw_or_rate=0),
                                                     vehicle_name=self.tracker).join()            

            # update the stored data for the next iteration
            ex_prev = ex
            ey_prev = ey
            ez_prev = ez
            eh_prev = eh
            Vx_prev = Vx

            # update plotting variables
            self.vx_store.append(Vx)
            self.vy_store.append(Vy)
            self.vz_store.append(Vz)
            self.yaw_rate_store.append(yaw_rate)

            
            # handle time and break
            prev_time = current_time
            self.curr_time = time.time()
            if (self.curr_time - self.init_time > self.time_thresh) or (cv2.waitKey(1) & 0xFF == ord('q')):
                print("Breaking Integrated PID controller")
                cv2.destroyAllWindows()
                # self.client.reset()
                break

    def bb_calculator(self, img) -> tuple:
        '''
        Iterates through the input image, and segments out the specified object mesh. 
        Further, draws a rectangular bounding box around the segmented object
        '''        
        png = cv2.imdecode(airsim.string_to_uint8_array(img), cv2.IMREAD_UNCHANGED)
        self.height, self.width = png.shape[:2]
        # print("height. width : ", self.height, self.width)
        flag = 0
        x_1, y_1 = -1, -1
        x_2, y_2 = -1, -1
        for row in range(self.height):
            for col in range(self.width):
                if flag == 0:
                    if png[row][col][0] > 0.0:
                        x_1, y_1 = row, col
                        flag = 1
                if flag == 1:
                    if png[row][col][0] > 0.0:
                        if col < y_1:
                            y_1 = col
                        if row > x_2:
                            x_2 = row
                        if col > y_2:
                            y_2 = col

        png = self.image_annotator(png, ([y_1, x_1], [y_2, x_2]))
        bb_center_x = (x_2 - x_1) / 2 + x_1
        bb_center_y = (y_2 - y_1) / 2 + y_1
        bb_area = (x_2 - x_1) * (y_2 - y_1)
        return bb_center_x, bb_center_y, bb_area, png

    def image_annotator(self, png, bb: tuple):
        '''
        Draws a crosshair in the center and the bounding box around an object
        '''
        png = cv2.rectangle(png, (bb[0][0], bb[0][1]), (bb[1][0], bb[1][1]), (0, 255, 0), thickness=1)
        png = cv2.drawMarker(png, (int(self.width / 2), int(self.height / 2)), color=(0, 0, 255),
                             markerType=cv2.MARKER_TILTED_CROSS, markerSize=5, thickness=1)
        return png

    

import airsim
import runner as run
# import chaser as chase
import visual_chaser as chase
import threading


def runtime(id):
    if id == 0:
        targ = run.OrbitNavigator(10, 20, 3, 1, [1, 0], 0)
        targ.sinusoidal_slalom()

    elif id == 1:
        trac = chase.IntegratedTracker("MAV", "HALO", tracking_distance=8, time_thresh=45,
                                       visual_tracking_distance_area=100, switch_time=10)
        # trac = chase.Tracker("MAV", "HALO")                               
        
        trac.start()


if __name__ == "__main__":
    client = airsim.MultirotorClient()
    print("Resetting client...")
    client.reset()
    threads = []
    for id in range(2):
        t = threading.Thread(target=runtime, args=(id,))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

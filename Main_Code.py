import cv2
import dlib
import time
import threading
import math
import tkinter as tk
from tkinter import ttk

carCascade = cv2.CascadeClassifier('myhaar.xml')
video = cv2.VideoCapture('cars.mp4')

WIDTH = int(1280/2)
HEIGHT = int(720/2)
MIN_BBOX_SIZE = 30
INTERVAL_SECONDS = 10  # Variable time interval
fps = 30
ppm = int(7) # pixels per real life meter
LOST_TRACKER_WINDOW = 30 # Frames to wait before considering tracker truly lost
car_traffic_threshhold = 5 # number of cars that need to pass in the measured timeframe in order for it to not be considered conjested
car_speed_threshhold = 20 # speed of cars that pass in measured timeframe for it to be considered conjested
medium_traffic_modifier = 1.5 # number of times that the car speed and quantity needs to be of tbe minumum for it to be considered mediocre traffic
image_y_threshhold = 200 # the area after we dont want the cars' speed to be tracked if the cameras view is tilted as the ppm will get lower further along the road


def estimateSpeed(location1, location2):
    d_pixels = math.sqrt(math.pow(location2[0] - location1[0], 2) + math.pow(location2[1] - location1[1], 2))
    d_meters = d_pixels / ppm
    speed = d_meters * fps * 3.6
    return speed

def trackMultipleObjects(car_count_text, avg_speed_text):
    rectangleColor = (0, 255, 0)
    frameCounter = 0
    currentCarID = 0

    carTracker = {}
    carNumbers = {}
    carLocation1 = {}
    carLocation2 = {}
    speed = [None] * 1000
    lostCarTrackers = {} # Dictionary to store lost trackers: {carID: {tracker, last_location, lost_frame}}

    last_time_interval = time.time() # relies on realtime operation
    all_speeds_interval = []
    cars_passed_interval_count = 0

    while True:
        start_time = time.time()
        rc, image = video.read()
        if type(image) == type(None):
            break

        image = cv2.resize(image, (WIDTH, HEIGHT))
        resultImage = image.copy()

        frameCounter += 1

        carIDtoDelete = []
        car_speeds_tk = {}
        reassociated_carIDs = set() # Keep track of re-associated carIDs in this frame (set makes duplicate values ignored)

        current_time = time.time() # depends on realtime operation
        if current_time - last_time_interval >= INTERVAL_SECONDS:
            avg_speed_interval_all = "N/A"
            if all_speeds_interval:
                avg_speed_interval_all = sum(all_speeds_interval) / len(all_speeds_interval)

            if isinstance(avg_speed_interval_all, (int, float)):
                avg_speed_text.set(f"{(avg_speed_interval_all/1.609):.2f} mph")

            car_count_text.set(cars_passed_interval_count)
            #print(cars_passed_interval_count)
            
            if cars_passed_interval_count <= car_traffic_threshhold and avg_speed_interval_all <= car_speed_threshhold: # reporting traffic status logic
                #print("Traffic could be conjested, as few cars have passed and they are moving slowly")
                traffic_status_text.set("Conjested, low speed/high car count")
            elif cars_passed_interval_count <= (car_traffic_threshhold * medium_traffic_modifier) and avg_speed_interval_all <= (car_speed_threshhold * medium_traffic_modifier):
                #print("Traffic seems mediocrely conjested")
                traffic_status_text.set("Mildly conjested")
            else:
                #print("Traffic seems unconjested")
                traffic_status_text.set("Unonjested, high speed/low car count")

            cars_passed_interval_count = 0 # reset
            all_speeds_interval = []
            last_time_interval = current_time

        # --- Check for Lost Trackers Timeout ---
        lost_carIDs_to_remove = []
        for lost_carID, lost_car_info in lostCarTrackers.items():
            if frameCounter - lost_car_info['lost_frame'] > LOST_TRACKER_WINDOW:
                cars_passed_interval_count += 1 # Increment count when truly lost
                lost_carIDs_to_remove.append(lost_carID)

        for lost_carID in lost_carIDs_to_remove:
            lostCarTrackers.pop(lost_carID) # pop removes and returns value in one go


        for carID in list(carTracker.keys()):
            trackingQuality = carTracker[carID].update(image)

            if trackingQuality < 7:
                carIDtoDelete.append(carID)
                continue

            trackedPosition = carTracker[carID].get_position()
            t_x = int(trackedPosition.left())
            t_y = int(trackedPosition.top())
            t_w = int(trackedPosition.width())
            t_h = int(trackedPosition.height())

            if t_w < MIN_BBOX_SIZE or t_h < MIN_BBOX_SIZE:
                carIDtoDelete.append(carID)
                continue


        for carID in carIDtoDelete:
            # Move tracker to lostCarTrackers instead of popping and incrementing count immediately
            lostCarTrackers[carID] = {
                'tracker': carTracker.pop(carID),
                'last_location': carLocation2.get(carID),  # Use .get() to avoid KeyError if carID is not in carLocation2
                'lost_frame': frameCounter
            }
            if carID in carLocation2: # Check if carID is in carLocation2 before popping
                carLocation2.pop(carID) # Clean up carLocation2
            carLocation1.pop(carID, None) # Clean up carLocation1


        if not (frameCounter % 10):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cars = carCascade.detectMultiScale(gray, 1.1, 13, 18, (24, 24))

            detected_car_locations = [] # Store locations of newly detected cars
            for (_x, _y, _w, _h) in cars:
                detected_car_locations.append(((int(_x), int(_y), int(_w), int(_h))))


            potential_reassociations = [] # List of tuples (detected_bbox_index, lost_carID, distance)

            for detected_bbox_index, (_x, _y, _w, _h) in enumerate(detected_car_locations):
                x = int(_x)
                y = int(_y)
                w = int(_w)
                h = int(_h)
                x_bar = x + 0.5 * w
                y_bar = y + 0.5 * h

                best_match_carID = None
                min_distance = float('inf')

                for lost_carID, lost_car_info in lostCarTrackers.items():
                    last_location = lost_car_info['last_location'] # Get last_location
                    if last_location is None: # Check if last_location is None
                        continue # Skip to the next lost car if last_location is None
                    last_tx, last_ty, last_tw, last_th = last_location # Unpack only if not None

                    last_tx_bar = last_tx + 0.5 * last_tw
                    last_ty_bar = last_ty + 0.5 * last_th

                    distance = math.sqrt((x_bar - last_tx_bar)**2 + (y_bar - last_ty_bar)**2) # Euclidean distance

                    if distance < min_distance and distance < 100: # Proximity threshold (adjust 100 as needed)
                        min_distance = distance
                        best_match_carID = lost_carID
                        potential_reassociations.append((detected_bbox_index, best_match_carID, min_distance))

            # Prioritize re-associations (e.g., closest match first - can be improved with more sophisticated logic)
            potential_reassociations.sort(key=lambda x: x[2]) # Sort by distance

            reassociated_detection_indices = set()
            for detection_index, carID_to_reassociate, distance in potential_reassociations:
                if detection_index not in reassociated_detection_indices and carID_to_reassociate not in carTracker: # Avoid double re-association
                    #print(f"Re-associating carID {carID_to_reassociate} with new detection.")
                    tracker = lostCarTrackers[carID_to_reassociate]['tracker'] # Retrieve tracker
                    x, y, w, h = detected_car_locations[detection_index]
                    tracker.start_track(image, dlib.rectangle(x, y, x + w, y + h)) # Restart tracker at new location

                    carTracker[carID_to_reassociate] = tracker # Add back to active trackers
                    carLocation1[carID_to_reassociate] = [x, y, w, h] # Update location
                    lostCarTrackers.pop(carID_to_reassociate) # Remove from lost trackers
                    reassociated_detection_indices.add(detection_index) # Mark detection as used
                    reassociated_carIDs.add(carID_to_reassociate)


            # Handle truly new detections (not re-associations)
            for detected_bbox_index, (_x, _y, _w, _h) in enumerate(detected_car_locations):
                if detected_bbox_index not in reassociated_detection_indices: # If detection wasn't re-associated
                    x = int(_x)
                    y = int(_y)
                    w = int(_w)
                    h = int(_h)
                    x_bar = x + 0.5 * w
                    y_bar = y + 0.5 * h


                    matchCarID = None
                    for carID in carTracker.keys(): # Check against currently tracked cars (redundant now, but kept for clarity)
                        trackedPosition = carTracker[carID].get_position()
                        t_x = int(trackedPosition.left())
                        t_y = int(trackedPosition.top())
                        t_w = int(trackedPosition.width())
                        t_h = int(trackedPosition.height())
                        t_x_bar = t_x + 0.5 * t_w
                        t_y_bar = t_y + 0.5 * t_h

                        if ((t_x <= x_bar <= (t_x + t_w)) and (t_y <= y_bar <= (t_y + t_h)) and (x <= t_x_bar <= (x + w)) and (y <= t_y_bar <= (y + h))):
                            matchCarID = carID
                            break # No need to check further if matched

                    if matchCarID is None:
                        # Truly new car
                        # print ('Creating new tracker ' + str(currentCarID))
                        tracker = dlib.correlation_tracker()
                        tracker.start_track(image, dlib.rectangle(x, y, x + w, y + h))
                        carTracker[currentCarID] = tracker
                        carLocation1[currentCarID] = [x, y, w, h]
                        currentCarID += 1


        for carID in list(carTracker.keys()):
            trackedPosition = carTracker[carID].get_position()

            t_x = int(trackedPosition.left())
            t_y = int(trackedPosition.top())
            t_w = int(trackedPosition.width())
            t_h = int(trackedPosition.height())

            if t_w < MIN_BBOX_SIZE or t_h < MIN_BBOX_SIZE:
                carIDtoDelete.append(carID) # Will be handled in the next carIDtoDelete loop
                continue

            cv2.rectangle(resultImage, (t_x, t_y), (t_x + t_w, t_y + t_h), rectangleColor, 4)

            carLocation2[carID] = [t_x, t_y, t_w, t_h]

        end_time = time.time()

        if not (end_time == start_time):
            fps = 1.0/(end_time - start_time)


        for i in list(carLocation1.keys()):
            if frameCounter % 1 == 0:
                if i not in carLocation2:
                    continue
                [x1, y1, w1, h1] = carLocation1[i]
                [x2, y2, w2, h2] = carLocation2[i]

                carLocation1[i] = [x2, y2, w2, h2]

                if [x1, y1, w1, h1] != [x2, y2, w2, h2]:
                    speed[i] = estimateSpeed([x1, y1, w1, h1], [x2, y2, w2, h2])

                    if speed[i] != None and y1 >= image_y_threshhold and i not in reassociated_carIDs: # Dont print speed for re-associated cars immediately
                        cv2.putText(resultImage, str(int(speed[i]/1.609)) + " mph", (int(x1 + w1/2), int(y1-5)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
                        car_speeds_tk[i] = speed[i]
                        all_speeds_interval.append(speed[i])
                        #print(f"Car ID {i} speed calculated: {speed[i]:.2f} mph")
                    else:
                        car_speeds_tk[i] = None

        cv2.imshow('Live feed', resultImage)

        if cv2.waitKey(33) == 27:
            break

    cv2.destroyAllWindows()
    window.destroy()
    video.release()


window = tk.Tk()
window.title("Traffic information")
window.geometry("300x300")

count_label = ttk.Label(window, text=f"Cars Passed ({INTERVAL_SECONDS}s):")
count_label.pack(padx=10, pady=5)

car_count_text = tk.StringVar()
car_count_display = ttk.Label(window, textvariable=car_count_text)
car_count_display.pack(padx=10, pady=5)

avg_speed_interval_label = ttk.Label(window, text=f"Avg Speed of all cars ({INTERVAL_SECONDS}s):")
avg_speed_interval_label.pack(padx=10, pady=5)

avg_speed_text = tk.StringVar()
avg_speed_display = ttk.Label(window, textvariable=avg_speed_text)
avg_speed_display.pack(padx=10, pady=5)

status_label = ttk.Label(window, text=f"Traffic status ({INTERVAL_SECONDS}s):")
status_label.pack(padx=10, pady=5)
traffic_status_text = tk.StringVar()
traffic_status_display = ttk.Label(window, textvariable=traffic_status_text)
traffic_status_display.pack()


def start_tracking_thread():
    trackMultipleObjects(car_count_text, avg_speed_text)

tracking_thread = threading.Thread(target=start_tracking_thread)
tracking_thread.daemon = True
tracking_thread.start()

window.mainloop()

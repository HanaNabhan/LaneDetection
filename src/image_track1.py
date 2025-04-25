#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import random
import onnxruntime as ort
import time

bridge = CvBridge()
NUM = 0

w = "/home/abdelrahman/Downloads/weights.onnx"
session = ort.InferenceSession(w, providers=['CPUExecutionProvider'])

# Define ROI box dimensions (width and height)
ROI_POINTS = np.array([[430, 270], [540, 270], [780, 460], [200, 460]])
# Real-world sizes (in meters) for the objects
OBJECT_SIZES = {
    'Car': (2.0, 1.5),  # (width, height)
    'Human': (0.5, 1),
    'Cone': (1, 0.5)
}

# Camera parameters
FOCAL_LENGTH = 700  # Example focal length in pixels, you should calibrate your camera to get the exact value

# Dictionary to store the last distances and timestamps for each object
object_tracking = {}

# Global variable to store the closest object in ROI
closest_object_in_roi = None

def image_callBack(msg: Image):
    # Convert the ROS Image message to an OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # Get the dimensions of the image
    height, width, _ = cv_image.shape

    # Define the crop area (from top to slightly above the bottom)
    crop_height = height - 15  # Crop 15 pixels from the bottom (adjust as needed)

    # Perform the crop
    cv_image = cv_image[:crop_height, :]

    detect(cv_image, session)

    cv2.waitKey(1)

def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, r, (dw, dh)

def point_in_polygon(point, polygon):
    """Check if a point is inside a polygon using the ray-casting algorithm."""
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def detect(img, session):
    global object_tracking, closest_object_in_roi
    names = ['Car', 'Cone', 'Human']
    colors = {name: [random.randint(0, 255) for _ in range(3)] for name in names}

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    image = img.copy()
    image, ratio, dwdh = letterbox(image, auto=False)
    image = image.transpose((2, 0, 1))
    image = np.expand_dims(image, 0)
    image = np.ascontiguousarray(image)

    im = image.astype(np.float32)
    im /= 255

    outname = [i.name for i in session.get_outputs()]
    inname = [i.name for i in session.get_inputs()]
    inp = {inname[0]: im}

    # ONNX inference
    outputs = session.run(outname, inp)[0]

    ori_images = [img.copy()]

    current_time = time.time()

    min_distance_in_roi = float('inf')
    closest_object_in_roi = None  # Reset the closest object in ROI

    for i, (batch_id, x0, y0, x1, y1, cls_id, score) in enumerate(outputs):
        image = ori_images[int(batch_id)]
        box = np.array([x0, y0, x1, y1])
        box -= np.array(dwdh * 2)
        box /= ratio
        box = box.round().astype(np.int32).tolist()
        cls_id = int(cls_id)
        score = round(float(score), 3)
        name = names[cls_id]
        color = tuple(colors[name])  # Ensure color is a tuple
        name += ' ' + str(score)

        # Calculate centroid of the bounding box
        centroid_x = (box[0] + box[2]) // 2
        centroid_y = (box[1] + box[3]) // 2

        # Draw centroid as a small circle or point
        cv2.circle(image, (centroid_x, centroid_y), 3, (0, 255, 255), -1)

        # Calculate distance to the object
        obj_type = name.split()[0]
        obj_width, obj_height = OBJECT_SIZES[obj_type]  # Get the object's real-world size
        pixel_height = box[3] - box[1]  # Height of the bounding box in pixels
        distance = (obj_height * FOCAL_LENGTH) / pixel_height  # Distance calculation

        # Check if any side of the object's bounding box is within the ROI parallelogram
        box_points = [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]
        if any(point_in_polygon(point, ROI_POINTS) for point in box_points):
            # Print object type and distance in terminal if the object is in the ROI
            print(f"{obj_type}: {distance:.2f} m")

            # Draw the bounding box and labels if the object is in the ROI
            cv2.rectangle(image, tuple(box[:2]), tuple(box[2:]), color, 2)
            cv2.putText(image, name, (box[0], box[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (225, 255, 255), thickness=1)
            cv2.putText(image, f'Distance: {distance:.2f} m', (box[0], box[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), thickness=1)
            cv2.putText(image, "IN ROI", (box[0], box[1] - 2), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0, 255, 0), thickness=2, bottomLeftOrigin=True)


            # Calculate speed if the object is a Car
            if obj_type == 'Car':
                # Calculate and print speed if we have previous distance data
                if obj_type in object_tracking:
                    prev_distance, prev_time = object_tracking[obj_type]
                    delta_distance = distance - prev_distance
                    delta_time = current_time - prev_time
                    if delta_time > 0:
                        speed = delta_distance / delta_time
                        print(f"{obj_type} Speed: {speed:.2f} m/s")
                        # Display speed on the image
                        cv2.putText(image, f'Speed: {speed:.2f} m/s', (box[0], box[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), thickness=1)

                # Update the tracking dictionary with the current distance and time
                object_tracking[obj_type] = (distance, current_time)

            # Check if this object is the closest one in the ROI
            if distance < min_distance_in_roi:
                min_distance_in_roi = distance
                closest_object_in_roi = (obj_type, distance)

    # Draw the ROI parallelogram on the image
    cv2.polylines(image, [ROI_POINTS], isClosed=True, color=(255, 0, 0), thickness=2)

    # If there is a closest object in the ROI, print its details; otherwise, indicate that the road is clear
    if closest_object_in_roi:
        print(f"Closest object in ROI: {closest_object_in_roi[0]}, Distance: {closest_object_in_roi[1]:.2f} m")
        rospy.set_param("lane", 1)
    else:
        rospy.set_param("lane", 0)
        print("The road is clear")

    cv2.imshow("Image Window", image)
    cv2.waitKey(1)



if __name__ == "__main__":
    rospy.init_node("Image_node")
    rospy.Subscriber("/image", Image, image_callBack, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")

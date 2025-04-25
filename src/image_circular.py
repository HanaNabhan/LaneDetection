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
w = "/home/mazen/Desktop/Ever/Submission_3/workspace/src/sub_3_pkg/src/best.onnx"
session = ort.InferenceSession(w, providers=['CPUExecutionProvider'])

last_processed_time = time.time()  # Initialize last processed time

# Initialize VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/mazen/Desktop/output.avi', fourcc, 10.0, (640, 480))

def image_callBack(msg: Image):
    global last_processed_time

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return

    if cv_image is None:
        rospy.logwarn("Received invalid image (NoneType) from ROS message.")
        return

    current_time = time.time()
    elapsed_time = current_time - last_processed_time

    if elapsed_time >= 1:
        image_with_od, object_names = detect_objects(cv_image, session)

        if image_with_od is None:
            rospy.logwarn("Object detection did not produce a valid image.")
            return

        # Debugging: Print the type and shape of the image
        rospy.loginfo(f"image_with_od type: {type(image_with_od)}, shape: {image_with_od.shape}")

        # Display the image with detections
        cv2.imshow("Object Detection", image_with_od)
        cv2.waitKey(1)  # Display frame for 1 millisecond

        last_processed_time = current_time

        # Reset and update detection parameters based on detected objects
        rospy.set_param("/Human_detected", 0)
        rospy.set_param("/Car_detected", 0)
        rospy.set_param("/Cone_detected", 0)
        if object_names:
            for obj in object_names:
                if obj == "Human":
                    rospy.set_param("/Human_detected", 1)
                elif obj == "Car":
                    rospy.set_param("/Car_detected", 1)
                elif obj == "Cone":
                    rospy.set_param("/Cone_detected", 1)


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)

def detect_objects(img, session):
    names = ['Car', 'Human', 'Cone']
    colors = {name: [random.randint(0, 255) for _ in range(3)] for name in names}
    object_names_in_roi = []

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

    outputs = session.run(outname, inp)[0]

    ori_images = [img.copy()]
    
    # Parameters for the inner ellipse (ROI)
    inner_center_coordinates = (150, 700)
    inner_axes_length = (300, 100)
    angle = 180
    start_angle = 0
    end_angle = 180

    # Parameters for the outer ellipse (ROI)
    distance_between_ellipses = 300  # Distance in pixels, adjust as necessary
    outer_axes_length = (inner_axes_length[0] + distance_between_ellipses, inner_axes_length[1] + distance_between_ellipses)

    # Create empty masks with the same dimensions as the image
    mask_inner = np.zeros_like(ori_images[0], dtype=np.uint8)
    mask_outer = np.zeros_like(ori_images[0], dtype=np.uint8)

    # Draw the inner and outer ellipses (ROI) on the masks
    cv2.ellipse(mask_inner, inner_center_coordinates, inner_axes_length, angle, start_angle, end_angle, (255, 255, 255), thickness=cv2.FILLED)
    cv2.ellipse(mask_outer, inner_center_coordinates, outer_axes_length, angle, start_angle, end_angle, (255, 255, 255), thickness=cv2.FILLED)

    object_names_in_roi = []

    for i, (batch_id, x0, y0, x1, y1_box, cls_id, score) in enumerate(outputs):
        image = ori_images[int(batch_id)]
        box = np.array([x0, y0, x1, y1_box])
        box -= np.array(dwdh * 2)
        box /= ratio
        box = box.round().astype(np.int32).tolist()
        cls_id = int(cls_id)
        score = round(float(score), 3)
        name = names[cls_id]
        color = tuple(colors[name])
        name_with_score = name + ' ' + str(score)

        center = ((box[0] + box[2] + 5) // 2, (box[1] + box[3] + 5) // 2)

        # Draw the inner and outer ellipses on the image
        cv2.ellipse(image, inner_center_coordinates, inner_axes_length, angle, start_angle, end_angle, (255, 0, 0), thickness=2)
        cv2.ellipse(image, inner_center_coordinates, outer_axes_length, angle, start_angle, end_angle, (0, 0, 255), thickness=2)

        # Check if the center of the bounding box is within the region between the inner and outer ellipses
        if mask_outer[center[1], center[0]].all() and not mask_inner[center[1], center[0]].all():
            cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), color, 2)
            cv2.putText(image, name_with_score, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (225, 255, 255), thickness=2)
            cv2.circle(image, center, 5, (0, 255, 0), -1)
            cv2.putText(image, "Centroid", (center[0] + 10, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thickness=2)
            object_names_in_roi.append(name)

    return ori_images[0], object_names_in_roi

if __name__ == "__main__":
    rospy.init_node("Image_node")
    rospy.set_param("/Human_detected", 0)
    rospy.set_param("/Car_detected", 0)
    rospy.set_param("/Cone_detected", 0)

    rospy.Subscriber("/image", Image, image_callBack, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("Shutdown requested. Exiting...")
    
    cv2.destroyAllWindows()  # Close the window

    out.release()  # Release the VideoWriter when done

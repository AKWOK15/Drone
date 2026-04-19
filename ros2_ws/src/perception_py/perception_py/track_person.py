#!/usr/bin/env python3
import depthai as dai
import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from ultralytics import YOLO
# Model input size must match what the blob was compiled for
import numpy as np
import time
#from perception_py.fps_counter import FPSCounter
from fps_counter import FPSCounter
class Track(Node):
    def __init__(self):
        super().__init__('track')
        #Queue size is 10
        self.point_publisher = self.create_publisher(Point, 'topic', 10)
        

    #publisher
    
    def set_point(self, msg):
        self.point_publisher.publish(msg)
def resizeAndPad(img, size, padColor=0):

    h, w = img.shape[:2]
    sh, sw = size

    # interpolation method
    if h > sh or w > sw: # shrinking image
        interp = cv2.INTER_AREA
    else: # stretching image
        interp = cv2.INTER_CUBIC

    # aspect ratio of image
    aspect = w/h  # if on Python 2, you might need to cast as a float: float(w)/h

    # compute scaling and pad sizing
    if aspect > 1: # horizontal image
        new_w = sw
        new_h = np.round(new_w/aspect).astype(int)
        pad_vert = (sh-new_h)/2
        pad_top, pad_bot = np.floor(pad_vert).astype(int), np.ceil(pad_vert).astype(int)
        pad_left, pad_right = 0, 0
    elif aspect < 1: # vertical image
        new_h = sh
        new_w = np.round(new_h*aspect).astype(int)
        pad_horz = (sw-new_w)/2
        pad_left, pad_right = np.floor(pad_horz).astype(int), np.ceil(pad_horz).astype(int)
        pad_top, pad_bot = 0, 0
    else: # square image
        new_h, new_w = sh, sw
        pad_left, pad_right, pad_top, pad_bot = 0, 0, 0, 0

    # set pad color
    if len(img.shape) is 3 and not isinstance(padColor, (list, tuple, np.ndarray)): # color image but only one color provided
        padColor = [padColor]*3

    # scale and pad
    scaled_img = cv2.resize(img, (new_w, new_h), interpolation=interp)
    scaled_img = cv2.copyMakeBorder(scaled_img, pad_top, pad_bot, pad_left, pad_right, borderType=cv2.BORDER_CONSTANT, value=padColor)

    return scaled_img
    
def main(args=None):    
    rclpy.init(args=args)
    #create preview Video
    #mp4v
    #First value is width, second is height
    DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
    mag_width = 0.1
    mag_height = 0.05
    FPS = 10
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    #Record frames 
    out_preview = cv2.VideoWriter('track_person.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
    )
    out_cropped = cv2.VideoWriter('cropped_track_person.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
    gun_model_path = 'weapon_detection_2.0.pt'
    gun_model = YOLO(gun_model_path)
    fps_counter = FPSCounter()
    try:
        track = Track()
        x_goal = 0.0
        y_goal = 0.0
        z_goal = 0.0
        pipeline = dai.Pipeline()

        # RGB camera to detect guns
        rgb_cam = pipeline.create(dai.node.ColorCamera)
        rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb_cam.setPreviewSize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])  # must match DET_INPUT_SIZE
        #Interleaved ture would be pixel by pixel, so each pixel's RGB values are stored togehter
        #False would be all R, then all G, then all B - planar
        #Most NNs expect planar 
        rgb_cam.setInterleaved(False)
        rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # Mono cameras for stereo depth
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Stereo depth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        # HIGH_DENSITY specifically configures the stereo matching algorithm to maximize the number of valid depth pixels — it trades off some accuracy/range for denser coverage. The other common preset is HIGH_ACCURACY, which does the opposite.
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # APplies median filter to raw disparity map
        #Replaces raw value with median value of neighbros 
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

        #Linking
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        
        # Person detection model (spatial = gives x,y,z coordinates)
        person_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        person_detection_network.setConfidenceThreshold(0.2)
        person_detection_network.setBlobPath('/home/aidankwok/Drone/person_model/yolov8n_openvino_2022.1_6shave.blob')
        #Ignore anything closer than 100 mm
        person_detection_network.setDepthLowerThreshold(100)
        # person_detection_network.setDepthUpperThreshold(20000)
        # YOLO-specific — must match how your model was trained/exported
        person_detection_network.setNumClasses(1)           # adjust to your class count
        #how many values to descibe boudning box, x_center, y_center, width, heigh 
        person_detection_network.setCoordinateSize(4)
        person_detection_network.setIouThreshold(0.5)

        #Samples depth from center 50% of the bounding box
        person_detection_network.setBoundingBoxScaleFactor(0.5)

        #Allows overwriting in case frames pile up

        person_detection_network.input.setBlocking(False)

        #Outputs
        xout_frame = pipeline.createXLinkOut()
        xout_frame.setStreamName("raw_frame")

        xout_detection = pipeline.createXLinkOut()
        xout_detection.setStreamName("detection")
        

        #Linking
        rgb_cam.preview.link(person_detection_network.input)
        stereo.depth.link(person_detection_network.inputDepth)
        
        person_detection_network.passthrough.link(xout_frame.input)
        person_detection_network.out.link(xout_detection.input)
        point_msg = Point()
    # Run
        print('made it to pipeline')
        with dai.Device(pipeline) as device:
            device.setLogLevel(dai.LogLevel.DEBUG)
            device.setLogOutputLevel(dai.LogLevel.DEBUG)
            q_frame = device.getOutputQueue(name="raw_frame", maxSize=4, blocking=False)
            q_detection = device.getOutputQueue(name="detection", maxSize=4, blocking=False)
            while rclpy.ok():
                img_frame = q_frame.get()
                detections = q_detection.get()
                frame = img_frame.getCvFrame()


                 
                for detection in detections.detections:
                    # Denormalize bounding box to frame pixel coordinates
                    x1 = int(detection.xmin * frame.shape[1])
                    y1 = int(detection.ymin * frame.shape[0])
                    x2 = int(detection.xmax * frame.shape[1])
                    y2 = int(detection.ymax * frame.shape[0])

                    # Spatial coordinates in mm
                    x_mm = detection.spatialCoordinates.x
                    y_mm = detection.spatialCoordinates.y
                    z_mm = detection.spatialCoordinates.z

                    # Publish point
                    point_msg.x = x_mm
                    point_msg.y = y_mm
                    point_msg.z = z_mm
                    track.set_point(point_msg)

                   # Magnify bounding box
                   # temp_mag_x1 = int(x1 - (x1*mag_width)) 
                   # temp_mag_y1 = int(y1 - (y1*mag_height))
                   # temp_mag_x2 = int(x2 + (x2*mag_width)) 
                   # temp_mag_y2 = int(y2 + (y2*mag_height))
                    
                    ##Check boundary conditions
                    #mag_x1 = temp_mag_x1 if temp_mag_x1 >=0 else 0
                    #mag_y1 = temp_mag_y1 if temp_mag_y1 >=0 else 0
                    #
                    #mag_x2 = temp_mag_x2 if temp_mag_x2 <= DET_INPUT_SIZE[0] else DET_INPUT_SIZE[0]
                    #mag_y2 = temp_mag_y2 if temp_mag_y2 <= DET_INPUT_SIZE[1] else DET_INPUT_SIZE[1]


                    #
                    #cropped = frame[mag_y1:mag_y2, mag_x1:mag_x2]
                    #cropped = resizeAndPad(cropped, (640, 640))
                    #

                    #prediction = gun_model.predict(cropped)
                    #for result in prediction:
                    #    out_cropped.write(result.plot()) 
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw depth and confidence label
                    label = f"Person {detection.confidence:.0%} | Depth: {z_mm:.0f} mm"
                    cv2.putText(frame, label, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                    # Draw crosshair at center of box
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                    # Always write frame (even with no detections)
                    fps_counter.tick()
                    frame = fps_counter.draw(frame)
                    out_preview.write(frame)                
                    rclpy.spin_once(track, timeout_sec=0.05)
    except KeyboardInterrupt:
        track.get_logger().info('Flight interrupted by user')
    except Exception as e:
        track.get_logger().error(f'An error occurred: {e}')
    finally:
        out_preview.release()
        out_cropped.release()
        track.destroy_node()
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()

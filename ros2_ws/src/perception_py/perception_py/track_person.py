#!/usr/bin/env python3
import depthai as dai
import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
import blobconverter
# Model input size must match what the blob was compiled for

print(f'dai version: {dai.__version__}')
class Track(Node):
    def __init__(self):
        super().__init__('track')
        #Queue size is 10
        self.point_publisher = self.create_publisher(Point, 'topic', 10)
        
    
    #publisher
    
    def set_point(self, msg):
        self.point_publisher.publish(msg)
    
def main(args=None):    
    rclpy.init(args=args)
    #create preview Video
    #mp4v
    DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
    FPS = 30
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    #Record frames 
    out_preview = cv2.VideoWriter('track_person.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
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
        
#        # Gun detection mode
#        gun_detection_network = pipeline.create(dai.node.YoloDetectionNetwork)
#        gun_detection_network.setConfidenceThreshold(0.5)
#        gun_detection_network.setBlobPath(gun_blob_path)
#        # YOLO-specific — must match how your model was trained/exported
#        gun_detection_network.setNumClasses(1)           # adjust to your class count
#        #how many values to descibe boudning box, x_center, y_center, width, heigh 
#        gun_detection_network.setCoordinateSize(4)
#        # gun_detection_network.setAnchors([...])          # must match your model's anchors
#        # gun_detection_network.setAnchorMasks({...})      # must match your model's anchor masks
#        gun_detection_network.setIouThreshold(0.5)
#
#        #Samples depth from center 50% of the bounding box
#        gun_detection_network.setBoundingBoxScaleFactor(0.5)
#
#        #Allows overwriting in case frames pile up
#        gun_detection_network.input.setBlocking(False)
        # Person detection model (spatial = gives x,y,z coordinates)
        person_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        # blob = dai.OpenVINO.Blob("/home/aidankwok/Drone/person_model/yolov8n_openvino_2022.1_6shave.blob")
        # print('hello')
        # for name, tensor in blob.networkInputs.items():
        #     print(f"Input: {name}, shape: {tensor.dims}")
        # for name, tensor in blob.networkOutputs.items():
        #     print(f"Output: {name}, shape: {tensor.dims}")
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


                print(f"frame shape: {frame.shape}, dtype: {frame.dtype}, min: {frame.min()}, max: {frame.max()}")
                print(f'detections: {detections}')
                 
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

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw depth and confidence label
                    label = f"Person {detection.confidence:.0%} | Z: {z_mm:.0f}mm"
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # Draw crosshair at center of box
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                # Always write frame (even with no detections)

                out_preview.write(frame)                
                rclpy.spin_once(track, timeout_sec=0.05)
    except KeyboardInterrupt:
        track.get_logger().info('Flight interrupted by user')
    except Exception as e:
        track.get_logger().error(f'An error occurred: {e}')
    finally:
        out_preview.release()
        track.destroy_node()
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import depthai as dai
import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

# Model input size must match what the blob was compiled for

# #import pid
# from pid.pid import PID

class Track(Node):
    def __init__(self):
        super().__init__('track')
        #Queue size is 10
        self.point_publisher = self.create_publisher(Point, 'topic', 10)
        
    
    #publisher
    
    def set_point(x, y, z):
        self.point_publisher.publish(x, y, z)
    
def main(args=None):

    
    
    rclpy.init(args=args)
    #create preview Video
    #mp4v
    DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
    FPS = 10
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    #Record frames 
    out_preview = cv2.VideoWriter('preview_depth.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
    try:
        track = Track()
        x_goal = 0.0
        y_goal = 0.0
        z_goal = 0.0
        
        
        # Get blob (or set blob_path manually if you have one)
        blob_path = '/home/aidankwok/Drone/gun_model/weights-2_openvino_2022.1_6shave.blob'

        # Create the pipeline
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

        # Gun detection model (spatial = gives x,y,z coordinates)
        spatial_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        spatial_detection_network.setConfidenceThreshold(0.2)
        spatial_detection_network.setBlobPath(blob_path)
        #Ignore anything closer than 100 mm
        spatial_detection_network.setDepthLowerThreshold(100)
        spatial_detection_network.setDepthUpperThreshold(5000)
        # YOLO-specific — must match how your model was trained/exported
        spatial_detection_network.setNumClasses(1)           # adjust to your class count
        #how many values to descibe boudning box, x_center, y_center, width, heigh 
        spatial_detection_network.setCoordinateSize(4)
        # spatial_detection_network.setAnchors([...])          # must match your model's anchors
        # spatial_detection_network.setAnchorMasks({...})      # must match your model's anchor masks
        spatial_detection_network.setIouThreshold(0.5)

        #Samples depth from center 50% of the bounding box
        spatial_detection_network.setBoundingBoxScaleFactor(0.5)

        #Allows overwriting in case frames pile up
        spatial_detection_network.input.setBlocking(False)
        #Object tracking using Kalman filter and hungarian algo 
        object_tracker = pipeline.create(dai.node.ObjectTracker)
        object_tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)

        #Outputs
        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("preview")

        tracker_output = pipeline.create(dai.node.XLinkOut)
        tracker_output.setStreamName("tracklets")

        #Linking
        rgb_cam.preview.link(spatial_detection_network.input)
        stereo.depth.link(spatial_detection_network.inputDepth)

        #passthrough is just passing along image 
        #frame on which tracking will be performed 
        spatial_detection_network.passthrough.link(object_tracker.inputTrackerFrame)
        #frame on which input detection was done
        spatial_detection_network.passthrough.link(object_tracker.inputDetectionFrame)
        #detection info of that frame 
        spatial_detection_network.out.link(object_tracker.inputDetections)

        object_tracker.passthroughTrackerFrame.link(xoutRgb.input)
        object_tracker.out.link(tracker_output.input)

        while rclpy.ok():
        # Run
            print('made it to pipeline')
            with dai.Device(pipeline) as device:
                # device.setLogLevel(dai.LogLevel.DEBUG)
                # device.setLogOutputLevel(dai.LogLevel.DEBUG)
                q_preview = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
                q_tracklets = device.getOutputQueue(name="tracklets", maxSize=4, blocking=False)
                while True:
                    img_frame = q_preview.get()
                    tracklets = q_tracklets.get()
                    frame = img_frame.getCvFrame()
                    tracklets_data = tracklets.tracklets
                    for t in tracklets_data:
                        roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                        x1 = int(roi.topLeft().x)
                        y1 = int(roi.topLeft().y)
                        x2 = int(roi.bottomRight().x)
                        y2 = int(roi.bottomRight().y)
            
                        coord_x = t.spatialCoordinates.x
                        coord_y = t.spatialCoordinates.y
                        coord_z = t.spatialCoordinates.z
                        #track.set_point(x, y, z)
                        print(f'coord_x: {coord_x}')
                        print(f'coord_y: {coord_y}')
                        print(f'coord_z: {coord_z}')

                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame, f'Depth: {coord_z:.2f} mm', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), thickness=2)
                        
                    #callbacks like subscribers don't run automaitcally in background
                    #only run when you give executor chance to process them
                    
                    rclpy.spin_once(track, timeout_sec=0.05)
                    out_preview.write(frame)
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

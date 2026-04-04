#!/usr/bin/env python3
import depthai as dai
import cv2


# Model input size must match what the blob was compiled for
DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
FPS = 10
#create preview Video
#mp4v
fourcc = cv2.VideoWriter_fourcc(*'avc1')
#Record frames 
out_preview = cv2.VideoWriter('preview_depth.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
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



# ImageManip to resize frames to model input size
# REQUIRED even if you don't want other operations —
# the NN needs exactly the right input dimensions
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setResize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
manip.initialConfig.setKeepAspectRatio(False)
#Max frame size in bytes
manip.setMaxOutputFrameSize(1228800)

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

object_tracker = pipeline.create(dai.node.ObjectTracker)
object_tracker.setDetectionLabelsToTrack([0])
object_tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)

#Preview output
xoutRgb = pipeline.createXLinkOut()
xoutRgb.setStreamName("preview")

tracker_output = pipeline.create(dai.node.XLinkOut)
tracker_output.setStreamName("tracklets")

#Linking
rgb_cam.preview.link(spatial_detection_network.input)
object_tracker.passthroughTrackerFrame.link(xoutRgb.input)
object_tracker.out.link(tracker_output.input)

spatial_detection_network.passthrough.link(object_tracker.inputTrackerFrame)
spatial_detection_network.passthrough.link(object_tracker.inputDetectionFrame)
spatial_detection_network.out.link(object_tracker.inputDetections)

stereo.depth.link(spatial_detection_network.inputDepth)
     



# Run
print('made it to pipeline')
with dai.Device(pipeline) as device:
    device.setLogLevel(dai.LogLevel.DEBUG)
    device.setLogOutputLevel(dai.LogLevel.DEBUG)
    preview = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
    tracklets = device.getOutputQueue(name="tracklets", maxSize=2, blocking=False)

    try:
        print('try block')
        while True:
            print('looping')
            img_frame = preview.get()
            track = tracklets.get()
            print('past queues')
            frame = img_frame.getCvFrame()
            tracklets_data = track.tracklets
            for t in tracklets_data:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                print(f'type x1: {type(x1)}')
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)
                print(f'x1: {x1}')
                print(f'x2: {x2}')
    
                coord_x = t.spatialCoordinates.x
                coord_y = t.spatialCoordinates.y
                coord_z = t.spatialCoordinates.z
                
                print(f'coord_x: {coord_x}')
                print(f'coord_y: {coord_y}')
                print(f'coord_z: {coord_z}')
                #detections.xmin outputs float between 0 and 1
                # print(f'x_min: {x_min}')
                # print(f'y_min: {y_min}')
                # print(f'x_max: {x_max}')
                # print(f'y_max: {y_max}')

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'Depth: {coord_z:.2f} mm', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), thickness=2)
                
                
            out_preview.write(frame)
    except KeyboardInterrupt:
        pass
    finally:
        out_preview.release()
        print('done')
        
        

#!/usr/bin/env python3
import depthai as dai
import cv2


# Model input size must match what the blob was compiled for
DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
FPS = 30
#create preview Video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('preview.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
# Get blob (or set blob_path manually if you have one)
blob_path = '/home/aidankwok/Drone/gun_model/weights-2_openvino_2022.1_6shave.blob'

# Create the pipeline
pipeline = dai.Pipeline()

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
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# RGB camera to detect guns
rgb_cam = pipeline.create(dai.node.ColorCamera)
rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
rgb_cam.setPreviewSize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])  # must match DET_INPUT_SIZE
#Interleaved ture would be pixel by pixel, so each pixel's RGB values are stored togehter
#False would be all R, then all G, then all B - planar
#Most NNs expect planar 
rgb_cam.setInterleaved(False)
rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

# ImageManip to resize frames to model input size
# REQUIRED even if you don't want other operations —
# the NN needs exactly the right input dimensions
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setResize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
manip.initialConfig.setKeepAspectRatio(False)
#Max frame size in bytes
manip.setMaxOutputFrameSize(1228800)
rgb_cam.preview.link(manip.inputImage)

# Gun detection model (spatial = gives x,y,z coordinates)
gun_model = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
gun_model.setConfidenceThreshold(0.2)
gun_model.setBlobPath(blob_path)
#Ignore anything closer than 100 mm
gun_model.setDepthLowerThreshold(100)
gun_model.setDepthUpperThreshold(5000)
# YOLO-specific — must match how your model was trained/exported
gun_model.setNumClasses(1)           # adjust to your class count
#how many values to descibe boudning box, x_center, y_center, width, heigh 
gun_model.setCoordinateSize(4)
# gun_model.setAnchors([...])          # must match your model's anchors
# gun_model.setAnchorMasks({...})      # must match your model's anchor masks
gun_model.setIouThreshold(0.5)

#Inputs to gun model
manip.out.link(gun_model.input)
stereo.depth.link(gun_model.inputDepth)

# Detection output (gives bounding boxes)
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")         # fixed: was setStream()

# Depth out output (spatial xyz coords)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutNN.setStreamName("detections")

#Outputs of gun model
gun_model.out.link(xoutNN.input)   
gun_model.passthroughDepth.link(xoutDepth.input)

# Disparity output (optional, for visualization)
disparity_out = pipeline.create(dai.node.XLinkOut)
disparity_out.setStreamName("disparity")
stereo.disparity.link(disparity_out.input)

#Get preview 
preview_out = pipeline.createXLinkOut()
preview_out.setStreamName("preview")
rgb_cam.preview.link(preview_out.input)
# Run

with dai.Device(pipeline) as device:
    # device.setLogLevel(dai.LogLevel.TRACE)
    # device.setLogOutputLevel(dai.LogLevel.TRACE)
    q_det = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
    # q_disp = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    q_prev = device.getOutputQueue(name="preview", maxSize=1, blocking=False)
    while True:
        print('waiting for detection')
        #halts until it receives message from device 
        in_det = q_det.get()
        detections = in_det.detections
        preview_frame = q_prev.get()
        preview_frame = preview_frame.getCvFrame()
        
        print("made it to for loop")
        for detection in detections:
            coord_x = detection.spatialCoordinates.x
            coord_y = detection.spatialCoordinates.y
            coord_z = detection.spatialCoordinates.z
            x_min = max(0, int(detection.xmin))
            y_min = max(0, int(detection.ymin))
            
            x_max = min(int(detection.xmax), 1)
            y_max = min(int(detection.ymax), 1)

            # Calculate coordinates
            # x = int(xmin*DET_INPUT_SIZE[0])
            # y = int(ymin*DET_INPUT_SIZE[1])
            # w = int(xmax*DET_INPUT_SIZE[0]-xmin*DET_INPUT_SIZE[0])
            # h = int(ymax*DET_INPUT_SIZE[1]-ymin*DET_INPUT_SIZE[1])
            bbox = (x_min, y_min, x_max, y_max)
            cv2.rectangle(preview_frame, bbox, (0, 0, 255), 2)
            cv2.putText(preview_frame, f'Depth: {coord_z}mm', (int(x_min), int(y_max)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255))
            
            print(f"Detection confidence: {detection.confidence:.2f}, depth: {coord_z:.0f}mm")
            
            

        # Visualize disparity
        # in_disp = q_disp.get()
        # disp_frame = in_disp.getCvFrame()
        # disp_norm = (disp_frame * (255 / stereo.getMaxDisparity())).astype('uint8')
        # disp_colored = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
        # cv2.imwrite("disparity.png", disp_colored)
        
        #Get preview
        # height, width, channels = preview_frame.shape
        # print(f'height: {height}')
        # print(f'width: {width}')
        out.write(preview_frame)
        
    out.release()
    print('done')
    
    

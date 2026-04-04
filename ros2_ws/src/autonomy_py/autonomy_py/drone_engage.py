#!/usr/bin/env python3
import depthai as dai
import cv2


# Model input size must match what the blob was compiled for
DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob
FPS = 30
#create preview Video
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('preview.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
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

#Linking
rgb_cam.preview.link(manip.inputImage)
#Inputs to gun model
manip.out.link(gun_model.input)
stereo.depth.link(gun_model.inputDepth)




# Depth output 
# xoutDepth = pipeline.createXLinkOut()
# xoutDepth.setStreamName("depth")
# gun_model.passthroughDepth.link(xoutDepth.input)         

# Detection output (spatial xyz coords)
xoutNN = pipeline.createXLinkOut()
xoutNN.setStreamName("detections")
gun_model.out.link(xoutNN.input)   

#Preview output
preview_out = pipeline.createXLinkOut()
preview_out.setStreamName("preview")
rgb_cam.preview.link(preview_out.input)

# Disparity output (optional, for visualization)
# disparity_out = pipeline.create(dai.node.XLinkOut)
# disparity_out.setStreamName("disparity")
# stereo.disparity.link(disparity_out.input)

# Run

with dai.Device(pipeline) as device:
    device.setLogLevel(dai.LogLevel.DEBUG)
    device.setLogOutputLevel(dai.LogLevel.DEBUG)
    q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    # q_disp = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    q_prev = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
    detections = None
    while True:
        q_name = device.getQueueEvent()
        print(f'q_name: {q_name}')
        if q_name == 'detections':
            in_det = q_det.get()
            detections = in_det.detections
            #Send ROS message here
        if q_name == 'preview':
            
            preview_frame = q_prev.get()
            preview_frame = preview_frame.getCvFrame()
            if detections:
                for detection in detections:
                    coord_x = detection.spatialCoordinates.x
                    coord_y = detection.spatialCoordinates.y
                    coord_z = detection.spatialCoordinates.z
                    
                    print(f'coord_x: {coord_x}')
                    print(f'coord_y: {coord_y}')
                    print(f'coord_z: {coord_z}')
                    #detections.xmin outputs float between 0 and 1
                    x_min = max(0, int(detection.xmin * DET_INPUT_SIZE[0]))
                    y_min = max(0, int(detection.ymin * DET_INPUT_SIZE[1]))
                    x_max = min(DET_INPUT_SIZE[0], int(detection.xmax * DET_INPUT_SIZE[0]))
                    y_max = min(DET_INPUT_SIZE[1], int(detection.ymax * DET_INPUT_SIZE[1]))
                    # print(f'x_min: {x_min}')
                    # print(f'y_min: {y_min}')
                    # print(f'x_max: {x_max}')
                    # print(f'y_max: {y_max}')

                    cv2.rectangle(preview_frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
                    cv2.putText(preview_frame, f'Depth: {coord_z} mm', (int(x_min), int(y_max)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255))
                    
                    print(f"Detection confidence: {detection.confidence:.2f}, depth: {coord_z:.0f}mm")
            
            out.write(preview_frame)
        
        # print("made it to for loop")
        
            
            

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
    
        
    out.release()
    print('done')
    
    

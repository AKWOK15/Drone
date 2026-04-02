#!/usr/bin/env python3
import depthai as dai
import cv2

# Model input size must match what the blob was compiled for
DET_INPUT_SIZE = (640, 640)  # adjust to match your gun model blob

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

# RGB camera
rgb_cam = pipeline.create(dai.node.ColorCamera)
rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
rgb_cam.setPreviewSize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])  # must match DET_INPUT_SIZE
rgb_cam.setInterleaved(False)
rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

# ImageManip to resize frames to model input size
# REQUIRED even if you don't want other operations —
# the NN needs exactly the right input dimensions
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setResize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
manip.initialConfig.setKeepAspectRatio(False)
manip.setMaxOutputFrameSize(1228800)
rgb_cam.preview.link(manip.inputImage)

# Gun detection model (spatial = gives x,y,z coordinates)
gun_model = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
gun_model.setConfidenceThreshold(0.75)
gun_model.setBlobPath(blob_path)
gun_model.setDepthLowerThreshold(100)
gun_model.setDepthUpperThreshold(5000)
# YOLO-specific — must match how your model was trained/exported
gun_model.setNumClasses(1)           # adjust to your class count
gun_model.setCoordinateSize(4)
# gun_model.setAnchors([...])          # must match your model's anchors
# gun_model.setAnchorMasks({...})      # must match your model's anchor masks
gun_model.setIouThreshold(0.5)

# Detection output (gives bounding boxes + spatial xyz coords)
det_out = pipeline.create(dai.node.XLinkOut)
det_out.setStreamName("det_out")         # fixed: was setStream()
gun_model.out.link(det_out.input)        # fixed: was undefined 'out'

# Disparity output (optional, for visualization)
disparity_out = pipeline.create(dai.node.XLinkOut)
disparity_out.setStreamName("disparity")
stereo.disparity.link(disparity_out.input)

#Get preview 
preview_out = pipeline.createXLinkOut()
preview_out.setStreamName("preview")
rgb_cam.preview.link(preview_out.input)
# Run
print("made it to pipeline")
with dai.Device(pipeline) as device:
    q_det = device.getOutputQueue(name="det_out", maxSize=1, blocking=False)
    q_disp = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    q_prev = device.getOutputQueue(name="preview", maxSize=1, blocking=False)

    # in_det = q_det.get()
    # detections = in_det.detections
    print("made it to for loop")
    # for detection in detections:
    #     coord_z = detection.spatialCoordinates.z
    #     print(f"Detection confidence: {detection.confidence:.2f}, depth: {coord_z:.0f}mm")

    # Visualize disparity
    # in_disp = q_disp.get()
    # disp_frame = in_disp.getCvFrame()
    # disp_norm = (disp_frame * (255 / stereo.getMaxDisparity())).astype('uint8')
    # disp_colored = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
    # cv2.imwrite("disparity.png", disp_colored)
    
    #Get preview
    preview_frame = q_prev.get()
    preview_frame = preview_frame.getCvFrame()
    cv2.imwrite("preview.png", preview_frame)
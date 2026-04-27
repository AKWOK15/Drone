#!/usr/bin/env python3
import depthai as dai
import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from ultralytics import YOLO
import numpy as np
import time
from fps_counter import FPSCounter
import numpy as np
from inference_sdk import InferenceHTTPClient 

class Track(Node):
	def __init__(self):
		#Node name
		super().__init__('track_person')
		self.point_publisher = self.create_publisher(Point, 'topic', 10)

	def set_point(self, msg):
		self.point_publisher.publish(msg)

def resizeAndPad(img, size, padColor=0):
	h, w = img.shape[:2]
	sh, sw = size

	interp = cv2.INTER_AREA if (h > sh or w > sw) else cv2.INTER_CUBIC

	aspect = w / h

	if aspect > 1:
		new_w = sw
		new_h = np.round(new_w / aspect).astype(int)
		scale = sw / w
		pad_vert = (sh - new_h) / 2
		pad_top = np.floor(pad_vert).astype(int)
		pad_bot = np.ceil(pad_vert).astype(int)
		pad_left, pad_right = 0, 0
	elif aspect < 1:
		new_h = sh
		new_w = np.round(new_h * aspect).astype(int)
		scale = sh / h
		pad_horz = (sw - new_w) / 2
		pad_left = np.floor(pad_horz).astype(int)
		pad_right = np.ceil(pad_horz).astype(int)
		pad_top, pad_bot = 0, 0
	else:
		new_h, new_w = sh, sw
		scale = sh / h
		pad_left, pad_right, pad_top, pad_bot = 0, 0, 0, 0

	if len(img.shape) == 3 and not isinstance(padColor, (list, tuple, np.ndarray)):
		padColor = [padColor] * 3

	scaled_img = cv2.resize(img, (new_w, new_h), interpolation=interp)
	scaled_img = cv2.copyMakeBorder(scaled_img, pad_top, pad_bot, pad_left, pad_right,
									borderType=cv2.BORDER_CONSTANT, value=padColor)

	return scaled_img, pad_left, pad_top, scale

def main(args=None):
	rclpy.init(args=args)

	DET_INPUT_SIZE = (640, 640)
	mag_width = 0.05
	mag_height = 0.0
	FPS = 10
	fourcc = cv2.VideoWriter_fourcc(*'avc1')
	out_preview = cv2.VideoWriter('track_person.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
	out_cropped = cv2.VideoWriter('cropped_track_person.mp4', fourcc, FPS, (DET_INPUT_SIZE[0], DET_INPUT_SIZE[1]))
	# 2. Connect to your workspace
	client = InferenceHTTPClient(
	api_url="https://serverless.roboflow.com",
	api_key="1dUvrWAbdffrk9p2hfU0"
	)
	fps_counter = FPSCounter()

	try:
		track = Track()
		x_goal = 0.0
		y_goal = 0.0
		z_goal = 0.0
		pipeline = dai.Pipeline()

		rgb_cam = pipeline.create(dai.node.ColorCamera)
		rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
		rgb_cam.setPreviewSize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
		rgb_cam.setInterleaved(False)
		rgb_cam.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
		rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

		mono_left = pipeline.create(dai.node.MonoCamera)
		mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
		mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

		mono_right = pipeline.create(dai.node.MonoCamera)
		mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
		mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

		stereo = pipeline.create(dai.node.StereoDepth)
		stereo.setLeftRightCheck(True)
		stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
		stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
		stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

		rotate_stereo_manip = pipeline.createImageManip()
		rotate_stereo_manip.initialConfig.setVerticalFlip(True)
		rotate_stereo_manip.initialConfig.setHorizontalFlip(True)
		rotate_stereo_manip.setFrameType(dai.ImgFrame.Type.RAW16)
		rotate_stereo_manip.setMaxOutputFrameSize(4147200)		

		mono_left.out.link(stereo.left)
		mono_right.out.link(stereo.right)

		person_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
		person_detection_network.setConfidenceThreshold(0.2)
		person_detection_network.setBlobPath('/home/aidankwok/Drone/person_model/yolov8n_openvino_2022.1_6shave.blob')
		person_detection_network.setDepthLowerThreshold(100)
		person_detection_network.setNumClasses(1)
		person_detection_network.setCoordinateSize(4)
		person_detection_network.setIouThreshold(0.5)
		person_detection_network.setBoundingBoxScaleFactor(0.5)
		person_detection_network.input.setBlocking(False)

		# Object tracker
		object_tracker = pipeline.create(dai.node.ObjectTracker)
		object_tracker.setDetectionLabelsToTrack([0])
		object_tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
		object_tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

		# Outputs
		xout_frame = pipeline.create(dai.node.XLinkOut)
		xout_frame.setStreamName("preview")

		tracker_output = pipeline.create(dai.node.XLinkOut)
		tracker_output.setStreamName("tracklets")

		# Linking
		rgb_cam.preview.link(person_detection_network.input)
		stereo.depth.link(rotate_stereo_manip.inputImage)
		rotate_stereo_manip.out.link(person_detection_network.inputDepth)

		# Frame on which tracking will be performed
		person_detection_network.passthrough.link(object_tracker.inputTrackerFrame)
		# Frame on which input detection was done
		person_detection_network.passthrough.link(object_tracker.inputDetectionFrame)
		# Detection info of that frame
		person_detection_network.out.link(object_tracker.inputDetections)

		object_tracker.passthroughTrackerFrame.link(xout_frame.input)
		object_tracker.out.link(tracker_output.input)

		point_msg = Point()

		print('made it to pipeline')
		with dai.Device(pipeline) as device:
			q_preview = device.getOutputQueue(name="preview", maxSize=4, blocking=False)
			q_tracklets = device.getOutputQueue(name="tracklets", maxSize=4, blocking=False)

			while rclpy.ok():
				img_frame = q_preview.get()
				tracklets = q_tracklets.get()
				frame = img_frame.getCvFrame()
				tracklets_data = tracklets.tracklets

				for t in tracklets_data:
					if t.status == dai.Tracklet.TrackingStatus.LOST:
						continue
					if t.status == dai.Tracklet.TrackingStatus.REMOVED:
						continue

					roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
					x1 = int(roi.topLeft().x)
					y1 = int(roi.topLeft().y)
					x2 = int(roi.bottomRight().x)
					y2 = int(roi.bottomRight().y)

					x_mm = t.spatialCoordinates.x
					y_mm = t.spatialCoordinates.y
					z_mm = t.spatialCoordinates.z

					point_msg.x = x_mm
					point_msg.y = y_mm
					point_msg.z = z_mm
					track.set_point(point_msg)
					print(f'id: {t.id}')

					# Draw bounding box
					cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

					# Draw track ID and depth
					label = f"ID:{t.id} | Depth: {z_mm:.0f} mm"
					cv2.putText(frame, label, (x1, y1 - 10),
								cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

					# Draw crosshair at center of box
					cx = (x1 + x2) // 2
					cy = (y1 + y2) // 2
					cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

					# Expand bounding box for gun detection crop
					temp_mag_x1 = int(x1 - (x1 * mag_width))
					temp_mag_y1 = int(y1 - (y1 * mag_height))
					temp_mag_x2 = int(x2 + (x2 * mag_width))
					temp_mag_y2 = int(y2 - (y2 - y1)//2)

					# Clamp to frame boundaries
					mag_x1 = max(temp_mag_x1, 0)
					mag_y1 = max(temp_mag_y1, 0)
					mag_x2 = min(temp_mag_x2, DET_INPUT_SIZE[0])
					mag_y2 = min(temp_mag_y2, DET_INPUT_SIZE[1])

					cropped = frame[mag_y1:mag_y2, mag_x1:mag_x2]

					# Resize + pad crop to 640x640; returns pad offsets and scale factor
					cropped_padded, pad_left, pad_top, scale = resizeAndPad(cropped, (640, 640))

					result = client.run_workflow(
						workspace_name="aidans-workspace-dqphd",
						workflow_id="general-segmentation-api-5",
						images={"image": cropped_padded},
						parameters={"classes": "Gun"},
						use_cache=True
					)                  
														
					predictions = result[0]['predictions']
										
					for pred in predictions.get("predictions"):
						# if len(result.boxes) == 0:
						# 	continue
						# box = result.boxes[0]
						# bb_x1, bb_y1, bb_x2, bb_y2 = map(int, box.xyxy[0])
						# out_cropped.write(result.plot())
						x, y, w, h = int(pred['x']), int(pred['y']), int(pred['width']), int(pred['height'])
						bb_x1, bb_y1 = x - w//2, y - h//2
						bb_x2, bb_y2 = x + w//2, y + h//2
						# Step 1: Remove padding offset (same top-left offset for all corners)
						np_x1 = bb_x1 - pad_left
						np_y1 = bb_y1 - pad_top
						np_x2 = bb_x2 - pad_left
						np_y2 = bb_y2 - pad_top

						# Step 2: Undo resize scale to get coords in crop-local space
						nr_x1 = int(np_x1 / scale)
						nr_y1 = int(np_y1 / scale)
						nr_x2 = int(np_x2 / scale)
						nr_y2 = int(np_y2 / scale)

						# Step 3: Translate from crop-local to original frame space
						og_x1 = nr_x1 + mag_x1
						og_y1 = nr_y1 + mag_y1
						og_x2 = nr_x2 + mag_x1
						og_y2 = nr_y2 + mag_y1

						cv2.rectangle(frame, (og_x1, og_y1), (og_x2, og_y2), (0, 0, 255), 3)

				# Always write frame (even with no detections) - outside for loop
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

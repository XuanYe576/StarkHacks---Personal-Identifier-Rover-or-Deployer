import argparse
from os import path
import sys
from time import perf_counter
from typing import Optional, Union

import cv2
import numpy as np

from api.deepsort import DeepSORTTracker
from api.yolo import YOLOPersonDetector

# constants
YOLO_MODEL = "./checkpoints/yolov7x.pt"
REID_MODEL = "./checkpoints/ReID.pb"
MAX_COS_DIST = 0.5
MAX_TRACK_AGE = 100


def camera_backends():
    if sys.platform == "darwin":
        return [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
    if sys.platform.startswith("win"):
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_ANY]


def video_output_codecs():
    if sys.platform == "darwin":
        return ["avc1", "mp4v"]
    if sys.platform.startswith("win"):
        return ["mp4v", "XVID", "avc1"]
    return ["avc1", "mp4v", "XVID"]


def video_writer_same_codec(video: cv2.VideoCapture, save_path: str) -> cv2.VideoWriter:
    """
    This function returns a VideoWriter object with the same codec as the input VideoCapture object
    """
    w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(video.get(cv2.CAP_PROP_FPS))
    fps = fps if fps > 0 else 30
    for codec_name in video_output_codecs():
        codec = cv2.VideoWriter_fourcc(*codec_name)
        writer = cv2.VideoWriter(save_path, codec, fps, (w, h))
        if writer.isOpened():
            return writer
        writer.release()
    raise RuntimeError("Unable to create VideoWriter with supported codecs")


def parse_source(source: str) -> Union[str, int]:
    source = source.strip()
    if source.lower() == "auto":
        return "auto"
    try:
        return int(source)
    except ValueError:
        return source


def open_video_source(source: Union[str, int], max_devices: int = 8) -> Optional[cv2.VideoCapture]:
    if source == "auto":
        for idx in range(max_devices - 1, -1, -1):
            for backend in camera_backends():
                cap = cv2.VideoCapture(idx, backend)
                if cap.isOpened():
                    print(f"Auto-selected camera index {idx} (backend={backend})")
                    return cap
                cap.release()
        return None

    if isinstance(source, int):
        for backend in camera_backends():
            cap = cv2.VideoCapture(source, backend)
            if cap.isOpened():
                return cap
            cap.release()
        return None

    cap = cv2.VideoCapture(source, cv2.CAP_ANY)
    if cap.isOpened():
        return cap
    cap.release()
    return None


def track_people(source: Union[str, int], save_path: Optional[str]):
    """
    Main function which implements the pipeline:
     1. Reads images from an input video stream
     2. Get detections of people in the input frame using YOLO
     3. Processes the detections along with previous tracks using DeepSORT
     4. Each output frame with refined bounding boxes to an output video stream
    """
    global YOLO_MODEL, REID_MODEL, MAX_COS_DIST, MAX_TRACK_AGE

    # initialize Yolo person detector and DeepSORT tracker
    detector = YOLOPersonDetector()
    detector.load(YOLO_MODEL)
    tracker = DeepSORTTracker(REID_MODEL, MAX_COS_DIST, MAX_TRACK_AGE)

    # initialize video stream objects
    video = open_video_source(source)
    if video is None:
        raise RuntimeError(f"Unable to open video source: {source}")

    output = None
    if save_path:
        output = video_writer_same_codec(video, save_path)

    # core processing loop
    frame_i = 0
    time_taken = 0
    while True:
        start = perf_counter()

        # read input video frame
        ret, frame = video.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # process YOLO detections
        detections = detector.detect(frame)
        try:
            bboxes, scores, _ = np.hsplit(detections, [4, 5])
            bboxes[:, 2:] = bboxes[:, 2:] - bboxes[:, :2]
            n_objects = detections.shape[0]
        except ValueError:
            bboxes = np.empty(0)
            scores = np.empty(0)
            n_objects = 0

        # track targets by refining with DeepSORT
        tracker.track(frame, bboxes, scores.flatten())

        # write to output video
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if output is not None:
            output.write(frame)

        # calculate FPS and display output frame
        frame_time = perf_counter() - start
        cv2.imshow("Detections", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("<< User has terminated the process >>")
            break
        time_taken += frame_time
        frame_i += 1
        print(
            f"Frame {frame_i}: "
            f"{n_objects} people - {int(frame_time*1000)} ms = {1/frame_time:.2f} Hz"
        )

    # print performance metrics
    print(
        f"\nTotal frames processed: {frame_i}"
        f"\nVideo processing time: {time_taken:.2f} s"
        f"\nAverage FPS: {frame_i/time_taken:.2f} Hz"
    )
    video.release()
    if output is not None:
        output.release()
    cv2.destroyWindow("Detections")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Track and ID People in a video",
        description="Use Yolov7 for detecting people in a video, assign IDs to detected"
        " people and track them as long as they are visible",
    )
    parser.add_argument(
        "--input-vid",
        type=str,
        default="./data/input.mp4",
        help="legacy alias for --source",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="",
        help='video source: file path, camera index (e.g. "0"), device path, or "auto"',
    )
    parser.add_argument(
        "--save-path",
        type=str,
        default="",
        help="path to save output video (optional). Leave empty for display only.",
    )

    args = parser.parse_args()
    source_arg = args.source if args.source else args.input_vid
    parsed_source = parse_source(source_arg)
    save_path = path.abspath(args.save_path) if args.save_path else None
    if isinstance(parsed_source, str) and parsed_source not in ("auto",) and not str(parsed_source).isdigit():
        parsed_source = path.abspath(parsed_source)
    start = perf_counter()

    # main pipeline
    track_people(parsed_source, save_path)

    print(f"Total time: {perf_counter()-start:.2f} s")

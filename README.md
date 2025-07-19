# Aerial Human Detection

This repository contains a Dockerized setup for **running YOLO-based human detection** on a Jetson Xavier using a **RealSense camera**.
> *Docker setup rn is not functional, use this ropo locally in xavier*


### 📜 Scripts available to run 
- `camera_feed_save.py` – Records raw-RGB feed of realsense to `raw_camera_output/raw_camera_output_year_month_day_hour_minute_seconds.mp4`.
- `cuda_test.py` – Verifies CUDA availability
- `live_yolo_display_output.py` – Runs YOLO and displays annotated frames in a window. (Not useful in ssh)
- `live_yolo_publish_output.py` – Publishes YOLO bounding boxes to a ROS topic which is `yolo_bounding_boxes`.
- `live_yolo_save_output.py` – Saves YOLO-annotated mp4 video output to `yolo_output/yolo_output_year_month_day_hour_minute_seconds.mp4`
---

### ⚙️ Setup & Usage

1. **Clone the repo**

   ```bash
   git clone https://github.com/Srindot/aerial-human-detection.git
   cd aerial-human-detection
   ```

2. **Open in VS Code** (`Not working, run it locally without docker`)

   - Press `Ctrl+Shift+P` → **Rebuild and Reopen in Container**
   - > Make sure to ssh into the xavier using vscode for the below step

3. **Run Scripts**

Open the Command Palette (`Ctrl+Shift+P`) → **Run Task**

Select and run any script listed above
   - `Session 1: roscore`: to run the roscore
   - `Session 2: launch realsense camera `: To start the realsense sdk to publish onto the ros topics
   - `Session 3: camera_feed_save`: To save the raw rgb feed of realsense as mp4 video.
   - `Session 4: live_yolo_save_output`: To save the rgb feed with annotation from the yolo to a mp4 file.
   - `Session 5: live_yolo_publish_output`: To publish the annotations only to a ros topic.
   - `Session 6: live_yolo_display_output`: To display the annotations and the rgb feed from the realsense real time in a window.



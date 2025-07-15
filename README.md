# Aerial Human Detection

This repository contains a Dockerized setup for **running YOLO-based human detection** on a Jetson Xavier using a **RealSense camera**.

### 📜 Scripts

- `camerafeed_record.py` – Records RGB feed to `.mp4`
- `cuda_test.py` – Verifies CUDA availability
- `live_yolo_display_output.py` – Runs YOLO and displays annotated frames
- `live_yolo_publish_output.py` – Publishes YOLO bounding boxes to a ROS topic
- `live_yolo_save_output.py` – Saves YOLO-annotated video output
- `save_raw_camera_feed.py` – Saves raw RGB feed

---

### ⚙️ Setup & Usage

1. **Clone the repo**

   ```bash
   git clone https://github.com/Srindot/aerial-human-detection.git
   cd aerial-human-detection
   ```

2. **Open in VS Code**

   - Press `Ctrl+Shift+P` → **Rebuild and Reopen in Container**

3. **Run Scripts**

   - Open the Command Palette (`Ctrl+Shift+P`) → **Run Task**
   - Select and run any script listed above

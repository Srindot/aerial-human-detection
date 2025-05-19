Model choosen:

1. **YOLOv8**
    
    - **Why**: Achieves 79% F1-score and 75% precision in human detection ([2](https://journal-isi.org/index.php/isi/article/view/944)), outperforming YOLOv5 while maintaining real-time capabilities.
        
    - **Tracking Integration**: Works seamlessly with Ultralytics' multi-object tracking API ([3](https://docs.ultralytics.com/modes/track/)), supporting BoT-SORT and ByteTrack algorithms.
        
    - **Variants**: Start with **YOLOv8s** (small) for testing, then scale to **YOLOv8m/l/x** if more accuracy is needed.

## Training Strategy

1. **Start with YOLOv8n** (nano) for rapid prototyping.
    
2. **Gradually scale up** to larger models (medium/large) if detection accuracy is insufficient.
    
3. **Use Roboflow** (mentioned in [2](https://journal-isi.org/index.php/isi/article/view/944)) for dataset preprocessing – it supports YOLO format and auto-augmentation.
    

## Critical Tips

- Enable **Mosaic augmentation** to simulate varied viewpoints during training.
    
- Fine-tune using **AdamW optimizer** with **Cosine learning rate decay** (standard in Ultralytics).
    
- For tracking, configure the `tracker_type` parameter in Ultralytics to match your scene density ([3](https://docs.ultralytics.com/modes/track/)).

## Setting up YOLO
Follow this to set it up [set up](Setting up env for YOLO)



  

## **Pipeline Overview**

1. **Install Required Packages**
    
    - `pyrealsense2` (RealSense SDK for Python)
        
    - `opencv-python` (for image processing and display)
        
    - `ultralytics` (for YOLOv8)
        
    - (Optional) Any additional dependencies for your platform
        
2. **Initialize RealSense Camera**
    
    - Start the RealSense pipeline to stream RGB frames.
        
3. **Load YOLOv8 Model**
    
    - Load a pre-trained YOLOv8 model (or your custom-trained weights).
        
4. **Process Frames in Real Time**
    
    - Capture frames from the RealSense camera.
        
    - Pass each frame to YOLOv8 for inference.
        
    - Draw bounding boxes and labels on detected persons.
        
5. **(Optional) Use Depth Data**
    
    - If you need distance information, retrieve and process the depth frame aligned with the RGB frame.
        
6. **Display or Use Results**
    
    - Show the annotated video stream or use the detections for further logic.



## How to Detect Only Persons with YOLOv8

**1. Use Class Filtering at Inference**

- When running inference, you can specify a class filter so the model only returns detections for the "person" class.
    
- In YOLOv8, the "person" class is typically class 0 in the COCO dataset.
    
- If you use the Ultralytics Python API, you can use the `classes` argument:
    

python

`results = model(source, classes=[0])  # Only detect 'person'`

**2. Train a Custom Model for Persons Only**

- For even better efficiency, you can fine-tune or retrain YOLOv8 on a dataset that only contains the "person" class.
    
- This reduces model size and inference complexity, as the model learns to ignore all other classes.
    
- Use a dataset with only person annotations, and set your YAML config to include just one class (`names: ['person']`).
    

**3. Post-Processing Filter**

- If you can't change the model or training, filter the results after inference to keep only detections with the "person" class ID.
    

**4. Use a Smaller Model Variant**

- For further speed gains, use a lighter YOLOv8 model (like `yolov8n` or `yolov8s`) since you only care about a single class[1](https://yolov8.org/how-to-make-yolov8-faster/)[2](https://keylabs.ai/blog/maximizing-object-detection-yolov8-performance-tips/)[7](https://docs.ultralytics.com/tasks/detect/).
    

## Benefits

- **Reduced computation:** The model spends less time evaluating unnecessary classes.
    
- **Faster inference:** Filtering out other classes streamlines processing.
    
- **Cleaner results:** Only relevant detections are returned, making downstream processing simpler.
    

## Summary Table

| Method                        | How It Works                                 | When to Use                           |
| ----------------------------- | -------------------------------------------- | ------------------------------------- |
| Class filtering at inference  | Use `classes=` in YOLOv8 API                 | Quick, no retraining needed           |
| Custom training (person only) | Train on person-only dataset                 | Maximum efficiency                    |
| Post-processing filter        | Remove non-person detections after inference | If model/class filtering not possible |
| Smaller model variant         | Use `yolov8n`/`yolov8s`                      | When hardware is limited              |




Some results 

default model with no fine tuning on a random picture of aerial view of people 
![[Pasted image 20250519041210.png]]
Issues, cannot detect cluttered people
Another have to test, how it works exactly over the person

## For real time yolo 
install this in the conda env 

```bash
	pip install ultralytics opencv-python pyrealsense2
```

and then run the realtime_yolo.py

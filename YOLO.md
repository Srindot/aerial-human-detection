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

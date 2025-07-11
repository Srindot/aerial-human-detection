
Some results 

default model with no fine tuning on a random picture of aerial view of people 
![[Pasted image 20250519041210.png]]
Issues, cannot detect cluttered people
Another have to test, how it works exactly over the person




**if you want your YOLOv8 model to detect people from both top and front (or any) angles, your fine-tuning dataset must include examples of both perspectives**. This ensures the model learns to generalize and does not "forget" how to detect people from previously learned viewpoints.

---

## Why is this important?

- **Catastrophic Forgetting:** If you fine-tune only on top-view images, the model adapts to that perspective and loses its ability to detect people from the front or other angles[5](https://blog.roboflow.com/how-to-train-yolov8-on-a-custom-dataset/).
    
- **Generalization:** Including a variety of perspectives (top, front, side, etc.) in your training/fine-tuning set helps the model generalize and perform well in all those scenarios[1](https://github.com/J3lly-Been/YOLOv8-HumanDetection)[5](https://blog.roboflow.com/how-to-train-yolov8-on-a-custom-dataset/).
    
- **Annotation Consistency:** Make sure your bounding boxes are consistent across all perspectives (e.g., always annotate the full body, not just the head or face), so the model learns the correct object extent.


## Yolo Fine tuning 

The YOLO algorithm has been fine tuned, to detect people from overhead view.
The data set which was used is for fine tuning is the first one in the [here](Datasets).
The final error was:
![[Pasted image 20250522160951.png]]
The results:
- normal view :
![[Pasted image 20250522161013.png]]


- overhead view:
![[Pasted image 20250522161046.png]]

So, the model works well in the overhead but loses all the memory of detecting from ahead.
Another issue is the height is not sufficient, and requires fine tuning from irl data.
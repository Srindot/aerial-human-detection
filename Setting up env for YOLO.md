
To set up yolo you have to run the following command

First, set up conda in your device

Then, create a new environment
```bash
conda create -n yolo python=3.8
```

Then, activate the env
and run this command to setup YOLO with cuda support
```bash
conda install -c pytorch -c nvidia -c conda-forge pytorch torchvision pytorch-cuda=11.8 ultralytics
```

Run the following command to check whether Yolo is setup or not:
```python 
import ultralytics

print("Ultralytics version:", ultralytics.__version__)
```

After that run an example from an online dataset: 

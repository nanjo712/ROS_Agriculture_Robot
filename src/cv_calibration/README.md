# YOLOv8 花瓶检测模型

+ capture_driver.py为相机驱动，内含去畸变代码

script文件夹中有关YOLO模型代码的说明

+ openvino_yolo.py为Ultralytics提供的使用内置OpenVINO接口优化推理的代码
+ openvino—test.py为使用OpenVINO接口手动实现图像预处理后处理并由YOLOv8推理的代码
+ test.py为Ultralytics的裸YOLOv8推理代码

前两者无明显性能差距，推理时间相对于test.py提高了20ms/帧，建议使用openvino_yolo.py
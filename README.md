kinect-test
===========

Test Kinect cameras using OpenCV and OpenNI2

usage:

```
args: none are required
    dev_id: the device id number. Typically 1600.
    min_distance: minimum distance to capture depth data in mm
    max_distance: max distance to capture depth data in mm
    quantization: use grayscale quantization or color quantization (more depth info), 0 or 1

knktest [dev_id] [min_distance] [max_distance] [quantization]
knktest 1600 725 10000 0
```

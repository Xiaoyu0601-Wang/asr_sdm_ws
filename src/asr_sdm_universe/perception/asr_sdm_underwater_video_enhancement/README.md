# Under Water Video Enhancement

## Overview
该算法基于asr_sdm_video_enhancement进行了改进，针对水下视频做了特别处理，大幅提高了图像处理速度。

## Dependencies
- rclcpp
- cv_bridge
- OpenCV
- sensor_msgs

## Run
为了确保参数正确传入，请通过launch文件启动该节点。

`ros2 launch asr_sdm_underwater_video_enhancement asr_sdm_underwater_video_enhancement.launch.xml`

## Paremeters
可以在config文件中设置以下参数。
| name                  | default           | type | description
| --------------------- | ----------------- | ---- | -------------------------------------------------------------------- |
| airlight              | `100.0`           | -    | 环境光; 该数值越大, 复原后的图像辉度越低, 反之除掉的噪音也越多.
| beta_b                | `0.5`             | -    | 蓝光在水中的衰减程度.
| beta_g                | `1.0`             | -    | 绿光在水中的衰减程度.
| beta_r                | `2.0`             | -    | 红光在水中的衰减程度.
| is_show_fps           | `true`            | bool | 用于设置是否在处理后的图像上显示帧数. 在代码调试阶段可以设置为真, 会占用一定的CPU资源.
| is_resize             | `false`           | bool | 用于设置是否对原始图像做rezise变换. 360p的视频流应该不需要做resize变换, 请实际测试.
| resize_scale          | `0.5`             | -    | resize参数为真时,可以设置resize的尺度, 0.5意味着缩小为原始尺寸的一半.

## Topics
订阅和发布的话题名称请在launch文件中设置。
- ### Subscribe Topic
    `/asr_sdm_underwater_video_enhancement/input/image`
- ### Publish Topic
    `/asr_sdm_underwater_video_enhancement/output/image`

## Test
在scripts文件夹下，准备了一个脚本用于测试。该脚本可以把视频文件以ros2 topic的形式发布出来。

可以在`VIDEO_EXTS`中添加必要的视频格式，同时该脚本也支持播放单个文件或者文件夹中的所有文件。
```
$ python3 play_video_as_ros_topic.py /path_to_videos/

$ python3 play_video_as_ros_topic.py /path_to_videos/cv_001.mp4
```
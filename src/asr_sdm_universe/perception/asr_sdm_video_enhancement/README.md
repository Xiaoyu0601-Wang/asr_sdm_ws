# ASR_SDM_VIDEO_ENHANCEMENT

This node subscribes to a camera topic and then uses image enhancement algorithms to improve the image quality.

## Input and Output

- Input Topic
    - /asr_sdm_video_enhancement/input/image (sensor_msgs::msg::Image)

- Output Topic
    - /asr_sdm_video_enhancement/output/image (sensor_msgs::msg::Image)

## Parameters

In the .cpp file, there are several parameters that can be modified.

- airlight
    - The default value is 255; a smaller value results in a brighter processed image.
- scale
    - This parameter allows you to scale the original image to ensure real-time processing.

## Test

A script is provided in the "scripts" directory that can directly play local video files and publish them as ROS topics.

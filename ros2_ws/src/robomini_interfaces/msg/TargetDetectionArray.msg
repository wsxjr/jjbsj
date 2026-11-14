# 一帧相机图像中的所有靶板识别结果
#
# header.stamp 来自原始相机图像，而不是识别完成时间。
# header.frame_id 使用原始相机图像的 frame_id。

std_msgs/Header header
TargetDetection[] detections

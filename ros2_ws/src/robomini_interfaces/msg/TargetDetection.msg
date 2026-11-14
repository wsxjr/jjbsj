# 单个黑色圆形靶板及其蓝色灯条识别结果
#
# 图像坐标以左上角为原点：
# center_x 向右增大，center_y 向下增大。
#
# state:
#   off     没有蓝色灯条
#   active  蓝色面积比例较低，通常为 2 根灯条
#   pending 蓝色面积比例较高，通常为 4 根灯条
#   unknown 比例处于过渡区，或靶板超出图像边缘

float64 center_x
float64 center_y
float64 radius
float64 dark_fraction
float64 blue_ratio
uint32 blue_pixels
uint32 circle_pixels
string state
bool clipped

# 카메라 핸들러가 내보내는 검출 하나. 영상 자체는 발행하지 않는다.
string label                          # 색 이름(red|blue|yellow) 또는 클래스(person|forklift)
float32 score
string camera                         # head_camera | base_camera | left_wrist_camera | right_wrist_camera
float32[4] bbox_xyxy                  # 픽셀 좌표 (x1, y1, x2, y2)
bool has_position
geometry_msgs/PointStamped position   # 물체 중심 (frame_id 에 표기, 보통 base_footprint)

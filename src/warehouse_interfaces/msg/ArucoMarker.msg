# 검출된 ArUco 마커 하나 (배열 header.frame_id 기준, 보통 base_footprint)
int32 id
float32 size               # 검은 정사각형 한 변 (m)
geometry_msgs/Pose pose    # 마커 중심, z축 = 마커가 바라보는 법선
float32 normal_yaw         # 기준 좌표계에서 본 법선 방향 (rad). 로봇이 정면으로 마주 보면 ±pi
float32 distance           # 카메라에서 마커 중심까지 (m)

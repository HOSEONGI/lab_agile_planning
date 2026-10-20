# LLM이 만든 작업 계획(RobotPlan)의 한 단계. Task Manager가 순서대로 실행한다.
string action        # navigate | pick | place | arm_pose | gripper | wait | say
string target        # pick: red|blue|yellow ; arm_pose: 자세 이름(ready|home|carry) ; gripper: open|close ; say: 말할 문장
string location      # navigate: 목적지 ; pick: 물체가 있는 작업대 ; place: 놓을 작업대 (locations.yaml 키)
string arm           # left | right | 비우면 자동
float32 duration     # wait: 초
string note          # 사용자에게 보여 줄 한 구절 (예: "픽업 작업대로 이동")

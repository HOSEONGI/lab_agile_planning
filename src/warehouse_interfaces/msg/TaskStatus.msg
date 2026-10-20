# Task Manager 상태 (웹 대시보드가 구독). /warehouse/task_status
std_msgs/Header header
string state             # idle | planning | executing | succeeded | failed | canceled
string utterance         # 원문 지령
string summary           # 계획 요약
PlanStep[] steps
int32 current_step       # 실행 중인 단계 (0 부터), 없으면 -1
string detail            # 현재 세부 상태 문장

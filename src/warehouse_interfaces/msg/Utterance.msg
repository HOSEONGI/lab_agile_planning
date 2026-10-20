# 사용자 지령 또는 로봇의 결과 문장 한 줄.
# /warehouse/utterance (사용자 → 로봇), /warehouse/narration (로봇 → 사용자)
std_msgs/Header header
string speaker           # user | robot
string text
string language          # ko | en

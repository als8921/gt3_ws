import math

def check_arm_limit(base_pos, end_pos_list, arm_length_limit):
    """
        Args:
            base_pos            로봇의 원점 위치
            end_pos_list        수행할 작업의 위치 리스트 (로봇 원점을 기준으로함)
            arm_length_limit    로봇팔 최대 길이
    """
    return [0 if math.dist(base_pos, end_pos) > arm_length_limit else 1 for end_pos in end_pos_list]

# 사용 예시
my_position = (1, 2, 3)
positions = [(1, 2, 4), (2, 3, 1), (5, 5, 5)]
threshold_distance = 4

output = check_arm_limit(my_position, positions, threshold_distance)
print(output)

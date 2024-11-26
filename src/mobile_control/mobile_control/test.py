data = "[1, (-2.46, 0.00, -1.52), (-1.73, 0.00, -0.89), (-1.73, 0.52, -0.89)],[2, (-2.46, 0.00, -1.52), (-1.73, 0.00, -0.89), (-1.73, 0.52, -0.89)],[2, (-2.46, 0.00, -1.52), (-1.73, 0.00, -0.89), (-1.73, 0.52, -0.89)]"

# 데이터에서 개별 리스트를 나누기
data_list = data.split("],[")

# 리스트의 양 끝의 대괄호를 제거
data_list[0] = data_list[0][1:]  # 첫 번째 리스트의 시작 대괄호 제거
data_list[-1] = data_list[-1][:-1]  # 마지막 리스트의 끝 대괄호 제거

# 각 리스트를 eval로 평가하여 처리
for item in data_list:
    item = "[" + item + "]"  # 다시 대괄호로 감싸기
    idx, startpos, endpos, height = eval(item)
    index = int(idx)
    h = float(height[1])
    x1, y1, x2, y2 = float(startpos[0]), float(startpos[2]), float(endpos[0]), float(endpos[2])
    print(f"{index} : {x1}, {y1}, {x2}, {y2}")

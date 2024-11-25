# msg = "[1; (-2.46, 0.00, -1.52); (-1.73, 0.00, -0.89); (-1.73, 0.52, -0.89)]"


# idx, startpos, endpos, height = msg.split(';')

# print(idx)
# print(startpos)
# print(endpos)
# print(height)

# x1, y1, x2, y2 = map(float, msg.data.split(';'))

msg = "[1, (-2.75, 0.00, -0.43), (-0.89, 0.00, -1.51), (-0.89, 0.80, -1.51)],[2, (-2.75, 0.00, -0.43), (-0.89, 0.00, -1.51), (-0.89, 0.80, -1.51)]"

a = eval(msg)
for idx, startpos, endpos, height in a:
    print(idx)
    print(startpos[0], startpos[2])
    print(endpos[0], endpos[2])
    print(height[1])
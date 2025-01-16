import math

def laser_scan_to_xy(front_scan, rear_scan, length, width):
    front_angle_offset = math.radians(45)
    front_x = []
    front_y = []

    for i, range in enumerate(front_scan.ranges):
        if range > front_scan.range_min and range < front_scan.range_max:
            angle = front_scan.angle_min + i * front_scan.angle_increment + front_angle_offset
            x = range * math.cos(angle) + (length / 2)
            y = range * math.sin(angle) + (width / 2)
            front_x.append(x)
            front_y.append(y)

    rear_angle_offset = math.radians(-135)
    rear_x = []
    rear_y = []

    for i, range in enumerate(rear_scan.ranges):
        if range > rear_scan.range_min and range < rear_scan.range_max:
            angle = rear_scan.angle_min + i * rear_scan.angle_increment + rear_angle_offset
            x = range * math.cos(angle) - (length / 2)
            y = range * math.sin(angle) - (width / 2)
            rear_x.append(x)
            rear_y.append(y)

    return [front_x, front_y],  [rear_x, rear_y]

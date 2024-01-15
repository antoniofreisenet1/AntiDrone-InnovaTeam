def map_to_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def is_valid_distance(distance):
    return 150 > distance > 1

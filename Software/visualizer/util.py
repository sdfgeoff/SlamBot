from mathutils import Vector

def get_deviation_and_average(samples: list[Vector]):
    average = sum(samples, Vector([0,0,0])) / len(samples)
    deviation = sum([(g - average).length_squared for g in samples]) / len(samples)

    return deviation, average

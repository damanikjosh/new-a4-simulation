import numpy as np



def find_nearest_point(target, points):
    min_dist = float('inf')
    min_i = None
    nearest_point = None
    for i, point in enumerate(points):
        dist = (target[0] - point[0])**2 + (target[1] - point[1])**2
        if dist < min_dist:
            min_dist = dist
            min_i = i
            nearest_point = point
    return nearest_point, min_i

def process_task(obj1_points, obj2_points, obj3_points):
    assert len(obj2_points) == len(obj3_points), "Number of points in obj2 and obj3 should be the same"

    obj2_reqs = np.zeros((len(obj2_points), len(obj1_points)))
    for i, point in enumerate(obj2_points):
        _, nearest_obj1_i = find_nearest_point(point, obj1_points)
        obj2_reqs[i, nearest_obj1_i] = 1

    obj3_reqs = np.eye(len(obj3_points), len(obj2_points))

    # Combine all objectives
    all_points = obj1_points + obj2_points + obj3_points
    all_types = ['surveillance'] * len(obj1_points) + ['search'] * len(obj2_points) + ['handling'] * len(obj3_points)
    all_reqs = np.zeros((len(all_points), len(all_points)))
    all_reqs[len(obj1_points):len(obj1_points)+len(obj2_points), :len(obj1_points)] = obj2_reqs
    all_reqs[len(obj1_points)+len(obj2_points):, len(obj1_points):len(obj1_points)+len(obj2_points)] = obj3_reqs
    all_done = np.zeros(len(all_points))
    return all_points, all_reqs, all_done, all_types


if __name__ == '__main__':
    from read_objectives import read_objectives

    obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    print(obj1_points)
    process_task(obj1_points, obj2_points, obj3_points)

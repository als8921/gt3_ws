import numpy as np

def get_start_end_index(clust):
    clust = np.array(clust)
    result = []
    for y in range(clust.shape[0]):
        start_point = -1
        end_point = -1

        for i in range(clust.shape[1]):
            if(start_point == -1 and not np.isnan(clust[y][i]).any()):
                start_point = i
            if(end_point == -1 and not np.isnan(clust[y][clust.shape[1] - 1 - i]).any()):
                end_point = clust.shape[1] - 1 - i
        result.append([y, start_point, end_point])

    return result
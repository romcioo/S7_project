# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 12:24:26 2020

@author: inaki
"""

## [Declare]

# ## [Boundaries]
x_bound = [-48.7, 55.7]
y_bound = [-16, 28.2]
# ## ![Boundaries]

## ![Declare]

def create(figType):
    visited = []
    xRng = x_bound[1] - x_bound[0]
    yRng = y_bound[1] - y_bound[0]
    if figType == "plane":
        xstps = int(xRng//.5 + (0 if (xRng%.5 == 0) else 1))
        ystps = int(yRng//.5 + (0 if (yRng%.5 == 0) else 1))
        for i in range(xstps):
            temp = []
            for j in range(ystps):
                temp.append([False, 0])
            visited.append(temp)
    elif figType == "cylinder":
        pass
    else:
        pass
    
    return visited

def visit(point, mat, figType):
    area = 0
    overlap = 0
    if point[0] >= x_bound[0] and point[0] <= x_bound[1] and point[1] >= y_bound[0] and point[1] <= y_bound[1]:
        if figType == "plane":
            xscal = point[0] - x_bound[0]
            yscal = point[1] - y_bound[0]
            pos = (xscal//.5, yscal//.5)
            if mat[pos[0]][pos[1]][0]:
                mat[pos[0]][pos[1]][1] += 1
                overlap = .5^2
            else:
                mat[pos[0]][pos[1]][0] = True
                area = .5^2
        if figType == "cylinder":
            pass
        else:
            pass
    
    return area, overlap, mat
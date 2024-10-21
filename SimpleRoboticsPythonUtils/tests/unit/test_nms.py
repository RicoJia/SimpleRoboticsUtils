#! /usr/bin/env python3
import numpy as np
from simple_robotics_python_utils.cv.nms import handcrafted_non_maximum_suppresion

def test_non_maximum_suppresion():
    # [x1, y1, x2, y2]
    boxes = np.array([
        [100, 100, 210, 210],
        [101, 101, 209, 209],
        [105, 105, 215, 215],
        [150, 150, 270, 270]
    ])

    scores = np.array([0.8, 0.79, 0.75, 0.9])
    overlapping_thre = 0.3
    selected_indices = handcrafted_non_maximum_suppresion(scores=scores, boxes=boxes, overlapping_thre=overlapping_thre)
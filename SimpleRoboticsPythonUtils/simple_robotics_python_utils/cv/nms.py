#! /usr/bin/env python3

import numpy as np
import typing


def handcrafted_non_maximum_suppresion(scores: np.ndarray, boxes: np.ndarray, overlapping_thre: float = 0.3) -> typing.List:
    # sort scores into descending order and store their indices
    score_order = scores.argsort()[::-1]

    # Store individual coordinates in lists.
    x1s = boxes[:, 0]
    y1s = boxes[:, 1]
    x2s = boxes[:, 2]
    y2s = boxes[:, 3]

    # find areas of each box
    areas = (x2s - x1s + 1) * (y2s - y1s + 1)
    keep = []

    while score_order.size > 0:
        keep.append(score_order[0])
        # find coords differences along x
        other_x2_this_x1 = x2s[score_order[1:]] - x1s[score_order[0]]
        this_x2_other_x1 = x2s[score_order[0]] - x1s[score_order[1:]]
        # rest of the score_order elements
        x_overlapping_lengths = np.maximum(np.minimum(other_x2_this_x1, this_x2_other_x1), 0)

        other_y2_this_y1 = y2s[score_order[1:]] - y1s[score_order[0]]
        this_y2_other_y1 = y2s[score_order[0]] - y1s[score_order[1:]]
        y_overlapping_lengths = np.maximum(np.minimum(other_y2_this_y1, this_y2_other_y1), 0)

        # find intersection area
        overlapping_areas = x_overlapping_lengths * y_overlapping_lengths
        ious = overlapping_areas / (areas[score_order[0]] + areas[score_order[1:]] - overlapping_areas)

        # TODO, why np.where returns an array?
        independent_box_indices = np.where(ious <= overlapping_thre)[0]
        # because ious, independent_box_indices are arrays starting at the next element, we want to add 1 here.
        score_order = score_order[independent_box_indices + 1]

    return keep

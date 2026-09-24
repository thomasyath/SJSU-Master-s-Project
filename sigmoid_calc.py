#!/usr/bin/env python3
import numpy as np

def sigmoid(w_max,rate):
    time_end = 5
    step = 1/rate
    steepness = 8
    half_time = 0.25
    input_array = np.arange(0,time_end,step)
    sigmoid_array = []
    sig_d = []
    for index, value in enumerate(input_array):
        sigmoid_array.append(w_max / (1 + np.exp(-steepness*(value - half_time))))
        sig_d.append(steepness*sigmoid_array[index]*(1-(sigmoid_array[index]/w_max)))
    return input_array, sigmoid_array, sig_d
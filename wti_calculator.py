#!/usr/bin/env python3
import numpy as np

def wti_calc(data):
    height = 1.1938
    weight = 360
    rf = 0.2286
    rb = 0.381
    rh = 0.6096
    width = 0.2794
    wti_a = data[0] * height / ((data[2]+weight)*rf)
    wti_p = data[0] * height / ((data[2]+weight)*rb)
    wti_mll = 0
    wti_mlr = 0
    if data[3] < 0:
        wti_mll = abs(data[1]) * height / (-(data[3]*width + weight/2)*rh)
        wti_mlr = 0
    elif data[3] > 0:
        wti_mll = 0
        wti_mlr = abs(data[1]) * height / ((abs(data[3])*width + weight/2)*rh)
    #rospy.loginfo(f"WTI_a: {wti_a}")
    #rospy.loginfo(f"WTI_p: {wti_p}")
    #rospy.loginfo(f"WTI_mll: {wti_mll}")
    #rospy.loginfo(f"WTI_mlr: {wti_mlr}")

    return [wti_a, wti_mll, wti_mlr, wti_p]
 
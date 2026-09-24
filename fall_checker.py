#!/usr/bin/env python3

def ft_deviation_check(ft_deviation):
    if ft_deviation[0] >= 70 or ft_deviation[4] >= 40:  #check for forward fall
        fall_state = "Forward"

    elif ft_deviation[1] >= 50 or ft_deviation[3] <= -60: #check for left fall
        fall_state = "Left"

    elif ft_deviation[1] <= -50 or ft_deviation[3] >= 80: #check for right fall
        fall_state = "Right"

    elif ft_deviation[0] <= -70 or ft_deviation[4] <= -50: #check for backward fall
        fall_state = "Backward"

    else: #No fall detected, send data to terminal to check forces and torques applied
        fall_state = "No fall"

    return fall_state

def ft_rate_check(ft_rate_change):
    if ft_rate_change[0] >= 1200 or ft_rate_change[4] >= 700:  #check for forward fall
        fall_state = "Forward"

    elif ft_rate_change[1] >= 1000 or ft_rate_change[3] <= -1000: #check for left fall
        fall_state = "Left"

    elif ft_rate_change[1] <= -1000 or ft_rate_change[3] >= 1200: #check for right fall
        fall_state = "Right"

    elif ft_rate_change[0] <= -1500 or ft_rate_change[4] <= -1000: #check for backward fall
        fall_state = "Backward"

    else: #No fall detected, send data to terminal to check forces and torques applied
        fall_state = "No fall"

    return fall_state
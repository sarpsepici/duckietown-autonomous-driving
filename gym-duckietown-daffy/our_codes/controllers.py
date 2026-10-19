import numpy as np

def PID_3params(dist, angl,
                dist_intg, angl_intg,
                dist_prev, angl_prev,
                K):
    # Expand params
    [Kv, Ka, Kad, Ti, Td] = K

    # P
    delta_action = (Kv * (0.25 - abs(dist))) * np.array([0.44, 0.0]) + \
                   (Kad * (dist) + Ka * angl) * np.array([0, 1])

    # I
    delta_action += (Kv/Ti) * (-1) * abs(dist_intg) * np.array([0.44, 0.0]) + \
                    ((Kad/Ti) * dist_intg + (Ka/Ti) * angl_intg) * np.array([0, 1])

    # D
    delta_action += (Kv/Td) * abs(dist - dist_prev) * (-1) * np.array([0.44, 0.0]) + \
                    ((Kad/Td) * (dist - dist_prev) + (Ka/Td) * (angl - angl_prev) * (-1)) * np.array([0, 1])

    return delta_action


def PID_9params(dist, angl,
        dist_intg, angl_intg,
        dist_prev, angl_prev,
        K, T, D):
    # expand arguements
    [Kv, Ka, Kad] = K
    [Tv, Ta, Tad] = T
    [Dv, Da, Dad] = D

    # P
    delta_action = (Kv * (0.25 - abs(dist))) * np.array([0.44, 0.0]) + \
                   (Kad * (dist) + Ka * angl) * np.array([0, 1])

    # I
    delta_action += Tv * (-1) * dist_intg * np.array([0.44, 0.0]) + \
                    (Tad * dist_intg + Ta * angl_intg) * np.array([0, 1])

    # D
    delta_action += (-1) * abs(dist - dist_prev) * Dv * np.array([0.44, 0.0]) + \
                    (Dad * (dist - dist_prev) + \
                    Da * (angl - angl_prev) * (-1)) * np.array([0, 1])

    return delta_action

def PID_weighted(dist, angl, dt,
                 dist_prev, angl_prev,
                 dist_intg, angl_intg,
                 K, w_dist, w_angl, L):

    [Kp, Kd, Ki] = K
    e = w_dist * dist + w_angl * angl
    e_prev = w_dist*dist_prev + w_angl*angl_prev
    e_intg = w_dist*dist_intg+w_angl*angl_intg
    delta_e_deriv = e-e_prev


    delta_action = (Kp * e + Kd * delta_e_deriv / dt + (Ki/2) * e_intg * dt) * np.array([0,1]) + L * np.array([1,0])

    return delta_action


# STEP CONTROLLER
# MARK 1

    # if (dist < -0.15) and (angl < 23 or angl > 27):
    #     action -= np.abs(angl - 25) * 0.02 * np.array([0, 1])
    # elif (dist < -0.15) and (angl > 23 and angl < 27):
    #     action += np.abs(dist + 0.1) * 0.3 * np.array([0.44, 0.0])
    # elif (dist > -0.15) and (angl < -2 or angl > 2):
    #      action += np.abs(angl) * 0.3 * np.array([0, 1])
    # if (dist > -0.05) and (angl < -23 or angl > -27):
    #      action += np.abs(angl + 25) * 0.03 * np.array([0, 1])
    # elif (dist > -0.05) and (angl > -23 and angl < -27):
    #      action += np.abs(dist + 0.1) * 0.03 * np.array([0.44, 0.0])
    # elif (dist < -0.05) and (angl < -2 or angl > 2):
    #     action -= np.abs(angl) * 0.03 * np.array([0, 1])
    # else:
    #     action += np.array([0.44, 0.0])

# MARK 2

    # if (dist < -0.15) and (angl < 23 or angl > 27):
    #     stt = 1
    #     action -= np.abs(angl - 25) * 0.05 * np.array([0, 1])
    # elif (dist < -0.15) and (angl > 23 and angl < 27):
    #     stt = 1
    #     action += np.abs(dist + 0.1) * 10 * np.array([0.44, 0.0])
    # elif stt==1 and (dist > -0.15) and (angl < -2 or angl > 2):
    #      action += 0.3 * np.array([0, 1])
    #      stt= 0
    # elif (dist > -0.05) and (angl < -23 or angl > -27):
    #      action += 0.3 * np.array([0, 1])
    # elif (dist > -0.05) and (angl > -23 and angl < -27):
    #      action += 0.3 * np.array([0.44, 0.0])
    # elif (dist < -0.05) and (angl < -2 or angl > 2):
    #     action -= 0.3 * np.array([0, 1])
    # else:
    #     action += 0.3 * np.array([0.44, 0.0])

# MARK 3 (correct)

    # if (dist < -0.15) :
    #     if (angl > 23 and angl < 27):
    #         action += 0.3 * np.array([0.44, 0.0])
    #     elif (angl < 23):
    #         action -= 0.3 * np.array([0, 1])
    #     else:
    #         action += 0.3 * np.array([0, 1])
    # if (dist > 0.15):
    #     if (angl > -23 and angl < -27):
    #         action += 0.3 * np.array([0.44, 0.0])
    #     elif (angl < -23):
    #         action += 0.3 * np.array([0, 1])
    #     else:
    #         action -= 0.3 * np.array([0, 1])
    # else:
    #     if (angl > -2 and angl < 2):
    #         action += 0.3 * np.array([0.44, 0.0])
    #     elif (angl < -2):
    #         action -= 0.3 * np.array([0, 1])
    #     else:
    #         action += 0.3 * np.array([0, 1])

# STEP CONTROLLER (dead zone)

    # action += np.array([0.08, 0]) + Kp * (error_dist) * np.array([0, -10]) + Td * (error_dist - error_dist_previous) * np.array([0, -10])

    # making tank close to right lane
    # if error_dist > 0.07:  # Means we are on right side, must go left
    #     action += np.array([0.44, 0]) + Kp * np.abs(error_dist) * np.array([0, 1]) + Td * np.abs(
    #         error_dist - error_dist_previous) * np.array([0, 1])  # left
    #     error_dist_previous = error_dist
    #     error_angle_previous = error_angle
    # elif error_dist < -0.07:  # we are on left , must go right
    #     action += np.array([0.44, 0]) + Kp * np.abs(error_dist) * np.array([0, -1]) + Td * np.abs(
    #         error_dist - error_dist_previous) * np.array([0, -1])  # right
    #     error_dist_previous = error_dist
    #     error_angle_previous = error_angle
    #
    # if error_dist > -0.1 and error_dist < 0.1:  # Making the tank parallel to right lane
    #     if error_angle < -0.1:  # we on left
    #         action -= Kpa * np.abs(error_angle) * np.array([0, 1]) + Tda * np.abs(
    #             error_angle - error_angle_previous) * np.array([0, 1])  # right
    #         error_dist_previous = error_dist
    #         error_angle_previous = error_angle
    #     elif error_angle > 0.1:  # we on right
    #         action += Kpa * np.abs(error_angle) * np.array([0, 1]) + Tda * np.abs(
    #             error_angle - error_angle_previous) * np.array([0, 1])  # left
    #         error_dist_previous = error_dist
    #         error_angle_previous = error_angle
    #     else:
    #         action += np.array([0.44, 0])
    #         error_dist_previous = error_dist
    #         error_angle_previous = error_angle

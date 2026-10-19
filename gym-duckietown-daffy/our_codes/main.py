#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""
# add library path to python path
import os
import sys
from pathlib import Path
code_path = os.getcwd()
sys.path.append(str(Path(code_path).parent))

# import controllers
from controllers import PID_3params, PID_9params, PID_weighted

import numpy as np
import argparse
import gym
import pyglet
from PIL import Image
from pyglet.window import key
from gym_duckietown.envs import DuckietownEnv
import time
import math

# from experiments.utils import save_img

parser = argparse.ArgumentParser()
parser.add_argument("--env-name", default="Duckietown")
# parser.add_argument("--map-name", default="udem1")
parser.add_argument("--map-name", default="small_loop")
parser.add_argument("--distortion", default=False, action="store_true")
parser.add_argument("--camera_rand", default=False, action="store_true")
parser.add_argument("--draw-curve", action="store_true", help="draw the lane following curve")
parser.add_argument("--draw-bbox", action="store_true", help="draw collision detection bounding boxes")
parser.add_argument("--domain-rand", action="store_true", help="enable domain randomization")
parser.add_argument("--dynamics_rand", action="store_true", help="enable dynamics randomization")
parser.add_argument("--frame-skip", default=1, type=int, help="number of frames to skip")
parser.add_argument("--seed", default=1, type=int, help="seed")
args = parser.parse_args()

if args.env_name and args.env_name.find("Duckietown") != -1:
    env = DuckietownEnv(
        seed=args.seed,
        map_name=args.map_name,
        draw_curve=args.draw_curve,
        draw_bbox=args.draw_bbox,
        domain_rand=args.domain_rand,
        frame_skip=args.frame_skip,
        distortion=args.distortion,
        camera_rand=args.camera_rand,
        dynamics_rand=args.dynamics_rand,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()

# env.start_pose = [[0.25,0,0.25], 0]
# env.user_tile_start = (1,0)
# print("---")
# print(env.road_tile_size)

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print("RESET")
        env.start_pose = [[0.25, 0, 0.25], 0]
        env.user_tile_start = (1, 0)
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.Q:
        env.close()
        sys.exit(0)

    # Take a screenshot
    # UNCOMMENT IF NEEDED - Skimage dependency
    # elif symbol == key.RETURN:
    #     print('saving screenshot')
    #     img = env.render('rgb_array')
    #     save_img('screenshot.png', img)


# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

dist_prev = 0
angl_prev = 0
dist_intg = 0
angl_intg = 0
mean_square_error = 0

# DEFINE YOUR CONTROLLER HERE
controller_name = 'PID_weighted'
if controller_name == 'PID_3params':
    Kv = 3      # proportional velocity by distance
    Kad = 15    # proportional angle by distance
    Ka = 0.2    # proportional angle by angle
    Ti = 1000
    Td = 10
    K = [Kv, Ka, Kad, Ti, Td]

if controller_name == 'PID_9params':
    Kv = 3      # proportional velocity by distance
    Kad = 20    # proportional angle by distance
    Ka = 0.1    # proportional angle by angle
    K = [Kv, Ka, Kad]

    Dv = 0.5    # derivative velocity by distance
    Dad = 5     # derivative angle by distance
    Da = 0.01   # derivative angle by angle
    D = [Dv, Dad, Da]

    Tv = 0.0005     # integrator velocity by distance
    Tad = 0.0001    # integrator angle by distance
    Ta = 0.00001    # integrator angle by angle
    T = [Tv, Tad, Ta]

if controller_name == 'PID_weighted':
    Kp = 5.83
    Kd = 0.04
    Ki = 0.2
    K = [Kp, Kd, Ki]
    w_dist = 0.7
    w_angl = 0.3
    v = 0.5
    dt = 0.0333


def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    wheel_distance = 0.102
    min_rad = 0.08

    global dist_prev
    global angl_prev
    global dist_intg
    global angl_intg
    global mean_square_error

    dist_intg = np.clip(dist_intg, -50, 50)     # ARW
    angl_intg = np.clip(angl_intg, -500, 500)   # ARW

    action = np.array([0.0, 0.0])

    dist = env.get_lane_pos2(env.cur_pos, env.cur_angle)[0]
    angl = env.get_lane_pos2(env.cur_pos, env.cur_angle)[2]

# PID CONTROLLER
    # Apply controller
    if controller_name == 'PID_3params':
        delta_action = PID_3params(dist=dist, angl=angl,
                                   dist_intg=dist_intg, angl_intg=angl_intg,
                                   dist_prev=dist_prev, angl_prev=angl_prev,
                                   K=K)

    if controller_name == 'PID_9params':
        delta_action = PID_9params(dist=dist, angl=angl,
                                   dist_intg=dist_intg, angl_intg=angl_intg,
                                   dist_prev=dist_prev, angl_prev=angl_prev,
                                   K=K, T=T, D=D)

    if controller_name == 'PID_weighted':
        delta_action = PID_weighted(dist=dist, angl=angl, dt=dt, dist_prev=dist_prev, angl_prev=angl_prev, dist_intg=dist_intg, angl_intg=angl_intg, K=K, w_dist=w_dist,
                                    w_angl=w_angl, L=v)


    # Update action
    action += delta_action

    # Record changes
    dist_prev = dist
    angl_prev = angl
    dist_intg += dist
    angl_intg += angl

    v1 = action[0]
    v2 = action[1]

    # Limit radius of curvature
    # if v1 == 0 or abs(v2 / v1) > (min_rad + wheel_distance / 2.0) / (min_rad - wheel_distance / 2.0):
    #    # adjust velocities evenly such that condition is fulfilled
    #    delta_v = (v2 - v1) / 2 - wheel_distance / (4 * min_rad) * (v1 + v2)
    #    v1 += delta_v
    #    v2 -= delta_v

    action[0] = v1
    action[1] = v2

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    obs, reward, done, info = env.step(action)

    # print(env.cur_angle*(180/math.pi) + env.get_lane_pos2(env.cur_pos,env.cur_angle)[2])
    # print("********")

    # STOPPING CRITERIA OF BRUTE FORCE ALG

    if -91 < (env.cur_angle * (180 / math.pi) + env.get_lane_pos2(env.cur_pos, env.cur_angle)[2]) < -89 :
        print("New Iteration")
        env.start_pose = [[0.25, 0, 0.25], 0]
        env.user_tile_start = (1, 0)
        env.reset()
        env.render()
        pyglet.app.exit()

    mean_square_error = mean_square_error + abs(dist) * abs(dist)
    print(mean_square_error)

    # ERROR VALUES TEXT FILE

    file = open("errorvalues_tot.txt", "a")
    string_error = str(dist)
    comma = ','
    comma_value = string_error + comma
    file.write(comma_value)
    file.close()

    if key_handler[key.RETURN]:

        im = Image.fromarray(obs)

        im.save("screen.png")

    if done:
        print("done!")
        env.reset()
        env.render()

    env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop

best_time = 1000
end = 1000
st = 0
cpt = 0



# BRUTE FORCE ALG

# for PID_weighted

for Kp_for in range(570, 591, 10):
    for Kd_for in range(3, 5, 1):
        for Ki_for in range(1, 3, 1):
            # the first iteration does not count,
            if not cpt == 0:
                st = time.time() # timing the beginning
                Kp = Kp_for/100
                Kd = Kd_for/100
                Ki = Ki_for/10
                env.start_pose = [[0.25, 0, 0.25], 0] # initializing the position
                env.user_tile_start = (1, 0)
                env.reset() # need to reset the environnement before starting the next iteration
                pyglet.app.run()
                end = time.time()
                # the first 2 iterations are faster, need to discard them to compare the others coherently
                if (end-st) < best_time and cpt > 2: # to print only the best so far
                    print("//////////////////////////////")
                    print(Ti)
                    print(Td)
                    best_time = (end - st)
            print(end - st)
            cpt+=1 # counting in which iteration we're at




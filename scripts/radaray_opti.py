#!/usr/bin/env python
import rospy

import sys
import actionlib


import radarays_ros.msg
import radarays_ros.srv

import cv_bridge
from cv_bridge import CvBridge

import cv2
import numpy as np

from sensor_msgs.msg import Image


# SKIMAGE Metrics
from skimage.metrics import structural_similarity
from skimage.metrics import peak_signal_noise_ratio
from skimage.metrics import normalized_mutual_information
from skimage.metrics import variation_of_information

# SKLEARN Metrics
from sklearn.metrics import mutual_info_score


# minimizer
from scipy.optimize import minimize

from scipy import optimize

from matplotlib import pyplot as plt
from tqdm import tqdm
import itertools



def to_param_vec(params : radarays_ros.msg.RadarParams):
    param_vec = []
    bounds = []
    param_vec.append(params.model.beam_width)
    bounds.append((0.01, 20.0))
    # param_vec.append(params.model.n_samples)
    # bounds.append((0.0, 1000.0))
    param_vec.append(params.model.n_reflections)
    bounds.append((0.0, 6.0))

    # wall
    mat = params.materials.data[1]
    param_vec.append(mat.velocity)
    bounds.append((0.0, 0.3))
    param_vec.append(mat.ambient)
    bounds.append((0.0, 1.0))
    param_vec.append(mat.diffuse)
    bounds.append((0.0, 1.0))
    param_vec.append(mat.specular)
    bounds.append((0.0, 5000.0))

    # glass
    mat = params.materials.data[3]
    param_vec.append(mat.velocity)
    bounds.append((0.0, 0.3))
    param_vec.append(mat.ambient)
    bounds.append((0.0, 1.0))
    param_vec.append(mat.diffuse)
    bounds.append((0.0, 1.0))
    param_vec.append(mat.specular)
    bounds.append((0.0, 5000.0))



    # for i, mat in enumerate(params.materials.data):
    #     # param_vec.append(mat.velocity)
    #     if i == 0:
    #         continue
        
    #     param_vec.append(mat.velocity)
    #     bounds.append((0.0, 0.3))
    #     param_vec.append(mat.ambient)
    #     bounds.append((0.0, 1.0))
    #     param_vec.append(mat.diffuse)
    #     bounds.append((0.0, 1.0))
    #     param_vec.append(mat.specular)
    #     bounds.append((0.0, 10000.0))

    return np.array(param_vec), bounds

def vec_to_params(params_init : radarays_ros.msg.RadarParams, param_vec):
    params_out = params_init
    
    params_out.model.beam_width = param_vec[0]
    # params_out.model.n_samples = int(param_vec[1])
    params_out.model.n_reflections = int(param_vec[2] + 0.5)

    offset_materials = 2

    # optimize wall and glass parameter

    # 1: wall, 3: glass
    params_out.materials.data[1].velocity = param_vec[offset_materials + 0 * 4 + 0]
    params_out.materials.data[1].ambient  = param_vec[offset_materials + 0 * 4 + 1]
    params_out.materials.data[1].diffuse  = param_vec[offset_materials + 0 * 4 + 2]
    params_out.materials.data[1].specular = param_vec[offset_materials + 0 * 4 + 3]

    params_out.materials.data[3].velocity = param_vec[offset_materials + 1 * 4 + 0]
    params_out.materials.data[3].ambient  = param_vec[offset_materials + 1 * 4 + 1]
    params_out.materials.data[3].diffuse  = param_vec[offset_materials + 1 * 4 + 2]
    params_out.materials.data[3].specular = param_vec[offset_materials + 1 * 4 + 3]

    # for i in range(len(params_out.materials.data)-1):
    #     params_out.materials.data[i+1].velocity = param_vec[offset_materials + i * 4 + 0]
    #     params_out.materials.data[i+1].ambient = param_vec[offset_materials + i * 4 + 1]
    #     params_out.materials.data[i+1].diffuse = param_vec[offset_materials + i * 4 + 2]
    #     params_out.materials.data[i+1].specular = param_vec[offset_materials + i * 4 + 3]

    return params_out
    
def grid_search(func, bounds, N):

    search_spaces = []
    for i in range(len(bounds)):
        search_spaces.append(np.linspace(bounds[0][0], bounds[0][1], N))

    best_permutation = None
    best_params = None
    best_score = 10000.0

    for permutation in itertools.product(range(N), repeat=len(bounds)):
        print(permutation)

        params = []

        for i,v in enumerate(permutation):
            params.append(search_spaces[i][v])

        params = np.array(params)
        score = func(params)

        if score < best_score:
            best_score = score
            best_permutation = permutation
            best_params = params

            print("New best found:")
            print("- permutation:", best_permutation)
            print("- params:", best_params)
            print("- score:", best_score)


    return best_params, best_score
    
client = None
br = None
service_name = 'get_radar_params'
action_name = 'gen_radar_image'

    
def simulate_image(params):
    global client, br

    goal = radarays_ros.msg.GenRadarImageGoal(params=params)

    polar_image = None

    while polar_image is None:
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        got_result = client.wait_for_result(timeout = rospy.Duration(5.0))
        
        if got_result:
            res = client.get_result()
            
            if not res is None:

                try:
                    polar_image = br.imgmsg_to_cv2(res.polar_image)
                except cv_bridge.core.CvBridgeError as ex:
                    print(res.polar_image)
                    print(ex)

                if len(polar_image.data) == 0:
                    # waited too long
                    connected = False
                    while not connected:
                        print("Reconnect due to wrong data")
                        client = actionlib.SimpleActionClient(server_node_name + "/" + action_name, radarays_ros.msg.GenRadarImageAction)
                        connected = client.wait_for_server(timeout= rospy.Duration(2.0))
                    polar_image = None
            else:
                # waited too long
                connected = False
                while not connected:
                    print("Reconnect due to wrong response")
                    client = actionlib.SimpleActionClient(server_node_name + "/" + action_name, radarays_ros.msg.GenRadarImageAction)
                    connected = client.wait_for_server(timeout= rospy.Duration(2.0))
                polar_image = None

        else:
            # waited too long
            connected = False
            while not connected:
                print("Reconnect due to missing goal return")
                client = actionlib.SimpleActionClient(server_node_name + "/" + action_name, radarays_ros.msg.GenRadarImageAction)
                connected = client.wait_for_server(timeout= rospy.Duration(2.0))
            polar_image = None

    return polar_image

def image_gen_and_compare(param_vec, params_init, real_image, objective_func, info={"iteration":0}):
    """Objective function"""
    iteration = info["iteration"]

    params = vec_to_params(params_init, param_vec)
    polar_image = simulate_image(params)
    score = objective_func(real_image, polar_image)

    if iteration % 100 == 0:
        print("Iteration: %d" % iteration)
        print("- params:")
        print(params)
        print("- score:", score)

    iteration += 1
    info["iteration"] = iteration

    return score

def radaray_opti(server_node_name = "radar_simulator", override_bounds = {}):
    global client, br
    
    # next step: get rid of bag file
    # - Tsm: T[v[0.863185,-1.164,1.49406], E[0.0149786, 0.00858233, 3.04591]]
    # - 

    br = CvBridge()

    print("Wait for real message...")
    radar_real_msg = rospy.wait_for_message("/Navtech/Polar", Image)
    radar_real = br.imgmsg_to_cv2(radar_real_msg)

    print("Got image", radar_real.shape)

    # cv2.imwrite("radar.png", radar_real)

    # radar_real = cv2.imread("radar.png", cv2.IMREAD_GRAYSCALE)

    # store image once
    
    get_radar_params = None

    

    print("Get current simulation params")
    rospy.wait_for_service(server_node_name + "/" + service_name)
    try:
        get_radar_params = rospy.ServiceProxy(server_node_name + "/" + service_name, radarays_ros.srv.GetRadarParams)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    resp = get_radar_params()
    params = resp.params

    print("Done fetching initial parameters.")

    print(params)
    param_vec, bounds = to_param_vec(params)

    print("param dim:", param_vec.shape)

    print("Bounds:")
    print(bounds)

    # override bounds
    for key, value in override_bounds.items():
        bounds[key] = value
    
    print(bounds)

    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(server_node_name + "/" + action_name, radarays_ros.msg.GenRadarImageAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print("Connected to Action Server.")

    options={'disp': True}

    objective_func = lambda real, sim: -mutual_info_score(real.flatten(), sim.flatten())
    func = lambda x: image_gen_and_compare(x, client, params, radar_real, objective_func)


     # do one test
    polar_image = simulate_image(params)
    score = objective_func(radar_real, polar_image)

    print("Test score:", score)

    plt.imshow(polar_image)
    plt.show()

    best_params, best_score = grid_search(func, bounds, 10)

    print("Result")
    print("- params:", best_params)
    print("- score:", best_score)

    return res

from multiprocessing import Process

if __name__ == '__main__':
    print("RadaRay Opti")

    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('radaray_opti_py')

        server_node_name = rospy.get_param('~server_node_name', 'radar_simulator')

        override_bounds_raw = rospy.get_param('~override_bounds', dict())

        override_bounds = {}

        for key, val in override_bounds_raw.items():
            override_bounds[eval(key)] = eval(val)

        print(override_bounds)

        res = radaray_opti(server_node_name = server_node_name, 
            override_bounds={5: (0.0, 2500.0)})
            
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

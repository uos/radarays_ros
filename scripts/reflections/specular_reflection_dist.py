import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.stats import norm, beta
from pert import PERT



def pert(x, a, b, c):
    pert = PERT(a, b, c)
    return pert.pdf(x)

# def mock(X, min_, max_, split_, mu_, P=1.0, num_rec=10, curr_depth=0):
#     sigma = 0.8

#     if curr_depth >= num_rec:
#         return P
#     else:
#         curr_depth = curr_depth + 1
#         if X > mu_:
#             chunk_size_ = (max_ - mu_) / curr_depth

#             if X > split_:
#                 return mock(X, split_ - chunk_size_, split_)
#             else:
#                 return mock(X, split_, split_ + chunk_size_)
#         else:
#             if X > split_:
#                 return mock(X)

#         S = max_ - min_
        
#         left_mid = mu_ + (min_ - mu_) / 2.0
#         right_mid = mu_ + (max_ - mu_) / 2.0

#         Pinner = P * sigma
#         Pouter = P * (1.0 - sigma)

    






    





    
def mean_binary_search(val, interval, depth_max, curr_depth=0):
    if curr_depth >= depth_max:
        return interval
    else:
        mid = (interval[1] + interval[0]) / 2.0
        
        if val > mid:
            print(curr_depth, [mid, interval[1]])
            return mean_binary_search(val, [mid, interval[1]], depth_max, curr_depth+1)
        else:
            print(curr_depth, [interval[0], mid])
            return mean_binary_search(val, [interval[0], mid], depth_max, curr_depth+1)


# use mu for first split instead of the mean
def valued_binary_search(val, interval, split_value, depth_max, curr_depth=0):
    if curr_depth >= depth_max:
        return interval
    else:
        mid = interval[1] * split_value + interval[0] * (1.0 - split_value)

        if val > mid:
            print(curr_depth, [mid, interval[1]])
            return valued_binary_search(val, [mid, interval[1]], split_value, depth_max, curr_depth+1)
        else:
            print(curr_depth, [interval[0], mid])
            return valued_binary_search(val, [interval[0], mid], split_value, depth_max, curr_depth+1)
    
def binary_probability_search(val, interval, var, height, depth_max, curr_depth=0):
    if curr_depth >= depth_max:
        return interval, P
    else:
        area = (interval[1] - interval[0]) * height

        mid = (interval[0] + interval[1]) / 2.0
        
        if val < mid:
            # left: P = var * P
            return binary_probability_search(val, [interval[0], mid], var, P * var, depth_max, curr_depth+1)
        else:
            # left: P = (1-var) * P
            return binary_probability_search(val, [mid, interval[1]], var, P * (1.0-var), depth_max, curr_depth+1)


def probability_search(val, interval, mu, var, depth_max, curr_depth=0):
    if curr_depth >= depth_max:
        return interval, 1.0
    else:
        if val < mu:
            # left
            area = 0.5
            width = mu - interval[0]
            height = area / width
            return binary_probability_search(val, [interval[0], mu], 1.0-var, height, depth_max, curr_depth+1)
        else:
            # right
            area = 0.5
            width = interval[1] - mu
            height = area / width
            return binary_probability_search(val, [mu, interval[1]], var, height, depth_max, curr_depth+1)

# M in [0,1]
def rectangle_split(rectangle, M):
    width = rectangle[1] - rectangle[0]
    height = rectangle[2]
    area = height * width

    area_left_right = area / 2.0

    width_left = width * M
    width_right = width * (1.0 - M)
    
    height_left = area_left_right / width_left
    height_right = area_left_right / width_right


    rectangle_left = [rectangle[0], rectangle[0] + width_left, height_left]
    rectangle_right = [rectangle[0] + width_left, rectangle[1], height_right]

    return rectangle_left, rectangle_right


def rec_rectangle_split(rectangle_list, M, num_splits, curr_depth=0):

    if curr_depth >= num_splits:
        return rectangle_list
    else:
        rectangle_list_new = []

        for rectangle in rectangle_list:
            rect_left, rect_right = rectangle_split(rectangle, M)
            rectangle_list_new.append(rect_left)
            rectangle_list_new.append(rect_right)

        return rec_rectangle_split(rectangle_list_new, M, num_splits, curr_depth+1)


def draw_rectangles(rectangle_list):
    
    X = []
    Y = []

    for rect in rectangle_list:
        X.append(rect[0])
        Y.append(rect[2])
        X.append(rect[1])
        Y.append(rect[2])

    X = np.array(X)
    Y = np.array(Y)

    plt.plot(X, Y)

if __name__ == '__main__':
    print("Specular Reflection Dist")

    
    x = np.linspace(0.0, 1.0, 1000)
    y = x**2
    plt.plot(x, y)




    # rect_left, rect_right = rectangle_split(rect, 0.2)



    # print(rect_left, rect_right)

    # bla = [rect_left, rect_right]

    # draw_rectangles(bla)
    # plt.show()


    # val = 0.8

    # bla1 = mean_binary_search(val, interval, depth_max = 5)
    # print("-", sum(bla1)/2.0 )

    # mu = 0.81
    # bla2 = valued_binary_search(val, interval, mu, depth_max = 5)
    # print("-", sum(bla2) / 2.0)

    # I, P = probability_search(val, interval, mu, 0.9, depth_max = 5)
    
    # print(I, P)



    exit()
    # min_ = 0.0
    # max_ = 1.0
    # mu = 0.0
    # sigma = 1.0
    
    # x = np.linspace(min_, max_, 1000)

    # fig, ax = plt.subplots()

    # y = mock(x, min_, max_, mu, sigma)
    # plt.plot(x, y)

    # plt.show()

    # exit()

    line_inc, = ax.plot(x, y, label='PDF')

    ax.set_ylim([0.0, 2.0])

    fig.subplots_adjust(bottom=0.15)

    # ax.set_ylim([-1.05, 1.05])
    ax_a = fig.add_axes([0.2, 0.08, 0.65, 0.03])
    # ax_b = fig.add_axes([0.2, 0.03, 0.65, 0.03])

    def render():
        y = pert(x, min_, a, max_)
        line_inc.set_ydata(y)
        fig.canvas.draw_idle()


    a_slider = Slider(
        ax=ax_a,
        label='a',
        valmin=min_,
        valmax=max_,
        valinit=a)
    
    def update_a(val):
        global a
        a = val
        render()

    a_slider.on_changed(update_a)

    # b_slider = Slider(
    #     ax=ax_b,
    #     label='b',
    #     valmin=0.0,
    #     valmax=20.0,
    #     valinit=b)

    # def update_b(val):
    #     global b
    #     b = val
    #     render()

    # b_slider.on_changed(update_b)

    

    plt.plot(x,y)
    plt.show()

    


    

import numpy as np

def forward(dist):
    return [[1, 0, dist], [0, 1, 0], [0, 0, 1]]

def left(angle):
    return [[np.cos(angle), -np.sin(angle), np.sin(angle)], [np.sin(angle), np.cos(angle), 1 - np.cos(angle)], [0, 0, 1]]

def right(angle):
    return [[np.cos(-angle), -np.sin(-angle), -np.sin(-angle)], [np.sin(-angle), np.cos(-angle), np.cos(-angle) - 1], [0, 0, 1]]

values = [.8, 1, .8, .4, .5]
#order = [L, F, L, R, F]:

turn_left = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
turn_right = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]


def main():

    one = np.matmul(left(.8), forward(1))
    two = np.matmul(one, left(.8))
    three = np.matmul(two, right(.4))
    four = np.matmul(three, forward(.5))

    # second turn's center of rotation
    print(str(np.matmul(np.matmul(one,turn_left), forward(1))))
    # third turn's center of rotation
    print(str(np.matmul(np.matmul(two, turn_right), forward(1))))
    print(str(four))

main()
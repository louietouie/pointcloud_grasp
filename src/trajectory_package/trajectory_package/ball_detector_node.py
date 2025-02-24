# import pyrealsense2 as rs
import numpy as np
import cv2

def main():

    print('chicken')
    
    # backgroundRemover = cv2.createBackgroundSubtractorKNN()

    # cap = cv2.VideoCapture(0)

    # while True:
    #     ret, frame = cap.read()

    #     cv2.imshow("Frame" ,frame)
    #     if cv2.waitKey(300) & 0xFF == 27:
    #         break

    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


# STATES
# detecting
# ready (game not started)
# error (ball lost, cant fit trajectory)

# MAIN PROCESS
# initialize with 2 camera positions
# begin loop
    # reset when player or robot hits ball
    # detect ball
        # narrow image search based on previous trajectory
    # triangulate 2 images to new position
    # add new position to data
    # estimate new trajectory (6 parameters) from data via RANSAC
    # publish trajectory

# QUESTIONS
# A: how to detect hit ball (aka reset data)
# B: would it be best to use the depth data from the realsense cams or just triangulate with the two color cams on realsense
    # if one camera loses sight of the ball this depth data would be very useful. FPS may be lower with depth data
    # at every detection, could also add 3 datapoints... 1 triangulated, 1 from cam1 depth, 1 from cam2 depth, and dump it into RANSAC and let it handle it

# OTHER
# Thought A:
    # If the cv detection algorithm is weak, I could also hold multiple hypothesis of possible trajectories (like particle filters?) and use the best fitting one (that looks most like a ballistic trajectory)
    # probably unnecessary. keeping all detections (even noise) in data array and then using RANSAN to get the best fitting trajectory should work
    # hungarian algorithm? optimization problem to assign new observations to existing object trajectories (used when multiple objects) or graph based approaches
# Thought B: I don't know if a Kalman filter would make sense here.
    # Kalman filters are useful because as you g# However, I don't really ever care about narrowing down the current position of the ball (say halfway across the table)
    # I could use a Kalman filter to get a more accurate prediction of the ball midway over the table by using a) the current measurement and b) the current model of where the ball should be based on the previous state
    # but I only care about the estimated trajectory, which takes into account all historical data at once to fit a curve.
    # so using the data to fit an estimated trajectory, plus the previous state (based on the same data) + a model to estimate the trajectory (et more confident in your current state via sensor info, you can use that state in a model to predict the next state
    # which will just be less accurate than our current estimate with new info), seems redundant\

# DETECTION ALGOS
# Option A - No AI - frame mask (remove everything that didn't change between this frame and last) - would still need to remove hands
    # + color mask
    # + ball shape mask
    # + ball likelihood in certain area based on previous data
    # + CNN to other things (a CNN that detects people/arms could remove moving people from a scene)
    # + select whatever is left that is largest
# CNN or YOLO
    # could train on Option A's labels
    # could do a detection on smaller images of the ball (easier to make training data), and then do detection multiple times per frame in a moving window
# Other
    # cv simpleblobdetector
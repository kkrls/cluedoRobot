import cv2
import rospy
import yaml
import os

# Infrastructure Reader Imports (Camera, Sensors, etc.)
from readers.camera_feed import CameraFeedReader

# Infrastructure Writer Imports (Movement, etc.)
from motion.movement import *

# Data Processing Interface Imports
from vision.vision import Vision
from colour.colour_detection import ColourDetection

# Robot Integrated Solver Imports
from integration.robot import Robot

# Environment Util Imports
from environment.room import Room

# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')

def read_input_points():
    path = f'{PROJECT_DIR}/world/input_points.yaml'
    with open(path) as opened_file:
        return yaml.safe_load(opened_file)

def write_character_id(character_id):
    path = f'{PROJECT_DIR}/output/cluedo_character.txt'
    with open(path, 'w') as opened_file:
        opened_file.write(character_id)

def save_character_screenshot(cv_image):
    path = f'{PROJECT_DIR}/output/cluedo_character.png'
    cv2.imwrite(path, cv_image)

##################### MAIN EXECUTION #####################
if __name__ == '__main__':
    
    # Runtime Util Variables
    in_simulation                   = True # False if code is running on real robot, True if code running in simulation
    main_debug                      = False
    live_feed                       = False
    mover_debug                     = False
    colour_detection_debug          = False
    
    # Load room entrances and center points data
    rooms = Room.load_rooms(read_input_points())
    
    # Initiate subscribers and readers
    camera_feed     = CameraFeedReader()
    colour_detector = ColourDetection(debug=colour_detection_debug)

    # Initiate and run ROS node
    rospy.init_node('group_project')
    
    # Initiate mobility publisher
    mover = Movement(debug=mover_debug)
    
    # Initiate integrated solver
    robot = Robot(camera_feed, colour_detector, mover, main_debug, live_feed, in_simulation)

    ############################################### MOVING TO GREEN ROOM ###############################################
    if main_debug and in_simulation:
        print("\nRunning code in simulation")
    elif main_debug and not in_simulation:
        print("\nRunning code on real robot")
    
    if main_debug:
        print("\nIdentifying rooms by colour")
        
    green_room, red_room = robot.move_to_green_room(rooms)    
    
    ############################################### CHARACTER RECOGNITION ###############################################
    
    space_angle = []
    n_search    = 1
    n_angles    = 0
    prev_angle  = 0
    faces       = []
    
    while len(faces) == 0:
        
        robot.search_room(n_search, n_angles, prev_angle, space_angle)
              
        faces, found_frame = robot.rotate_and_search_for_character_frame(forward_step_size=0.5)
        
        # No faces and no frame detected - Change position in room starting from center
        if len(faces) == 0 and found_frame == False:            
            if main_debug:
                print("\nMoving back to center of room")

            mover.goTo(green_room.center)
            
            n_search += 1
            n_angles = 0
        # No faces but frame detected - Change position in room
        elif len(faces) == 0 and found_frame == True:
            n_search += 1
            n_angles = 0
            
        # Faces detected
        if (len(faces) > 0):
            if main_debug:
                print("\nDetected Face")
                
            # Retrieve the image containing the first face contained within the bounding box
            current_camera_frame = camera_feed.cv_image
            face_image = Vision.image_from_face_bounding_box(image=camera_feed.cv_image, face=faces[0])

            # Classify the face
            character_in_frame = Vision.classify_face(face_image)
            if (live_feed):
                Vision.add_bound_box_to_faces_in_image(image=camera_feed.cv_image, faces=faces, threshold=1500)
                robot.display_camera_feed()

            # Store identified results as per the brief
            if character_in_frame[0] is not None and character_in_frame[1] is not None:
                write_character_id(character_id=character_in_frame[0])
                save_character_screenshot(current_camera_frame)
                if main_debug:
                    print(f"\nIdentified Face: {character_in_frame[0]}")
                break
            
            faces = []
            found_frame = False
    
    ############################################### CLEANUP ###############################################
    cv2.destroyAllWindows()
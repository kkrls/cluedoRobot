import random

from motion.movement import Movement
from colour.colour_detection import ColourDetection
from readers.camera_feed import CameraFeedReader
from vision.vision import Vision

class Robot():
    """
    Class which contains methods that integrate various reader and publisher functions
    to handle repetitive behaviours and execution steps (like looking for a frame)
    """
    
    def __init__(self, camera: CameraFeedReader, colour_detector: ColourDetection, mover: Movement, debug=False, live_feed=False, in_simulation=False):
        self.debug = debug
        self.live_feed = live_feed
        self.in_simulation = in_simulation
        self.camera = camera
        self.colour_detector = colour_detector
        self.mover = mover
        
        # Additional environment knowledge
        self.red_room = None
        self.green_room = None
    
    def display_camera_feed(self):
        """
        Displays the camera feed
        """
        if (self.camera.cv_image is not None and self.live_feed):
            Vision.show_image_in_window(image=self.camera.cv_image, image_title="TurtleBot Camera Feed")
    
    def move_to_green_room(self, rooms: list):
        """
        Go to each of the room entrances and check for the colour.
        If green, move to its center, if red go to other room, confirm its green and go to its center.
        Returns the green room and red room `Room` objects (in that order) populated with the right data.
        """
        found_green_room = False
        found_red_room = False
        red_room = None
        green_room = None
        # Move between room entrances until both rooms' colours have been identified
        while not found_green_room and not found_red_room:
            for i, room in enumerate(rooms):
                if (self.debug):
                    print(f"\nMoving to room #{i+1} entrance at point {room.entrance}\n")
                self.mover.goTo(room.entrance)
                # Instead of one 360 degree rotation, it splits up into 10 36 degree rotation and runs colour identification
                for _ in range(10):
                    room_colour = self.colour_detector.identify_colour(self.camera.cv_image)
                    if (room_colour == "green"):
                        room.setColor(room_colour)
                        green_room = room
                        found_green_room = True
                        # Edge case: when the robot finds the green room first, it won't set the red room object
                        # and thus room navigation may fail
                        if (found_red_room == False):
                            red_room = rooms[i + 1]
                            red_room.setColor("red")
                            found_red_room = True
                        # Move to center of green room
                        self.mover.goTo(green_room.center)
                        break
                    elif (room_colour == "red"):
                        room.setColor(room_colour)
                        red_room = room
                        found_red_room = True
                        break
                    if (not self.in_simulation):
                        # Rotate slower on the real robot
                        self.mover.rotate(angle=36.1, time=2)
                    else:
                        self.mover.rotate(angle=36.1, time=1)
                if (found_green_room):
                    break
        
        if (self.debug):
            print(f"\n[{green_room.color}] room center point: {green_room.center}\n")
            print(f"[{red_room.color}] room center point: {red_room.center}\n")
            
        self.red_room = red_room
        self.green_room = green_room
        
        return green_room, red_room
    
    def rotate_and_search_for_character_frame(self, forward_step_size: float):
        """
        Rotate the robot at its current position and check whether it can see a character frame. If it can,
        the robot will change its angle such that the frame is in the center of the camera and then will slowly approach it.
        Returns a list of faces which were detected and whether a frame was found.
        """
        
        found_frame = False
        centered_frame = False
        n = 10 # Number of steps to take when doing a 360 rotation
        
        if self.debug:
            print("\nDoing a 360 while scanning")
        
        for _ in range(n + 1):
            
            # Live camera feed window continuous re-render
            self.display_camera_feed()
                
            faces = Vision.detect_faces_in_image(image=self.camera.cv_image)
            if (len(faces) > 0):
                break
            
            # Check whether in current frame at current angle there is a frame in sight
            frame_position = self.colour_detector.character_frame_colour_position_in_image(self.camera.cv_image)                
            if (frame_position is not None):
                
                if self.debug:
                    print(f"\nDetected frame in {frame_position} of camera image")
                
                if (frame_position == "CENTER"):
                    found_frame = True
                    centered_frame = True
                
                if (frame_position != "CENTER"):
                    
                    if self.debug:
                        print(f"\nCentering frame")
                        
                    # Rotate until the character frame is in the center of the camera
                    for step in range(37):
                        # Live camera feed window continuous re-render
                        self.display_camera_feed()
                            
                        frame_position = self.colour_detector.character_frame_colour_position_in_image(self.camera.cv_image)
                        
                        if (frame_position == "CENTER"):
                            if self.debug:
                                print(f"\nCentered frame")
                            found_frame = True
                            centered_frame = True
                            break
                        
                        if (not self.in_simulation):
                            self.mover.rotate(angle=8, time=2)
                        else:
                            self.mover.rotate(angle=10, time=1)
                            
                # The frame is in the center of the image, so check whether a face can be detected
                if (centered_frame):
                    faces = Vision.detect_faces_in_image(image=self.camera.cv_image)
                    if (len(faces) > 0):
                        break
                    else:
                        # Move towards frame until face is detected
                        # or forward translation was interrupted by obstacle detector
                        obstacle_ahead = False
                        while (len(faces) == 0 and frame_position is not None):
                            self.display_camera_feed()
                            status = self.mover.forwardUntilPotentialCollisionAbort()
                            frame_position = self.colour_detector.character_frame_colour_position_in_image(self.camera.cv_image)                                  
                            faces = Vision.detect_faces_in_image(image=self.camera.cv_image)
                            
                            # If obstacle ahead, stop advancing and take distance from it
                            if (status < 0):
                                if self.debug:
                                    print(f"\nDistance to obstacle lower than limit but frame in sight")
                                
                                obstacle_ahead = True
                                self.mover.rotate(angle=180, time=2)
                                self.mover.forward(distance=1.3, time=3)
                                break
                            
                                
                        if (len(faces) > 0 or frame_position is None or obstacle_ahead):
                            break
                    
            if (not self.in_simulation):
                self.mover.rotate(angle=36, time=2)
            else:
                self.mover.rotate(angle=36, time=1)
        
        return faces, found_frame
    
    def move_to_next_point_in_room(self):   
        a = [-90, -45, 0, 45, 90]
        a = a[random.randint(0,len(a)-1)]
        if a != 0:
            if (not self.in_simulation):
                self.mover.rotate(angle=a, time=10*abs(abs(a)/360) + 1)
            else:
                self.mover.rotate(angle=a, time=10*abs(abs(a)/360))
        
        while (self.mover.distance > 1.5):
            if self.debug:
                print("\nMoving 1")
            
            if (not self.in_simulation):
                self.mover.forward(distance=1, time=5)
            else:
                self.mover.forward(distance=1, time=2)
            # Avoid exiting the room
            if self.mover.dist((self.mover.pose['x'], self.mover.pose['y']), self.green_room.entrance) < 2:
                if self.debug:
                    print("\nMoving away from room exit")
                Movement.navigate_room(self.mover, self.green_room, self.red_room)
                a = random.uniform(-180, 180)
                
                if (not self.in_simulation):
                    self.mover.rotate(angle=a, time=10*abs(abs(a)/360) + 1)
                else:
                    self.mover.rotate(angle=a, time=10*abs(abs(a)/360))
    
    def search_room(self, n_search, n_angles, prev_angle, space_angle):
        
        if self.debug:
            print("\nFacing wall and scanning room from its center point")
            
        # Current distance from wall to start search is predetermined at 2 >> 
        # it could use scan_dist values to select between this and random methods
        space_angle, space_dist, scan_dist = self.mover.find_spaces(steps=12)
        
        if n_search == 1 or n_search == 2:
            wall_dist = 2
            search_tolerance = 0.65
            
            if n_angles == 0:
                if n_search == 1:
                    search_steps = 4
                elif n_search == 2:
                    search_steps = 12
            
                if self.debug:
                    print(f"\nStarting search #{n_search}: steps={search_steps}")
                    
                self.mover.face_wall(wall_distance=wall_dist)
                space_angle, space_dist, all_distances = self.mover.find_spaces(dist=wall_dist+search_tolerance)
                
                if self.debug:
                    print("\nFound ", space_angle, " angles")

            angle = space_angle[n_angles]
            rot = abs(angle - prev_angle)
            
            if self.debug:
                print("\nRotating to ", angle)
                
            self.mover.rotate(angle=angle, time=0.25+angle*0.05)
            self.mover.fullStop(0.5)
            prev_angle = angle
            n_angles += 1
            
            if self.debug:
                print("\Moving to space")
            reps = 0
            while self.mover.range_distance >= 1.25:
                reps += 1
                self.mover.forward(velocity=0.3, time=0.5)
                moved_dist = 0.3*0.5*reps
                if moved_dist > 3:
                    break
            
            self.mover.fullStop(1)

        else:
            self.move_to_next_point_in_room()
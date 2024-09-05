from __future__ import division
import cv2
import numpy as np

class ColourDetection():

    # Colour sensitivity, reduced from the usual 10 as cones can sometimes appear in some very specific light conditions
    sensitivity = 5

    # Upper and lower bounds for green mask (different from lab, unsure why)
    hsv_green_lower = np.array([36 - sensitivity, 0, 0])
    hsv_green_upper = np.array([86 + sensitivity, 255, 255])
    
    # Upper and lower bounds for red mask
    hsv_red_lower = np.array([0, 100, 100])
    hsv_red_upper = np.array([0 + sensitivity, 255, 255])
    
    # Upper and lower bounds for purple mask
    hsv_purple_lower = np.array([130 - sensitivity, 100, 100])
    hsv_purple_upper = np.array([140 + sensitivity, 255, 255])
    
    # Upper and lower bounds for blue mask
    hsv_blue_lower = np.array([90 - sensitivity, 100, 100])
    hsv_blue_upper = np.array([110 + sensitivity, 255, 255])
    
    # Upper and lower bounds for yellow mask
    hsv_yellow_lower = np.array([22, 100, 20])
    hsv_yellow_upper = np.array([30 + sensitivity, 255, 255])

    def __init__(self, debug=False):
        self.debug = debug
        self.green_found = False
        self.red_found = False
        self.colour = ""

    @staticmethod
    def white_balance(image):
        """
        Method to apply automatic white balance to ensure robust solution against lighting changes
        Adapted from: https://stackoverflow.com/questions/46390779/automatic-white-balancing-with-grayworld-assumption
        """
        result = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        avg_a = np.average(result[:, :, 1])
        avg_b = np.average(result[:, :, 2])
        result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
        return result
    
    def identify_colour(self, image):
        """
        Method to identify green and red circles
        """

        wb_corrected_img = ColourDetection.white_balance(image)
        hsv_image = cv2.cvtColor(wb_corrected_img, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv_image, self.hsv_green_lower, self.hsv_green_upper)
        mask_red = cv2.inRange(hsv_image, self.hsv_red_lower, self.hsv_red_upper)
        
        greencontours, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        redcontours, _ = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(greencontours) > 0:
            g = max(greencontours, key=cv2.contourArea)
            if cv2.contourArea(g) > 750: 
                self.green_found = True
                self.colour = "green"
                if (self.debug):
                    print("\nIdentified the GREEN room")
                return "green"

        if len(redcontours) > 0:
            r = max(redcontours, key=cv2.contourArea)
            if cv2.contourArea(r) > 750:
                self.red_found = True 
                self.colour = "red"
                if (self.debug):
                    print("\nIdentified the RED room")
                return "red"
    
    def character_frame_colour_position_in_image(self, image):
        """
        Method to identify the coloured character frames by masking each specific colour found in the asset character files.
        Returns a string which states in which side of the input image the colour frame is found: "LEFT", "RIGHT", "CENTER".
        Returns None if a character frame of the given colours is not present in the input image.
        """

        wb_corrected_img = ColourDetection.white_balance(image)
        hsv_image = cv2.cvtColor(wb_corrected_img, cv2.COLOR_BGR2HSV)

        mask_red = cv2.inRange(hsv_image, self.hsv_red_lower, self.hsv_red_upper)
        mask_purple = cv2.inRange(hsv_image, self.hsv_purple_lower, self.hsv_purple_upper)
        mask_blue = cv2.inRange(hsv_image, self.hsv_blue_lower, self.hsv_blue_upper)
        mask_yellow = cv2.inRange(hsv_image, self.hsv_yellow_lower, self.hsv_yellow_upper)
        
        red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        purple_contours, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        contours = [red_contours, purple_contours, blue_contours, yellow_contours]
        
        img_height, img_width = image.shape[:2]
        input_image_center = ((int)(img_width/2), (int)(img_height/2))

        for contour in contours:
            if len(contour) > 0:
                
                col = max(contour, key=cv2.contourArea)
                contour_area = cv2.contourArea(col)
                
                if (self.debug):
                    print(f"\nContour observed with area {contour_area}")
                    
                if contour_area > 500:
                    
                    # Use the image moments to find the centroid of the coloured frame contour
                    M = cv2.moments(col)
                    cx = int(M['m10'] / M['m00'])
                    
                    if (cx >= (input_image_center[0] - 50) and cx <= (input_image_center[0] + 50)):
                        # Frame colour is in center of image
                        if (self.debug):
                            print("\nIdentified character frame colour in the CENTER of the input image")
                        return "CENTER"
                    elif (cx <= input_image_center[0]):
                        # Frame colour is in left-hand side of image
                        if (self.debug):
                            print("\nIdentified character frame colour in the LEFT side of the input image")
                        return "LEFT"
                    elif (cx >= input_image_center[0]):
                        # Frame colour is in right-hand side of image
                        if (self.debug):
                            print("\nIdentified character frame colour in the RIGHT side of the input image")
                        return "RIGHT"
        
        if (self.debug):
            print("\nNo character frame colour identified in the input image")
            
        return None
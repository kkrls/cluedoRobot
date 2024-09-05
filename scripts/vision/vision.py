from __future__ import division
import cv2
import numpy as np
import glob
import os

class Vision():
    """
    ### Class which provides an interface with methods to process the CV2 image data in various ways:
        - check if specific colour is in image (TODO: Konstantinos - as part of circle recognition)
        - calculates average of RGB or HSV values in image 
        - detects faces in image
        - labels face in image against the available cluedo character asset files
    """
    
    @staticmethod
    def classify_face(face_image, output_label=False):
        """
        Given an input image of a face in cv2 format, compare the input face with all available faces in `cluedo_images/`
        and determine which character is present. 
        
        Returns a list of length 2 where the first element is the label of the character
        and the second element is a cv2 image object of the captured face.
        
        If `output_label` is set to true, then the method will output the character label to the terminal running the script.
        """
        images_folder = os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images')
        result = [None, face_image] # label of identified cluedo character, cv2 image
        max_similarity = float('-inf')
        
        for filepath in glob.iglob(f"{images_folder}/*.png"):
            
            # Get label and image of asset file
            asset_character_image = cv2.imread(filepath)
            asset_character_label = filepath.split('/')[-1].replace(".png", "") # scarlet, plum, etc.
            
            # Find face in given asset image
            asset_character_face = Vision.detect_faces_in_image(asset_character_image)
            
            # Get image of bounded face from asset
            asset_character_face_image = Vision.image_from_face_bounding_box(asset_character_image, asset_character_face[0])
            
            # Compare asset face to given face by comparing their histograms
            dim = (face_image.shape[0], face_image.shape[1])
            asset_character_face_image = cv2.resize(asset_character_face_image, dim, interpolation=cv2.INTER_AREA)
            
            # Calculate similarity between the asset image face and the input image face
            similarity = Vision.compare_img_histograms(asset_character_face_image, face_image)
            
            # If the similarity between the input face and the current asset face is max, update computed class
            if (similarity > max_similarity):
                result[0] = asset_character_label
                max_similarity = similarity
                
        if (output_label == True):
            print(f"Detected Character -> {result[0]}")
        
        return result
            
    @staticmethod
    def detect_faces_in_image(image):
        """
        Get a list of coordinates of faces in the input image using a default trained Haarcascade Frontal Face Classifier.
        Returns an empty list if no faces are detected in the input image.
        
        Sources: 
        
            1. https://towardsdatascience.com/face-detection-in-2-minutes-using-opencv-python-90f89d7c0f81
            2. https://stackoverflow.com/questions/4440283/how-to-choose-the-cascade-file-for-face-detection
        """
        if image is None:
            return []
        
        # Load face detector model
        classifier = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # The cascade classifier only works on grayscale images, so convert input to grayscale
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces in input image
        faces = classifier.detectMultiScale(
            grayscale_image, 
            scaleFactor=1.1, 
            minNeighbors=10,
            minSize=(10, 10)
        ) # contains a list of coordinates for the rectangles which contain faces
        
        return faces

    @staticmethod
    def image_from_face_bounding_box(image, face):
        """
        Returns an image object of the face in a bounding box
        """
        return image[face[1]:face[1]+face[3], face[0]:face[0]+face[2]]

    @staticmethod
    def add_bound_box_to_faces_in_image(image, faces, threshold=1000):
        """
        Add bounding box rectangles to the face coordinates in the given input image.
        The `threshold` dictates the area of the rectangle (higher values = bigger rectangle = face is closer on z-axis)
        """
        for (x, y, w, h) in faces:
            if (w * h > threshold):
                image = cv2.rectangle(image, (x,y), (x+w, y+h), (255, 0, 0), 2)
        return
        
    @staticmethod
    def show_image_in_window(image, image_title):
        """
        Displays the given image data in a cv2 window identified by `image_title`.
        
        NOTE THAT THIS IS A BLOCKING CALL - Program execution will only resume after a key press
        """
        cv2.imshow(image_title, image)
        cv2.waitKey(1)
    
    @staticmethod
    def compare_img_histograms(image1, image2):
        """
        Calculates the similarity between two images by comparing their histograms in the RGB colour space.
        """
        histogram_image1 = cv2.calcHist([image1], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
        cv2.normalize(histogram_image1, histogram_image1, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        
        histogram_image2 = cv2.calcHist([image2], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
        cv2.normalize(histogram_image2, histogram_image2, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        
        similarity = cv2.compareHist(histogram_image1, histogram_image2, cv2.HISTCMP_INTERSECT)
        
        return similarity
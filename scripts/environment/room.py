class Room():
    
    def __init__(self, entrance, center):
        self.entrance = entrance
        self.center = center
        self.color = None
    
    def setColor(self, color):
        self.color = color
    
    @staticmethod
    def load_rooms(points):
        """
        Returns a list of Rooms which are initialised with the coodinates of the rooms from the input points.
        """
        rooms = []
        rooms.append(Room(points["room1_entrance_xy"], points["room1_centre_xy"]))
        rooms.append(Room(points["room2_entrance_xy"], points["room2_centre_xy"]))
        return rooms
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import actionlib
from actionlib_msgs.msg import *
from math import pi
import numpy as np
import random, math
from environment.room import Room

class Movement():
    def __init__(self, frequency=10, debug=False):
        self.debug = debug
        if self.debug:
            print("Starting Movement Constructor")
        rospy.on_shutdown(self.shutdown)
        
        self.frequency = frequency
        self.rate = rospy.Rate(self.frequency)
        self.lock=False
        self.halt=False
        # laser
        self.center = 10
        self.left = 10
        self.right = 10
        self.distance = None
        self.range_distance = None
        self.wall_ahead = False
        
        if self.debug:
            print("Frequency set")
        
        self.lastTwist = Twist()
        self.newTwist = Twist()
        
        if self.debug:
            print("Twists initialized")
        
        self.velocityPub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        if self.debug:
            print("Velocity Publisher initialized")
        self.bumperSub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.handleCollision)
        if self.debug:
            print("Bumper Subscriber initialized")
        self.poseSub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        if self.debug:
            print("Pose Subscriber initialized")
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallback)
        if self.debug:
            print("Laser Subscriber initialized")
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if self.debug:
            print("Simple Action Client initialized")
        self.fullStop()
        self.goTo((0,0), timeout=1)
    
    def poseCallback(self,data=None):
        pose = {"x": data.pose.pose.position.x,"y": data.pose.pose.position.y,"angle": (180/pi) * np.arcsin(2* (data.pose.pose.orientation.w*data.pose.pose.orientation.z-data.pose.pose.orientation.x*data.pose.pose.orientation.z))}
        #if self.debug:
        #    print("Current Pose:",pose)
        self.pose = pose
        try:
            dist = np.sqrt((self.pose["x"]-self.goal["x"])**2 + (self.pose["y"]-self.goal["y"])**2)
            if self.goal["angle"] is None and dist<0.15:
                self.fullStop()
        except:
            pass

    def laserCallback(self, data=None):
        """
        left / right = -/+ ~30deg
        range max = 4.5~5 m      
        at 0.4m/s if wall_ahead at 0.99m >> full stop at 0.75~0.8m
        at 0.5m/s if wall_ahead at 0.97m >> full stop at 0.65m
        
        UPDATE: robot stops at wall_dist=0.7 to prevent collisions,
                but if moving the robot in a loop to move robot select a higher value (min. wall_dist=1)
        """
        self.center = data.ranges[len(data.ranges)//2]
        if not (0.1 < self.center < 5): self.center = 10
        self.left = data.ranges[len(data.ranges)-1]
        if not (0.1 < self.left < 5): self.left = 10
        self.right = data.ranges[0]
        if not (0.1 < self.right < 5): self.right = 10
        
        self.distance = min([self.left, self.center, self.right])
        
        # handle potential collision >> self.range_distance instead of self.distance to avoid blind spots
        self.range_distance = min(data.ranges)
        if self.range_distance < 0.7:
            self.wall_ahead = True
            if self.debug:
                print("Potential collision detected")
            try:
                self.move_base.cancel_all_goals()
            except:
                if self.debug:
                    print("\nFailed to cancel all move base goals")
            finally:
                self.fullStop()
    
    def handleCollision(self,data=None):
        try:
            self.move_base.cancel_all_goals()
        except:
            pass
        if self.debug:
            print("Collision Detected")
        self.halt = True
        self.move(twist = self.flipTwist((self.newTwist)), emergency=True, time=1)
        self.fullStop()
        self.halt = False
        self.rotate(angle=360, time=10)
    
    def walls_visible(self, dist=10):
        scan_LCR = [self.left, self.center, self.right]
        bin_LCR = [0]*3
        for i, val in enumerate(scan_LCR):
            bin_LCR[i] = 0 if val >= dist else 1    
        count_walls = sum(bin_LCR)

        return count_walls, bin_LCR, scan_LCR

    def face_wall(self, wall_distance=None, angle=None):
        if wall_distance is not None:
            if (self.debug):
                print(f"Approaching wall {self.center:.3f}")
            while self.range_distance > wall_distance:
                self.forward(velocity=0.3, time=0.5)
            
            self.fullStop(1)
        
        centered = False
        
        if (self.debug):
            print(f"Facing wall {self.center:.3f}")
            
        i = 0
        while not centered and i < 15:
            if (self.left > self.center and self.center < self.right):
                i+=1
                # adjust center
                if self.left < self.right:
                    self.rotate(angle=1, time=0.5)
                else:
                    self.rotate(angle=-1, time=0.5)
                if (self.debug):
                    print(f"Tuning center... {self.left:.3f} {self.center:.3f} {self.right:.3f}")
                if abs(self.right - self.left) <= (0.2*self.range_distance):
                    centered = True
                    break
            
            elif (self.left < self.center and self.center > self.right) or self.distance == 10:
                i+=1
                if self.left < self.right:
                    self.rotate(angle=45, time=2)
                else:
                    self.rotate(angle=-45, time=2)
                if (self.debug):
                    print(f"Facing empty direction... {self.left:.3f} {self.center:.3f} {self.right:.3f}")
            
            else:
                i+=1
                if self.left < self.center:
                    self.rotate(angle=10, time=0.5)
                if self.right < self.center:
                    self.rotate(angle=-10, time=0.5)
                if (self.debug):
                    print(f"Centering... {self.left:.3f} {self.center:.3f} {self.right:.3f}")
        self.fullStop(1)

        if wall_distance is not None:
            while not self.wall_ahead and abs(wall_distance - self.distance) >= 0.3:
                if (self.debug):
                    print(f"moving to wall distance: {self.center:.3f}")
                if self.center > wall_distance:
                    self.forward(velocity=0.25, time=0.5)
                if self.distance < wall_distance:
                    self.forward(velocity=-0.25, time=0.5)

        self.fullStop(1)

        if (self.debug):
            print("Centered")
            
        if angle != None:
            self.rotate(angle, time=2)
            if (self.debug):
                print(f"at angle: {angle}")
            self.fullStop(1)
            
    def find_spaces(self, find_empty=True, steps=4, dist=10, all_dist=[]):
        """
        Find empty spaces or obstacles around robot at a certain distance.
        Returns lists with only the identified angles and corresponding distances, and a list with all the distances at all steps.
        """
        angle = 360/steps
        
        if len(all_dist) == 0:
            for i in range(steps):
                count_walls, bin_LCR, scan_LCR = self.walls_visible(dist=dist)
                all_dist.append(scan_LCR)
                self.rotate(angle, time=8*1/steps)
                self.fullStop(0.15)
            if self.debug:
                print(f"all distances with step={steps} and wall_dist={dist}:")
                for x in all_dist:
                    print(x)
                    
            self.fullStop(1)

        flat_distances = [item for sublist in all_dist for item in sublist]
        if steps > 8:
            if self.debug:
                print(f"swapping values")
            for i, elem in enumerate(flat_distances):
                if i % 3 == 2:
                    idx1 = i
                    idx2 = i+1 if (i != len(flat_distances)-1) else 0
                    flat_distances[idx1], flat_distances[idx2] = flat_distances[idx2], flat_distances[idx1]
                         
        space_angle = []
        space_dist = []
        for i, val in enumerate(flat_distances):
            new_angle = 360/(steps*3)
            prev_val = flat_distances[i-1] if i != 0 else flat_distances[-1]
            post_val = flat_distances[i+1] if (i != len(flat_distances)-1) else flat_distances[0]
            
            if find_empty and (val > dist or val == 10):
                if prev_val > dist and post_val > dist:
                    space_angle.append(i*new_angle)
                    space_dist.append(val)
                else:
                    continue                
            elif not find_empty and val < dist:
                if prev_val < dist and post_val < dist:
                    space_angle.append(i*new_angle)
                    space_dist.append(val)
                else:
                    continue

        return space_angle, space_dist, all_dist
    
    def rotate(self, angle=None,velocity=None, time=1):
        try:
            self.move_base.cancel_all_goals()
        except:
            pass
        if velocity == None and angle == None:
            raise ValueError("velocity and distance cannot both be None")

        if velocity != None and angle != None:
            raise ValueError("velocity and distance cannot both be given")
        
        if velocity == None and angle is not None:
            velocity = (angle * (pi/180)) / time # radians per second
        
        if time < 1/self.frequency:
            time = 1/self.frequency
        

        twist = Twist()
        
        twist.linear.x=0
        twist.linear.y=0
        twist.linear.z=0
        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=velocity
        
        self.move(twist,time)
        self.fullStop()
        
        return

    def forward(self, distance=None, velocity=None, time=1):
        try:
            self.move_base.cancel_all_goals()
        except:
            pass
        if velocity == None and distance == None:
            raise ValueError("velocity and distance cannot both be None")

        if velocity != None and distance != None:
            raise ValueError("velocity and distance cannot both be given")
        
        if time < 1/self.frequency:
            time = 1/self.frequency
        
        if velocity == None and distance is not None:
            velocity = distance/time

        twist = Twist()
        
        twist.linear.x=velocity
        twist.linear.y=0
        twist.linear.z=0
        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=0
        
        self.move(twist,time)
        return
    
    def forwardUntilPotentialCollisionAbort(self):
        """
        Function similar to `.forward()` but with a check for collision before initiating the translation.
        Uses the robot laser callback data.
        Returns 0 if the translation was successful, -1 if interrupted because of collision detection.
        """
        reps = 0
        while self.range_distance >= 0.6:
            reps += 1
            self.forward(velocity=0.3, time=0.5)
            moved_dist = 0.3*0.5*reps
            if moved_dist > 3:
                break
        
        if (self.range_distance < 0.6):
            return -1
        
        return 0

    def fullStop(self,time=0):
        try:
            self.move_base.cancel_all_goals()
        except:
            pass
        if time < 1/self.frequency:
            time = 1/self.frequency
        
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        
        self.move(twist, time)
        return
    
    def flipTwist(self,twist):
        if twist == None:
            return
        flipped = Twist()
        
        flipped.linear.x = twist.linear.x * -1
        flipped.linear.y = twist.linear.y * -1
        flipped.linear.z = twist.linear.z * -1
        flipped.angular.x = twist.angular.x * -1
        flipped.angular.y = twist.angular.y * -1
        flipped.angular.z = twist.angular.z * -1
        
        return flipped
    
    def move(self, twist = None, time=1, emergency=False):
        if twist is None:
            return
        if emergency:
            self.halt = False
            self.lock = False
        if self.lock:
            return
        # if self.wall_ahead:
        #     return
        self.lock = True
        self.oldTwist = self.newTwist
        self.newTwist = twist
        # Stop Twist received, execute immediately with priority
        if (twist.linear.x == 0 and twist.angular.z == 0):
            self.velocityPub.publish(self.newTwist)
        else:
            #if self.debug:
            #    print("Moving for", time, "seconds with twist:\n",twist)
            for _ in range(int(time/(1/self.frequency))):
                if self.halt:
                    return
                self.velocityPub.publish(self.newTwist)
                self.rate.sleep()
        self.lock = False
    
    def goTo(self, point, angle=90, timeout=0):
        self.goal = {"x":point[0],"y":point[1], "angle":angle}
        if self.debug:
            print("Moving to:",point[0],point[1],angle)
        if angle is None:
            angle=0
        angle = float(angle)
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(angle/2.0), 'r4' : np.cos(angle/2.0)}
        if self.debug:
            print("Quaternion Computed")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = Pose(Point(float(point[0]), float(point[1]), 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        if self.debug:
            print("Goal Computed")
        self.move_base.send_goal(goal)
        if self.debug:
            print("Goal Sent")
        
        #if timeout==0:
        #    return
        
        result = self.move_base.wait_for_result(rospy.Duration(timeout)) 
        state = self.move_base.get_state()

        if self.debug:
            print("Goal Attempted")
        
        if self.debug:
            print("result:",result)
            print("state:",state)
        
        if result and state == GoalStatus.SUCCEEDED:
            if self.debug:
                print("Succeeded")
            self.fullStop()
            return True
        else:
            self.move_base.cancel_all_goals()
            if self.debug:
                print("Failed")
            self.fullStop()
            return False
    
    def shutdown(self):
        self.fullStop()
    
    
    @staticmethod
    def dist(a, b):
        x1, y1 = a
        x2, y2 = b
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance
    
    @staticmethod
    def navigate_room(movement, roomA: Room, roomB: Room):
        """
        Moves the input infrastructure randomly around the room 
        and returns the point coordinates to which the infrastructure translated
        """
        
        points = []
        weights = {}
        
        for i in range(100):
            point = (random.uniform(movement.pose['x'] -1, movement.pose['x'] +1),
                     random.uniform(movement.pose['y'] -1, movement.pose['y'] +1))
            
            weight = Movement.dist(point, roomA.entrance)**(2) 
            #+ Movement.dist(point, roomA.center) * 0.1 
            + Movement.dist(point, roomB.entrance) ** 2 
            + Movement.dist(point, roomB.center)** 2 
            + random.uniform(-1,1)
            
            points.append(point)
            weights[point]=(weight)
            
        point = max(points, key=lambda x:(weights[x]))
        movement.goTo(point)
        
        return point

if __name__ == '__main__':
    rospy.init_node("mover", anonymous=True)
    mov = Movement(debug=True)
    mov.rotate(angle=90, time=3)
    mov.forward(distance=1, time=5)
    mov.goTo(point=(0,0))
import math

SCANS_PER_SEC = 10
NUM_BRANCHES = 3
MAX_ANGLE = 34
DIRECTIONS = [0, MAX_ANGLE / 2, -MAX_ANGLE / 2, MAX_ANGLE, -MAX_ANGLE]
RANGE = 0.25 #distance in x and y direction that the current position can be from goal_position
speed = 5 #miles per hour
path = [-1,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
a = -1

#position is a 2-object list with  distance of the radius first, followed by the angle in degrees
start_position = [0,0]

#Example data: given index 
target_angle = 0 #angle_from_index
target_distance = 4 #index of array to find value
goal_position = []

class State:
    def __init__(self, position, speed, direction):
        self.position = position
        self.speed = speed
        self.direction = direction
    
    def printState(self):
        print("Current position = " + str(self.position[0]) + ", " + str(self.position[1]))

def max_depth(distance):
    global speed
    global SCANS_PER_SEC

    depth = distance / (speed / SCANS_PER_SEC)
    return depth * 1.5 #multiplied by 2 to allow to swing out wide

def main():
    global goal_position

    goal_position = calc_xy_from_polar(target_distance, target_angle)
    print(str(goal_position))
    origin_state = State(start_position, speed, 0)
    origin_state.printState()
    search(origin_state)


def calc_xy_from_polar(radius, angle):
    xPos = radius * math.sin(math.radians(angle))       #TODO: Check signs!  
    yPos = radius * math.cos(math.radians(angle))
    return [xPos, yPos]


def calc_new_position(current_state, direction_change):
    global SCANS_PER_SEC
    new_direction = current_state.position[1] + direction_change
    distance_traveled = speed / SCANS_PER_SEC

    xPos = current_state.position[0] + distance_traveled * math.sin(new_direction)
    yPos = current_state.position[1] + distance_traveled * math.cos(new_direction)

    return[xPos, yPos]

#Checks to see if current position is in a range
def in_range(current_position, goal_position):
    global RANGE

    if(current_position[0] > goal_position[0] - RANGE
        and current_position[0] < goal_position[0] + RANGE
        and current_position[1] < goal_position[1] + (RANGE)
        and current_position[1] > goal_position[1] - (RANGE)):
        return True

def search(current_state):
    global NUM_BRANCHES
    global a
    global path
    global goal_position
    global target_distance
 
    a = a + 1
    for i in range(NUM_BRANCHES):
        path[a] = i

        print(path)

        #Current position is in range of goal
        if(in_range(current_state.position, goal_position)):
            return True

        #Place boundaries
        if(current_state.position[1] < 0):
            a = a - 1
            return False


        #Set max depth to 5
        if(a >= max_depth(target_distance)):
            print("FALSE")
            a = a - 1
            return False
        if(current_state.position[0] == goal_position[0] and current_state.position[1] == goal_position[1]):
            return True
        
        print("Current State:")
        current_state.printState()

        print(str(i) + " - " + "Direction of turn: " + str(DIRECTIONS[i]))
        print("New direction: " + str(current_state.direction + DIRECTIONS[i]))

        new_position = calc_new_position(current_state, DIRECTIONS[i])
        newState = State(new_position, speed, current_state.direction + DIRECTIONS[i])

        print("New State:")
        newState.printState()

        if(new_position[1] < current_state.position[1]):
            print("BACKWARDS")
            continue

        if(search(newState)):
            print("FOUND")
            return True
        

    print("NO OPTION HERE")
    a = a - 1
    return False

    

    #new_pos = calc_new_position(current_state.position, xVel, yVel)





#  CODE FROM DISPARITY EXTENDER:
#   Given an index in the array of the longest disparity distance
#   target_index = self.find_widest_disparity_index()
#   target_angle = self.adjust_angle_for_car_side(target_angle)
#   steering_percentage = self.degrees_to_steering_percentage(target_angle)
#   msg.angle = steering_percentage
#   1.0 - ((degrees + max_angle) / (2 * max_angle))
#   new_pos = calc_new_position(current_state.position, xVel, yVel)


# This is the arc width of the full LIDAR scan data, in degrees
#        self.scan_width = 270.0


main()
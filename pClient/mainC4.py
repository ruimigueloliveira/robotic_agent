import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    closest_direction = "N"
    start = False
    in_left = 0
    in_right = 0
    current_out_left = 0
    current_out_right = 0
    movement_model_x = 13
    movement_model_y = 27
    media_x = 13
    media_y = 27
    previous_theta = 0
    movement_model_theta = 0
    origin_x = 0
    origin_y = 0
    sensors_x = 13
    sensors_y = 27
    wall_diameter = 0.1
    robot_radius = 0.5

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    # Initializion
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.start == False:
                self.start = True
                self.origin_x = self.measures.x
                self.origin_y = self.measures.y 

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.main()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotorsUpdate(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.main()

    # Main 
    def main(self):
        self.localization()
        self.movement()

    # Calculation of final coordinates
    def localization(self):
        self.movementModel()
        self.nearestDirectionEstimate()
        self.sensorsCorrection()


        gps_x = self.measures.x - (self.origin_x - 13)
        gps_y = self.measures.y - (self.origin_y - 27)

        self.media_x = (self.movement_model_x + self.sensors_x) / 2
        self.media_y = (self.movement_model_y + self.sensors_y) / 2

        dif_x = abs(gps_x - self.media_x)
        dif_y = abs(gps_y - self.media_y)
        

        print("\n###############################################")
        print("front: ", self.measures.irSensor[0])
        print("left: ", self.measures.irSensor[1])
        print("right: ", self.measures.irSensor[2])
        print("back: ", self.measures.irSensor[3])
        print("gps x                : ", round(gps_x,1))
        # print("movement model x     : ", round(self.movement_model_x,1))
        # print("sensors x            : ", self.sensors_x)
        print("media x              : ", round(self.media_x,1))
        print("dif x                : ", round(dif_x,1))
        print("---------------------------------------------")
        print("gps y                : ", round(gps_y,1))
        # print("movement model y     : ", round(self.movement_model_y,1))
        # print("sensors y            : ", self.sensors_y)
        print("media y              : ", round(self.media_y,1))
        print("dif y                : ", round(dif_y,1))
        # print("---------------------------------------------")
        # print("compass              : ", self.measures.compass)
        # print("movement model theta : ", math.degrees(self.movement_model_theta))
        print("###############################################")

    # Calculation of coordinates and theta from movement model
    def movementModel(self):
        # Local Variables
        previous_x = self.movement_model_x
        previous_y = self.movement_model_y
        previous_theta = self.movement_model_theta
        previous_out_left = self.current_out_left
        previous_out_right = self.current_out_right

        # Lin Calculation
        self.current_out_left = (self.in_left + previous_out_left)/2
        self.current_out_right = (self.in_right + previous_out_right)/2
        lin = (self.current_out_left + self.current_out_right)/2

        # Theta Calculation
        rot = (self.current_out_right - self.current_out_left)/1
        
        if abs((previous_theta + rot) + math.radians(self.measures.compass)) == abs((previous_theta + rot)) + abs(math.radians(self.measures.compass)): # Check if both angles have the same sign for mean
            self.movement_model_theta = ((previous_theta + rot) + math.radians(self.measures.compass))/2 
        else:
            self.movement_model_theta = (previous_theta + rot)
        
        if self.movement_model_theta < -(math.pi):
            self.movement_model_theta = -(self.movement_model_theta)
        elif self.movement_model_theta > math.pi:
            self.movement_model_theta = -(self.movement_model_theta)

        # Current coordinates
        self.movement_model_x = previous_x + (lin * math.cos(previous_theta))
        self.movement_model_y = previous_y + (lin * math.sin(previous_theta))

    # Correction of coordinates from sensors
    def sensorsCorrection(self):
        # Local Variables
        closest_x_wall_coordinate = 0
        closest_x_wall_distance = 0
        closest_y_wall_coordinate = 0
        closest_y_wall_distance = 0

        # In case there is no correction the sensors value is the same as the movement model
        self.sensors_x = self.movement_model_x
        self.sensors_y = self.movement_model_y
        
        # Direction N
        if self.closest_direction == "N":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

        # Direction S
        if self.closest_direction == "S":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius
        
        # Direction W
        if self.closest_direction == "W":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

        # Direction E
        if self.closest_direction == "E":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius
        
        # # print("closest_x_wall_coordinate: ", closest_x_wall_coordinate)
        # # print("closest_y_wall_coordinate: ", closest_y_wall_coordinate)

    # Robot movement algorithm
    def movement(self):

        if self.measures.irSensor[0] > 2\
            or self.measures.irSensor[1]   > 2.7\
            or self.measures.irSensor[2]  > 2.7:
            # print("Alto Perigo")
            if self.measures.irSensor[1] > self.measures.irSensor[2]:
                self.driveMotorsUpdate(0.15,-0.13)
            else:
                self.driveMotorsUpdate(-0.13,0.15)

        elif self.measures.irSensor[0] > 1.1\
            or self.measures.irSensor[1]   > 2.7\
            or self.measures.irSensor[2]  > 2.7:
            # print("Medio Perigo")
            if self.measures.irSensor[1] > self.measures.irSensor[2]:
                self.driveMotorsUpdate(0.15, -0.06)
            else: 
                self.driveMotorsUpdate(-0.06,0.15)

        elif self.measures.irSensor[0] > 0.6\
            or self.measures.irSensor[1]   > 2.6\
            or self.measures.irSensor[2]  > 2.6:
            # print("Baixo Perigo")
            if self.measures.irSensor[1] > self.measures.irSensor[2]:
                self.driveMotorsUpdate(0.15,0.08)
            else: 
                self.driveMotorsUpdate(0.08,0.15)

        elif self.measures.irSensor[0] > 0.5\
            or self.measures.irSensor[1]   > 2.1\
            or self.measures.irSensor[2]  > 2.1:
            # print("Sem Perigo")
            if self.measures.irSensor[1] > self.measures.irSensor[2]:
                self.driveMotorsUpdate(0.15,0.145)
            else:
                self.driveMotorsUpdate(0.145,0.15)
        
        else:
            self.driveMotorsUpdate(0.15,0.15)

    # Drive Motors and aand updating the power input variables for the movement model
    def driveMotorsUpdate(self, in_left, in_right):
        self.in_left = in_left
        self.in_right = in_right
        self.driveMotors(self.in_left , self.in_right)

    # Nearest direction estimate
    def nearestDirectionEstimate(self):
        ls = [0,90,180,-180,-90]
        closest_direction_val = min(ls, key=lambda x:abs(x-math.degrees(self.movement_model_theta)))
        # print("closest_direction_val: ", closest_direction_val)
        if closest_direction_val == 0:
            self.closest_direction = "N"
        elif closest_direction_val == 90:
            self.closest_direction = "W"
        elif closest_direction_val == 180 or closest_direction_val == -180:
            self.closest_direction = "S"
        elif closest_direction_val == -90:
            self.closest_direction = "E"
        # print("closest direction: ", self.closest_direction)

    # Writing output files
    def writeOutputFiles(self):
        with open(outfilemap, 'w') as out:
            out.write("map")
            out.write('\n')

        with open(outfilepath, 'w') as out:
            out.write("path")
            out.write('\n')

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
outfilemap = "outfilemap"
outfilepath = "outfilepath"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f1") and i != len(sys.argv) - 1:
        outfilemap = (sys.argv[i + 1])
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f2") and i != len(sys.argv) - 1:
        outfilepath = (sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

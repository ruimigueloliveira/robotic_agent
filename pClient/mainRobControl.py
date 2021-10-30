
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14


class MyRob(CRobLinkAngs):
    flag = 0
    lap = 0
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
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
                self.control()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.control()
            
    def control(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        print("    Voltas feitas: ", self.lap)

        print("front: ",  self.measures.irSensor[center_id])
        print("left: ", self.measures.irSensor[left_id])
        print("right: ", self.measures.irSensor[right_id])

        if (self.flag == 2) and (self.measures.ground == 0):
            self.flag = 0
            self.lap = self.lap + 1
            
        if(self.measures.ground == 2):
            self.flag = 2

        if self.measures.irSensor[center_id] > 2\
            or self.measures.irSensor[left_id]   > 2.6\
            or self.measures.irSensor[right_id]  > 2.6:
            print("Alto Perigo")
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                self.driveMotors(0.15,-0.14)
            else:
                self.driveMotors(-0.14,0.15)

        elif self.measures.irSensor[center_id] > 1.1\
            or self.measures.irSensor[left_id]   > 2.5\
            or self.measures.irSensor[right_id]  > 2.5:
            print("Medio Perigo")
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                self.driveMotors(0.15,-0.07)
            else: 
                self.driveMotors(-0.07,0.15)

        elif self.measures.irSensor[center_id] > 0.6\
            or self.measures.irSensor[left_id]   > 2.3\
            or self.measures.irSensor[right_id]  > 2.3:
            print("Baixo Perigo")
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                self.driveMotors(0.15,0.07)
            else: 
                self.driveMotors(0.07,0.15)

        elif self.measures.irSensor[center_id] > 0.5\
            or self.measures.irSensor[left_id]   > 2.1\
            or self.measures.irSensor[right_id]  > 2.1:
            print("Sem Perigo")
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                self.driveMotors(0.15,0.14)
            else:
                self.driveMotors(0.14,0.15)
        else:
            self.driveMotors(0.15,0.15)


    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
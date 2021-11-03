
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14


class MyRob(CRobLinkAngs):
    flag = 0
    stop = False
    xorigem = 0
    yorigem = 0
    rodando = False
    direcao = "North"
    
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

            if self.flag == 0:
                self.xorigem = self.measures.x
                self.yorigem = self.measures.y
                self.flag = 1
                print("x origem", self.xorigem)
                print("y origem", self.yorigem)

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
                self.mapping()
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
                self.mapping()
            
    def mapping(self):
        # self.driveMotors(-0.01,0.01)
        # print("self.measures.compass: ", self.measures.compass)
        if self.direcao == "North":
            self.goingNorth()
        elif self.direcao == "West":
            self.goingWest()
        elif self.direcao == "South":
            self.goingSouth()
        elif self.direcao == "East":
            self.goingEast()


    def goingNorth(self):
        if self.rodando == True:
            if self.measures.compass == -91 or self.measures.compass == -90 or self.measures.compass == -89:
                self.direcao = "West"
                self.rodando = False
                self.stop = False
                self.xorigem = self.xorigem - 2
            else:
                self.driveMotors(0.01,-0.01)
        elif self.measures.x == self.xorigem and self.measures.irSensor[0] < 2.0:
            print("front: ",  self.measures.irSensor[0])
            print("celula x")
            self.xorigem = self.xorigem + 2
            self.stop = False
        elif self.measures.x == self.xorigem and self.measures.irSensor[0] > 2.0:
            print("front: ",  self.measures.irSensor[0])
            self.driveMotors(0.00,0.00)
            print("celula x perigo")
            self.xorigem = self.xorigem + 2
            print("para crg")
            self.rodando = True
            self.stop = True
        elif (self.measures.x != self.xorigem) and self.stop == False:
            if self.measures.compass > 0:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < 0:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == 0:
                self.driveMotors(0.04,0.04)
        else:
            print("estranho")

    def goingWest(self):
        if self.rodando == True:
            if self.measures.compass == -179 or self.measures.compass == 180 or self.measures.compass == 179:
                self.direcao = "South"
                self.rodando = False
                self.stop = False
                self.yorigem = self.yorigem + 2
            else:
                self.driveMotors(0.01,-0.01)
        elif self.measures.y == self.yorigem and self.measures.irSensor[0] < 2.0:
            print("celula y")
            self.yorigem = self.yorigem - 2
            self.stop = False
        elif self.measures.y == self.yorigem and self.measures.irSensor[0] > 2.0:
            self.driveMotors(0.00,0.00)
            print("celula y perigo")
            self.yorigem = self.yorigem - 2
            print("para crg")
            self.rodando = True
            self.stop = True
        elif (self.measures.y != self.yorigem) and self.stop == False:
            if self.measures.compass > -90:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < -90:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == -90:
                self.driveMotors(0.04,0.04)
        else:
            print("estranho")

    def goingSouth(self):
        # print("self.measures.x: ", self.measures.x)
        # print("self.xorigem: ", self.xorigem)
        if self.rodando == True:
            print("rodando")
            if self.measures.compass == 91 or self.measures.compass == 90 or self.measures.compass == 89:
                self.direcao = "East"
                self.rodando = False
                self.stop = False
                self.xorigem = self.xorigem + 2
            else:
                self.driveMotors(0.01,-0.01)

        elif self.measures.x == self.xorigem and self.measures.irSensor[0] < 2.0:
            print("\ncelula x\n")
            self.xorigem = self.xorigem - 2
            self.stop = False
        elif self.measures.x == self.xorigem and self.measures.irSensor[0] > 2.0:
            self.driveMotors(0.00,0.00)
            print("celula x perigo")
            self.xorigem = self.xorigem - 2
            print("para crg")
            self.rodando = True
            self.stop = True
        elif (self.measures.x != self.xorigem) and self.stop == False:
            # print("self.measures.compass: ", self.measures.compass)
            if self.measures.compass > -179 and self.measures.compass < -1 :
                # print("para vira direita")
                self.driveMotors(0.03,0.04)
            elif self.measures.compass > 1 and self.measures.compass < 179:
                # print("para vira esquerda")
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == 180 or self.measures.compass == -180:
                # print("siga em frentee")
                self.driveMotors(0.04,0.04)
        else:
            print("estranho")

    def goingEast(self):
        if self.rodando == True:
            print("rodando")
            if self.measures.compass == -1 or self.measures.compass == 0 or self.measures.compass == 1:
                self.direcao = "North"
                self.rodando = False
                self.stop = False
                self.yorigem = self.yorigem - 2
            else:
                self.driveMotors(0.01,-0.01)

        elif self.measures.y == self.yorigem and self.measures.irSensor[0] < 2.0:
            print("\ncelula y\n")
            self.yorigem = self.yorigem + 2
            self.stop = False
        elif self.measures.y == self.yorigem and self.measures.irSensor[0] > 2.0:
            self.driveMotors(0.00,0.00)
            print("celula y perigo")
            self.yorigem = self.yorigem + 2
            print("para crg")
            self.rodando = True
            self.stop = True
        elif (self.measures.y != self.yorigem) and self.stop == False:
            if self.measures.compass > 90:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < 90:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == 90:
                self.driveMotors(0.04,0.04)
        else:
            print("estranho")


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

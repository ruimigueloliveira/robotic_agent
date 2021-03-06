import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math
import pathfinder
import itertools

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    rodando = False
    direcao = "North"
    center_id = 0
    left_id = 1
    right_id = 2
    back_id = 3
    proximadirecao = ""
    xorigemmatriz = 13
    yorigemmatriz = 27
    xorigemtransformada = 13
    yorigemtransformada = 27
    movement_model_x = 13
    movement_model_y = 27
    xpontoatual = 13
    ypontoatual = 27
    sensors_x = 13
    sensors_y = 27
    matrix = []
    astar_maze = []
    nosparavisitar = []
    nosvisitados = []
    closest_direction = "N"
    start = False
    in_left = 0
    in_right = 0
    current_out_left = 0
    current_out_right = 0
    previous_theta = 0
    movement_model_theta = 0
    origin_x = 0
    origin_y = 0
    wall_diameter = 0.1
    robot_radius = 0.5
    myCompass = 0
    numBeacons = 0
    beaconNumberList = []
    beaconCoordinateList = []
    finalpath = []
    end = False
    exploring = True

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

            if self.start == False:
                self.start = True

                self.origin_x = self.measures.x
                self.origin_y = self.measures.y 
                
                # Output file oh the map
                rows, colums = 55, 27
                self.matrix = [[" " for x in range(rows)] for y in range(colums)]
                self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "I"
                
                # Astar Maze
                rows, colums = 55, 100
                self.astar_maze = [[1 for x in range(rows)] for y in range(colums)]

                # Inicial evaluation
                self.evaluateNorth((13,27))
                
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

    # Main function       
    def main(self):

        try:
            self.checkEnd()
            if self.end == False and self.measures.collision == False: 
                self.localization()

                if self.direcao == "North":
                    self.goingNorth()
                elif self.direcao == "West":
                    self.goingWest()
                elif self.direcao == "South":
                    self.goingSouth()
                elif self.direcao == "East":
                    self.goingEast()
                    
            elif self.end == True:
                print("end")
                self.end = True
                self.finish()
                
            elif self.measures.collision == True:
                print("bateu")
                self.end = True
                self.finish()
                self.writePathOutputFile()
                
        except IndexError:
            print("An IndexError exception occurred")
            self.end = True
            self.finish()
            self.writePathOutputFile()
            
        except:
            print("An exception occurred")
            self.end = True
            self.finish()
            self.writePathOutputFile()
        
    # Robot movement when it needs to deviate to its left
    def auxGoLeft(self):

        difBetWeenTarget = 0
        difToCellCenter = 0
        inX = 0.000
        inY = 0.150

        if self.direcao == "North" or self.direcao == "South":
            difBetWeenTarget = abs((self.xpontoatual) - (self.xorigemtransformada))
            difToCellCenter = abs((self.ypontoatual) - (round(self.ypontoatual)))
        elif self.direcao == "West" or self.direcao == "East":
            difBetWeenTarget = abs((self.ypontoatual) - (self.yorigemtransformada))
            difToCellCenter = abs((self.xpontoatual) - (round(self.xpontoatual)))

        if difToCellCenter > 0.30:
            inX = 0.120
        elif difToCellCenter > 0.25:
            inX = 0.136
        elif difToCellCenter > 0.20:
            inX = 0.138
        elif  difToCellCenter > 0.15:
            inX = 0.140
        elif  difToCellCenter > 0.1:
            inX = 0.142
        else:
            inX = 0.144
        
        if (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to deviate to its right
    def auxGoRight(self):

        difBetWeenTarget = 0
        difToCellCenter = 0
        inX = 0.150
        inY = 0.000

        if self.direcao == "North" or self.direcao == "South":
            difBetWeenTarget = abs((self.xpontoatual) - (self.xorigemtransformada))
            difToCellCenter = abs((self.ypontoatual) - (round(self.ypontoatual)))
        elif self.direcao == "West" or self.direcao == "East":
            difBetWeenTarget = abs((self.ypontoatual) - (self.yorigemtransformada))
            difToCellCenter = abs((self.xpontoatual) - (round(self.xpontoatual)))

        if difToCellCenter > 0.30:
            inY = 0.120
        elif difToCellCenter > 0.25:
            inY = 0.136
        elif difToCellCenter > 0.20:
            inY = 0.138
        elif  difToCellCenter > 0.15:
            inY = 0.140
        elif  difToCellCenter > 0.1:
            inY = 0.142
        else:
            inY = 0.144
        
        if (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go forward
    def auxGoFront(self, inX, inY):

        difBetWeenTarget = 0
        if self.direcao == "North" or self.direcao == "South":
            difBetWeenTarget = abs((self.xpontoatual) - (self.xorigemtransformada))
        elif self.direcao == "West" or self.direcao == "East":
            difBetWeenTarget = abs((self.ypontoatual) - (self.yorigemtransformada))

        if (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go backwards
    def auxGoBack(self):
        
        self.driveMotorsUpdate(-0.01,-0.01)

    # Robot movement when it needs to rotate North
    def auxRotateNorth(self):

        if self.myCompass >= -180 and self.myCompass < 0:
            if self.myCompass >= 0-30:
                self.driveMotorsUpdate(-0.009, 0.009)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 0:
            if self.myCompass <= 0+30:
                self.driveMotorsUpdate(0.009, -0.009)
            else:
                self.driveMotorsUpdate(0.150, -0.150)

    # Robot movement when it needs to rotate West
    def auxRotateWest(self):

        if self.myCompass >= -180 and self.myCompass < 90:
            if self.myCompass >= 90-30:
                self.driveMotorsUpdate(-0.009, 0.009)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 90:
            if self.myCompass <= 90+30:
                self.driveMotorsUpdate(0.009, -0.009)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(0.150, -0.150)
    
    # Robot movement when it needs to rotate South
    def auxRotateSouth(self):

        if self.myCompass >= 0 and self.myCompass < 180:
            if self.myCompass >= 180-30:
                self.driveMotorsUpdate(-0.009, 0.009)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 0 and self.myCompass > -180:
            if self.myCompass <= -180+30:
                self.driveMotorsUpdate(0.009, -0.009)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(-0.150, 0.150)

    # Robot movement when it needs to rotate East
    def auxRotateEast(self):

        if self.myCompass >= -180 and self.myCompass < -90:
            if self.myCompass >= -90-30:
                self.driveMotorsUpdate(-0.009, 0.009)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > -90:
            if self.myCompass <= -90+30:
                self.driveMotorsUpdate(0.009, -0.009)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(-0.150, 0.150)

    # Check if all the positions have benn visited or there is no time left
    def checkEnd(self):

        if self.measures.time >= int(self.simTime):
            print("\nfim do tempo")
            self.end = True
            self.finish()
            self.writePathOutputFile()
            self.writeMazesOutputFiles()
            
    # Find next point based on A* algorithm
    def findNextPoint(self):

        if len(self.nosparavisitar) > 0:
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            start = (xround, yround)
            end = ()
            minlen = 27* 55
            end = 0
            for i in self.nosparavisitar:
                path = pathfinder.search(self.astar_maze, 1, start, i)
                if len(path) < minlen:
                    minlen = len(path)
                    end = i
            path = pathfinder.search(self.astar_maze, 1, start, end)
            pontoatual = path[0]
            pontoseguinte = path[1]
            x1 = pontoatual[0]
            x2 = pontoseguinte[0]
            y1 = pontoatual[1]
            y2 = pontoseguinte[1]
            if x1 == x2:
                if y1 > y2:
                    self.rodando = True
                    self.proximadirecao = "East"
                else:
                    self.rodando = True
                    self.proximadirecao = "West"
            if y1 == y2:
                if x1 > x2:
                    self.rodando = True
                    self.proximadirecao = "South"
                else:
                    self.rodando = True
                    self.proximadirecao = "North"

            # In case the robot does not change the previous direction
            if self.proximadirecao == self.direcao:
                self.rodando = False
                if self.proximadirecao == "North":
                    self.xorigemtransformada = self.xorigemtransformada + 2
                elif self.proximadirecao == "West":
                    self.yorigemtransformada = self.yorigemtransformada + 2
                elif self.proximadirecao == "South":
                    self.xorigemtransformada = self.xorigemtransformada - 2
                elif self.proximadirecao == "East":
                    self.yorigemtransformada = self.yorigemtransformada - 2
        
        elif len(self.nosparavisitar) == 0:

            if ((round(self.xpontoatual) == 13) and (round(self.ypontoatual) == 27)):
                print("\nfim")
                self.end = True
                self.finish()
                self.writePathOutputFile()
                print("tempo restante: ", abs(self.measures.time - int(self.simTime)))
                    
            else:
                print("\ntodas as posicoes foram descobertas mas nao estou na posicao inicial")
                self.exploring = False
                self.nosparavisitar.append((13,27))
                self.findNextPoint()

    # Checks if the current position is a beacon and adds its coordinate to a list
    def appendBeacon(self, xround, yround):
        if self.exploring == True and self.end == False:
            if (self.measures.ground != -1) and (self.beaconNumberList.count(self.measures.ground) == 0):
                beacon_id = self.measures.ground
                self.beaconNumberList.append(beacon_id)
                self.beaconCoordinateList.append([beacon_id, xround, yround])
                for triple in self.beaconCoordinateList:
                    if triple[0] == self.measures.ground:
                        print("beacon: ", triple)
            
    # Main function when the compass is 0
    def goingNorth(self):

        if self.rodando == True:
            # print("rodando")

            if self.proximadirecao == "East":
                if self.myCompass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    self.auxRotateEast()

            elif self.proximadirecao == "West":
                if self.myCompass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    self.auxRotateWest()

            elif self.proximadirecao == "South":
                if (self.myCompass == 180) or ((self.myCompass == -180)):
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    self.auxRotateSouth()
    
        elif (abs(self.xpontoatual - self.xorigemtransformada) < 0.06) or (self.measures.irSensor[0] > 2.1):

            if self.measures.irSensor[0] > 2.1:
                self.driveMotorsUpdate(0.0, 0.0)
            
            if self.myCompass < 0+10 and self.myCompass > 0-10:
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.appendBeacon(xround, yround)
                self.drawMapNorth(xround, yround)
                self.evaluateNorth((xround, yround))
                self.drawAstarMazeNorth((xround, yround))
                self.findNextPoint()
                self.yorigemmatriz = self.yorigemmatriz + 2

            else:
                if self.myCompass > 0:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < 0:
                    self.driveMotorsUpdate(-0.01, 0.01)
            
        else:

            if self.xpontoatual <= self.xorigemtransformada:
                if abs(self.ypontoatual - round(self.ypontoatual)) >= 0.1:
                    if self.ypontoatual > round(self.ypontoatual):
                        self.auxGoRight()
                    elif self.ypontoatual < round(self.ypontoatual):
                        self.auxGoLeft()
                elif abs(self.ypontoatual - round(self.ypontoatual)) < 0.1:
                    if self.myCompass > 0:
                        self.auxGoFront(0.150,0.140)
                    elif self.myCompass < 0:
                        self.auxGoFront(0.140,0.150)
                    elif self.myCompass == 0:
                        self.auxGoFront(0.150,0.150)

            elif self.xpontoatual > self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 90
    def goingWest(self):

        if self.rodando == True:
            # print("rodando")

            if self.proximadirecao == "South":
                if (self.myCompass == 180) or (self.myCompass == -180):
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    self.auxRotateSouth()
                        
            elif self.proximadirecao == "North":        
                if self.myCompass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    self.auxRotateNorth()
            
            elif self.proximadirecao == "East":
                if self.myCompass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    self.auxRotateEast()

        elif (abs(self.ypontoatual - self.yorigemtransformada) < 0.06) or (self.measures.irSensor[0] > 2.1):

            if self.measures.irSensor[0] > 2.1:
                self.driveMotorsUpdate(0.0, 0.0)

            if self.myCompass < 90+10 and self.myCompass > 90-10:
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.appendBeacon(xround, yround)
                self.drawMapWest(xround, yround)
                self.evaluateWest((xround, yround))
                self.drawAstarMazeWest((xround, yround))
                self.findNextPoint()
                self.xorigemmatriz = self.xorigemmatriz - 2

            else:
                if self.myCompass > 90:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < 90:
                    self.driveMotorsUpdate(-0.01, 0.01)

        else:

            if self.ypontoatual <= self.yorigemtransformada:
                if abs(self.xpontoatual - round(self.xpontoatual)) >= 0.1:
                    if self.xpontoatual > round(self.xpontoatual):
                        self.auxGoLeft()
                    elif self.xpontoatual < round(self.xpontoatual):
                        self.auxGoRight()
                elif abs(self.xpontoatual - round(self.xpontoatual)) < 0.1:
                    if self.myCompass > 90:
                        self.auxGoFront(0.150,0.140)
                    elif self.myCompass < 90:
                        self.auxGoFront(0.140,0.150)
                    elif self.myCompass == 90:
                        self.auxGoFront(0.150,0.150)
            elif self.ypontoatual > self.yorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 180 or -180
    def goingSouth(self):

        if self.rodando == True:
            # print("rodando")
            
            if self.proximadirecao == "West":
                if self.myCompass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    self.auxRotateWest()

            elif self.proximadirecao == "East":
                if self.myCompass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    self.auxRotateEast()
            
            elif self.proximadirecao == "North":
                if self.myCompass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    self.auxRotateNorth()

        elif (abs(self.xpontoatual - self.xorigemtransformada) < 0.06) or (self.measures.irSensor[0] > 2.1):

            if self.measures.irSensor[0] > 2.1:
                self.driveMotorsUpdate(0.0, 0.0)
            
            if (self.myCompass < 180+10 and self.myCompass > 180-10) or (self.myCompass < -180+10 and self.myCompass > -180-10):
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.appendBeacon(xround, yround)
                self.drawMapSouth(xround, yround)
                self.evaluateSouth((xround, yround))
                self.drawAstarMazeSouth((xround, yround))
                self.findNextPoint()
                self.yorigemmatriz = self.yorigemmatriz - 2

            else:
                if (self.myCompass < 0) and (self.myCompass > -180):
                    self.driveMotorsUpdate(0.01, -0.01)
                elif (self.myCompass > 0) and (self.myCompass < 180):
                    self.driveMotorsUpdate(-0.01, 0.01)

        else:
            
            if self.xpontoatual >= self.xorigemtransformada:
                if abs(self.ypontoatual - round(self.ypontoatual)) >= 0.1:
                    if self.ypontoatual > round(self.ypontoatual):
                        self.auxGoLeft()
                    elif self.ypontoatual < round(self.ypontoatual):
                        self.auxGoRight()
                elif abs(self.ypontoatual - round(self.ypontoatual)) < 0.1:
                    if (self.myCompass > 0 and self.myCompass > 180) or (self.myCompass < 0 and self.myCompass > -180):
                        self.auxGoFront(0.150,0.140)
                    elif (self.myCompass > 0 and self.myCompass < 180) or (self.myCompass < 0 and self.myCompass < -180):
                        self.auxGoFront(0.140,0.150)
                    elif self.myCompass == 180 or self.myCompass == -180:
                        self.auxGoFront(0.150,0.150)

            elif self.xpontoatual < self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is -90
    def goingEast(self):

        if self.rodando == True:
            # print("rodando")
            
            if self.proximadirecao == "North":
                if self.myCompass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    self.auxRotateNorth()

            elif self.proximadirecao == "South":
                if (self.myCompass == -180) or (self.myCompass == 180):
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    self.auxRotateSouth()
                    
            elif self.proximadirecao == "West":
                if self.myCompass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    self.auxRotateWest()

        elif (abs(self.ypontoatual - self.yorigemtransformada) < 0.06) or (self.measures.irSensor[0] > 2.1):

            if self.measures.irSensor[0] > 2.1:
                self.driveMotorsUpdate(0.0, 0.0)
            
            if self.myCompass < -90+10 and self.myCompass > -90-10:
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.appendBeacon(xround, yround)
                self.drawMapEast(xround, yround)
                self.evaluateEast((xround, yround))
                self.drawAstarMazeEast((xround, yround))
                self.findNextPoint()
                self.xorigemmatriz = self.xorigemmatriz + 2

            else:
                if self.myCompass > -90:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < -90:
                    self.driveMotorsUpdate(-0.01, 0.01)

        else:

            if self.ypontoatual >= self.yorigemtransformada:
                if abs(self.xpontoatual - round(self.xpontoatual)) >= 0.1:
                    if self.xpontoatual > round(self.xpontoatual):
                        self.auxGoRight()
                    elif self.xpontoatual < round(self.xpontoatual):
                        self.auxGoLeft()
                elif abs(self.xpontoatual - round(self.xpontoatual)) < 0.1:
                    if self.myCompass > -90:
                        self.auxGoFront(0.150,0.140)
                    elif self.myCompass < -90:
                        self.auxGoFront(0.140,0.150)
                    elif self.myCompass == -90:
                        self.auxGoFront(0.150,0.150)

            elif self.ypontoatual < self.yorigemtransformada:
                self.auxGoBack()

    # Updates list of visited and unvisited points while going North
    def evaluateNorth(self, point):
        
        if self.exploring == True and self.end == False:
            if self.nosvisitados.count(point) == 0:
                self.nosvisitados.append(point)

            if(self.measures.irSensor[self.left_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]+2))

            if(self.measures.irSensor[self.right_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]-2))

            if(self.measures.irSensor[self.center_id] < 1.0):
                if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]+2, point[1]))
            
            if self.measures.irSensor[self.back_id] < 1.0:
                if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]-2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        self.nosvisitados = list(dict.fromkeys(self.nosvisitados))
        self.nosparavisitar = list(dict.fromkeys(self.nosparavisitar))

    # Updates list of visited and unvisited points while going West
    def evaluateWest(self, point):

        if self.exploring == True and self.end == False:
            if self.nosvisitados.count(point) == 0:
                self.nosvisitados.append(point)

            if(self.measures.irSensor[self.left_id] < 1.0):
                if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]-2, point[1]))

            if(self.measures.irSensor[self.right_id] < 1.0):
                if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]+2, point[1]))

            if(self.measures.irSensor[self.center_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]+2))

            if(self.measures.irSensor[self.back_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]-2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        self.nosvisitados = list(dict.fromkeys(self.nosvisitados))
        self.nosparavisitar = list(dict.fromkeys(self.nosparavisitar))

    # Updates list of visited and unvisited points while going South 
    def evaluateSouth(self, point):

        if self.exploring == True and self.end == False:
            if self.nosvisitados.count(point) == 0:
                self.nosvisitados.append(point)

            if(self.measures.irSensor[self.right_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]+2))

            if(self.measures.irSensor[self.left_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]-2))

            if(self.measures.irSensor[self.center_id] < 1.0):
                if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]-2, point[1]))

            if(self.measures.irSensor[self.back_id] < 1.0):
                if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]+2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        self.nosvisitados = list(dict.fromkeys(self.nosvisitados))
        self.nosparavisitar = list(dict.fromkeys(self.nosparavisitar))

    # Updates list of visited and unvisited points while going East 
    def evaluateEast(self, point):

        if self.exploring == True and self.end == False:
            if self.nosvisitados.count(point) == 0:
                self.nosvisitados.append(point)

            if(self.measures.irSensor[self.right_id] < 1.0):
                if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]-2, point[1]))

            if(self.measures.irSensor[self.left_id] < 1.0):
                if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                    self.nosparavisitar.append((point[0]+2, point[1]))
            
            if(self.measures.irSensor[self.center_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]-2))

            if(self.measures.irSensor[self.back_id] < 1.0):
                if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                    self.nosparavisitar.append((point[0], point[1]+2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        self.nosvisitados = list(dict.fromkeys(self.nosvisitados))
        self.nosparavisitar = list(dict.fromkeys(self.nosparavisitar))

    # Draw map outfile while going North 
    def drawMapNorth(self, xround, yround):

        if self.exploring == True and self.end == False:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

            if self.measures.irSensor[self.center_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            if self.measures.irSensor[self.left_id] > 1.1:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.right_id] > 1.1:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.back_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

            for beacon in self.beaconCoordinateList:
                id = beacon[0]
                x = beacon[1]
                y = beacon[2]
                if (xround == x) and (yround == y):
                    self.matrix[self.xorigemmatriz][self.yorigemmatriz] = str(id)

            self.writeMazesOutputFiles()

    # Draw map outfile while going West 
    def drawMapWest(self, xround, yround):

        if self.exploring == True and self.end == False:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

            if self.measures.irSensor[self.center_id] > 1.1:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.left_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            if self.measures.irSensor[self.right_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            if self.measures.irSensor[self.back_id] > 1.1:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

            for beacon in self.beaconCoordinateList:
                id = beacon[0]
                x = beacon[1]
                y = beacon[2]
                if (xround == x) and (yround == y):
                    self.matrix[self.xorigemmatriz][self.yorigemmatriz] = str(id)

            self.writeMazesOutputFiles()

    # Draw map outfile while going South 
    def drawMapSouth(self, xround, yround):

        if self.exploring == True and self.end == False:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

            if self.measures.irSensor[self.center_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            if self.measures.irSensor[self.left_id] > 1.1:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.right_id] > 1.1:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.back_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

            for beacon in self.beaconCoordinateList:
                id = beacon[0]
                x = beacon[1]
                y = beacon[2]
                if (xround == x) and (yround == y):
                    self.matrix[self.xorigemmatriz][self.yorigemmatriz] = str(id)

            self.writeMazesOutputFiles()

    # Draw map outfile while going East 
    def drawMapEast(self, xround, yround):

        if self.exploring == True and self.end == False:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

            if self.measures.irSensor[self.center_id] > 1.1:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            if self.measures.irSensor[self.left_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            if self.measures.irSensor[self.right_id] > 1.1:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
            else:
                self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            if self.measures.irSensor[self.back_id] > 1.1:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
            else:
                self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

            for beacon in self.beaconCoordinateList:
                id = beacon[0]
                x = beacon[1]
                y = beacon[2]
                if (xround == x) and (yround == y):
                    self.matrix[self.xorigemmatriz][self.yorigemmatriz] = str(id)

            self.writeMazesOutputFiles()
    
    # Draw A* Maze while going North 
    def drawAstarMazeNorth(self, point):

        if self.exploring == True and self.end == False:
            self.astar_maze[point[0]][point[1]] = 0
            if(self.measures.irSensor[self.center_id] < 1.0):
                self.astar_maze[point[0]+1][point[1]] = 0
                self.astar_maze[point[0]+2][point[1]] = 0
            if(self.measures.irSensor[self.left_id] < 1.0):
                self.astar_maze[point[0]][point[1]+1] = 0
                self.astar_maze[point[0]][point[1]+2] = 0
            if(self.measures.irSensor[self.right_id] < 1.0):
                self.astar_maze[point[0]][point[1]-1] = 0
                self.astar_maze[point[0]][point[1]-2] = 0
            if(self.measures.irSensor[self.back_id] < 1.0):
                self.astar_maze[point[0]-1][point[1]] = 0
                self.astar_maze[point[0]-2][point[1]] = 0
    
    # Draw A* Maze while going West 
    def drawAstarMazeWest(self, point):

        if self.exploring == True and self.end == False:
            self.astar_maze[point[0]][point[1]] = 0
            if(self.measures.irSensor[self.center_id] < 1.0):
                self.astar_maze[point[0]][point[1]+1] = 0
                self.astar_maze[point[0]][point[1]+2] = 0
            if(self.measures.irSensor[self.left_id] < 1.0):
                self.astar_maze[point[0]-1][point[1]] = 0
                self.astar_maze[point[0]-2][point[1]] = 0
            if(self.measures.irSensor[self.right_id] < 1.0):
                self.astar_maze[point[0]+1][point[1]] = 0
                self.astar_maze[point[0]+2][point[1]] = 0
            if(self.measures.irSensor[self.back_id] < 1.0):
                self.astar_maze[point[0]][point[1]-1] = 0
                self.astar_maze[point[0]][point[1]-2] = 0

    # Draw A* Maze while going South
    def drawAstarMazeSouth(self, point):

        if self.exploring == True and self.end == False:
            self.astar_maze[point[0]][point[1]] = 0
            if(self.measures.irSensor[self.center_id] < 1.0):
                self.astar_maze[point[0]-1][point[1]] = 0
                self.astar_maze[point[0]-2][point[1]] = 0
            if(self.measures.irSensor[self.left_id] < 1.0):
                self.astar_maze[point[0]][point[1]-1] = 0
                self.astar_maze[point[0]][point[1]-2] = 0
            if(self.measures.irSensor[self.right_id] < 1.0):
                self.astar_maze[point[0]][point[1]+1] = 0
                self.astar_maze[point[0]][point[1]+2] = 0
            if(self.measures.irSensor[self.back_id] < 1.0):
                self.astar_maze[point[0]+1][point[1]] = 0
                self.astar_maze[point[0]+2][point[1]] = 0
            
    # Draw A* Maze while going East 
    def drawAstarMazeEast(self, point):

        if self.exploring == True and self.end == False:
            self.astar_maze[point[0]][point[1]] = 0
            if(self.measures.irSensor[self.center_id] < 1.0):
                self.astar_maze[point[0]][point[1]-1] = 0
                self.astar_maze[point[0]][point[1]-2] = 0
            if(self.measures.irSensor[self.left_id] < 1.0):
                self.astar_maze[point[0]+1][point[1]] = 0
                self.astar_maze[point[0]+2][point[1]] = 0
            if(self.measures.irSensor[self.right_id] < 1.0):
                self.astar_maze[point[0]-1][point[1]] = 0
                self.astar_maze[point[0]-2][point[1]] = 0
            if(self.measures.irSensor[self.back_id] < 1.0):
                self.astar_maze[point[0]][point[1]+1] = 0
                self.astar_maze[point[0]][point[1]+2] = 0
            
    # Print Astar Maze
    def printAstarMaze(self):

        rotated = list(zip(*self.astar_maze))[::-1]
        print('\n'.join([''.join(['{:}'.format(item) for item in row]) 
            for row in rotated]))

    # Calculation of final coordinates
    def localization(self):
        
        self.movementModel()
        self.nearestDirectionEstimate()
        self.sensorsCorrection()

        self.movement_model_x = self.sensors_x
        self.movement_model_y = self.sensors_y
        
        self.myCompass = round(math.degrees(self.movement_model_theta))
        self.xpontoatual = round(self.movement_model_x, 2)
        self.ypontoatual = round(self.movement_model_y, 2)

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

        self.sensors_x = self.movement_model_x
        self.sensors_y = self.movement_model_y
        
        # Direction N
        if self.closest_direction == "N":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

        # Direction S
        elif self.closest_direction == "S":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius
        
        # Direction W
        elif self.closest_direction == "W":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

        # Direction E
        elif self.closest_direction == "E":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.0:
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.0:
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

    # Nearest direction estimate
    def nearestDirectionEstimate(self):

        ls = [0,90,180,-180,-90]
        closest_direction_val = min(ls, key=lambda x:abs(x-math.degrees(self.movement_model_theta)))
        if closest_direction_val == 0:
            self.closest_direction = "N"
        elif closest_direction_val == 90:
            self.closest_direction = "W"
        elif closest_direction_val == 180 or closest_direction_val == -180:
            self.closest_direction = "S"
        elif closest_direction_val == -90:
            self.closest_direction = "E"

    # Drive Motors and and updating the power input variables for the movement model
    def driveMotorsUpdate(self, in_left, in_right):

        self.in_left = in_left
        self.in_right = in_right
        self.driveMotors(self.in_left , self.in_right)

    # Writing maze output files
    def writeMazesOutputFiles(self):
        
        if self.exploring == True and self.end == False:
            with open('astar_map.txt', 'w') as out:
                rotated = list(zip(*self.astar_maze))[::-1]
                out.write('\n'.join([''.join(['{:}'.format(item) for item in row]) 
                    for row in rotated]))

            with open(outfilemap, 'w') as out:
                    for i in self.matrix:
                        out.write(''.join(i))
                        out.write('\n')
            
    # Writing maze output files
    def writePathOutputFile(self):
        print("calculando a path...")
        all_beacons_permutations = list(itertools.permutations(self.beaconCoordinateList))

        minpathlen = 27* 55
        for list_combination in all_beacons_permutations:
            beacons_list_coordinates = []

            for i in range(len(list_combination)):
                beacon = list_combination[i]
                point = beacon[1], beacon[2]
                beacons_list_coordinates.append(point)

            beacons_list_coordinates.append(beacons_list_coordinates[0])

            if beacons_list_coordinates[0] == (13,27):
                path = []
                for i in range(len(beacons_list_coordinates)-1):
                    p = pathfinder.search(self.astar_maze, 1, beacons_list_coordinates[i], beacons_list_coordinates[i+1])
                    path = path + p
                path = [v for i, v in enumerate(path) if i == 0 or v != path[i-1]]
                if len(path) < minpathlen:
                    minpathlen = len(path)
                    self.finalpath = path

        print("final path: ", self.finalpath)
        formated_path = []
        for node in self.finalpath:
            beacon_id = -1
            for beacon in self.beaconCoordinateList:
                if (node[0]==beacon[1]) and (node[1]==beacon[2]):
                    beacon_id = beacon[0]
            formated_point = (beacon_id, node[0]-13, node[1]-27)
            formated_path.append(formated_point)
        open(outfilepath, 'w').close()
        with open(outfilepath, 'w') as out:
            for node in formated_path:
                if node[0] != -1:
                    out.write(str(node[1]) + ' ' +  str(node[2]) + ' #' + str(node[0]))
                    out.write('\n')
                else:
                    out.write(str(node[1]) + ' ' +  str(node[2]))
                    out.write('\n')
        print("path atualizada")

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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
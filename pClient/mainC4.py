import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math
import pathfinder

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
    sensors_correction = False

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

        
        self.checkEnd()
        self.localization()

        if self.direcao == "North":
            self.goingNorth()
        elif self.direcao == "West":
            self.goingWest()
        elif self.direcao == "South":
            self.goingSouth()
        elif self.direcao == "East":
            self.goingEast()
    
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

        if  difToCellCenter > 0.20:
            # print("maximo desvio")
            inX = 0.110
        elif  difToCellCenter > 0.15:
            # print("frande desvio")
            inX = 0.135
        elif  difToCellCenter > 0.10:
            # print("medio desvio")
            inX = 0.140
        elif  difToCellCenter > 0.05:
            # print("pequeno desvio")
            inX = 0.145
        
        
        if (difBetWeenTarget) < 0.4:
            self.driveMotorsUpdate(inX/2,inY/2)
        elif (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        elif (difBetWeenTarget) < 0.1:
            self.driveMotorsUpdate(inX/10,inY/10)
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

        if  difToCellCenter > 0.20:
            # print("maximo desvio")
            inY = 0.110
        elif  difToCellCenter > 0.15:
            # print("grande desvio")
            inY = 0.135
        elif  difToCellCenter > 0.10:
            # print("medio desvio")
            inY = 0.140
        elif  difToCellCenter > 0.05:
            # print("pequeno desvio")
            inY = 0.145
        
        if (difBetWeenTarget) < 0.4:
            self.driveMotorsUpdate(inX/2,inY/2)
        elif (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        elif (difBetWeenTarget) < 0.1:
            self.driveMotorsUpdate(inX/10,inY/10)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go forward
    def auxGoFront(self, inX, inY):
        
        difBetWeenTarget = 0
        if self.direcao == "North" or self.direcao == "South":
            difBetWeenTarget = abs((self.xpontoatual) - (self.xorigemtransformada))
        elif self.direcao == "West" or self.direcao == "East":
            difBetWeenTarget = abs((self.ypontoatual) - (self.yorigemtransformada))

        if (difBetWeenTarget) < 0.4:
            self.driveMotorsUpdate(inX/2,inY/2)
        elif (difBetWeenTarget) < 0.3:
            self.driveMotorsUpdate(inX/3,inY/3)
        elif (difBetWeenTarget) < 0.2:
            self.driveMotorsUpdate(inX/5,inY/5)
        elif (difBetWeenTarget) < 0.1:
            self.driveMotorsUpdate(inX/10,inY/10)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go backwards
    def auxGoBack(self):
        # print("go back")
        self.driveMotorsUpdate(-0.01,-0.01)

    # Robot movement when it needs to rotate North
    def auxRotateNorth(self):
        if self.myCompass >= -180 and self.myCompass < 0:
            if self.myCompass >= 0-5:
                self.driveMotorsUpdate(-0.005, 0.005)
            elif self.myCompass >= 0-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 0:
            if self.myCompass <= 0+5:
                self.driveMotorsUpdate(0.005, -0.005)
            elif self.myCompass <= 0+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)

    # Robot movement when it needs to rotate West
    def auxRotateWest(self):
        if self.myCompass >= -180 and self.myCompass < 90:
            if self.myCompass >= 90-5:
                self.driveMotorsUpdate(-0.005, 0.005)
            elif self.myCompass >= 90-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 90:
            if self.myCompass <= 90+5:
                self.driveMotorsUpdate(0.005, -0.005)
            elif self.myCompass <= 90+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(0.150, -0.150)
    
    # Robot movement when it needs to rotate South
    def auxRotateSouth(self):
        if self.myCompass >= 0 and self.myCompass < 180:
            if self.myCompass >= 180-5:
                self.driveMotorsUpdate(-0.005, 0.005)
            elif self.myCompass >= 180-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 0 and self.myCompass > -180:
            if self.myCompass <= -180+5:
                self.driveMotorsUpdate(0.005, -0.005)
            elif self.myCompass <= -180+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(-0.150, 0.150)

    # Robot movement when it needs to rotate East
    def auxRotateEast(self):
        if self.myCompass >= -180 and self.myCompass < -90:
            if self.myCompass >= -90-5:
                self.driveMotorsUpdate(-0.005, 0.005)
            elif self.myCompass >= -90-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > -90:
            if self.myCompass <= -90+5:
                self.driveMotorsUpdate(0.005, -0.005)
            elif self.myCompass <= -90+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(-0.150, 0.150)

    # Check if all the positions have benn visited or there is no time left
    def checkEnd(self):

        if self.measures.time >= int(self.simTime):
            print("\nfim do tempo")

            self.writePathOutputFile()
            self.writeMazesOutputFiles()
            self.finish()
        
        

            

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
                    # print("vou para east")
                    self.rodando = True
                    self.proximadirecao = "East"
                else:
                    # print("vou para west")
                    self.rodando = True
                    self.proximadirecao = "West"
            if y1 == y2:
                if x1 > x2:
                    # print("vou para south")
                    self.rodando = True
                    self.proximadirecao = "South"
                else:
                    # print("vou para north")
                    self.rodando = True
                    self.proximadirecao = "North"

            # In case the robot does not change the previous direction
            if self.proximadirecao == self.direcao:
                # print("In case the robot does not change the previous direction")
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
                print("tempo restante: ", abs(self.measures.time - int(self.simTime)))

                self.writePathOutputFile()
                self.writeMazesOutputFiles()
                self.finish()

            else:
                print("\ntodas as posicoes foram descobertas mas nao estou na posicao inicial")
                self.nosparavisitar.append((13,27))
                self.findNextPoint()


    # Checks if the current position is a beacon and adds its coordinate to a list
    def appendBeacon(self, xround, yround):
        if (self.measures.ground != -1) and (self.beaconNumberList.count(self.measures.ground) == 0):
            beacon_id = self.measures.ground
            self.beaconNumberList.append(beacon_id)
            self.beaconCoordinateList.append([beacon_id, xround, yround])
            for triple in self.beaconCoordinateList:
                if triple[0] == self.measures.ground:
                    print("beacon: ", triple)

            # Create path
            posinicial = [13,27]
            minpathlen = 27* 55
            orderedlist = []
            triple = []
            aux_list = self.beaconCoordinateList.copy()
            for j in range(len(aux_list), 0 ,-1):
                for t in aux_list:
                    coordinates = (t[1], t[2])
                    path = pathfinder.search(self.astar_maze, 1, posinicial, coordinates)
                    if len(path) < minpathlen:
                        minpathlen = len(path)
                        end = coordinates
                        triple = t
                orderedlist.append(end)
                minpathlen = 27* 55
                posinicial = end
                aux_list.remove(triple)
            orderedlist.append([13,27])
            orderedlist.insert(0, [13,27])
            self.finalpath = []
            for i in range(len(orderedlist)-1):
                path = pathfinder.search(self.astar_maze, 1, orderedlist[i], orderedlist[i+1])
                self.finalpath = self.finalpath + path
            self.finalpath = [v for i, v in enumerate(self.finalpath) if i == 0 or v != self.finalpath[i-1]]
            self.writePathOutputFile()
        
    # Main function when the compass is 0
    def goingNorth(self):

        # print("\ngoing N")
        # print("transformda: ", self.xorigemtransformada)

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
    
        elif (abs(self.xpontoatual - self.xorigemtransformada) < 0.06) or self.measures.irSensor[0] > 2.0:
            
            if self.myCompass < 0+5 and self.myCompass > 0-5:
                # print("celula")
                # self.driveMotorsUpdate(0.0, 0.0)
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
                if abs(self.ypontoatual - round(self.ypontoatual)) >= 0.05:
                    # print("nao esta no centro da celula")
                    if self.ypontoatual > round(self.ypontoatual):
                        # print("se esta acima do centro da celula")
                        self.auxGoRight()
                    elif self.ypontoatual < round(self.ypontoatual):
                        # print("se esta abaixo do centro da celula")
                        self.auxGoLeft()
                elif abs(self.ypontoatual - round(self.ypontoatual)) < 0.05:
                    # print("se esta no centro da celula")
                    if self.myCompass > 0:
                        self.auxGoFront(0.150,0.145)
                    elif self.myCompass < 0:
                        self.auxGoFront(0.145,0.150)
                    elif self.myCompass == 0:
                        self.auxGoFront(0.150,0.150)
            elif self.xpontoatual > self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 90
    def goingWest(self):

        # print("\ngoing W")
        # print("transformda: ", self.yorigemtransformada)

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

        elif (abs(self.ypontoatual - self.yorigemtransformada) < 0.06) or self.measures.irSensor[0] > 2.0:

            if self.myCompass < 90+5 and self.myCompass > 90-5:   
                # print("celula")
                # self.driveMotorsUpdate(0.0, 0.0)
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
                if abs(self.xpontoatual - round(self.xpontoatual)) >= 0.05:
                    # print("nao esta no centro da celula")
                    if self.xpontoatual > round(self.xpontoatual):
                        # print("se esta direita do centro da celula")
                        self.auxGoLeft()
                    elif self.xpontoatual < round(self.xpontoatual):
                        # print("se esta esquerda do centro da celula")
                        self.auxGoRight()
                elif abs(self.xpontoatual - round(self.xpontoatual)) < 0.05:
                    # print("se esta no centro da celula")
                    if self.myCompass > 90:
                        self.auxGoFront(0.150,0.145)
                    elif self.myCompass < 90:
                        self.auxGoFront(0.145,0.150)
                    elif self.myCompass == 90:
                        self.auxGoFront(0.150,0.150)
            elif self.ypontoatual > self.yorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 180 or -180
    def goingSouth(self):

        # print("\ngoing S")
        # print("transformda: ", self.xorigemtransformada)

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

        elif (abs(self.xpontoatual - self.xorigemtransformada) < 0.06) or self.measures.irSensor[0] > 2.0:
            
            if (self.myCompass < 180+5 and self.myCompass > 180-5) or (self.myCompass < -180+5 and self.myCompass > -180-5):
                # print("celula")
                # self.driveMotorsUpdate(0.0, 0.0)
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
                if abs(self.ypontoatual - round(self.ypontoatual)) >= 0.05:
                    # print("nao esta no centro da celula")
                    if self.ypontoatual > round(self.ypontoatual):
                        # print("se esta acima do centro da celula")
                        self.auxGoLeft()
                    elif self.ypontoatual < round(self.ypontoatual):
                        # print("se esta abaixo do centro da celula")
                        self.auxGoRight()
                elif abs(self.ypontoatual - round(self.ypontoatual)) < 0.05:
                    # print("se esta no centro da celula")
                    if (self.myCompass > 0 and self.myCompass > 180) or (self.myCompass < 0 and self.myCompass > -180):
                        self.auxGoFront(0.150,0.145)
                    elif (self.myCompass > 0 and self.myCompass < 180) or (self.myCompass < 0 and self.myCompass < -180):
                        self.auxGoFront(0.145,0.150)
                    elif self.myCompass == 180 or self.myCompass == -180:
                        self.auxGoFront(0.150,0.150)

            elif self.xpontoatual < self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is -90
    def goingEast(self):

        # print("\ngoing E")
        # print("transformda: ", self.yorigemtransformada)

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

        elif (abs(self.ypontoatual - self.yorigemtransformada) < 0.06) or self.measures.irSensor[0] > 2.0:
            
            if self.myCompass < -90+5 and self.myCompass > -90-5:
                # print("celula")
                # self.driveMotorsUpdate(0.0, 0.0)
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
                if abs(self.xpontoatual - round(self.xpontoatual)) >= 0.05:
                    # print("nao esta no centro da celula")
                    if self.xpontoatual > round(self.xpontoatual):
                        # print("se esta esqeurda do centro da celula")
                        self.auxGoRight()
                    elif self.xpontoatual < round(self.xpontoatual):
                        # print("se esta diretia do centro da celula")
                        self.auxGoLeft()
                elif abs(self.xpontoatual - round(self.xpontoatual)) < 0.05:
                    # print("se esta no centro da celula")
                    if self.myCompass > -90:
                        self.auxGoFront(0.150,0.145)
                    elif self.myCompass < -90:
                        self.auxGoFront(0.145,0.150)
                    elif self.myCompass == -90:
                        self.auxGoFront(0.150,0.150)

            elif self.ypontoatual < self.yorigemtransformada:
                self.auxGoBack()

    # Updates list of visited and unvisited points while going North
    def evaluateNorth(self, point):
        # print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        

        if(self.measures.irSensor[self.left_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.right_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.center_id] < 1.1):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))
        
        if self.measures.irSensor[self.back_id] < 1.1:
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

    # Updates list of visited and unvisited points while going West
    def evaluateWest(self, point):
        # print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        

        if(self.measures.irSensor[self.left_id] < 1.1):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.right_id] < 1.1):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.center_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.back_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

    # Updates list of visited and unvisited points while going South 
    def evaluateSouth(self, point):
        # print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        

        if(self.measures.irSensor[self.right_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.left_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.center_id] < 1.1):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.back_id] < 1.1):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

    # Updates list of visited and unvisited points while going East 
    def evaluateEast(self, point):
        # print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        
        if(self.measures.irSensor[self.right_id] < 1.1):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.left_id] < 1.1):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))
        
        if(self.measures.irSensor[self.center_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))


        if(self.measures.irSensor[self.back_id] < 1.1):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

    # Draw map outfile while going North 
    def drawMapNorth(self, xround, yround):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if self.measures.irSensor[self.center_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
        if self.measures.irSensor[self.left_id] > 1.2:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.right_id] > 1.2:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.back_id] > 1.2:
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

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if self.measures.irSensor[self.center_id] > 1.2:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.left_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
        if self.measures.irSensor[self.right_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
        if self.measures.irSensor[self.back_id] > 1.2:
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

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if self.measures.irSensor[self.center_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
        if self.measures.irSensor[self.left_id] > 1.2:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.right_id] > 1.2:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.back_id] > 1.2:
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

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if self.measures.irSensor[self.center_id] > 1.2:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
        if self.measures.irSensor[self.left_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
        if self.measures.irSensor[self.right_id] > 1.2:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
        if self.measures.irSensor[self.back_id] > 1.2:
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
        self.astar_maze[point[0]][point[1]] = 0
        if(self.measures.irSensor[self.center_id] < 1.1):
            self.astar_maze[point[0]+1][point[1]] = 0
            self.astar_maze[point[0]+2][point[1]] = 0
        if(self.measures.irSensor[self.left_id] < 1.1):
            self.astar_maze[point[0]][point[1]+1] = 0
            self.astar_maze[point[0]][point[1]+2] = 0
        if(self.measures.irSensor[self.right_id] < 1.1):
            self.astar_maze[point[0]][point[1]-1] = 0
            self.astar_maze[point[0]][point[1]-2] = 0
        if(self.measures.irSensor[self.back_id] < 1.1):
            self.astar_maze[point[0]-1][point[1]] = 0
            self.astar_maze[point[0]-2][point[1]] = 0
    
    # Draw A* Maze while going West 
    def drawAstarMazeWest(self, point):
        self.astar_maze[point[0]][point[1]] = 0
        if(self.measures.irSensor[self.center_id] < 1.1):
            self.astar_maze[point[0]][point[1]+1] = 0
            self.astar_maze[point[0]][point[1]+2] = 0
        if(self.measures.irSensor[self.left_id] < 1.1):
            self.astar_maze[point[0]-1][point[1]] = 0
            self.astar_maze[point[0]-2][point[1]] = 0
        if(self.measures.irSensor[self.right_id] < 1.1):
            self.astar_maze[point[0]+1][point[1]] = 0
            self.astar_maze[point[0]+2][point[1]] = 0
        if(self.measures.irSensor[self.back_id] < 1.1):
            self.astar_maze[point[0]][point[1]-1] = 0
            self.astar_maze[point[0]][point[1]-2] = 0

    # Draw A* Maze while going South
    def drawAstarMazeSouth(self, point):
        self.astar_maze[point[0]][point[1]] = 0
        if(self.measures.irSensor[self.center_id] < 1.1):
            self.astar_maze[point[0]-1][point[1]] = 0
            self.astar_maze[point[0]-2][point[1]] = 0
        if(self.measures.irSensor[self.left_id] < 1.1):
            self.astar_maze[point[0]][point[1]-1] = 0
            self.astar_maze[point[0]][point[1]-2] = 0
        if(self.measures.irSensor[self.right_id] < 1.1):
            self.astar_maze[point[0]][point[1]+1] = 0
            self.astar_maze[point[0]][point[1]+2] = 0
        if(self.measures.irSensor[self.back_id] < 1.1):
            self.astar_maze[point[0]+1][point[1]] = 0
            self.astar_maze[point[0]+2][point[1]] = 0
            
    # Draw A* Maze while going East 
    def drawAstarMazeEast(self, point):
        self.astar_maze[point[0]][point[1]] = 0
        if(self.measures.irSensor[self.center_id] < 1.1):
            self.astar_maze[point[0]][point[1]-1] = 0
            self.astar_maze[point[0]][point[1]-2] = 0
        if(self.measures.irSensor[self.left_id] < 1.1):
            self.astar_maze[point[0]+1][point[1]] = 0
            self.astar_maze[point[0]+2][point[1]] = 0
        if(self.measures.irSensor[self.right_id] < 1.1):
            self.astar_maze[point[0]-1][point[1]] = 0
            self.astar_maze[point[0]-2][point[1]] = 0
        if(self.measures.irSensor[self.back_id] < 1.1):
            self.astar_maze[point[0]][point[1]+1] = 0
            self.astar_maze[point[0]][point[1]+2] = 0
            
    # Print Astar Maze
    def printAstarMaze(self):
        # print('\n'.join([''.join(['{:}'.format(item) for item in row]) 
        #     for row in self.astar_maze]))

        rotated = list(zip(*self.astar_maze))[::-1]

        print('\n'.join([''.join(['{:}'.format(item) for item in row]) 
            for row in rotated]))

    # Calculation of final coordinates
    def localization(self):
        
        self.movementModel()
        self.nearestDirectionEstimate()
        self.sensorsCorrection()

        # testx = self.movement_model_x
        # testy = self.movement_model_y
            
        # self.movement_model_x = (self.movement_model_x*(3/4)) + (self.sensors_x*(1/4))
        # self.movement_model_y = (self.movement_model_y*(3/4)) + (self.sensors_y*(1/4))

        self.movement_model_x = self.sensors_x
        self.movement_model_y = self.sensors_y

        self.myCompass = round(math.degrees(self.movement_model_theta))

        self.xpontoatual = round(self.movement_model_x, 2)
        self.ypontoatual = round(self.movement_model_y, 2)

        if self.sensors_correction == True:
            self.sensors_correction = False

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
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.4:
                # print("\nFront correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.4:
               # print("\nBack correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.4:
               # print("\nLeft correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.4:
                # print("\nRight correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

        # Direction S
        elif self.closest_direction == "S":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.4:
                # print("\nFront correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.4:
               # print("\nBack correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.4:
               # print("\nLeft correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.4:
                # print("\nRight correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius
        
        # Direction W
        elif self.closest_direction == "W":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.4:
                # print("\nFront correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.4:
               # print("\nBack correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.4:
               # print("\nLeft correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.4:
                # print("\nRight correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

        # Direction E
        elif self.closest_direction == "E":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3.4:
                # print("\nFront correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3.4:
               # print("\nBack correction")
                self.sensors_correction = True
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3.4:
               # print("\nLeft correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3.4:
                # print("\nRight correction")
                self.sensors_correction = True
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

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

    # Drive Motors and and updating the power input variables for the movement model
    def driveMotorsUpdate(self, in_left, in_right):
        self.in_left = in_left
        self.in_right = in_right
        self.driveMotors(self.in_left , self.in_right)

    # Writing maze output files
    def writeMazesOutputFiles(self):

        # with open('astar_map.txt', 'w') as out:
        #     rotated = list(zip(*self.astar_maze))[::-1]
        #     out.write('\n'.join([''.join(['{:}'.format(item) for item in row]) 
        #         for row in rotated]))

        with open(outfilemap, 'w') as out:
                for i in self.matrix:
                    out.write(''.join(i))
                    out.write('\n')
        
    # Writing maze output files
    def writePathOutputFile(self):
        
        # print("finalpath: ", self.finalpath)

        formated_path = []
        for node in self.finalpath:
            beacon_id = -1
            for beacon in self.beaconCoordinateList:
                if (node[0]==beacon[1]) and (node[1]==beacon[2]):
                    beacon_id = beacon[0]
            formated_point = (beacon_id, node[1]-27, -(node[0]-13))
            formated_path.append(formated_point)

        # print("formated_path: ", formated_path)
        open(outfilepath, 'w').close()
        with open(outfilepath, 'w') as out:
            for node in formated_path:
                if node[0] != -1:
                    out.write(str(node[1]) + ' ' +  str(node[2]) + ' #' + str(node[0]))
                    out.write('\n')
                else:
                    out.write(str(node[1]) + ' ' +  str(node[2]))
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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
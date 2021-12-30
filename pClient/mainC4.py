import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math

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
    matrix = []
    astar_maze = []
    nosparavisitar = []
    nosvisitados = []
    xorigemtransformada = 13
    yorigemtransformada = 27

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
    myCompass = 0

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
                rows, colums = 55, 27
                self.matrix = [[" " for x in range(rows)] for y in range(colums)]
                self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "I"
                self.evaluateNorth((self.media_x,self.media_y))
                rows, colums = 55, 27
                self.astar_maze = [[1 for x in range(rows)] for y in range(colums)]
                
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

        print("\n########################################################################")
        self.writeOutputFiles()
        self.localization()

        print("\nfront: ", self.measures.irSensor[0])
        print("left: ", self.measures.irSensor[1])
        print("right: ", self.measures.irSensor[2])
        print("back: ", self.measures.irSensor[3])
        print("\nx   : ", self.xpontoatual)
        print("y   : ", self.ypontoatual)
        print("compass : ", self.myCompass)
        print("\n")

        if self.direcao == "North":
            self.goingNorth()
        elif self.direcao == "West":
            self.goingWest()
        elif self.direcao == "South":
            self.goingSouth()
        elif self.direcao == "East":
            self.goingEast()
            
        self.checkEnd()

        self.printAstarMaze()

        print("########################################################################")
    
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
        
        print("go left")
        print("difBetWeenTarget: ", difBetWeenTarget)
        print("difToCellCenter: ", difToCellCenter)

        if difToCellCenter >= 0.2:
            inX = 0.133
        elif difToCellCenter >= 0.1:
            inX = 0.140
        elif difToCellCenter >= 0.05:
            inX = 0.145
        else:
            inX = 0.149
        
        if difBetWeenTarget <= 0.3:
            self.driveMotorsUpdate(inX/15,inY/15)
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
        
        print("go right")
        print("difBetWeenTarget: ", difBetWeenTarget)
        print("difToCellCenter: ", difToCellCenter)

        if difToCellCenter >= 0.2:
            inY = 0.133
        elif difToCellCenter >= 0.1:
            inY = 0.140
        elif difToCellCenter >= 0.05:
            inY = 0.145
        else:
            inY = 0.149
        
        if difBetWeenTarget <= 0.3:
            self.driveMotorsUpdate(inX/15,inY/15)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go forward
    def auxGoFront(self):

        inX = 0.150
        inY = 0.150
        
        dif = 0
        if self.direcao == "North" or self.direcao == "South":
            dif = abs((self.xpontoatual) - (self.xorigemtransformada))
        elif self.direcao == "West" or self.direcao == "East":
            dif = abs((self.ypontoatual) - (self.yorigemtransformada))
        print("go front")
        print("la difrenca: ", dif)

        if (dif) <= 0.3:
            self.driveMotorsUpdate(inX/15,inY/15)
        else:
            self.driveMotorsUpdate(inX,inY)

    # Robot movement when it needs to go backwards
    def auxGoBack(self):

        print("ja passei")
        print("go back")
        self.driveMotorsUpdate(-0.01,-0.01)

    # Robot movement when it needs to rotate North
    def auxRotateNorth(self):
        if self.myCompass >= -180 and self.myCompass < 0:
            if self.myCompass >= 0-5:
                self.driveMotorsUpdate(-0.001, 0.001)
            elif self.myCompass >= 0-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 0:
            if self.myCompass <= 0+5:
                self.driveMotorsUpdate(0.001, -0.001)
            elif self.myCompass <= 0+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)

    # Robot movement when it needs to rotate West
    def auxRotateWest(self):
        if self.myCompass >= -180 and self.myCompass < 90:
            if self.myCompass >= 90-5:
                self.driveMotorsUpdate(-0.001, 0.001)
            elif self.myCompass >= 90-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > 90:
            if self.myCompass <= 90+5:
                self.driveMotorsUpdate(0.001, -0.001)
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
                self.driveMotorsUpdate(-0.001, 0.001)
            elif self.myCompass >= 180-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 0 and self.myCompass > -180:
            if self.myCompass <= -180+5:
                self.driveMotorsUpdate(0.001, -0.001)
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
                self.driveMotorsUpdate(-0.001, 0.001)
            elif self.myCompass >= -90-25:
                self.driveMotorsUpdate(-0.010, 0.010)
            else:
                self.driveMotorsUpdate(-0.150, 0.150)
        elif self.myCompass <= 180 and self.myCompass > -90:
            if self.myCompass <= -90+5:
                self.driveMotorsUpdate(0.001, -0.001)
            elif self.myCompass <= -90+25:
                self.driveMotorsUpdate(0.010, -0.010)
            else:
                self.driveMotorsUpdate(0.150, -0.150)
        else:
            self.driveMotorsUpdate(-0.150, 0.150)

    # Main function when the compass is 0
    def goingNorth(self):

        print("going N")
        print("transformda: ", self.xorigemtransformada)

        if self.rodando == True:
            print("rodando")

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
    
        elif round(self.xpontoatual, 1) == (self.xorigemtransformada):
            
            if self.myCompass == 0:
                print("celula")
                
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.evaluateNorth((xround, yround))
                self.drawMapNorth()

                # Left
                if (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.left_id] < 1.2):
                    print("WEST por visitar")
                    self.rodando = True
                    self.proximadirecao = "West"  
                
                # Center 
                elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.center_id] < 1.2):
                    print("NORTH por visitar")
                    self.proximadirecao = "North"
                    self.xorigemtransformada = self.xorigemtransformada + 2

                # Right
                elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.right_id] < 1.2):
                    self.rodando = True
                    print("EAST por visitar")
                    self.proximadirecao = "East"

                # Back
                elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.2):
                    self.rodando = True
                    print("SOUTH por visitar")
                    self.proximadirecao = "South"
                
                # In case all surrounding positions have already been visited
                else:
                    print("NORTH POR EXCLUSAO")

                    # Center
                    if self.measures.irSensor[self.center_id] < 1.2:
                        print("vou em frente - North")
                        self.proximadirecao = "North"
                        self.xorigemtransformada = self.xorigemtransformada + 2

                    # Left
                    elif self.measures.irSensor[self.left_id] < 1.2:
                        print("vou esquerda - West")
                        self.rodando = True
                        self.proximadirecao = "West"
                    
                    # Right
                    elif self.measures.irSensor[self.right_id] < 1.2:
                        print("vou direita - East")
                        self.rodando = True
                        self.proximadirecao = "East"
                    
                    # Back
                    elif self.measures.irSensor[self.back_id] < 1.2:
                        print("vou tras - South")
                        self.rodando = True
                        self.proximadirecao = "South"

            else:
                if self.myCompass > 0:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < 0:
                    self.driveMotorsUpdate(-0.01, 0.01)

        elif round(self.xpontoatual,1) != (self.xorigemtransformada):

            print("andar")
            
            if self.xpontoatual <= self.xorigemtransformada:
                print("estou atras")

                # se esta acima do centro da celula
                if self.ypontoatual > round(self.ypontoatual):
                    print("se esta acima do centro da celula")
                    self.auxGoRight()
                    
                # se esta abaixo do centro da celula
                elif self.ypontoatual < round(self.ypontoatual):
                    print("se esta abaixo do centro da celula")
                    self.auxGoLeft()

                # se esta no centro da celula
                elif self.ypontoatual == round(self.ypontoatual):
                    print("se esta no centro da celula")
                    self.auxGoFront()
                        
            elif self.xpontoatual > self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 90
    def goingWest(self):

        print("going W")
        print("transformda: ", self.yorigemtransformada)

        if self.rodando == True:
            print("rodando")

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

        elif round(self.ypontoatual,1) == (self.yorigemtransformada):

            if self.myCompass == 90:     
                print("celula")
                
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.evaluateWest((xround, yround))
                self.drawMapWest()

                # Left
                if (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.left_id]   < 1.2):
                    print("SOUTH por visitar")
                    self.rodando = True
                    self.proximadirecao = "South"

                # Center
                elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.center_id] < 1.2):
                    print("WEST por visitar")
                    self.proximadirecao = "West"
                    self.yorigemtransformada = self.yorigemtransformada + 2

                # Right
                elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.right_id]  < 1.2):
                    print("NORTH por visitar")
                    self.rodando = True
                    self.proximadirecao = "North"

                # Back
                elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.back_id]   < 1.2):
                    print("EAST por visitar")
                    self.rodando = True
                    self.proximadirecao = "East"
                
                # In case all surrounding positions have already been visited
                else:
                    print("WEST POR EXCLUSAO")
        
                    # Center
                    if self.measures.irSensor[self.center_id] < 1.2:
                        print("vou em frente - West")
                        self.proximadirecao = "West"
                        self.yorigemtransformada = self.yorigemtransformada + 2

                    # Left
                    elif self.measures.irSensor[self.left_id] < 1.2:
                        print("vou esquerda - South")
                        self.rodando = True
                        self.proximadirecao = "South"

                    # Right
                    elif self.measures.irSensor[self.right_id] < 1.2:
                        print("vou direita - North")
                        self.rodando = True
                        self.proximadirecao = "North"

                    # Back
                    elif self.measures.irSensor[self.back_id] < 1.2:
                        print("vou tras - East")
                        self.rodando = True
                        self.proximadirecao = "East"
            
            else:
                if self.myCompass > 90:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < 90:
                    self.driveMotorsUpdate(-0.01, 0.01)

        elif round(self.ypontoatual,1) != (self.yorigemtransformada):
            
            print("andar")

            if self.ypontoatual <= self.yorigemtransformada:
                print("estou atras")

                # se esta na celula a direita
                if self.xpontoatual > round(self.xpontoatual):
                    print("se esta na celula a direita")
                    self.auxGoLeft()

                # se esta na celula a esquerda   
                elif self.xpontoatual < round(self.xpontoatual):
                    print("se esta na celula a esquerda")
                    self.auxGoRight()

                # se esta no centro da celula
                elif self.xpontoatual == round(self.xpontoatual):
                    print("se esta no centro da celula")
                    self.auxGoFront()

            elif self.ypontoatual > self.yorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is 180 or -180
    def goingSouth(self):

        print("going S")
        print("transformda: ", self.xorigemtransformada)

        if self.rodando == True:
            print("rodando")
            
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

        elif round(self.xpontoatual, 1) == (self.xorigemtransformada):
            
            if (self.myCompass == 180) or (self.myCompass == -180):
                print("celula")
                
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.evaluateSouth((xround, yround))
                self.drawMapSouth()

                # Left
                if (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.left_id] < 1.2):
                    print("EAST por visitar")
                    self.rodando = True
                    self.proximadirecao = "East"

                # Center
                elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.center_id] < 1.2): 
                    print("SOUTH por visitar")
                    self.proximadirecao = "South"
                    self.xorigemtransformada = self.xorigemtransformada - 2

                # Right
                elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.right_id] < 1.2):
                    print("WEST por visitar")
                    self.rodando = True
                    self.proximadirecao = "West"

                # Back
                elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.2):
                    print("NORTH por visitar")
                    self.rodando = True
                    self.proximadirecao = "North"
                
                # In case all surrounding positions have already been visited
                else:
                    print("SOUTH POR EXCLUSAO")

                    # Left
                    if self.measures.irSensor[self.left_id] < 1.2:
                        print("vou esquerda - East")
                        self.rodando = True
                        self.proximadirecao = "East"

                    # Center
                    elif self.measures.irSensor[self.center_id] < 1.2:
                        print("vou em frente - South")
                        self.proximadirecao = "South"
                        self.xorigemtransformada = self.xorigemtransformada - 2

                    # Right
                    elif self.measures.irSensor[self.right_id] < 1.2:
                        print("vou direita - West")
                        self.rodando = True
                        self.proximadirecao = "West"

                    # Back
                    elif self.measures.irSensor[self.back_id] < 1.2:
                        print("vou tras - North")
                        self.rodando = True
                        self.proximadirecao = "North"

            else:
                if (self.myCompass < 0) and (self.myCompass > -180):
                    self.driveMotorsUpdate(0.01, -0.01)
                elif (self.myCompass > 0) and (self.myCompass < 180):
                    self.driveMotorsUpdate(-0.01, 0.01)

        elif round(self.xpontoatual,1) != (self.xorigemtransformada):
            
            print("andar")

            if self.xpontoatual >= self.xorigemtransformada:
                print("estou atras")

                # se esta acima do centro da celula
                if self.ypontoatual > round(self.ypontoatual):
                    print("se esta acima do centro da celula")
                    self.auxGoLeft()
        
                # se esta abaixo do centro da celula
                elif self.ypontoatual < round(self.ypontoatual):
                    print("se esta abaixo do centro da celula")
                    self.auxGoRight()

                # se esta no centro da celula
                elif self.ypontoatual == round(self.ypontoatual):
                    print("se esta no centro da celula")
                    self.auxGoFront()

            elif self.xpontoatual < self.xorigemtransformada:
                self.auxGoBack()

    # Main function when the compass is -90
    def goingEast(self):

        print("going E")
        print("transformda: ", self.yorigemtransformada)

        if self.rodando == True:
            print("rodando")
            
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

        elif round(self.ypontoatual,1) == (self.yorigemtransformada):
            
            if self.myCompass == -90:
                print("celula")
                
                xround = round(self.xpontoatual)
                yround = round(self.ypontoatual)
                self.evaluateEast((xround, yround))
                self.drawMapEast()
                
                # Left
                if (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.left_id]  < 1.2):
                    print("NORTH por visitar")
                    self.rodando = True
                    self.proximadirecao = "North"
                
                # Center
                elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.center_id] < 1.2):
                    print("EAST por visitar")
                    self.proximadirecao = "East"
                    self.yorigemtransformada = self.yorigemtransformada - 2

                # Right
                elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.right_id] < 1.2):
                    print("South por visitar")
                    self.rodando = True
                    self.proximadirecao = "South" 
                
                # Back
                elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.back_id] < 1.2):
                    print("WEST esta por visitar")
                    self.rodando = True
                    self.proximadirecao = "West"
                
                # In case all surrounding positions have already been visited
                else:
                    print("EAST POR EXCLUSAO")

                    # Center
                    if self.measures.irSensor[self.center_id] < 1.2:
                        print("vou em frente - East")
                        self.proximadirecao = "East"
                        self.yorigemtransformada = self.yorigemtransformada - 2

                    # Left
                    elif self.measures.irSensor[self.left_id] < 1.2:
                        self.rodando = True
                        print("vou esquerda - North")
                        self.proximadirecao = "North"

                    # Right
                    elif self.measures.irSensor[self.right_id] < 1.2:
                        print("vou direita - South")
                        self.rodando = True
                        self.proximadirecao = "South"
                    
                    # Back
                    elif self.measures.irSensor[self.right_id] < 1.2:
                        print("vou tras - West")
                        self.rodando = True
                        self.proximadirecao = "West"
            
            else:
                if self.myCompass > -90:
                    self.driveMotorsUpdate(0.01, -0.01)
                elif self.myCompass < -90:
                    self.driveMotorsUpdate(-0.01, 0.01)

        elif round(self.ypontoatual,1) != (self.yorigemtransformada):
            
            print("andar")

            if self.ypontoatual >= self.yorigemtransformada:
                print("estou atras")

                # se esta na celula a direita
                if self.xpontoatual > round(self.xpontoatual):
                    print("se esta na celula a direita")
                    self.auxGoRight()
                
                # se esta na celula a esquerda   
                elif self.xpontoatual < round(self.xpontoatual):
                    print("se esta na celula a esquerda")
                    self.auxGoLeft()

                # se esta no centro da celula
                elif self.xpontoatual == round(self.xpontoatual):
                    self.auxGoFront()

            elif self.ypontoatual < self.yorigemtransformada:
                self.auxGoBack()

    # Updates list of visited and unvisited points while going North
    def evaluateNorth(self, point):
    
        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if self.measures.irSensor[self.back_id] < 1.2:
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.center_id] < 1.2):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.right_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.left_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    # Updates list of visited and unvisited points while going West
    def evaluateWest(self, point):

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.center_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.right_id] < 1.2):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.left_id] < 1.2):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    # Updates list of visited and unvisited points while going South 
    def evaluateSouth(self, point):

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.2):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.center_id] < 1.2):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.right_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.left_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    # Updates list of visited and unvisited points while going East 
    def evaluateEast(self, point):

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.center_id] < 1.2):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.right_id] < 1.2):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.left_id] < 1.2):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    # Draw map outfile and astar maze while going North 
    def drawMapNorth(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        self.astar_maze[self.xorigemmatriz][self.yorigemmatriz] = 0
        
        if self.measures.irSensor[self.center_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz+1] = 0

        if self.measures.irSensor[self.left_id] > 1.1:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz-1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.right_id] > 1.1:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz+1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.back_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz-1] = 0

        if self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

        self.yorigemmatriz = self.yorigemmatriz + 2

    # Draw map outfile and astar maze while going West 
    def drawMapWest(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        self.astar_maze[self.xorigemmatriz][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.center_id] > 1.1:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz-1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.left_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz-1] = 0

        if self.measures.irSensor[self.right_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz+1] = 0

        if self.measures.irSensor[self.back_id] > 1.1:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz+1][self.yorigemmatriz] = 0

        if self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

        self.xorigemmatriz = self.xorigemmatriz - 2

    # Draw map outfile and astar maze while going South 
    def drawMapSouth(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        self.astar_maze[self.xorigemmatriz][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.center_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz-1] = 0

        if self.measures.irSensor[self.left_id] > 1.1:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz+1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.right_id] > 1.1:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz-1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.back_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz+1] = 0

        if self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

        self.yorigemmatriz = self.yorigemmatriz - 2

    # Draw map outfile and astar maze while going East 
    def drawMapEast(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        self.astar_maze[self.xorigemmatriz][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.center_id] > 1.1:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz+1][self.yorigemmatriz] = 0

        if self.measures.irSensor[self.left_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz+1] = 0

        if self.measures.irSensor[self.right_id] > 1.1:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"
            self.astar_maze[self.xorigemmatriz][self.yorigemmatriz-1] = 0

        if self.measures.irSensor[self.back_id] > 1.1:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"
            self.astar_maze[self.xorigemmatriz-1][self.yorigemmatriz] = 0

        self.xorigemmatriz = self.xorigemmatriz + 2

        if self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

    # Call of the finish function when all positions have been visited
    def checkEnd(self):
        if len(self.nosparavisitar) == 0:
            print("TODAS AS POSICOES FORAM ENCONTRADAS")
            print("TEMPO RESTANTE: ", 5000-self.measures.time)
            self.finish()
        if self.measures.time >= 5000:
            print("FIM DO TEMPO")
            self.finish()

    # Print Astar Maze
    def printAstarMaze(self):
        print('\n'.join([''.join(['{:}'.format(item) for item in row]) 
            for row in self.astar_maze]))

    # Calculation of final coordinates
    def localization(self):
        self.movementModel()
        self.nearestDirectionEstimate()
        self.sensorsCorrection()

        # self.media_x = self.movement_model_x 
        # self.media_y = self.movement_model_y 

        self.media_x = self.movement_model_x
        self.media_y = self.movement_model_y

        self.myCompass = round(math.degrees(self.movement_model_theta))
        self.xpontoatual = round(self.media_x,2)
        self.ypontoatual = round(self.media_y,2)

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
        
        # Direction N
        if self.closest_direction == "N":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2.7:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.movement_model_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2.7:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.movement_model_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2.7:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.movement_model_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2.7:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.movement_model_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

        # Direction S
        elif self.closest_direction == "S":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2.7:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.movement_model_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2.7:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.movement_model_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2.7:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.movement_model_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2.7:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.movement_model_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius
        
        # Direction W
        elif self.closest_direction == "W":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2.7:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.movement_model_y = closest_y_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2.7:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.movement_model_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2.7:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.movement_model_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2.7:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.movement_model_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

        # Direction E
        elif self.closest_direction == "E":
            
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 2.7:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.movement_model_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 2.7:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.movement_model_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 2.7:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.movement_model_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 2.7:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.movement_model_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius
        
        print("closest_x_wall_coordinate: ", closest_x_wall_coordinate)
        print("closest_y_wall_coordinate: ", closest_y_wall_coordinate)

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

    # Drive Motors and and updating the power input variables for the movement model
    def driveMotorsUpdate(self, in_left, in_right):
        self.in_left = in_left
        self.in_right = in_right
        self.driveMotors(self.in_left , self.in_right)

    # Writing output files
    def writeOutputFiles(self):
        with open(outfilemap, 'w') as out:
                for i in self.matrix:
                    out.write(''.join(i))
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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
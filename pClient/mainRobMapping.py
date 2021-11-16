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
    center_id = 0
    left_id = 1
    right_id = 2
    back_id = 3
    proximadirecao = ""
    xorigemmatriz = 13
    yorigemmatriz = 27
    matrix = 0
    nosparavistitar = []
    nosvisitados = []
    xorigemtransformada = 13
    yorigemtransformada = 27
    xorigemdesvio = 0
    yorigemdesvio = 0
    xdesvio = 0
    ydesvio = 0
    xorigemdesvioatribuido = False
    yorigemdesvioatribuido = False

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
                rows, colums = 55, 27
                self.matrix = [[" " for x in range(rows)] for y in range(colums)] 
                self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "I"

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

        self.xpontoatual = self.measures.x - (self.xorigem - 13)
        self.ypontoatual = self.measures.y - (self.yorigem - 27)

        if self.direcao == "North":
            self.goingNorth()
        elif self.direcao == "West":
            self.goingWest()
        elif self.direcao == "South":
            self.goingSouth()
        elif self.direcao == "East":
            self.goingEast()

    def goingNorth(self):

        if self.yorigemdesvioatribuido == False:
            self.yorigemdesvio = self.ypontoatual
            print("self.yorigemdesvio", self.yorigemdesvio)
            self.yorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "East":
                if self.measures.compass == -90 or self.measures.compass == -91 or self.measures.compass == -89:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                    print("desvio y: ", self.ydesvio)
                else:
                    self.driveMotors(0.01,-0.01)
            elif self.proximadirecao == "West":
                if self.measures.compass == 90 or self.measures.compass == 91 or self.measures.compass == 89:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                    print("desvio y: ", self.ydesvio)
                else:
                    self.driveMotors(-0.01,0.01)

        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.6:
            self.xdesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateNorth(point)
            # self.drawMapNorth()
            self.xorigemtransformada = self.xorigemtransformada + 2
            self.stop = False

        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] > 1.6:
            self.xdesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateNorth(point)
            # self.drawMapNorth()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            if self.measures.irSensor[self.left_id] > self.measures.irSensor[self.right_id]:
                self.proximadirecao = "East"
            elif self.measures.irSensor[self.left_id] < self.measures.irSensor[self.right_id]:
                self.proximadirecao = "West"
            else:
                self.proximadirecao = "East"

        elif ((self.xpontoatual - self.xdesvio) != (self.xorigemtransformada)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.6 :
            self.xdesvio = 0
            if self.measures.compass > 0:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < 0:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == 0:
                self.driveMotors(0.07,0.07)

        else:
            self.driveMotors(0.01,0.01)

    def goingWest(self):

        if self.xorigemdesvioatribuido == False:
            self.xorigemdesvio = self.xpontoatual
            print("self.xorigemdesvio", self.xorigemdesvio)
            self.xorigemdesvioatribuido = True
        
        if self.rodando == True:
            if self.proximadirecao == "South":
                if self.measures.compass == 180 or self.measures.compass == -179 or self.measures.compass == 179:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                    print("desvio x: ", self.xdesvio)
                else:
                    self.driveMotors(-0.01,+0.01)
            elif self.proximadirecao == "North":
                if self.measures.compass == 0 or self.measures.compass == -1 or self.measures.compass == 1:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                    print("desvio x: ", self.xdesvio)
                else:
                    self.driveMotors(0.01,-0.01)

        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.6:
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateWest(point)
            # self.drawMapWest()
            self.yorigemtransformada = self.yorigemtransformada + 2
            self.stop = False

        elif (self.ypontoatual- self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] > 1.6:
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateWest(point)
            # self.drawMapWest()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            if self.measures.irSensor[self.left_id] > self.measures.irSensor[self.right_id]:
                self.proximadirecao = "North"
            elif self.measures.irSensor[self.left_id] < self.measures.irSensor[self.right_id]:
                self.proximadirecao = "South"
            else:
                self.proximadirecao = "South"

        elif ((self.ypontoatual - self.ydesvio) != (self.yorigemtransformada)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.6:
            if self.measures.compass > 90:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < 90:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == 90:
                self.driveMotors(0.07,0.07)

        else:
            self.driveMotors(0.01,0.01)

    def goingSouth(self):

        if self.yorigemdesvioatribuido == False:
            self.yorigemdesvio = self.ypontoatual
            print("self.yorigemdesvio", self.yorigemdesvio)
            self.yorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "West":
                if self.measures.compass == 90 or self.measures.compass == 91 or self.measures.compass == 89:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                    print("desvio y: ", self.ydesvio)
                else:
                    self.driveMotors(0.01,-0.01)
            elif self.proximadirecao == "East":
                if self.measures.compass == -90 or self.measures.compass == -91 or self.measures.compass == -89:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                    print("desvio y: ", self.ydesvio)
                else:
                    self.driveMotors(-0.01,0.01)

        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.6:
            self.xdesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateSouth(point)
            # self.drawMapSouth()
            self.xorigemtransformada = self.xorigemtransformada - 2
            self.stop = False

        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] > 1.6:
            self.xdesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateSouth(point)
            # self.drawMapSouth()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            if self.measures.irSensor[self.left_id] > self.measures.irSensor[self.right_id]:
                self.proximadirecao = "West"
            elif self.measures.irSensor[self.left_id] < self.measures.irSensor[self.right_id]:
                self.proximadirecao = "East"
            else:
                self.proximadirecao = "West"
        elif ( (self.xpontoatual - self.xdesvio) != (self.xorigem)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.6:
            if self.measures.compass > -180 and self.measures.compass < -1 :
                self.driveMotors(0.04,0.03)
            elif self.measures.compass > 1 and self.measures.compass < 180:
                self.driveMotors(0.03,0.04)
            elif (self.measures.compass == 180) or (self.measures.compass == -180):
                self.driveMotors(0.07,0.07)
        else:
            self.driveMotors(0.01,0.01)

    def goingEast(self):

        if self.xorigemdesvioatribuido == False:
            self.xorigemdesvio = self.xpontoatual
            print("self.xorigemdesvio", self.xorigemdesvio)
            self.xorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "North":
                if self.measures.compass == 0 or self.measures.compass == -1 or self.measures.compass == 1:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                    print("desvio x: ", self.xdesvio)
                else:
                    self.driveMotors(-0.01,0.01)
            elif self.proximadirecao == "South":
                if self.measures.compass == 180 or self.measures.compass == -179 or self.measures.compass == 179:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                    print("desvio x: ", self.xdesvio)
                else:
                    self.driveMotors(0.01,-0.01)

        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.6:
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateEast(point)
            # self.drawMapEast()
            self.yorigemtransformada = self.yorigemtransformada - 2
            self.stop = False

        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] > 1.6:
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual)
            self.evaluateEast(point)
            # self.drawMapEast()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            if self.measures.irSensor[self.left_id] > self.measures.irSensor[self.right_id]:
                self.proximadirecao = "South"
            elif self.measures.irSensor[self.left_id] < self.measures.irSensor[self.right_id]:
                self.proximadirecao = "North"
            else:
                self.proximadirecao = "South"

        elif ((self.ypontoatual - self.ydesvio) != (self.yorigem)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.6:
            if self.measures.compass > -90:
                self.driveMotors(0.04,0.03)
            elif self.measures.compass < -90:
                self.driveMotors(0.03,0.04)
            elif self.measures.compass == -90:
                self.driveMotors(0.07,0.07)
        else:
            self.driveMotors(0.01,0.01)

    def evaluateNorth(self, point):
        x = point[0]
        y = point[1]

        print("x:", x)
        print("y:", y)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)
            if self.nosparavistitar.count(point) > 0:
                self.nosparavistitar.remove(point)

        if(self.measures.irSensor[self.back_id] < 1.6):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavistitar.append((x-2, y))

        if(self.measures.irSensor[self.center_id] < 1.6):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavistitar.append((x+2, y))

        if(self.measures.irSensor[self.right_id] < 1.6):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavistitar.append((x, y-2))

        if(self.measures.irSensor[self.left_id] < 1.6):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavistitar.append((x, y+2))

        # print("nos para visitar: ",  list(dict.fromkeys(self.nosparavistitar)))
        # print("nos visitados: ", list(dict.fromkeys(self.nosvisitados)))

    def evaluateWest(self, point):
        x = point[0]
        y = point[1]

        print("x:", x)
        print("y:", y)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)
            if self.nosparavistitar.count(point) > 0:
                self.nosparavistitar.remove(point)

        if(self.measures.irSensor[self.back_id] < 1.6):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavistitar.append((x, y-2))

        if(self.measures.irSensor[self.center_id] < 1.6):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavistitar.append((x, y+2))

        if(self.measures.irSensor[self.right_id] < 1.6):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavistitar.append((x+2, y))

        if(self.measures.irSensor[self.left_id] < 1.6):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavistitar.append((x-2, y))

        # print("nos para visitar: ",  list(dict.fromkeys(self.nosparavistitar)))
        # print("nos visitados: ", list(dict.fromkeys(self.nosvisitados)))

    def evaluateSouth(self, point):
        x = point[0]
        y = point[1]

        print("x:", x)
        print("y:", y)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)
            if self.nosparavistitar.count(point) > 0:
                self.nosparavistitar.remove(point)

        if(self.measures.irSensor[self.back_id] < 1.6):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavistitar.append((x+2, y))

        if(self.measures.irSensor[self.center_id] < 1.6):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavistitar.append((x-2, y))

        if(self.measures.irSensor[self.right_id] < 1.6):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavistitar.append((x, y+2))

        if(self.measures.irSensor[self.left_id] < 1.6):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavistitar.append((x, y-2))

        # print("nos para visitar: ",  list(dict.fromkeys(self.nosparavistitar)))
        # print("nos visitados: ", list(dict.fromkeys(self.nosvisitados)))


    def evaluateEast(self, point):
        x = point[0]
        y = point[1]

        print("x:", x)
        print("y:", y)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)
            if self.nosparavistitar.count(point) > 0:
                self.nosparavistitar.remove(point)

        if(self.measures.irSensor[self.back_id] < 1.6):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavistitar.append((x, y+2))

        if(self.measures.irSensor[self.center_id] < 1.6):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavistitar.append((x, y-2))

        if(self.measures.irSensor[self.right_id] < 1.6):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavistitar.append((x-2, y))

        if(self.measures.irSensor[self.left_id] < 1.6):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavistitar.append((x+2, y))

        # print("nos para visitar: ",  list(dict.fromkeys(self.nosparavistitar)))
        # print("nos visitados: ", list(dict.fromkeys(self.nosvisitados)))

    def drawMapNorth(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        
        if(self.measures.irSensor[self.center_id] > 1.6):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.left_id] > 1.6):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.right_id] > 1.6):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.back_id] > 1.6):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        # self.printMatrix()

        self.yorigemmatriz = self.yorigemmatriz + 2

    def drawMapWest(self):

        # print("posicao mapa west:", (self.xorigemmatriz, self.yorigemmatriz))

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.6):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.left_id] > 1.6):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.right_id] > 1.6):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.back_id] > 1.6):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        self.xorigemmatriz = self.xorigemmatriz - 2

        # self.printMatrix()

    def drawMapSouth(self):

        # print("posicao mapa south:", (self.xorigemmatriz, self.yorigemmatriz))
        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.6):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.left_id] > 1.6):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.right_id] > 1.6):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.back_id] > 1.6):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        # self.printMatrix()

        self.yorigemmatriz = self.yorigemmatriz - 2

    def drawMapEast(self):

        # print("posicao mapa east:", (self.xorigemmatriz, self.yorigemmatriz))

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.6):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.left_id] > 1.6):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.right_id] > 1.6):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.back_id] > 1.6):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        self.xorigemmatriz = self.xorigemmatriz + 2

        # self.printMatrix()


    def printMatrix(self):
        print('\n'.join([''.join(['{:}'.format(item) for item in row]) 
            for row in self.matrix]))


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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

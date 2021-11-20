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
    nosparavisitar = []
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


        print("self.measures.ground: ", self.measures.ground)

        with open('mapping.out', 'w') as out:
                for i in self.matrix:
                    out.write(''.join(i))
                    out.write('\n')
        
        self.xpontoatual = self.measures.x - (self.xorigem - 13)
        self.ypontoatual = self.measures.y - (self.yorigem - 27)

        if self.direcao == "North":
            self.goingNorth()
            self.checkEnd()
        elif self.direcao == "West":
            self.goingWest()
            self.checkEnd()
        elif self.direcao == "South":
            self.goingSouth()
            self.checkEnd()
        elif self.direcao == "East":
            self.goingEast()
            self.checkEnd()
        
    def goingNorth(self):

        if self.yorigemdesvioatribuido == False:
            self.yorigemdesvio = self.ypontoatual
            self.yorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "East":
                if self.measures.compass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass < 0) and (self.measures.compass >= -40):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass < -40) and (self.measures.compass >= -70):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass < -70) and (self.measures.compass >= -80):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass < -80) and (self.measures.compass > -90):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)

            elif self.proximadirecao == "West":
                if self.measures.compass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass > 0) and (self.measures.compass <= 40):
                        self.driveMotors(-0.15, 0.15)
                    elif (self.measures.compass > 40) and (self.measures.compass <= 70):
                        self.driveMotors(-0.04, 0.04)
                    elif (self.measures.compass > 70) and (self.measures.compass <= 80):
                        self.driveMotors(-0.02, 0.02)
                    elif (self.measures.compass > 80) and (self.measures.compass < 90):
                        self.driveMotors(-0.005, 0.005)
                    else:
                        self.driveMotors(-0.03, 0.03)

            elif self.proximadirecao == "South":
                if self.measures.compass == 180 or self.measures.compass == -180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass > 0) and (self.measures.compass <= 130):
                        self.driveMotors(-0.15, 0.15)
                    elif (self.measures.compass > 130) and (self.measures.compass <= 160):
                        self.driveMotors(-0.04, 0.04)
                    elif (self.measures.compass > 160) and (self.measures.compass <= 170):
                        self.driveMotors(-0.02, 0.02)
                    elif (self.measures.compass > 170) and (self.measures.compass < 180):
                        self.driveMotors(-0.005, 0.005)
                    else:
                        self.driveMotors(-0.03, 0.03)
            
        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.5 :
            self.driveMotors(0.00,0.00)
            self.xdesvio = 0
            point = (self.xpontoatual - self.xdesvio, self.ypontoatual)
            self.evaluateNorth(point)
            self.drawMapNorth()
            self.stop = False
            
            if (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("WEST por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.center_id] < 1.5):
                print("NORTH por visitar")
                self.proximadirecao = "North"
                self.xorigemtransformada = self.xorigemtransformada + 2
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                self.rodando = True
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                self.rodando = True
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            else:
                print("NORTH POR EXCLUSAO")

                if self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - North")
                    self.proximadirecao = "North"
                    self.xorigemtransformada = self.xorigemtransformada + 2

                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - East")
                    self.rodando = True
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.left_id] < 1.5:
                    self.rodando = True
                    print("vou esquerda - West")
                    self.proximadirecao = "West"
                
        elif ((self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.5):
            self.xdesvio = 0
            point = (self.xpontoatual - self.xdesvio, self.ypontoatual)
            self.evaluateNorth(point)
            self.drawMapNorth()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True

            if (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("WEST por visitar")
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.left_id] < 1.5:
                    print("vou esquerda - West")
                    self.proximadirecao = "West"

                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - North")
                    self.proximadirecao = "North"
                    self.xorigemtransformada = self.xorigemtransformada + 2
                else:
                    print("vou tras - South")
                    self.proximadirecao = "East"
     
        elif ((self.xpontoatual - self.xdesvio) != (self.xorigemtransformada)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.5 :
            if self.measures.compass > 0:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < 0:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == 0:
                if (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

        else:
            if self.measures.compass > 0:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < 0:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == 0:
                if (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

    def goingWest(self):

        if self.xorigemdesvioatribuido == False:
            self.xorigemdesvio = self.xpontoatual
            self.xorigemdesvioatribuido = True
        
        if self.rodando == True:
            if self.proximadirecao == "South":
                if self.measures.compass == 180 or self.measures.compass == -180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if (self.measures.compass > 90) and (self.measures.compass <= 130):
                        self.driveMotors(-0.15, 0.15)
                    elif (self.measures.compass > 130) and (self.measures.compass <= 160):
                        self.driveMotors(-0.04, 0.04)
                    elif (self.measures.compass > 160) and (self.measures.compass <= 170):
                        self.driveMotors(-0.02, 0.02)
                    elif (self.measures.compass > 170) and (self.measures.compass < 180):
                        self.driveMotors(-0.005, 0.005)
                    else:
                        self.driveMotors(-0.03, 0.03)
                        
            elif self.proximadirecao == "North":
                if self.measures.compass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if (self.measures.compass > 50) and (self.measures.compass <= 90):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass > 20) and (self.measures.compass <= 50):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass > 10) and (self.measures.compass <= 20):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass > 0) and (self.measures.compass <= 10):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)
            
            elif self.proximadirecao == "East":
                if self.measures.compass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if(self.measures.compass >= 0):
                        self.driveMotors(0.03, -0.03)
                    elif (self.measures.compass < 0) and (self.measures.compass >= -40):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass < -40) and (self.measures.compass >= -70):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass < -70) and (self.measures.compass >= -80):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass < -80) and (self.measures.compass > -90):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)

        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.5:
            self.driveMotors(0.00,0.00)
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual - self.ydesvio) 
            self.evaluateWest(point)
            self.drawMapWest()
            self.stop = False

            if (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("SOUTH por visitar")
                self.rodando = True
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.center_id] < 1.5):
                print("WEST por visitar")
                self.proximadirecao = "West"
                self.yorigemtransformada = self.yorigemtransformada + 2
            elif (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("EAST por visitar")
                self.rodando = True
                self.proximadirecao = "East"
            else:
                print("WEST POR EXCLUSAO")

                if self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - West")
                    self.proximadirecao = "West"
                    self.yorigemtransformada = self.yorigemtransformada + 2

                elif self.measures.irSensor[self.left_id] < 1.5:
                    self.rodando = True
                    print("vou esquerda - South")
                    self.proximadirecao = "South"
                
                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - North")
                    self.rodando = True
                    self.proximadirecao = "North"

        elif (self.ypontoatual- self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.5:
            self.ydesvio = 0
            point = (self.xpontoatual, self.ypontoatual - self.ydesvio)
            self.evaluateWest(point)
            self.drawMapWest()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("EAST por visitar")
                self.proximadirecao = "East"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - West")
                    self.proximadirecao = "West"
                    self.yorigemtransformada = self.yorigemtransformada + 2

                elif self.measures.irSensor[self.left_id] < 1.5:
                    print("vou esquerda - South")
                    self.proximadirecao = "South"
                
                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - North")
                    self.proximadirecao = "North"

                elif self.measures.irSensor[self.back_id] < 1.5:
                    print("vou pa tras - East")
                    self.proximadirecao = "East"
                
        elif ((self.ypontoatual - self.ydesvio) != (self.yorigemtransformada)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.5:
            if self.measures.compass > 90:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < 90:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == 90:

                if (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

        else:
            if self.measures.compass > 90:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < 90:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == 90:

                if (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

    def goingSouth(self):

        if self.yorigemdesvioatribuido == False:
            self.yorigemdesvio = self.ypontoatual
            self.yorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "West":
                if self.measures.compass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass < 180) and (self.measures.compass >= 140):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass < 140) and (self.measures.compass >= 110):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass < 110) and (self.measures.compass >= 100):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass < 100) and (self.measures.compass > 90):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)

            elif self.proximadirecao == "East":
                if self.measures.compass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass < -140) and (self.measures.compass >= -180):
                        self.driveMotors(-0.15, 0.15)
                    elif (self.measures.compass < -110) and (self.measures.compass >= -140):
                        self.driveMotors(-0.04, 0.04)
                    elif (self.measures.compass < -100) and (self.measures.compass >= -110):
                        self.driveMotors(-0.02, 0.02)
                    elif (self.measures.compass < -90) and (self.measures.compass > -100):
                        self.driveMotors(-0.005, 0.005)
                    else:
                        self.driveMotors(-0.03, 0.03)
            
            elif self.proximadirecao == "North":
                if self.measures.compass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                    self.yorigemdesvioatribuido = False
                    self.ydesvio = self.ypontoatual - self.yorigemdesvio
                else:
                    if (self.measures.compass > 50) and (self.measures.compass <= 180):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass > 20) and (self.measures.compass <= 50):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass > 10) and (self.measures.compass <= 20):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass > 0) and (self.measures.compass <= 10):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)

        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.5:
            self.driveMotors(0.00,0.00)
            self.xdesvio = 0
            point = (self.xpontoatual - self.xdesvio, self.ypontoatual)
            self.evaluateSouth(point)
            self.drawMapSouth()
            self.stop = False

            if (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.center_id] < 1.5): 
                print("SOUTH por visitar")
                self.proximadirecao = "South"
                self.xorigemtransformada = self.xorigemtransformada - 2
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("EAST por visitar")
                self.rodando = True
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("WEST por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            else:
                print("SOUTH POR EXCLUSAO")

                

                if self.measures.irSensor[self.left_id] < 1.5:
                    self.rodando = True
                    print("vou esquerda - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.left_id] < 1.5:
                    self.rodando = True
                    print("vou esquerda - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - South")
                    self.proximadirecao = "South"
                    self.xorigemtransformada = self.xorigemtransformada - 2
  
        elif (self.xpontoatual - self.xdesvio) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.5:
            self.xdesvio = 0
            point = (self.xpontoatual - self.xdesvio, self.ypontoatual)
            self.evaluateSouth(point)
            self.drawMapSouth()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("WEST por visitar")
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - West")
                    self.proximadirecao = "West"

                elif self.measures.irSensor[self.left_id] < 1.5:
                    print("vou esquerda - East")
                    self.proximadirecao = "East"
                
                elif self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - South")
                    self.proximadirecao = "South"
                    self.yorigemtransformada = self.yorigemtransformada - 2

                elif self.measures.irSensor[self.back_id] < 1.5:
                    print("vou pa tras - North")
                    self.proximadirecao = "North"

        elif ((self.xpontoatual - self.xdesvio) != (self.xorigem)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.5:
            if self.measures.compass > -180 and self.measures.compass < -1 :
                self.driveMotors(0.07,0.05)
            elif self.measures.compass > 1 and self.measures.compass < 180:
                self.driveMotors(0.05,0.07)
            elif (self.measures.compass == 180) or (self.measures.compass == -180):
                if (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)
        
        else:
            if self.measures.compass > -180 and self.measures.compass < -1 :
                self.driveMotors(0.07,0.05)
            elif self.measures.compass > 1 and self.measures.compass < 180:
                self.driveMotors(0.05,0.07)
            elif (self.measures.compass == 180) or (self.measures.compass == -180):
                if (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.xpontoatual - self.xdesvio) - (self.xorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

    def goingEast(self):

        if self.xorigemdesvioatribuido == False:
            self.xorigemdesvio = self.xpontoatual
            self.xorigemdesvioatribuido = True

        if self.rodando == True:
            if self.proximadirecao == "North":
                if self.measures.compass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if (self.measures.compass > -90) and (self.measures.compass <= -50):
                        self.driveMotors(-0.15, 0.15)
                    elif (self.measures.compass > -50) and (self.measures.compass <= -20):
                        self.driveMotors(-0.04, 0.04)
                    elif (self.measures.compass > -20) and (self.measures.compass <= -10):
                        self.driveMotors(-0.02, 0.02)
                    elif (self.measures.compass > -10) and (self.measures.compass < 0):
                        self.driveMotors(-0.005, 0.005)
                    else:
                        self.driveMotors(-0.03, 0.03)

            elif self.proximadirecao == "South":
                if self.measures.compass == -180 or self.measures.compass == 180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if (self.measures.compass < -90) and (self.measures.compass >= -130):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass < -130) and (self.measures.compass >= -160):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass < -160) and (self.measures.compass >= -170):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass < -170) and (self.measures.compass >= -180):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)

            elif self.proximadirecao == "West":
                if self.measures.compass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                    self.xorigemdesvioatribuido = False
                    self.xdesvio = self.xpontoatual - self.xorigemdesvio
                else:
                    if(self.measures.compass <= 0):
                        self.driveMotors(0.03, -0.03)
                    elif (self.measures.compass > 0) and (self.measures.compass <= 40):
                        self.driveMotors(0.15, -0.15)
                    elif (self.measures.compass > 40) and (self.measures.compass <= 70):
                        self.driveMotors(0.04, -0.04)
                    elif (self.measures.compass > 70) and (self.measures.compass <= 80):
                        self.driveMotors(0.02, -0.02)
                    elif (self.measures.compass > 80) and (self.measures.compass < 90):
                        self.driveMotors(0.005, -0.005)
                    else:
                        self.driveMotors(0.03, -0.03)
                        
        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.5:
           
            self.driveMotors(0.0,0.0)
            self.driveMotors(0.00,0.00)
            self.ydesvio = 0
            point = (self.xpontoatual - self.ydesvio, self.ypontoatual)
            self.evaluateEast(point)
            self.drawMapEast()
            self.stop = False

            # self.yorigemtransformada = self.yorigemtransformada - 2

            
            if (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual-2)) > 0) and (self.measures.irSensor[self.center_id] < 1.5):
                print("EAST por visitar")
                self.proximadirecao = "East"
                self.yorigemtransformada = self.yorigemtransformada - 2
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("WEST esta por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            else:
                print("EAST POR EXCLUSAO")
                
                if self.measures.irSensor[self.center_id] < 1.5:
                    self.proximadirecao = "East"
                    self.yorigemtransformada = self.yorigemtransformada - 2
                elif self.measures.irSensor[self.left_id] < 1.5:
                    self.rodando = True
                    print("vou esquerda - North")
                    self.proximadirecao = "North"
                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - South")
                    self.rodando = True
                    self.proximadirecao = "South"

        elif (self.ypontoatual - self.ydesvio) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.5:
            self.ydesvio = 0
            point = (self.xpontoatual - self.ydesvio, self.ypontoatual)
            self.evaluateEast(point)
            self.drawMapEast()
            self.driveMotors(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((self.xpontoatual+2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.left_id] < 1.5):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((self.xpontoatual-2,self.ypontoatual)) > 0) and (self.measures.irSensor[self.right_id] < 1.5):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((self.xpontoatual,self.ypontoatual+2)) > 0) and (self.measures.irSensor[self.back_id] < 1.5):
                print("WEST esta por visitar")
                self.proximadirecao = "West"
            else:
                print("todas as posicoes a volta descobertas")
                if self.measures.irSensor[self.left_id] < 1.5:
                    print("vou esquerda - North")
                    self.proximadirecao = "North"
                
                elif self.measures.irSensor[self.right_id] < 1.5:
                    print("vou direita - South")
                    self.proximadirecao = "South"

                elif self.measures.irSensor[self.center_id] < 1.5:
                    print("vou em frente - East")
                    self.proximadirecao = "East"
                    self.yorigemtransformada = self.yorigemtransformada - 2   

                elif self.measures.irSensor[self.back_id] < 1.5:
                    print("vou pa tras - West")
                    self.proximadirecao = "West"         

        elif ((self.ypontoatual - self.ydesvio) != (self.yorigem)) and self.stop == False and self.measures.irSensor[self.center_id] < 1.5:
            if self.measures.compass > -90:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < -90:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == -90:
                # print("la diferenca: ", abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada)))
                if (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)
        
        else:
            if self.measures.compass > -90:
                self.driveMotors(0.07,0.05)
            elif self.measures.compass < -90:
                self.driveMotors(0.05,0.07)
            elif self.measures.compass == -90:
                if (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.45:
                    self.driveMotors(0.15,0.15)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.15:
                    self.driveMotors(0.04,0.04)
                elif (abs((self.ypontoatual - self.ydesvio) - (self.yorigemtransformada))) > 0.02:
                    self.driveMotors(0.01,0.01)
                else:
                    self.driveMotors(0.005,0.005)

    def evaluateNorth(self, point):
        x = round(point[0])
        y = round(point[1])
    
        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if self.measures.irSensor[self.back_id] < 1.5:
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavisitar.append((x-2, y))

        if(self.measures.irSensor[self.center_id] < 1.5):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavisitar.append((x+2, y))

        if(self.measures.irSensor[self.right_id] < 1.5):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavisitar.append((x, y-2))

        if(self.measures.irSensor[self.left_id] < 1.5):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavisitar.append((x, y+2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove((x, y))

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateWest(self, point):
        x = round(point[0])
        y = round(point[1])

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.5):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavisitar.append((x, y-2))

        if(self.measures.irSensor[self.center_id] < 1.5):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavisitar.append((x, y+2))

        if(self.measures.irSensor[self.right_id] < 1.5):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavisitar.append((x+2, y))

        if(self.measures.irSensor[self.left_id] < 1.5):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavisitar.append((x-2, y))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateSouth(self, point):
        x = round(point[0])
        y = round(point[1])

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.5):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavisitar.append((x+2, y))

        if(self.measures.irSensor[self.center_id] < 1.5):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavisitar.append((x-2, y))

        if(self.measures.irSensor[self.right_id] < 1.5):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavisitar.append((x, y+2))

        if(self.measures.irSensor[self.left_id] < 1.5):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavisitar.append((x, y-2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateEast(self, point):
        x = round(point[0])
        y = round(point[1])

        print("\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.5):
            if self.nosvisitados.count((x, y+2)) == 0:
                self.nosparavisitar.append((x, y+2))

        if(self.measures.irSensor[self.center_id] < 1.5):
            if self.nosvisitados.count((x, y-2)) == 0:
                self.nosparavisitar.append((x, y-2))

        if(self.measures.irSensor[self.right_id] < 1.5):
            if self.nosvisitados.count((x-2, y)) == 0:
                self.nosparavisitar.append((x-2, y))

        if(self.measures.irSensor[self.left_id] < 1.5):
            if self.nosvisitados.count((x+2, y)) == 0:
                self.nosparavisitar.append((x+2, y))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def drawMapNorth(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"
        
        if(self.measures.irSensor[self.center_id] > 1.4):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.left_id] > 1.4):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.right_id] > 1.4):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.back_id] > 1.4):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if  self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

        self.yorigemmatriz = self.yorigemmatriz + 2

    def drawMapWest(self):

        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.4):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.left_id] > 1.4):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.right_id] > 1.4):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.back_id] > 1.4):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        self.xorigemmatriz = self.xorigemmatriz - 2

        if  self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

    def drawMapSouth(self):
        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.4):
            # print("parede em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre em frente ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.left_id] > 1.4):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.right_id] > 1.4):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.back_id] > 1.4):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if  self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

        self.yorigemmatriz = self.yorigemmatriz - 2

    def drawMapEast(self):
        self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.center_id] > 1.4):
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "-"
        else:
            self.matrix[self.xorigemmatriz+1][self.yorigemmatriz] = "X"

        if(self.measures.irSensor[self.left_id] > 1.4):
            # print("parede a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "|"
        else:
            # print("livre a esquerda ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz+1] = "X"

        if(self.measures.irSensor[self.right_id] > 1.4):
            # print("parede a direira ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "|"
        else:
            # print("livre a direita ")
            self.matrix[self.xorigemmatriz][self.yorigemmatriz-1] = "X"

        if(self.measures.irSensor[self.back_id] > 1.4):
            # print("parede a tras ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "-"
        else:
            # print("livre a tras ")
            self.matrix[self.xorigemmatriz-1][self.yorigemmatriz] = "X"

        self.xorigemmatriz = self.xorigemmatriz + 2

        if  self.matrix[13][27] != "I":
            self.matrix[13][27] = "I"

    def checkEnd(self):
        if len(self.nosparavisitar) == 0:
            print("TODAS AS POSICOES FORAM ENCONTRDAS")
            sys.exit()
    
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

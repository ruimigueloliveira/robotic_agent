import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math

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

            # To test GPS
            if self.start == False:
                self.start = True
                self.origin_x = self.measures.x
                self.origin_y = self.measures.y 

            if self.flag == 0:
                self.xorigem = self.media_x
                self.yorigem = self.media_y
                self.flag = 1
                rows, colums = 55, 27
                self.matrix = [[" " for x in range(rows)] for y in range(colums)]
                self.matrix[self.xorigemmatriz][self.yorigemmatriz] = "I"

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
            
    def main(self):

        self.localization()
        self.myCompass = round(math.degrees(self.movement_model_theta))
        # print(self.myCompass)

        # with open(outfile, 'w') as out:
        #         for i in self.matrix:
        #             out.write(''.join(i))
        #             out.write('\n')

        gps_x = self.measures.x - (self.origin_x - 13)
        gps_y = self.measures.y - (self.origin_y - 27)

        self.xpontoatual = round(gps_x,1)
        self.ypontoatual = round(gps_y,1)

        # dif_x = abs(gps_x - self.xpontoatual)
        # dif_y = abs(gps_y - self.ypontoatual)

        print("\n###############################################")
        print("front: ", self.measures.irSensor[0])
        print("left: ", self.measures.irSensor[1])
        print("right: ", self.measures.irSensor[2])
        print("back: ", self.measures.irSensor[3])
        # print("gps x                : ", gps_x)
        # print("movement model x     : ", round(self.movement_model_x,1))
        # print("sensors x            : ", self.sensors_x)
        print("media x              : ", self.xpontoatual)
        # print("dif x                : ", dif_x)
        print("---------------------------------------------")
        # print("gps y                : ", gps_y)
        # print("movement model y     : ", round(self.movement_model_y,1))
        # print("sensors y            : ", self.sensors_y)
        print("media y              : ", self.ypontoatual)
        # print("dif y                : ", dif_y)
        print("---------------------------------------------")
        print("compass              : ", self.measures.compass)
        print("movement model theta : ", self.myCompass)
        print("###############################################")

        if self.direcao == "North":
            self.goingNorth()
        elif self.direcao == "West":
            self.goingWest()
        elif self.direcao == "South":
            self.goingSouth()
        elif self.direcao == "East":
            self.goingEast()
            
        self.checkEnd()
    
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
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    if (self.myCompass < 0) and (self.myCompass >= -40):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass < -40) and (self.myCompass >= -70):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass < -70) and (self.myCompass >= -80):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass < -80) and (self.myCompass > -90):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            elif self.proximadirecao == "West":
                if self.myCompass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    if (self.myCompass > 0) and (self.myCompass <= 40):
                        self.driveMotorsUpdate(-0.15, 0.15)
                    elif (self.myCompass > 40) and (self.myCompass <= 70):
                        self.driveMotorsUpdate(-0.04, 0.04)
                    elif (self.myCompass > 70) and (self.myCompass <= 80):
                        self.driveMotorsUpdate(-0.02, 0.02)
                    elif (self.myCompass > 80) and (self.myCompass < 90):
                        self.driveMotorsUpdate(-0.001, 0.001)
                    else:
                        self.driveMotorsUpdate(-0.03, 0.03)

            elif self.proximadirecao == "South":
                if self.myCompass == 180 or self.myCompass == -180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz - 2
                else:
                    if (self.myCompass > 0) and (self.myCompass <= 130):
                        self.driveMotorsUpdate(-0.15, 0.15)
                    elif (self.myCompass > 130) and (self.myCompass <= 160):
                        self.driveMotorsUpdate(-0.04, 0.04)
                    elif (self.myCompass > 160) and (self.myCompass <= 170):
                        self.driveMotorsUpdate(-0.02, 0.02)
                    elif (self.myCompass > 170) and (self.myCompass < 180):
                        self.driveMotorsUpdate(-0.001, 0.001)
                    else:
                        self.driveMotorsUpdate(-0.03, 0.03)

        elif (self.xpontoatual) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.3:


            print("celula")
            
            self.driveMotorsUpdate(0.00,0.00)
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateNorth(point)
            self.drawMapNorth()
            self.stop = False
            
            if (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("WEST por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.center_id] < 1.3):
                print("NORTH por visitar")
                self.proximadirecao = "North"
                self.xorigemtransformada = self.xorigemtransformada + 2
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                self.rodando = True
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                self.rodando = True
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            else:
                print("NORTH POR EXCLUSAO")

                if self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - North")
                    self.proximadirecao = "North"
                    self.xorigemtransformada = self.xorigemtransformada + 2

                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - East")
                    self.rodando = True
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.left_id] < 1.3:
                    self.rodando = True
                    print("vou esquerda - West")
                    self.proximadirecao = "West"
                
        elif ((self.xpontoatual) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.3):

            print("perigo")
            
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            
            self.evaluateNorth(point)
            self.drawMapNorth()
            self.driveMotorsUpdate(0.00,0.00)
            self.rodando = True
            self.stop = True

            if (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("WEST por visitar")
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.left_id] < 1.3:
                    print("vou esquerda - West")
                    self.proximadirecao = "West"

                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - North")
                    self.proximadirecao = "North"
                    self.xorigemtransformada = self.xorigemtransformada + 2
                else:
                    print("vou tras - South")
                    self.proximadirecao = "South"
     
        elif ((self.xpontoatual) != (self.xorigemtransformada)) and self.stop == False:
            print("andar")

            if self.ypontoatual > round(self.ypontoatual):
                self.driveMotorsUpdate(0.07,0.05)
            elif self.ypontoatual < round(self.ypontoatual):
                self.driveMotorsUpdate(0.05,0.07)
            elif self.ypontoatual == round(self.ypontoatual):

                if self.myCompass > 0:
                    self.driveMotorsUpdate(0.07,0.05)
                elif self.myCompass < 0:
                    self.driveMotorsUpdate(0.05,0.07)
                elif self.myCompass == 0:
                    if (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.45:
                        print("1")
                        self.driveMotorsUpdate(0.15,0.15)
                    elif (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.15:
                        print("2")
                        self.driveMotorsUpdate(0.04,0.04)
                    elif (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.02:
                        print("3")
                        self.driveMotorsUpdate(0.01,0.01)


    def goingWest(self):

        print("going W")
        print("transformda: ", self.yorigemtransformada)

        if self.rodando == True:
            print("rodando")
            if self.proximadirecao == "South":
                if self.myCompass == 180 or self.myCompass == -180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    if (self.myCompass > 90) and (self.myCompass <= 130):
                        self.driveMotorsUpdate(-0.15, 0.15)
                    elif (self.myCompass > 130) and (self.myCompass <= 160):
                        self.driveMotorsUpdate(-0.04, 0.04)
                    elif (self.myCompass > 160) and (self.myCompass <= 170):
                        self.driveMotorsUpdate(-0.02, 0.02)
                    elif (self.myCompass > 170) and (self.myCompass < 180):
                        self.driveMotorsUpdate(-0.001, 0.001)
                    else:
                        self.driveMotorsUpdate(-0.03, 0.03)
                        
            elif self.proximadirecao == "North":
                
                if self.myCompass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    if (self.myCompass > 50) and (self.myCompass <= 90):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass > 20) and (self.myCompass <= 50):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass > 10) and (self.myCompass <= 20):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass > 0) and (self.myCompass <= 10):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)
            
            elif self.proximadirecao == "East":
                if self.myCompass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz + 2
                else:
                    if(self.myCompass >= 0):
                        self.driveMotorsUpdate(0.03, -0.03)
                    elif (self.myCompass < 0) and (self.myCompass >= -40):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass < -40) and (self.myCompass >= -70):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass < -70) and (self.myCompass >= -80):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass < -80) and (self.myCompass > -90):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            

        elif (self.ypontoatual) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.3:
            print("celula")
            self.driveMotorsUpdate(0.00,0.00)
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateWest(point)
            self.drawMapWest()
            self.stop = False

            if (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("SOUTH por visitar")
                self.rodando = True
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.center_id] < 1.3):
                print("WEST por visitar")
                self.proximadirecao = "West"
                self.yorigemtransformada = self.yorigemtransformada + 2
            elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("EAST por visitar")
                self.rodando = True
                self.proximadirecao = "East"
            else:
                print("WEST POR EXCLUSAO")

                if self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - West")
                    self.proximadirecao = "West"
                    self.yorigemtransformada = self.yorigemtransformada + 2

                elif self.measures.irSensor[self.left_id] < 1.3:
                    self.rodando = True
                    print("vou esquerda - South")
                    self.proximadirecao = "South"
                
                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - North")
                    self.rodando = True
                    self.proximadirecao = "North"

        elif (self.ypontoatual) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.3:
            print("perigo")
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateWest(point)
            self.drawMapWest()
            self.driveMotorsUpdate(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("EAST por visitar")
                self.proximadirecao = "East"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - West")
                    self.proximadirecao = "West"
                    self.yorigemtransformada = self.yorigemtransformada + 2

                elif self.measures.irSensor[self.left_id] < 1.3:
                    print("vou esquerda - South")
                    self.proximadirecao = "South"
                
                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - North")
                    self.proximadirecao = "North"

                elif self.measures.irSensor[self.back_id] < 1.3:
                    print("vou pa tras - East")
                    self.proximadirecao = "East"
                
        elif ((self.ypontoatual) != (self.yorigemtransformada)) and self.stop == False:
            print("andar")

            if self.xpontoatual > round(self.xpontoatual):
                self.driveMotorsUpdate(0.05,0.07)
            elif self.xpontoatual < round(self.xpontoatual):
                self.driveMotorsUpdate(0.07,0.05)
            elif self.xpontoatual == round(self.xpontoatual):

                if self.myCompass > 90:
                    self.driveMotorsUpdate(0.07,0.05)
                elif self.myCompass < 90:
                    self.driveMotorsUpdate(0.05,0.07)
                elif self.myCompass == 90:
                    if (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.45:
                        self.driveMotorsUpdate(0.15,0.15)
                    elif (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.15:
                        self.driveMotorsUpdate(0.04,0.04)
                    elif (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.02:
                        self.driveMotorsUpdate(0.01,0.01)

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
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    if (self.myCompass < 180) and (self.myCompass >= 140):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass < 140) and (self.myCompass >= 110):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass < 110) and (self.myCompass >= 100):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass < 100) and (self.myCompass > 90):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            elif self.proximadirecao == "East":
                if self.myCompass == -90:
                    self.direcao = "East"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    if (self.myCompass < -140) and (self.myCompass >= -180):
                        self.driveMotorsUpdate(-0.15, 0.15)
                    elif (self.myCompass < -110) and (self.myCompass >= -140):
                        self.driveMotorsUpdate(-0.04, 0.04)
                    elif (self.myCompass < -100) and (self.myCompass >= -110):
                        self.driveMotorsUpdate(-0.02, 0.02)
                    elif (self.myCompass < -90) and (self.myCompass > -100):
                        self.driveMotorsUpdate(-0.001, 0.001)
                    else:
                        self.driveMotorsUpdate(-0.03, 0.03)
            
            elif self.proximadirecao == "North":
                
                if self.myCompass == 0:
                    self.direcao = "North"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.yorigemmatriz = self.yorigemmatriz + 2
                else:
                    if (self.myCompass > 50) and (self.myCompass <= 180):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass > 20) and (self.myCompass <= 50):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass > 10) and (self.myCompass <= 20):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass > 0) and (self.myCompass <= 10):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            

        elif (self.xpontoatual) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] < 1.3:
            print("celula")
            self.driveMotorsUpdate(0.00,0.00)
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateSouth(point)
            self.drawMapSouth()
            self.stop = False

            if (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.center_id] < 1.3): 
                print("SOUTH por visitar")
                self.proximadirecao = "South"
                self.xorigemtransformada = self.xorigemtransformada - 2
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("EAST por visitar")
                self.rodando = True
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("WEST por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            else:
                print("SOUTH POR EXCLUSAO")

                

                if self.measures.irSensor[self.left_id] < 1.3:
                    self.rodando = True
                    print("vou esquerda - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.left_id] < 1.3:
                    self.rodando = True
                    print("vou esquerda - East")
                    self.proximadirecao = "East"

                elif self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - South")
                    self.proximadirecao = "South"
                    self.xorigemtransformada = self.xorigemtransformada - 2
  
        elif (self.xpontoatual) == (self.xorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.3:
            print("perigo")
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateSouth(point)
            self.drawMapSouth()
            self.driveMotorsUpdate(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("EAST por visitar")
                self.proximadirecao = "East"
            elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("WEST por visitar")
                self.proximadirecao = "West"
            elif (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            else:
                print("todas as posicoes a volta descobertas")

                if self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - West")
                    self.proximadirecao = "West"

                elif self.measures.irSensor[self.left_id] < 1.3:
                    print("vou esquerda - East")
                    self.proximadirecao = "East"
                
                elif self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - South")
                    self.proximadirecao = "South"
                    self.yorigemtransformada = self.yorigemtransformada - 2

                elif self.measures.irSensor[self.back_id] < 1.3:
                    print("vou pa tras - North")
                    self.proximadirecao = "North"

        elif ((self.xpontoatual) != (self.xorigemtransformada)) and self.stop == False:
            print("andar")

            if self.ypontoatual > round(self.ypontoatual):
                self.driveMotorsUpdate(0.05,0.07)
            elif self.ypontoatual < round(self.ypontoatual):
                self.driveMotorsUpdate(0.07,0.05)
            elif self.ypontoatual == round(self.ypontoatual):

                if self.myCompass > -180 and self.myCompass < -1:
                    self.driveMotorsUpdate(0.07,0.05)
                elif self.myCompass > 1 and self.myCompass < 180:
                    self.driveMotorsUpdate(0.05,0.07)
                elif (self.myCompass == 180) or (self.myCompass == -180):
                    if (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.45:
                        self.driveMotorsUpdate(0.15,0.15)
                    elif (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.15:
                        self.driveMotorsUpdate(0.04,0.04)
                    elif (abs((self.xpontoatual) - (self.xorigemtransformada))) > 0.02:
                        self.driveMotorsUpdate(0.01,0.01)

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
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    if (self.myCompass > -90) and (self.myCompass <= -50):
                        self.driveMotorsUpdate(-0.15, 0.15)
                    elif (self.myCompass > -50) and (self.myCompass <= -20):
                        self.driveMotorsUpdate(-0.04, 0.04)
                    elif (self.myCompass > -20) and (self.myCompass <= -10):
                        self.driveMotorsUpdate(-0.02, 0.02)
                    elif (self.myCompass > -10) and (self.myCompass < 0):
                        self.driveMotorsUpdate(-0.001, 0.001)
                    else:
                        self.driveMotorsUpdate(-0.03, 0.03)

            elif self.proximadirecao == "South":
                if self.myCompass == -180 or self.myCompass == 180:
                    self.direcao = "South"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    if (self.myCompass < -90) and (self.myCompass >= -130):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass < -130) and (self.myCompass >= -160):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass < -160) and (self.myCompass >= -170):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass < -170) and (self.myCompass >= -180):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            elif self.proximadirecao == "West":
                if self.myCompass == 90:
                    self.direcao = "West"
                    self.proximadirecao = ""
                    self.rodando = False
                    self.stop = False
                    self.xorigemmatriz = self.xorigemmatriz - 2
                else:
                    if(self.myCompass <= 0):
                        self.driveMotorsUpdate(0.03, -0.03)
                    elif (self.myCompass > 0) and (self.myCompass <= 40):
                        self.driveMotorsUpdate(0.15, -0.15)
                    elif (self.myCompass > 40) and (self.myCompass <= 70):
                        self.driveMotorsUpdate(0.04, -0.04)
                    elif (self.myCompass > 70) and (self.myCompass <= 80):
                        self.driveMotorsUpdate(0.02, -0.02)
                    elif (self.myCompass > 80) and (self.myCompass < 90):
                        self.driveMotorsUpdate(0.001, -0.001)
                    else:
                        self.driveMotorsUpdate(0.03, -0.03)

            

        elif (self.ypontoatual) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] < 1.3:
            print("celula")
            self.driveMotorsUpdate(0.0,0.0)
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateEast(point)
            self.drawMapEast()
            self.stop = False
            
            if (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("NORTH por visitar")
                self.rodando = True
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((xround,yround-2)) > 0) and (self.measures.irSensor[self.center_id] < 1.3):
                print("EAST por visitar")
                self.proximadirecao = "East"
                self.yorigemtransformada = self.yorigemtransformada - 2
            elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("WEST esta por visitar")
                self.rodando = True
                self.proximadirecao = "West"
            else:
                print("EAST POR EXCLUSAO")
                
                if self.measures.irSensor[self.center_id] < 1.3:
                    self.proximadirecao = "East"
                    self.yorigemtransformada = self.yorigemtransformada - 2
                elif self.measures.irSensor[self.left_id] < 1.3:
                    self.rodando = True
                    print("vou esquerda - North")
                    self.proximadirecao = "North"
                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - South")
                    self.rodando = True
                    self.proximadirecao = "South"

        elif (self.ypontoatual ) == (self.yorigemtransformada) and self.measures.irSensor[self.center_id] >= 1.3:
            print("perigo")
            xround = round(self.xpontoatual)
            yround = round(self.ypontoatual)
            point = (xround, yround)
            self.evaluateEast(point)
            self.drawMapEast()
            self.driveMotorsUpdate(0.00,0.00)
            self.rodando = True
            self.stop = True
            
            if (self.nosparavisitar.count((xround+2,yround)) > 0) and (self.measures.irSensor[self.left_id] < 1.3):
                print("NORTH por visitar")
                self.proximadirecao = "North"
            elif (self.nosparavisitar.count((xround-2,yround)) > 0) and (self.measures.irSensor[self.right_id] < 1.3):
                print("SOUTH por visitar")
                self.proximadirecao = "South"
            elif (self.nosparavisitar.count((xround,yround+2)) > 0) and (self.measures.irSensor[self.back_id] < 1.3):
                print("WEST esta por visitar")
                self.proximadirecao = "West"
            else:
                print("todas as posicoes a volta descobertas")
                if self.measures.irSensor[self.left_id] < 1.3:
                    print("vou esquerda - North")
                    self.proximadirecao = "North"
                
                elif self.measures.irSensor[self.right_id] < 1.3:
                    print("vou direita - South")
                    self.proximadirecao = "South"

                elif self.measures.irSensor[self.center_id] < 1.3:
                    print("vou em frente - East")
                    self.proximadirecao = "East"
                    self.yorigemtransformada = self.yorigemtransformada - 2   

                elif self.measures.irSensor[self.back_id] < 1.3:
                    print("vou pa tras - West")
                    self.proximadirecao = "West"         

        elif ((self.ypontoatual) != (self.yorigemtransformada)) and self.stop == False:
            print("andar")

            if self.xpontoatual > round(self.xpontoatual):
                self.driveMotorsUpdate(0.07,0.05)
            elif self.xpontoatual < round(self.xpontoatual):
                self.driveMotorsUpdate(0.05,0.07)
            elif self.xpontoatual == round(self.xpontoatual):

                if self.myCompass > -90:
                    self.driveMotorsUpdate(0.07,0.05)
                elif self.myCompass < -90:
                    self.driveMotorsUpdate(0.05,0.07)
                elif self.myCompass == -90:
                    if (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.45:
                        self.driveMotorsUpdate(0.15,0.15)
                    elif (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.15:
                        self.driveMotorsUpdate(0.04,0.04)
                    elif (abs((self.ypontoatual) - (self.yorigemtransformada))) > 0.02:
                        self.driveMotorsUpdate(0.01,0.01)

    def evaluateNorth(self, point):
    
        print("\n\n\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if self.measures.irSensor[self.back_id] < 1.3:
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.center_id] < 1.3):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.right_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.left_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateWest(self, point):

        print("\n\n\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.center_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.right_id] < 1.3):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.left_id] < 1.3):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateSouth(self, point):

        print("\n\n\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.3):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

        if(self.measures.irSensor[self.center_id] < 1.3):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.right_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.left_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        while self.nosparavisitar.count(point) > 0:
            self.nosparavisitar.remove(point)

        print("NOS PARA VISITAR: ", list(dict.fromkeys(self.nosparavisitar))) 

    def evaluateEast(self, point):

        print("\n\n\nPONTO ATUAL: ",point)

        if self.nosvisitados.count(point) == 0:
            self.nosvisitados.append(point)

        if(self.measures.irSensor[self.back_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]+2)) == 0:
                self.nosparavisitar.append((point[0], point[1]+2))

        if(self.measures.irSensor[self.center_id] < 1.3):
            if self.nosvisitados.count((point[0], point[1]-2)) == 0:
                self.nosparavisitar.append((point[0], point[1]-2))

        if(self.measures.irSensor[self.right_id] < 1.3):
            if self.nosvisitados.count((point[0]-2, point[1])) == 0:
                self.nosparavisitar.append((point[0]-2, point[1]))

        if(self.measures.irSensor[self.left_id] < 1.3):
            if self.nosvisitados.count((point[0]+2, point[1])) == 0:
                self.nosparavisitar.append((point[0]+2, point[1]))

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
            self.finish()

    # Calculation of final coordinates
    def localization(self):
        self.movementModel()
        self.nearestDirectionEstimate()
        self.sensorsCorrection()

        # self.media_x = self.movement_model_x 
        # self.media_y = self.movement_model_y 

        self.media_x = (self.movement_model_x + self.sensors_x) / 2
        self.media_y = (self.movement_model_y + self.sensors_y) / 2

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
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

        # Direction S
        if self.closest_direction == "S":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3:
                # print("Front correction")
                closest_x_wall_distance = 1/self.measures.irSensor[0]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3:
                # print("Back correction")
                closest_x_wall_distance = 1/self.measures.irSensor[3]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3:
                # print("Left correction")
                closest_y_wall_distance = 1/self.measures.irSensor[1]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3:
                # print("Right correction")
                closest_y_wall_distance = 1/self.measures.irSensor[2]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius
        
        # Direction W
        if self.closest_direction == "W":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

        # Direction E
        if self.closest_direction == "E":
            # Front correction
            if (self.measures.irSensor[0] > self.measures.irSensor[3]) and self.measures.irSensor[0] > 3:
                # print("Front correction")
                closest_y_wall_distance = 1/self.measures.irSensor[0]
                closest_y_wall_coordinate = round(self.movement_model_y - 1)
                self.sensors_y = closest_y_wall_coordinate + self.wall_diameter + closest_y_wall_distance + self.robot_radius

            # Back correction
            elif (self.measures.irSensor[0] < self.measures.irSensor[3]) and self.measures.irSensor[3] > 3:
                # print("Back correction")
                closest_y_wall_distance = 1/self.measures.irSensor[3]
                closest_y_wall_coordinate = round(self.movement_model_y + 1)
                self.sensors_y = closest_y_wall_coordinate - self.wall_diameter - closest_y_wall_distance - self.robot_radius

            # Left correction
            if (self.measures.irSensor[1] > self.measures.irSensor[2]) and self.measures.irSensor[1] > 3:
                # print("Left correction")
                closest_x_wall_distance = 1/self.measures.irSensor[1]
                closest_x_wall_coordinate = round(self.movement_model_x - 1)
                self.sensors_x = closest_x_wall_coordinate + self.wall_diameter + closest_x_wall_distance + self.robot_radius

            # Right correction
            elif (self.measures.irSensor[1] < self.measures.irSensor[2]) and self.measures.irSensor[2] > 3:
                # print("Right correction")
                closest_x_wall_distance = 1/self.measures.irSensor[2]
                closest_x_wall_coordinate = round(self.movement_model_x + 1)
                self.sensors_x = closest_x_wall_coordinate - self.wall_diameter - closest_x_wall_distance - self.robot_radius
        
        # # print("closest_x_wall_coordinate: ", closest_x_wall_coordinate)
        # # print("closest_y_wall_coordinate: ", closest_y_wall_coordinate)

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
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

import math
import serial
import time
ultrasoundsList = [0, 0, 0, 0]
# The step variable indicates where we are in the map (before the balls, at the first block,...)
step = 0


    # receive serial text from arduino and publish it to '/arduino' message
def get_arduino_message1(data1):
    global ultrasoundsList
    ultrasound1 = data1.range
    #print('capteur 1')
    # print(data1.range)
    ultrasoundsList[0] = data1.range
#        time.sleep(1)

def get_arduino_message2(data2):
    global ultrasoundsList
    #print('capteur 2')
    #print(data2.range)
    ultrasoundsList[1] = data2.range

  #      time.sleep(1)

def get_arduino_message3(data3):
    global ultrasoundsList
    #print('capteur 3')
    #print(data3.range)
    ultrasoundsList[2] = data3.range

 #       time.sleep(1)

def get_arduino_message4(data4):
    global ultrasoundsList
    #print('capteur 4')
    #print(data4.range)
    ultrasoundsList[3] = data4.range

#        time.sleep(1)

def navigate(data):
    global ultrasoundsList
    global step
    ultrasoundR = ultrasoundsList[0]
    ultrasoundL = ultrasoundsList[1]
    ultrasoundF = ultrasoundsList[2]
    ultrasoundB = ultrasoundsList[3]
    print("capteurR : " + str(ultrasoundR))
    print("capteurL : " + str(ultrasoundL))
    print("capteurF : " + str(ultrasoundF))
    print("capteurB : " + str(ultrasoundB))


    #verifier que l'encodeur a pas encore parcouru 21 cm (le chiffre depend de la ou commence le robot)
    if ultrasoundL != 0.000 and ultrasoundF != 0.000 and ultrasoundB != 0.000:
        print("testultrasound" + str(step))
        #Pour le petit robot
        if step == 0:
            if ultrasoundL < 0.09 and ultrasoundB < 0.845 and ultrasoundF > 0.32:
                # fonction pour aller tout droit
                print("avant les balles")

            #Pour ulstrasoundF faut voir si il detecte le bloc qui est devant
            elif ultrasoundL < 0.09 and ultrasoundB > 0.845 and ultrasoundF < 0.97:
                print("on est aux balles")
                step = 1
                time.sleep(2)
                #Il faudra prendre les balles et aller vers la droite ici pour se mettre sur la ligne du premier bloc
            elif ultrasoundL > 0.09:
                print("Faut se decaler vers la gauche")
                #Aller vers la gauche while on est pas assez proche des balles
        #Pour le grand robot
        elif step == 1:
            if ultrasoundL > 0.27 and ultrasoundB < 0.94:
            #
                print ("avant le premier bloc")
            #
            elif ultrasoundL < 0.27 and ultrasoundL > 0.21 and ultrasoundB > 0.94:

                print("on peut prendre le premier bloc")



if __name__ == '__main__':
    try:
        rospy.init_node('car_driver', anonymous=True)
        rospy.Subscriber('/ultrasound1', Range, get_arduino_message1)
      #  time.sleep(1)
        rospy.Subscriber('/ultrasound2', Range, get_arduino_message2)
        rospy.Subscriber('/ultrasound3', Range, get_arduino_message3)
        rospy.Subscriber('/ultrasound4', Range, get_arduino_message4)
        rospy.Subscriber('/ultrasound4', Range, navigate)
        rospy.spin()
        print("test")

    except rospy.ROSInterruptException:
        pass

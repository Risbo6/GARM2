#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from playsound import playsound
from random import randint
from math import degrees
import re 
from numpy import sign, array 
from numpy.linalg import norm 

def wordChecker(myString = '', keyWords = []):
    #Exemple d'utilisation : ['boris/jean', 'mange/bois']

    keyWordsSpaced = []
    myString = myString.lower() #La string est convertie en lowecase
    myString = ' ' + myString + ' ' #Ajoute espace au début et à la fin
    for i in keyWords:
        keyWordsSpaced = []

        #Ajoute un espace avant chaque mot. Sinon active proc dans le mot désactive par ex.
        for k in range(len(i.split('/'))):
            keyWordsSpaced.append(i.split('/')[k]) 
            keyWordsSpaced[k] = ' ' + keyWordsSpaced[k]

            
            
        if any(x in myString for x in keyWordsSpaced):
            #print(keyWordsSpaced)
            pass
        else:
            return 0

    return 1


path = '/home/other/catkin_ws/src/vocal/scripts/voicepack/'


class parameters():
    def __init__(self):
        self.voiceControl = True
        self.isFirstInterraction = True
        self.mode = 'manuel'
        self.speed_pourcent = 50 #Vitesse en pourcentage
        self.max_speed_ms = 1.5/100 #Vitesse max en m/s. Sera mutliplié par le pourcentage.
        self.max_speed_rotational_rad = 1/100 #Vitesse rot max en rad/s. Sera mutliplié par le pourcentage.
        self.linear_speed = 0 #Vaut 1, 0 ou -1. Est multiplié par max_speed_ms et speed_pourcent.
        self.rotational_speed = 0 #Vaut 1, 0 ou -1. Est multiplié par max_speed_rotational_rad et speed_pourcent.

        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_orientation_yaw = 0 #Degrés

        self.predefined_move = False
        self.wished_rotation = 0
        self.start_orientation_yaw = 0
        self.wished_deplacement = 0
        self.start_pose_x = 0
        self.start_pose_y = 0

        self.yaw = 0
        self.yaw_old = 0
        self.i = 0
        self.i_offset = 0
        self.isFirstYawOffset = True
        self.yawOffset = 0
        



def text_callback(data):
    incoming_message = data.data
    global params

    if params.voiceControl:

        if (wordChecker(incoming_message, ['activ', 'mode/commande/contrôle/pilote/navigation', 'manuel/main'])
            or wordChecker(incoming_message, ['désactiv/off', 'auto/l''auto'])
            or wordChecker(incoming_message, ['désactiv', 'mode/commande/contrôle/pilote/navigation', 'auto/l''auto'])):

            params.mode = 'manuel'

            i = randint(1, 7)
            playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')
            i = randint(1, 2)
            playsound(path+'autopilot/autopilot_disengage_'+str(i)+'.mp3')
            i = randint(1, 5)
            playsound(path+'awaiting/awaiting_order_'+str(i)+'.mp3')

        elif (wordChecker(incoming_message, ['activ', 'mode/commande/contrôle/pilote/navigation', 'auto/l''auto'])
            or wordChecker(incoming_message, ['activ/on', 'auto/l''auto'])
            or wordChecker(incoming_message, ['désactiv', 'mode/commande/contrôle/pilote/navigation', 'manuel/main'])):

            params.mode = 'automatique'

            i = randint(1, 7)
            playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')

            i = randint(1, 2)
            playsound(path+'autopilot/autopilot_engage_'+str(i)+'.mp3')




        if params.mode == 'manuel' :
            ##Paramètres de vitesse
            if (wordChecker(incoming_message, ['augment/amplifi/agrandi/monte', 'de', 'vitesse']) and any(x.isdigit() for x in incoming_message)):
                params.speed_pourcent = params.speed_pourcent + int(re.findall('\d+', incoming_message)[-1])

                if params.speed_pourcent >= 100 :
                    params.speed_pourcent = 100
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')

                elif params.speed_pourcent <= 10 :
                    params.speed_pourcent = 10
                    i = randint(1, 2)
                    playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')

                else:
                    i = randint(1, 6)
                    playsound(path+'speed/increase_speed_'+str(i)+'.mp3')



            elif (wordChecker(incoming_message, ['diminue/baisse/rédui/limit','de', 'vitesse']) and any(x.isdigit() for x in incoming_message)):
                params.speed_pourcent = params.speed_pourcent - int(re.findall('\d+', incoming_message)[-1])

                if params.speed_pourcent >= 100 :
                    params.speed_pourcent = 100
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')

                elif params.speed_pourcent <= 10 :
                    params.speed_pourcent = 10
                    i = randint(1, 2)
                    playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')

                else:
                    i = randint(1, 7)
                    playsound(path+'speed/reduce_speed_'+str(i)+'.mp3')


            elif (wordChecker(incoming_message, ['vitesse']) and any(x.isdigit() for x in incoming_message)):
                old_speed = params.speed_pourcent
                params.speed_pourcent = int(re.findall('\d+', incoming_message)[-1])

                if params.speed_pourcent >= 100 :
                    params.speed_pourcent = 100
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')

                elif params.speed_pourcent <= 10 :
                    params.speed_pourcent = 10
                    i = randint(1, 2)
                    playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')

                elif old_speed > params.speed_pourcent:
                    i = randint(1, 7)
                    playsound(path+'speed/reduce_speed_'+str(i)+'.mp3')

                elif old_speed < params.speed_pourcent:
                    i = randint(1, 6)
                    playsound(path+'speed/increase_speed_'+str(i)+'.mp3')


            elif wordChecker(incoming_message, ['augment/amplifi/agrandi/monte', 'vitesse']):
                params.speed_pourcent = params.speed_pourcent + 10

                if params.speed_pourcent >= 100 :
                    params.speed_pourcent = 100
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')

                elif params.speed_pourcent <= 10 :
                    params.speed_pourcent = 10
                    i = randint(1, 2)
                    playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')

                else:
                    i = randint(1, 6)
                    playsound(path+'speed/increase_speed_'+str(i)+'.mp3')

            elif wordChecker(incoming_message, ['diminue/baisse/rédui/limit', 'vitesse']):
                params.speed_pourcent = params.speed_pourcent - 10

                if params.speed_pourcent >= 100 :
                    params.speed_pourcent = 100
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')

                elif params.speed_pourcent <= 10 :
                    params.speed_pourcent = 10
                    i = randint(1, 2)
                    playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')

                else:
                    i = randint(1, 7)
                    playsound(path+'speed/reduce_speed_'+str(i)+'.mp3')


            elif wordChecker(incoming_message, ['max', 'vitesse']):
                params.speed_pourcent = 100
                i = randint(1, 4)
                playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')


            elif wordChecker(incoming_message, ['min', 'vitesse']) or wordChecker(incoming_message, ['préci/lentement/doucement']):
                params.speed_pourcent = 10
                i = randint(1, 2)
                playsound(path+'speed/speed_minimum_'+str(i)+'.mp3')


            elif wordChecker(incoming_message, ['vitesse', 'actuel/en ce moment/quel/maitenant']):

                if int(round(params.speed_pourcent/10)) == 0 :
                    playsound(path+'moving/all_stop.mp3')
            
                elif int(round(params.speed_pourcent/10)) == 1 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/10_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 2 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/20_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 3 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/30_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 4 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/40_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 5 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/50_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 6 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/60_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 7 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/70_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 8 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/80_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 9 :
                    i = randint(1, 2)
                    playsound(path+'pourcent/90_pourcent_'+str(i)+'.mp3')

                elif int(round(params.speed_pourcent/10)) == 10 :
                    i = randint(1, 4)
                    playsound(path+'speed/speed_maximum_'+str(i)+'.mp3')





            

            if (wordChecker(incoming_message, ['avan/tout droit']) and any(x.isdigit() for x in incoming_message)):
                
                params.wished_deplacement= int(re.findall('\d+', incoming_message)[-1])
                params.predefined_move = True
                params.start_pose_x = params.current_pose_x
                params.start_pose_y = params.current_pose_y

                params.rotational_speed = 0
                params.linear_speed = 1

                i = randint(1, 7)
                playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')

            elif wordChecker(incoming_message, ['avan/tout droit']):
                params.linear_speed = 1
                params.rotational_speed = 0

                playsound(path+'moving/manoeuvring.mp3')
                playsound(path+'moving/forward.mp3')


            elif (wordChecker(incoming_message, ['recul/arrière']) and any(x.isdigit() for x in incoming_message)):
                
                params.wished_deplacement= int(re.findall('\d+', incoming_message)[-1])
                params.predefined_move = True
                params.start_pose_x = params.current_pose_x
                params.start_pose_y = params.current_pose_y

                params.rotational_speed = 0
                params.linear_speed = -1

                i = randint(1, 7)
                playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')

            elif wordChecker(incoming_message, ['recul/arrière']):
                params.linear_speed = -1
                params.rotational_speed = 0

                playsound(path+'moving/manoeuvring.mp3')
                playsound(path+'moving/back.mp3')



            elif wordChecker(incoming_message, ["stop/arrêt/t'arrêt/s'arrêt"]):
                params.linear_speed = 0
                params.rotational_speed = 0

                i = randint(1, 3)
                playsound(path+'moving/stop_'+str(i)+'.mp3')


            elif (wordChecker(incoming_message, ['gauche']) and any(x.isdigit() for x in incoming_message)):
                

                params.wished_rotation = int(re.findall('\d+', incoming_message)[-1])
                params.predefined_move = True
                params.start_orientation_yaw = params.current_orientation_yaw

                params.rotational_speed = 1
                params.linear_speed = 0

                i = randint(1, 7)
                playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')

                if params.wished_rotation == 180:
                    playsound(path+'moving/turning_around.mp3')
 

            elif (wordChecker(incoming_message, ['droite/180']) and any(x.isdigit() for x in incoming_message)):
                
                params.wished_rotation = int(re.findall('\d+', incoming_message)[-1])
                params.predefined_move = True
                params.start_orientation_yaw = params.current_orientation_yaw

                params.rotational_speed = -1
                params.linear_speed = 0

                i = randint(1, 7)
                playsound(path+'acknowledge/acknowledgment_positive_'+str(i)+'.mp3')
                if params.wished_rotation == 180:
                    playsound(path+'moving/turning_around.mp3')


            elif wordChecker(incoming_message, ['gauche']):
                params.rotational_speed = 1
                params.linear_speed = 0


                playsound(path+'moving/manoeuvring.mp3')
                playsound(path+'moving/left.mp3')   



            elif wordChecker(incoming_message, ['droite']):
                params.rotational_speed = -1
                params.linear_speed = 0

                playsound(path+'moving/manoeuvring.mp3')
                playsound(path+'moving/right.mp3')



            if wordChecker(incoming_message, ['allum/on/active', 'lumière/lampe']):

                playsound(path+'lights/lights_on.mp3')

            elif wordChecker(incoming_message, ['étein/désactiv/off', 'lumière/lampe']):

                playsound(path+'lights/lights_off.mp3')
                


            



    

    

    if wordChecker(incoming_message, ['activ', 'commande/reconnaissance/contrôle', 'vocal']):
        params.voiceControl = True

        i = randint(1, 2)
        playsound(path+'voicecontrol/voice_control_on_'+str(i)+'.mp3')

        if params.isFirstInterraction :
            i = randint(1, 5)
            playsound(path+'greetings/greetings_'+str(i)+'.mp3')
            isFirstInterraction = False


    elif wordChecker(incoming_message, ['désactiv', 'commande/reconnaissance/contrôle', 'vocal']):
        params.voiceControl = False

        i = randint(1, 2)
        playsound(path+'voicecontrol/voice_control_off_'+str(i)+'.mp3')

        i = randint(1, 2)
        playsound(path+'goodbye/goodbye_'+str(i)+'.mp3')

        params.isFirstInterraction = True


    elif wordChecker(incoming_message, ['commande/reconnaissance/contrôle', 'vocal']):
        i = randint(1, 4)
        playsound(path+'comply/cannot_comply_'+str(i)+'.mp3')






def odom_callback(data):
    global params

    params.current_pose_x = data.pose.pose.position.x
    params.current_pose_y = data.pose.pose.position.y   
    

    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    

    if params.isFirstYawOffset == True :
        params.yawOffset = degrees(euler[2])+180
        params.isFirstYawOffset = False

    params.yaw = (degrees(euler[2])-params.yawOffset)%360
    
    if sign(params.yaw-params.yaw_old) == -1:
        if abs(params.yaw-params.yaw_old) > 200:
            params.i+=1
            params.i_offset = params.i*360

    if sign(params.yaw-params.yaw_old) == 1:
        if abs(params.yaw-params.yaw_old) > 200:
            params.i-=1
            params.i_offset = params.i*360
    
    params.current_orientation_yaw = params.yaw+params.i_offset

    params.yaw_old = (degrees(euler[2])-params.yawOffset)%360

    
    

def main():
    global params

    rospy.init_node('Verity', anonymous=True)
    rospy.Subscriber("/speech_recognition/final_result", String, text_callback)
    #rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/odom/filtered", Odometry, odom_callback)

    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if params.mode == 'manuel':
            vel_msg = Twist()
            vel_msg.linear.x = params.linear_speed*params.max_speed_ms*params.speed_pourcent
            vel_msg.angular.z = params.rotational_speed*params.max_speed_rotational_rad*params.speed_pourcent
            


            Kp = 1/50
            #Rotation d'un angle voulu
            if params.predefined_move == True:
                if params.rotational_speed == 1 : #Tourne à gauche
                    Error = abs(params.start_orientation_yaw+params.wished_rotation-params.current_orientation_yaw)
                    

                    #print(Error*Kp)

                    if abs(Error*Kp) < (params.max_speed_rotational_rad*params.speed_pourcent):
                        vel_msg.angular.z = params.rotational_speed*abs(Error*Kp)

                        if vel_msg.angular.z < 0.05: #Vitesse minimum, sinon le robot va tourner à 0.0001
                            vel_msg.angular.z = 0.05


                    else :
                        vel_msg.angular.z = params.rotational_speed*params.max_speed_rotational_rad*params.speed_pourcent


                    print(vel_msg.angular.z)


                    if (params.start_orientation_yaw+params.wished_rotation <= params.current_orientation_yaw) :
                        print("movement complete")
                        
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        pub_vel.publish(vel_msg)
                        params.predefined_move = False
                        params.rotational_speed = 0
                        
                        i = randint(1, 3)
                        playsound(path+'complete/complete_'+str(i)+'.mp3')

                

                if params.rotational_speed == -1 : #Tourne à droite
                    #vel_msg.angular.z = params.rotational_speed*params.max_speed_rotational_rad*params.speed_pourcent
                    Error = abs(params.current_orientation_yaw - params.start_orientation_yaw + params.wished_rotation)

                    #print(abs(Error*Kp))
                    if abs(Error*Kp) < (params.max_speed_rotational_rad*params.speed_pourcent):
                        vel_msg.angular.z = params.rotational_speed*abs(Error*Kp)
                        

            

                        if abs(vel_msg.angular.z) < 0.05: #Vitesse minimum, sinon le robot va tourner à 0.0001
                            vel_msg.angular.z = 0.05*params.rotational_speed

                        

                    else :
                        vel_msg.angular.z = params.rotational_speed*params.max_speed_rotational_rad*params.speed_pourcent

                    print(vel_msg.angular.z)


                    if (params.start_orientation_yaw-params.wished_rotation >= params.current_orientation_yaw) :
                        print("movement complete")

                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        pub_vel.publish(vel_msg)
                        params.predefined_move = False
                        params.rotational_speed = 0
                    

                        i = randint(1, 3)
                        playsound(path+'complete/complete_'+str(i)+'.mp3')

            #Déplacement d'un longueur voulue

                

                if params.linear_speed == 1 :
                    v = array([params.start_pose_x-params.current_pose_x, params.start_pose_y-params.current_pose_y])
                    #print((norm(v,2)))
                    vel_msg.linear.x = params.linear_speed*params.max_speed_ms*params.speed_pourcent
                    if (norm(v,2) >= params.wished_deplacement) :
                        print("movement complete")

                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        pub_vel.publish(vel_msg)
                        params.predefined_move = False
                        params.linear_speed = 0
                    
                        i = randint(1, 3)
                        playsound(path+'complete/complete_'+str(i)+'.mp3')


                if params.linear_speed == -1 :
                    v = array([params.start_pose_x-params.current_pose_x, params.start_pose_y-params.current_pose_y])
                    vel_msg.linear.x = params.linear_speed*params.max_speed_ms*params.speed_pourcent
                    if (norm(v,2) >= params.wished_deplacement) :
                        print("movement complete")

                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        pub_vel.publish(vel_msg)
                        params.predefined_move = False
                        params.linear_speed = 0
                    
                        i = randint(1, 3)
                        playsound(path+'complete/complete_'+str(i)+'.mp3')





            pub_vel.publish(vel_msg)




        r.sleep()


if __name__ == '__main__':
    params = parameters()
    main()

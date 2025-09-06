# Python code for Multiple Color Detection
import RPi.GPIO as GPIO


import argparse
import serial
import time
import numpy as np
import cv2
import math

cameraActual=100

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#GPIO.setup(26,GPIO.OUT)
#GPIO.output(26,GPIO.LOW)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.LOW)

GPIO.setup(19,GPIO.IN)
GPIO.setup(26,GPIO.IN)

led=True

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--picamera", type=int, default=-1,
	help="whether or not the Raspberry Pi camera should be used")
ap.add_argument("-l", "--log", type=str, default="log.txt",
	help="path to output log file")
args = vars(ap.parse_args())

slope=1
mingreen=10
minred =10
counter = 0
limitCounter = 30
parking =0 
position="near"
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
GPIO.output(18,GPIO.LOW)
time.sleep(1)

GPIO.output(18,led)
#GPIO.output(26,GPIO.HIGH)

#ser.reset_input_buffer()
# Capturing video through webcam

#webcam = cv2.VideoCapture(0)
redarea = 0
greanarea = 0
KNOWN_DISTANCE = 94  # centimeter
KNOWN_WIDTH = 5  # centimeter
# Start a while loop
def FocalLength(measured_distance, real_width, width_in_rf_image):
    focal_length = (width_in_rf_image* measured_distance)/ real_width
    return focal_length

def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length)/face_width_in_frame
    return distance
camera1index =2
cameraParking2index=0
webcam = cv2.VideoCapture(camera1index)
run =True
# Setting 

#rect hieght
pcpf=30
pcpn=40




while run==True:
    
    while (parking == 0):
      #  webcam = cv2.VideoCapture(2)

       # print ("Camera 1")
        GPIO.output(18,led)
        led= not led
        
        #print(ser.readline())
    # line = ser.readline().decode('utf-8').rstrip()
        redcct=0
        parkingcct=0
        greencct=0
        redy=0
        greeny=0
        width0  = webcam.get(cv2.CAP_PROP_FRAME_WIDTH)

    # print (width0)
        hight0  = webcam.get(cv2.CAP_PROP_FRAME_HEIGHT)

    # print (hight0)
        # Reading the video from the
        # webcam in image frames
        

        _, imageFrame = webcam.read()
        #    imageFrame = cv2.resize(imageFrame, (800, 300))
        imageFrame = imageFrame[125:250,:]
            # Convert the imageFrame in
            # BGR(RGB color space) to
            # HSV(hue-saturation-value)
            # color space
        
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        #not red its fosia color for parking
        red_lower = np.array([170, 50, 50], np.uint8)
        red_upper = np.array([179, 182, 170], np.uint8)
        #red_lower = np.array([170, 100, 80], np.uint8)
        #red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        
        
        

        # Set range for green color and
        # define mask
        green_lower = np.array([64,100, 40], np.uint8)
        green_upper = np.array([69, 190, 100], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)


        #not red its fosia color for parking
        parking_lower = np.array([155, 127 ,50], np.uint8)
        parking_upper = np.array([164, 199, 153], np.uint8)
        parking_mask = cv2.inRange(hsvFrame, parking_lower, parking_upper)


        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=red_mask)

        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                    mask=green_mask)
        # For barking color
        parking_mask = cv2.dilate(parking_mask, kernal)
        res_parking = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=parking_mask)

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask

        counter = counter +1
        mingreen=10
        minred =10
#RED
        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)


        for pic, contour in enumerate(contours):

            area = cv2.contourArea(contour)
        # redarea = area
            pt1 = (320, 0)
            pt2 = (320, 480)

            color = (255, 255, 0)
            cv2.line(imageFrame, pt1, pt2,color)
            if (area > 300):
                redcct=redcct+1
                x, y, w, h = cv2.boundingRect(contour)
                redarea = w 
               # print (w)
                dd= int(Distance_finder(60.16,5,w/10))
                pt1 = (x, y)
                pt2 = (320, 480)
                if x>320 :
                    pt1 = (x+w, y)
                pt3 = (320,y)
            # print (pt1)
            #  slope = (480-y) / (320 -x)
                color = (0, 0, 255)
                cv2.line(imageFrame, pt1, pt2,color)
                cv2.line(imageFrame, pt3, pt1,color)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                        (x + w, y + h),
                                        (0, 0, 255), 2)


                z=0
                if x>320 :
                    z=x-320+w
                else:
                    z=320-x
                theta = math.degrees(math.atan((z/10)/dd))
                cv2.putText(imageFrame, "L="+str(z/10) , (x+int((z/2)), y+30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))
                cv2.putText(imageFrame, "R d="+str(dd) +" @="+str(int(theta)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))



        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
#GREEN
        for pic, contour in enumerate(contours):

            area = cv2.contourArea(contour)
        # greanarea = area
            if (area > 300):
                greencct=greencct+1
                x, y, w, h = cv2.boundingRect(contour)
                greanarea =  w
                pt1 = (x, y)
                pt2 = (320, 480)
                if x>320 :
                    pt1 = (x+w, y)
                pt3 = (320,y)
            # slope = (480-y) / (320 -x)
                color = (0, 255, 0)
                cv2.line(imageFrame, pt1, pt2,color)
                cv2.line(imageFrame, pt3, pt1,color)
                dd=int( Distance_finder(60.16,5,w/10))
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                        (x + w, y + h),
                                        (0, 255, 0), 2)


                z=0
                if x>320 :
                    z=x-320 +w
                else:
                    z=320-x
                theta = math.degrees(math.atan((z/10)/dd))

                cv2.putText(imageFrame, "L="+str(z/10) , (x+int(z/2), y+30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))
                cv2.putText(imageFrame, "G d="+str(dd) +" @="+str(int(1)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 0))








#PARKING
        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(parking_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        
        for pic, contour in enumerate(contours):

            area = cv2.contourArea(contour)
        # redarea = area
            pt1 = (320, 0)
            pt2 = (320, 480)

            color = (255, 255, 0)
            cv2.line(imageFrame, pt1, pt2,color)
            if (area > 300):
                parkingcct=parkingcct+1
                x, y, w, h = cv2.boundingRect(contour)

               # print (w)
            #    print (h)
                parkingwidth = h
             #   print(parkingwidth)
                dd= int(Distance_finder(60.16,5,w/10))
                pt1 = (x, y)
                pt2 = (320, 480)
                if x>320 :
                    pt1 = (x+w, y)
                pt3 = (320,y)
            # print (pt1)
            #  slope = (480-y) / (320 -x)
                color = (0, 0, 255)
                cv2.line(imageFrame, pt1, pt2,color)
                cv2.line(imageFrame, pt3, pt1,color)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                        (x + w, y + h),
                                        (0, 255, 255), 2)


                z=0
                if x>320 :
                    z=x-320+w
                else:
                    z=320-x
                theta = math.degrees(math.atan((z/10)/dd))
                cv2.putText(imageFrame, "L="+str(z/10) , (x+int((z/2)), y+30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))
                cv2.putText(imageFrame, "R d="+str(dd) +" @="+str(int(theta)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))




        if greencct < mingreen :
            mingreen =greencct
        if redcct < minred :
            minred =redcct



        state = "9"





        cond1=0
        cond2=0
        cond3=0
        cond4=0

        if position == "far" :
            cond1=32  #1
            cond2=25  #2
            cond3=35  #3
            cond4=30  #3
        elif position == "near" :
            cond1=60  #1
            cond2=32  #2
            cond3=60  #3
            cond4=32  #3
        


     #   print ("GREEN "+ str(greanarea)+"   "+" Red"+ str(redarea))
        if mingreen >0 and minred >0 and cond1>  redarea >greanarea :
            state ="ERG"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

        elif mingreen >0 and minred >0 and redarea> greanarea > cond2  :
            state ="RGE"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    ser.write(b"RGET\n")
                    counter =0
                    mingreen=10
                    minred =10 

       
        elif mingreen >0 and minred >0 and redarea > cond3 and  greanarea < cond4   :
            state ="REG"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

           
                

        elif mingreen >0 and minred >0 and cond1> greanarea > redarea :
            state ="EGR"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

        elif mingreen >0 and minred >0 and greanarea > redarea> cond2  :
            state ="GRE"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

        elif mingreen >0 and minred >0 and greanarea > cond3 and  redarea < cond4    :
            state ="GER"

            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"
                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

           
                
                
        elif mingreen >= 1 and minred == 0 :
            state ="GGG"
            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"

                

                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 
        
        
        elif  minred >=1 and mingreen ==0 :
            state ="RRR"
            if counter > limitCounter :
                #ser.write(b"8\n")
            # line = ser.readline().decode('utf-8').rstrip()

                    if position == "far" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpf:
                            toSend = state + "T"
                        elif parkingcct > 0  and parkingwidth <pcpf:
                            toSend = state+ "H"
                        else :
                            toSend= state+ "E"


                    elif position == "near" :
                        toSend = ""
                        if parkingcct > 0  and parkingwidth > pcpn:
                            toSend = state+"T"
                        elif parkingcct > 0  and parkingwidth <pcpn:
                            toSend = state+"H"
                        else :
                            toSend= state+"E"

                

                    print(toSend)
                    print("---------------------------------------------")
                    rr =""+toSend+"\n"
                    ser.write(rr.encode('utf-8'))
                    counter =0
                    mingreen=10
                    minred =10 

        
        else :
            print ("EEEE")


            mingreen=10
            minred =10
            counter =0


        greencct = 0
        redctt=0



    # cv2.startWindowThread()
        # Program Termination
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            run=False
            break
        if GPIO.input(26)==True and GPIO.input(19)==True:  #position="near"
            GPIO.output(18,GPIO.HIGH)
            parking =1
            print("Parking")
        if GPIO.input(26)==False and GPIO.input(19)==False:
            position="near"
         #   print("Near")
        elif GPIO.input(26)==True and GPIO.input(19)==False:
            position="far"
          #  print("Far")
        elif GPIO.input(26)==False and GPIO.input(19)==True:
            position="centre"
            print("Center")

    webcam.release()
    cv2.destroyAllWindows()




#----------------------------------------------------------------------------------------------




    webcam = cv2.VideoCapture(cameraParking2index)
    while (parking == 1):
    # print ("Camera 2")
        #print(ser.readline())
    # line = ser.readline().decode('utf-8').rstrip()
        redcct=0
        greencct=0
        parkingcct=0
        parkingwidth=0
        redy=0
        greeny=0
        width0  = webcam.get(cv2.CAP_PROP_FRAME_WIDTH)

    # print (width0)
        hight0  = webcam.get(cv2.CAP_PROP_FRAME_HEIGHT)

    # print (hight0)
        # Reading the video from the
        # webcam in image frames
        _, imageFrame = webcam.read()
       # imageFrame = cv2.resize(imageFrame, (800, 300))
        imageFrame = imageFrame[200:500,:]
        #imageFrame = imageFrame[150:300,:]
        # Convert the imageFrame in
        # BGR(RGB color space) to
        # HSV(hue-saturation-value)
        # color space
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        #not red its fosia color for parking
        barking_lower = np.array([155, 127 ,50], np.uint8)
        barking_upper = np.array([164, 199, 153], np.uint8)

        barking_mask = cv2.inRange(hsvFrame, barking_lower, barking_upper)
        
        
        





        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        # For barking color
        barking_mask = cv2.dilate(barking_mask, kernal)
        res_barking = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=barking_mask)





        # Creating contour to track barking color
        contours, hierarchy = cv2.findContours(barking_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

        tmppp=0
        tmppb=0
        for pic, contour in enumerate(contours):

            area = cv2.contourArea(contour)
        # barkingarea = area
            pt1 = (320, 0)
            pt2 = (320, 480)

            color = (255, 0, 0)
            cv2.line(imageFrame, pt1, pt2,color)
            if (area > 300):
                redcct=redcct+1
                x, y, w, h = cv2.boundingRect(contour)
                redarea = w * h
            # print (w)
                dd= int(Distance_finder(60.16,5,w/10))
                pt1 = (x, y)
                pt2 = (320, 480)
                if x>320 :
                    pt1 = (x+w, y)
                pt3 = (320,y)
            # print (pt1)
            #  slope = (480-y) / (320 -x)
                color = (0, 0, 255)
                cv2.line(imageFrame, pt1, pt2,color)
                cv2.line(imageFrame, pt3, pt1,color)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                        (x + w, y + h),
                                        (0, 0, 255), 2)


                z=0
                if x>320 :
                    z=x-320+w
                    tmppp = z/10
                else:
                    z=320-x
                    tmppb = z/10

                theta = math.degrees(math.atan((z/10)/dd))
                cv2.putText(imageFrame, "L="+str(z/10) , (x+int((z/2)), y+30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))
                cv2.putText(imageFrame, "R d="+str(dd) +" @="+str(int(theta)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255))
                
                if (tmppp > 0 and tmppb>0  )   :
                   # print ("----" + str(tmppp) + "----" + str(tmppb)) 

                    xcv = (tmppp+tmppb)/2
                    if tmppp>tmppb:
                        xcv = tmppp - xcv
                    else :
                        xcv = tmppb - xcv
                    xcv = xcv*10
                    xcv = int(xcv)

                    if tmppp>tmppb:
                        
                        
                        cameraActual=xcv
                        if xcv >=100 :
                            xcv=99
                        print (xcv)
                        ser.write(str(xcv).encode('utf-8'))
                    # time.sleep(1)
                        pt10 = (320+xcv, 0)
                    
                        pt20 = (320+xcv, 480)
                    else :
                        
                        cameraActual=xcv
                        if xcv >=100 :
                            xcv=99
                        print (-1 * xcv)
                        ser.write(str(-1 * xcv).encode('utf-8'))
                        #time.sleep(1)
                    # ser.write(-1 * xcv)
                        pt10 = (320-xcv, 0)
                    
                        pt20 = (320-xcv, 480)


                    #print (xcv)
                    
                    pt10 = (320-xcv, 0)
                
                    pt20 = (320-xcv, 480)

                    color00 = (255, 210, 0)
                    cv2.line(imageFrame, pt10, pt20,color00)
                    tmppp = 0
                    tmppb = 0
                elif (tmppp > 0 and tmppb>=0  and cameraActual>99):#      positive
                    print ("98")
                    ser.write(str(99).encode('utf-8'))

                elif (tmppp >= 0 and tmppb>0  and cameraActual>60 ) :#     negative
                    print ("-99")
                    ser.write(str(-99).encode('utf-8'))
                else:
                    xa=0

                








        counter = counter +1
        minred =10

        if greencct < mingreen :
            mingreen =greencct
        if redcct < minred :
            minred =redcct



        state = "9"










        if mingreen >0 and minred >0 and greanarea > redarea :
            state ="8"

        else :
            mingreen=10
            minred =10
            counter =0


        greencct = 0
        redctt=0

        if GPIO.input(26)==False and GPIO.input(19)==False:  #position="near"
            GPIO.output(18,GPIO.HIGH)
            parking =0
            print("Parking")

    # cv2.startWindowThread()
        # Program Termination
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            run=False
            break

    
    webcam.release()
    cv2.destroyAllWindows()
    webcam = cv2.VideoCapture(camera1index)
import RPi.GPIO as GPIO
from time import sleep
import math


GPIO.setwarnings(False)

#pin_A = 18     # Channel A   
#pin_B = 17     # Channel B
#pin_X = 22     # Channel X 


pin_A = 12
pin_B = 6


Encoder_Count = 0      # Encoder Count variable
nb_tour = 0

GPIO.setmode (GPIO.BCM)
GPIO.setup (pin_A, GPIO.IN, )    # use ext pullup
GPIO.setup (pin_B, GPIO.IN, )    # use ext pullup
#GPIO.setup (pin_X, GPIO.IN, )    # use ext pullup
   
def do_Encoder(channel):
   global Encoder_Count
   if GPIO.input(pin_B) == 1:
      Encoder_Count += 1
   else:
      Encoder_Count -= 1

def do_Index(channel):
   global Encoder_Count
   Encoder_Count = 0
        

GPIO.add_event_detect (pin_A, GPIO.FALLING, callback=do_Encoder)   # Interrupt
#GPIO.add_event_detect (pin_X, GPIO.FALLING, callback=do_Index)   # Index interrupt



try:

    while(True):
        wdeg = Encoder_Count * 0.175781
        if Encoder_Count < 1:
            Encoder_Count = 2048
            nb_tour -= 1


        if Encoder_Count > 2048:
            Encoder_Count = 1
            nb_tour += 1


	#	if wdeg%(2*360) == 0 : 
	#	     nb_tour += 1





        print ('WindDir = {0:0.0f}'.format(wdeg))
	#	print(nb_tour)
        sleep(0.01)


finally : 
    GPIO.cleanup()
   

   



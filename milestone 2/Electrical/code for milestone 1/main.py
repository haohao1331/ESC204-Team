from electrical import *
from keyboard import *
from time import sleep


leftDIR = 3
leftSTEP =  2
rightDIR = 21
rightSTEP = 20
topDIR = 5
topSTEP = 6


delay = 0.03

step = 200
ministep = 100

#wheelForward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,step)

while(True):
   char = getch()
   if(char == "W"):
      wheelForward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,step)
   if(char == "S"):
      wheelBackward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,step)
   if(char == "A"):
      turn(rightDIR,rightSTEP,delay,step)
   if(char == "D"):
      turn(leftDIR,leftSTEP,delay,step)
   if(char == "w"):
      wheelForward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,ministep)
   if(char == "s"):
      wheelBackward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,ministep)
   if(char == "d"):
      turn(rightDIR,rightSTEP,delay,ministep)
   if(char == "e"):
      turn(leftDIR,leftSTEP,delay,ministep)

   if(char == "Q"):
      back(rightDIR,rightSTEP,delay,step)
   if(char == "E"):
      back(leftDIR,leftSTEP,delay,step)
   if(char == "q"):
      back(rightDIR,rightSTEP,delay,ministep)
   if(char == "a"):
      back(leftDIR,leftSTEP,delay,ministep)
   if(char == "k"):
      turn(topDIR, topSTEP,0.015,ministep)
   if(char == "l"):
      back(topDIR,topSTEP,0.015,ministep)
   sleep(0.1)

   if(char == " "):
      break
   print("done loop")
print("done")
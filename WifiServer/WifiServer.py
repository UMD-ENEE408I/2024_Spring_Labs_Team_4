#This is the new updated version of the wifi code, it makes the nvidia the server and the robots the connectors,
#this is nessassary so two robots can run the course at the same time, and it should also be faster since the nvidia is doing the comutation

#Reference: https://dev.to/idris_jimoh/create-a-network-server-with-python-2lo2


import socket               
# Import socket module

#include the newSoundLocalization
import newSoundLocalization as sound
import camera2 as camera



s = socket.socket()         
# Create a socket object
host = socket.gethostname() 
print(s)
# Get local machine name

port = 8090         
# Reserve a port for your service.
print(socket.gethostbyname(socket.gethostname()))
print("binding", host)
s.bind(('', port))        
# Bind to the port

s.listen(2)                 
# Now wait for client connection.
print("Here")
while True:
   c, addr = s.accept()     
# Establish connection with client.
   print('Got connection from', addr)

   #recieve 100 chars message
   messageSent = c.recv(100)

   
   print("This is the message from the robot:", messageSent)

   #if the robots message is SOUND(number) then go through the phases of the sound reading
   #record left
   if "SOUND1" in str(messageSent):
      print("phase1")
      sound.record_file("left_file.wav")
   #record right
   elif "SOUND2" in str(messageSent):
      print("phase2")
      sound.record_file("right_file.wav")
   #determine which way to go
   elif "SOUND3" in str(messageSent):
      print("phase3")
      dir = sound.compare_sounds("left_file.wav", "right_file.wav")
      print(dir)
      while("ack" not in str(messageSent)):
         c.send(bytes(dir, 'utf-8'))
         messageSent = c.recv(10)
         print(messageSent)
      #looks for closing message
      #messageSent = c.recv(100)

   #This is the code for the maze
   elif "MAZEBLOCK" in str(messageSent):
      #check if there is a found block

      #UNCOMMENT THIS TO USE THE FINDBLOCK CODE IN CAMERA2
      #val = camera.findBlock()
      #RECOMMENT THIS TO USE THE FINDBLOCK CODE IN CAMERA2
      val = 'B'
      #send the block over wifi
      while("ack" not in str(messageSent)):
         c.send(bytes(val, 'utf-8'))
         messageSent = c.recv(10)
         print(messageSent)


   
   c.close()                
# Output the message and Close the connection
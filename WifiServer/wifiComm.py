#import module
import os
from ipaddress import IPv4Address
import requests 
import time
  

#Switch the computers wifi to the specified network for the robot
#https://www.geeksforgeeks.org/how-to-connect-wifi-using-python/
#scan available Wifi networks
#os.system('cmd /c "netsh wlan show networks"')
#input Wifi name
#name_of_router = 'PatConnect'
#connect to the given wifi network
#os.system(f'''cmd /c "netsh wlan connect name={name_of_router}"''')

#Make sure connected to right network before continuing


#Run code based on what the websever says to do.
#while (True):
    # Making a GET request 
#r = requests.get('http://192.168.4.1/RIGHT') 
    # check status code for response received 
    # success code - 200 
#print(r) 
    # print content of request 
#print(r.content)
r = requests.get('http://192.168.4.1/')
count = 0
while(True):
    count = count + 1
    #based on the content go left or right:
    if "RIGHT" in str(r.content):
        try:
            r = requests.get('http://192.168.4.1/LEFT')
            print(r.content)
        except requests.exceptions.Timeout as e:
            print(f'Timeout occurred: {str(e)}')
        time.sleep(2)
    elif "LEFT" in str(r.content):
        try:
            r = requests.get('http://192.168.4.1/RIGHT')
            print(r.content)
        except requests.exceptions.Timeout as e:
            print(f'Timeout occurred: {str(e)}')
        time.sleep(2)
    else:
        print("Nothing")
        try:
            r = requests.get('http://192.168.4.1/RIGHT') 
        except requests.exceptions.Timeout as e:
            print(f'Timeout occurred: {str(e)}')
        #print(r.content)
        time.sleep(2)
    print(count)
          


    
    
import dartv2b
import time

if __name__ == "__main__":
    mybot = dartv2b.DartV2()

    # place your work here
    print ("Sonar Microcode Version : ",mybot.sonars.get_version())
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    for itst in range(10): 
        time.sleep(0.5)
        print ("Sonars front diag",mybot.sonars.read_diag_both())
    
    mybot.end() # clean end of the robot mission


# The following script communicates the values of inputs into the Arduino shield of the 2013 Bulldogs Racing car.
# By Alex Carrillo <alejandro.carrillo@yale.edu>

import serial
import curses
from datetime import datetime


#ser=serial.Serial('/dev/tty.usbmodem411')
ser=serial.Serial('/dev/tty.usbmodem621')

stdscr=curses.initscr()
stdscr.keypad(1)
stdscr.nodelay(1)
stdscr.leaveok(1)
curses.noecho()
curses.cbreak()

stdscr=curses.initscr()
stdscr.keypad(1)
stdscr.nodelay(1)
stdscr.leaveok(1)

pad = curses.newpad(100, 100)

rpm = 0
temp = 0
battlow = 0
fuelin = 0
throttlein = 0
bmsfault = 0
clutch = 0
assist = 0
brake = 0
engineEnable = 0



while 1:
    if stdscr.getch()==ord('q'):
        break
    line=ser.readline()
    items=line.split()
    if items[0] == "RPM":
        rpm = items[1]
    elif items[0] == "Temp":
        temp = items[1]
    elif items[0] == "BattLow":
        battlow = items[1]
    elif items[0] == "FuelIn":
        fuelin = items[1]
    elif items[0] == "ThrottleIn":
        throttlein = items[1]
    elif items[0] == "BMS_Fault":
        bmsfault = items[1]
    elif items[0] == "Clutch":
        clutch = items[1]
    elif items[0] == "AssistIn":
        assist = items[1]
    elif items[0] == "BrakeIn":
        brake = items[1]
    elif items[0] == "EngineEnable":
        engineEnable = items[1]
    pad.addstr(0,0, "RPM:");pad.addstr(0,20,str(rpm))
    pad.addstr(1,0, "Temp:"); pad.addstr(1,20,str(temp))
    pad.addstr(2,0, "Low Battery:");pad.addstr(2,20,str(battlow))
    pad.addstr(3,0, "FuelIn:"); pad.addstr(3,20,str(fuelin))
    pad.addstr(4,0, "Throttle:"); pad.addstr(4,20,str(throttlein))
    pad.addstr(5,0, "BMS Fault:"); pad.addstr(5,20,str(bmsfault))
    pad.addstr(6,0, "Clutch:"); pad.addstr(6,20,str(clutch))
    pad.addstr(7,0, "Assist In:"); pad.addstr(7,20,str(assist))
    pad.addstr(8,0, "Brake In:"); pad.addstr(8,20,str(brake))
    pad.addstr(9,0, "Engine Enable:"); pad.addstr(9,20,str(engineEnable))
    stdscr.refresh()
    pad.refresh( 0,0, 5,5, 20,75)
    #print "RPM\t",rpm,"\tTemp\t",temp,"\tLow Battery\t",battlow,"\tFuelIn\t",fuelin,"\tThrottle\t",throttlein,"\tBMS Fault\t",bmsfault,"\tClutch\t",clutch,"\tAssist In\t",assist,"\tBrake In\t",brake

# Quit!
curses.nocbreak(); stdscr.keypad(0); curses.echo()
curses.endwin()

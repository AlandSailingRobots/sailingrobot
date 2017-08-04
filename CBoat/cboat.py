
#!/usr/bin/python
import socket
import curses
import traceback
import math

TCP_IP = '127.0.0.1'
TCP_PORT = 9600
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

# Boat state
boat_heading = 0
boat_pitch = 0
boat_roll = 0

wind_dir = 0
wind_speed = 0

gps_heading = 0
gps_speed = 0
gps_lat = .0
gps_lon = .0

wp_id = 0
wp_lat = .0
wp_lon = .0
wp_radius = 0
wp_distance = .0
wp_bearing = 0

compassRadius = 10

def createStateDisplay( screen ):
    display = screen.subwin( 16, 31, 0, 0 )
    display.box()
    return screen

def createAppStateDisplay( screen ):
    appDisplay = screen.subwin( 7, 31, 25, 0 )
    appDisplay.box()
    return appDisplay

def createBoatDisplay( screen ):
    boatDisplay = screen.subwin( 32, 62, 0, 32 )
    boatDisplay.box()
    return boatDisplay

def createWaypointDisplay( screen ):
    waypointDisplay = screen.subwin( 7, 31, 17, 0)
    waypointDisplay.box()
    return waypointDisplay

def drawStateDisplay( stateDisplay ):
    stateDisplay.addstr( 0, 4, "Boat State")
    stateDisplay.addstr( 1, 1, "Compass Data" )
    stateDisplay.addstr( 2, 2, "Heading:" ); stateDisplay.addstr( 2, 20, str(boat_heading) )
    stateDisplay.addstr( 3, 2, "Pitch:" ); stateDisplay.addstr( 3, 20, str(boat_pitch) )
    stateDisplay.addstr( 4, 2, "Roll:" ); stateDisplay.addstr( 4, 20, str(boat_roll) )

    stateDisplay.addstr( 6, 1, "Wind Data" )
    stateDisplay.addstr( 7, 2, "Direction:" ); stateDisplay.addstr( 7, 20, str(wind_dir) )
    stateDisplay.addstr( 8, 2, "Speed:" ); stateDisplay.addstr( 8, 20, str(wind_speed) )

    stateDisplay.addstr( 10, 1, "GPS Data" )
    stateDisplay.addstr( 11, 2, "Heading:" ); stateDisplay.addstr( 11, 20, str(gps_heading) )
    stateDisplay.addstr( 12, 2, "Speed:" ); stateDisplay.addstr( 12, 20, str(gps_speed) )
    stateDisplay.addstr( 13, 2, "Latitude:" ); stateDisplay.addstr( 13, 20, str(gps_lat) )
    stateDisplay.addstr( 14, 2, "Longitude:" ); stateDisplay.addstr( 14, 20, str(gps_lon) )

def drawAppStateDisplay( appDisplay ):
    appDisplay.addstr( 0, 4, "Application State")
    appDisplay.addstr( 1, 1, "Connected:" ); appDisplay.addstr( 1, 20, "TODO" )
    appDisplay.addstr( 2, 1, "Last updated:" ); appDisplay.addstr( 2, 20, "TODO" )

def drawBoatDisplay( boatDisplay ):
    boatDisplay.addstr( 10, 10, "TODO boat display")

def drawWaypointDisplay( waypointDisplay ):
    waypointDisplay.addstr( 0, 4, "Waypoint Info")
    waypointDisplay.addstr( 1, 1, "Waypoint ID:"); waypointDisplay.addstr( 1, 20, str(wp_id) )
    waypointDisplay.addstr( 2, 1, "Latitude:"); waypointDisplay.addstr( 2, 20, str(wp_lat) )
    waypointDisplay.addstr( 3, 1, "Longitude:"); waypointDisplay.addstr( 3, 20, str(wp_lon) )
    waypointDisplay.addstr( 4, 1, "Distance:"); waypointDisplay.addstr( 4, 20, str(wp_distance) )
    waypointDisplay.addstr( 5, 1, "Bearing:"); waypointDisplay.addstr( 5, 20, str(wp_bearing) )

def drawCompass( compassDisplay ):
    radius = 15
    for part in range(0, 360):
        angle = math.radians(part)
        x = math.cos(angle) * radius + radius + 5
        y = math.sin(angle) * radius + radius + 5
        compassDisplay.addch(int(y*.6), int(x), '*')

    for line in range(0, radius):
        compassDisplay.addch( 20 + line, radius + 5, "|"  )
        compassDisplay.addch( 20 - line, radius + 5, "|"  )

    #for i in range(1,360):
     #   x = int(compassRadius + ( compassRadius * math.cos( math.radians(i) ) )) + 31 + 10
      #  y = int(compassRadius + ( compassRadius * math.sin( math.radians(i) ) )) + 10
       # compassDisplay.addstr( y, x, "*")

def drawUI( scr, stateDisplay, boatDisplay, appDisplay, waypointDisplay ):
    drawStateDisplay( stateDisplay )
    drawBoatDisplay( boatDisplay )
    drawAppStateDisplay( appDisplay )
    drawWaypointDisplay( waypointDisplay )

    stateDisplay.refresh()
    boatDisplay.refresh()
    appDisplay.refresh()
    waypointDisplay.refresh()
    scr.refresh()

def parseData( data ):
    global boat_heading, boat_pitch, boat_roll, wind_dir, wind_speed, gps_heading, gps_speed, gps_lat, gps_lon, wp_id, wp_bearing, wp_distance, wp_lat, wp_lon, wp_radius
    array = data.split( "," )
    boat_heading = int(array[0])
    boat_pitch = int(array[1])
    boat_roll = int(array[2])
    wind_dir = int(array[3])
    wind_speed = int(array[4])
    gps_heading = int(array[5])
    gps_speed = float(array[6])
    gps_lat = float(array[7])
    gps_lon = float(array[8])
    wp_id = int(array[9])
    wp_lat = float(array[10])
    wp_lon = float(array[11])
    wp_radius = int(array[12])
    wp_distance = float(array[13])
    wp_bearing = float(array[14])

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))

    stdscr = curses.initscr()
    try:
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(1)

        begin_x = 20; begin_y = 7
        height = 5; width = 40
        boatStateWin = createStateDisplay( stdscr )
        appDisplay = createAppStateDisplay( stdscr )
        boatDisplay = createBoatDisplay( stdscr )
        waypointDisplay = createWaypointDisplay( stdscr )

        drawUI( stdscr, boatStateWin, boatDisplay, appDisplay, waypointDisplay )
        while True:
            parseData( s.recv(BUFFER_SIZE) )
            drawUI( stdscr, boatStateWin, boatDisplay, appDisplay, waypointDisplay )

        s.close()
        stdscr.keypad(0)
        curses.echo()
        curses.nocbreak()
        curses.endwin() 
    except:
        stdscr.keypad(0)
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        traceback.print_exc() 

# executes main
if __name__ == "__main__":main()
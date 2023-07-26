# Test Stand GUI and Controls
# Date: 05/29/2023
# By: Eric Weissman
# Description: This code generates a GUI for the test stand controls and the control logic for all the motors.
# offers both manual control and automatic controls for the test stand.

# TODO:
#  allow for g-code entries for automatic controls


# import used packages
import serial
import time
import tkinter
import RPi.GPIO as GPIO
import threading
from threading import Lock, Thread
import time
from time import sleep
import numpy as np



RPtoggle = False

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# Define motor pins
# Rotating Platform
RPPUL = 29 #Pulse
RPDIR = 31 #Direction
RPENA = 33 #Enable 

# Left Actuator
LPUL = 15#40
LDIR = 13#38
LENA = 11#36

# Right Actuator
RPUL = 40#15
RDIR = 38#13
RENA = 36 #11

# Limit Switches
LSR = 35
LSL = 37

# Wirefeeder
wf_A = 12
wf_B = 32

# Motor Encoder
MEPinA = 22
MEPinB = 16

# Stop variable
global stopALL
stopALL = False

# Setting up motor pins
motorPins = [RPPUL, RPDIR, RPENA, LPUL, LDIR, LENA, RPUL, RDIR, RENA]
for pin in motorPins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
GPIO.output(RPENA, GPIO.HIGH)
GPIO.output(LENA, GPIO.HIGH)
GPIO.output(RENA, GPIO.HIGH)
GPIO.setup(wf_A, GPIO.OUT)
GPIO.setup(wf_B, GPIO.OUT)
GPIO.setup(LSR, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(LSL, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

# Setting PWM pins for the wire feeder
pwmA = GPIO.PWM(wf_A, 1000)
pwmB = GPIO.PWM(wf_B, 1000)
pwmB.start(0)
pwmA.start(0)

# Setting rotary encoder interrupt pins as pull up
GPIO.setup(MEPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MEPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables for wire feeder speed feedback
global counter
global L
global PeriWF
counter = 0 # Counts the number of steps read by the rotary encoder
PeriWF = 3.1415926535 * 1.18  # Circumference in inches
L = PeriWF / 600           # Distance per pulse

# Wire feeder variables to determine direction
global forward
global backward
global stop
forward = False
backward = False
stop = False

global stopWF
stopWF = False

# Variables for actuators
global CurrentHeight
CurrentHeight = 0

# Variable for printing function
global PrintToggle
PrintToggle = False
global RPcounter
global RProtations
RPcounter = 0 # Counts the number of steps taken
RProtations = 0 # Counts the number of rotations done
global numRotations

# Encoder counter function.

def update_counter(channel):
    '''This function determines which direction the encoder is spinning by 
    checking what value pin B is when pin A is rising. It updates the global 
    counter variable accordingly.'''
    global counter
    state_A = GPIO.input(MEPinA)
    state_B = GPIO.input(MEPinB)

    if state_B == 1:
        counter = counter + 1
    else:
        counter = counter - 1
        
# Setting interrupt functions for the encoder. 
'''This detects when pin A is
 rising and calls the update_counter function to update the counter whenever
 pin A rises.'''
GPIO.add_event_detect(MEPinA, GPIO.RISING, callback=update_counter)


# Define motor functions

# Rotating platform functions

def RPCW(TargetAngleSpeed):
    '''Clockwise rotation: This function takes in a target angular speed and 
    modifies the stepper motor delay to match the target speed. The delay is 
    calculated based on the encoder steps/rev setting and the pitch of the 
    slew drive. While the platform toggle is true and the stop all flag is false, 
    it will continuously step the motor. For every full rotation, a global 
    variable is updated to keep track of rotations for the automatic print function.
    This function returns when the platform toggle is false.'''
    global RPcounter
    global RProtations
    global RPtoggle
    print("CW")
    GPIO.output(RPENA, GPIO.LOW) # Turn motor on
    GPIO.output(RPDIR, GPIO.HIGH) # Set direction to clockwise
    delay = 16*28.125 / (TargetAngleSpeed*130) # Calculate the delay based on teh target angular speed of the platform
    delay = delay / 1000
    while RPtoggle and not stopALL:
        GPIO.output(RPPUL, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(RPPUL, GPIO.LOW)
        time.sleep(delay)
        RPcounter = RPcounter+1 # Counts the number of steps
        #print(RPcounter)
        if RPcounter == 400*130: # Counts the number of rotations
            RProtations = RProtations + 1 # Update number of rotations
            #print(RProtations)
            RPcounter = 0 # Reset step counter
    if RPtoggle == False: # Exits the thread and turns off the motor
        print("Thread terminated")
        GPIO.output(RPENA, GPIO.HIGH)
        return
    return
        
def RPCCW(TargetAngleSpeed):
    '''Counter-clockwise rotation: This function takes in a target angular speed 
    and modifies the stepper motor delay to match the target speed. The delay 
    is calculated based on the encoder steps/rev setting and the pitch of the 
    slew drive. While the platform toggle is true and the stop all flag is false, 
    it will continuously step the motor. For every full rotation, a global 
    variable is updated to keep track of rotations for the automatic print function.
    This function returns when the platform toggle is false.'''
    global RPtoggle
    global RPcounter
    global RProtations
    print("CCW")
    GPIO.output(RPENA, GPIO.LOW) # Turn motor on
    GPIO.output(RPDIR, GPIO.LOW) # Set direction to clockwise
    delay = 16*28.125 / (TargetAngleSpeed*130) # Calculate the delay based on the target angular speed of the platform
    delay = delay / 1000
    while RPtoggle and not stopALL:
        GPIO.output(RPPUL, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(RPPUL, GPIO.LOW)
        time.sleep(delay)
        RPcounter = RPcounter+1 # Counts the number of steps
        #print(RPcounter)
        if RPcounter == 400*130: # Counts the number of rotations
            #print(RProtations)
            RProtations = RProtations + 1 # Update number of rotations
            RPcounter = 0 # Reset step counter
    if RPtoggle == False: # Exits the thread and turns off the motor
        print("Thread terminated")
        GPIO.output(RPENA, GPIO.HIGH)
        return
    return

def RPANGLE():
    '''This function reads global variables for target angle, target angular
    speed, and spin direction. The target steps are calculated based on the
    slew drive pitch and the steps/rev on the encoder. The delay is calculated
    based on the target speed. The function checks which direction to spin
    then rotates the motor the desired number of steps at the desired speed
    and returns when done.'''
    global TargetAngle
    global TargetAngleSpeed
    global RotateDirectionState
    GPIO.output(RPENA, GPIO.LOW)
    TargetSteps = 130*TargetAngle*17.777 # Calculates the number of steps to take for a desired angle of rotation
    TargetSteps = int(TargetSteps) # Converts the number of steps to an integer
    delay = 8*28.125 / (TargetAngleSpeed*130) # Calculates the delay based on teh target angular speed
    delay = delay / 1000
    if RotateDirectionState == 0: # Checks which direction to turn
        GPIO.output(RPDIR, GPIO.LOW)
    else:
        GPIO.output(RPDIR, GPIO.HIGH)
    for i in range(TargetSteps-1): # Loops for the number of steps calculated
        GPIO.output(RPPUL, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(RPPUL, GPIO.LOW)
        time.sleep(delay)
        if stopALL:
            return
    return

# Linear actuator functions
def ZeroRActuator():
    '''This function modifies the global variable keeping track of the current
    height and resets it to 0. It also updates the height label for the GUI
    whenever the current height variable changes. It returns after setting
    the GUI label.'''
    global CurrentHeight
    # Turn motor on and set direction to down
    GPIO.output(RENA, GPIO.LOW)
    GPIO.output(RDIR, GPIO.LOW)
    steps = 6400
    stepsR = 0
    stepsL = 0
    # Run motors down until the limit switch is hit (Commented out for now because of faulty limit switch)
    '''while not GPIO.input(LSR) and not stopALL:
        GPIO.output(RPUL, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(RPUL, GPIO.LOW)
        time.sleep(0.00001)
        stepsR = stepsR + 1'''
    CurrentHeight = 0
    print("Right Actuator Zeroed")
    ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update the current height label
    GPIO.output(RENA, GPIO.HIGH) 
    return



def ZeroLActuator():
    '''This function modifies the global variable keeping track of the current
    height and resets it to 0. It returns after height is reset.'''
    global CurrentHeight
    # Turn motor on and set direction to down
    GPIO.output(LENA, GPIO.LOW)
    GPIO.output(LDIR, GPIO.LOW)
    steps = 6400
    stepsR = 0
    # Run motors down until the limit switch is hit (Commented out for now because of faulty limit switch)
    '''while not GPIO.input(LSL) and not stopALL:
        GPIO.output(LPUL, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(LPUL, GPIO.LOW)
        time.sleep(0.00001)
        stepsL = stepsL + 1'''
    CurrentHeight = 0
    print(GPIO.input(LSL))
    print("Left Actuator Zeroed")
    GPIO.output(LENA, GPIO.HIGH)
    return

def Up():
    '''This function runs a for loop for the length corresponding to a 0.5mm
    height increase in the linear actuators. It updates the global height
    variable and updates the GUI label with updated height. The function returns
    after the for loop ends.'''
    global CurrentHeight
    # Turn motors on and set direction to up
    GPIO.output(LENA, GPIO.LOW)
    GPIO.output(LDIR, GPIO.HIGH)
    GPIO.output(RENA, GPIO.LOW)
    GPIO.output(RDIR, GPIO.HIGH)
    steps = 200
    # Go up by 0.5 mm
    for i in range(int(steps/8-1)):
        GPIO.output(LPUL, GPIO.HIGH)
        GPIO.output(RPUL, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(LPUL, GPIO.LOW)
        GPIO.output(RPUL, GPIO.LOW)
        time.sleep(0.001)
        if stopALL:
            return
    CurrentHeight = CurrentHeight + 0.5 # Update current height
    ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update the current height 
    # Turn motors off
    '''GPIO.output(LENA, GPIO.HIGH)
    GPIO.output(RENA, GPIO.HIGH)'''
    return

def Down():
    '''This function runs a for loop for the length corresponding to a 0.5mm
    height decrease in the linear actuators. It updates the global height
    variable and updates the GUI label with updated height. The function returns
    after the for loop ends.'''
    global CurrentHeight
    # Turn motors on and set direction to down
    GPIO.output(LENA, GPIO.LOW)
    GPIO.output(LDIR, GPIO.LOW)
    GPIO.output(RENA, GPIO.LOW)
    GPIO.output(RDIR, GPIO.LOW)
    steps = 200
    # Go down by 0.5 mm
    for i in range(int(steps/8-1)):
        GPIO.output(LPUL, GPIO.HIGH)
        GPIO.output(RPUL, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(LPUL, GPIO.LOW)
        GPIO.output(RPUL, GPIO.LOW)
        time.sleep(0.001)
        if stopALL:
            return
    CurrentHeight = CurrentHeight - 0.5 # Update current height
    ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update the current height 
    # Turn motors off
    '''GPIO.output(LENA, GPIO.HIGH)
    GPIO.output(RENA, GPIO.HIGH)'''
    return

def Target():
    '''This function calculates an error by looking at the global variables
    for current height and target height. It converts the error in mm to
    a number of steps for the stepper motors to take. It checks the sign
    of the error to determine which direction the motors need to turn then
    runs the motors in a for loop for the calculated number of steps. The
    function returns when target height has been reached.'''
    global CurrentHeight
    global TargetHeight
    GPIO.output(LENA, GPIO.LOW)
    GPIO.output(RENA, GPIO.LOW)
    Error = TargetHeight - CurrentHeight # Calculate the error between the current height and the target height
    # Convert the error from mm to steps and integer
    steps = Error*50
    steps = int(steps)
    # If error is positive, go up, else go down
    if Error > 0:
        GPIO.output(LDIR, GPIO.HIGH)
        GPIO.output(RDIR, GPIO.HIGH)
        for i in range(steps-1):
            GPIO.output(LPUL, GPIO.HIGH)
            GPIO.output(RPUL, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(LPUL, GPIO.LOW)
            GPIO.output(RPUL, GPIO.LOW)
            time.sleep(0.001)
            if stopALL:
                return
    elif Error < 0 :
        GPIO.output(LDIR, GPIO.LOW)
        GPIO.output(RDIR, GPIO.LOW)
        for i in range(abs(steps)-1):
            GPIO.output(LPUL, GPIO.HIGH)
            GPIO.output(RPUL, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(LPUL, GPIO.LOW)
            GPIO.output(RPUL, GPIO.LOW)
            time.sleep(0.001)
            if stopALL:
                return
    CurrentHeight = TargetHeight # Update current height
    ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update current height label
    '''GPIO.output(LENA, GPIO.HIGH)
    GPIO.output(RENA, GPIO.HIGH)'''
    return

# Wire feeder functions
def WFforward():
    '''This function spins the wire feeder forward based on the global variable
    for target speed in in/s. It uses a low-pass filter to clean up the signal
    then calculates an error between the target speed and the current filtered
    speed. The output speed is saturated at the PWM limits (percentage from 0-100).
    Using this value, the motor then spins at that calculated PWM percent. The 
    function also checks for stop all and stop wire feeder flags. It returns when
    any stop flag becomes true.'''
    global counter
    global L
    global forward
    global backward
    global stop
    global TargetSpeed
    # Set all the storing variables for previous outputs, errors, speeds, filtered speeds, and time
    oldtime = 0
    oldcount = 0
    prev2Error = 0
    prevError = 0
    prev3Output = 0
    prev2Output = 0
    prevOutput = 0
    currenttime = 0
    starttime = time.perf_counter()
    previousSpeed = 0
    previousSpeedf = 0
    Error = 0
    Output = 0
    speedWF = 0
    speedWFf = 0
    
    # Loop the encoder filter and controller
    while forward and not stopALL:
        time.sleep(0.016) # Delay for stability
        
        # Rotary encoder filter
        currenttime = time.perf_counter() # Get current time
        dt = currenttime - oldtime # Find change in time to find the time step
        dcount = abs(counter - oldcount) # Find change in encoder steps from last time step
        dL = dcount * L # Distance traveled since last step
        speedWF = dL/dt # Calculate the current speed
        speedWFf = 0.9274 * previousSpeedf + 0.03681 * speedWF + 0.0359 * previousSpeed # Low-pass filter for the speed measurement to eliminate noise
        print("True Speed: "+ str(speedWFf))
        
        # Wire feeder controller
        Error = TargetSpeed - speedWFf # Current error
        Output = 71.48 * Error - 120.5 * prevError + 50.32 * prev2Error  + 1.591* prevOutput - 0.6641 * prev2Output + 0.07266 * prev3Output # Controller function
        Output = max(min(Output, 100), 0) # Saturation
        
        # Send the controller output to the wire feeder
        pwmA.start(Output)
        pwmB.start(0)
        
        # Store the current variables for the next time step
        oldtime = currenttime
        oldcount = counter
        prev2Error = prevError
        prevError = Error
        prev3Output = prev2Output
        prev2Output = prevOutput
        prevOutput = Output
        previousSpeed = speedWF
        previousSpeedf = speedWFf
        
        # Check for the stop variables
        if stopALL:
            pwmB.start(0)
            pwmA.start(0)
            return
        if stopWF:
            pwmB.start(0)
            pwmA.start(0)
            return
    return

def WFbackward():
    '''This function spins the wire feeder backwards based on the global variable
    for target speed in in/s. It uses a low-pass filter to clean up the signal
    then calculates an error between the target speed and the current filtered
    speed. The output speed is saturated at the PWM limits (percentage from 0-100).
    Using this value, the motor then spins at that calculated PWM percent. The 
    function also checks for stop all and stop wire feeder flags. It returns when
    any stop flag becomes true.'''
    global counter
    global L
    global forward
    global backward
    global stop
    global TargetSpeed
    # Set all the storing variables for previous outputs, errors, speeds, filtered speeds, and time
    oldtime = 0
    oldcount = 0
    prev2Error = 0
    prevError = 0
    prev3Output = 0
    prev2Output = 0
    prevOutput = 0
    currenttime = 0
    starttime = time.perf_counter()
    previousSpeed = 0
    previousSpeedf = 0
    Error = 0
    Output = 0
    speedWF = 0
    speedWFf = 0
    
    # Loop the encoder filter and controller
    while backward:
        time.sleep(0.016) # Delay for stability
        
        # Rotary encoder speed measurement filter
        currenttime = time.perf_counter() # Get current time
        dt = currenttime - oldtime # Find change in time to find the time step
        dcount = abs(counter - oldcount) # Find change in encoder steps from last time step
        dL = dcount * L # Distance traveled since last step
        speedWF = dL/dt # Calculate the current speed
        speedWFf = 0.9274 * previousSpeedf + 0.03681 * speedWF + 0.0359 * previousSpeed # Low-pass filter for the speed measurement to eliminate noise
        
        # Wire feeder controller
        Error = TargetSpeed - speedWFf # Current error
        Output = 71.48 * Error - 120.5 * prevError + 50.32 * prev2Error  + 1.591* prevOutput - 0.6641 * prev2Output + 0.07266 * prev3Output # Controller
        Output = max(min(Output, 100), 0) # Saturation

        # Send the controller output to the wire feeder
        pwmB.start(Output)
        pwmA.start(0)
        
        # Store the current variables for the next time step
        oldtime = currenttime
        oldcount = counter
        prev2Error = prevError
        prevError = Error
        prev3Output = prev2Output
        prev2Output = prevOutput
        prevOutput = Output
        previousSpeed = speedWF
        previousSpeedf = speedWFf
        
        # Check for the stop variables
        if stopALL:
            pwmB.start(0)
            pwmA.start(0)
            return
        if stopWF:
            pwmB.start(0)
            pwmA.start(0)
            return
    return
    
def WFstop():
    '''This function turns off the wire feeder when stop flag is true and returns.'''
    while stop:
        pwmB.start(0)
        pwmA.start(0)
    return

def PRINT():
    '''This function lowers the height of the platform by the global variable
    DeltaH every time the platform makes a complete rotation. It also starts
    the wire feeder forward according to the target angular speed variable.
    The speed ramps up from 0.5in/s to the target speed based on a hyperbolic 
    tangent function to allow the beginning wire to have more time to heat up.
    The function uses global variables for DeltaH and number of rotations to
    determine how long each for loop should run and to determine when to trigger functions.
    A dummy variable stores the current rotation count and increases only after
    the platform decreases in height while the global rotation variable increases
    before. This way we can tell the platform to decrease only when these two variables
    don't match. A second dummy variable is used to store the rotations and is
    used to check if it's the last rotation. For the last rotation, the platform
    decreases height by 4 times DeltaH and stops the wire feeder. The function
    returns after the routine is finished.'''
    global RProtations
    global DeltaH
    global TargetAngleSpeed
    global PrintToggle
    global RPcounter
    global CurrentHeight
    global TargetSpeed
    # Turn motors on and set direction to down
    GPIO.output(LENA, GPIO.LOW)
    GPIO.output(RENA, GPIO.LOW)
    GPIO.output(LDIR, GPIO.LOW)
    GPIO.output(RDIR, GPIO.LOW)
    stepsDown = DeltaH*50 # Calculate the amount of steps to take at each rotation based on the desired delta h
    stepsDown =  int(stepsDown) # Integer
    
    # Set wire feeder direction to forward
    global forward
    global backward
    global stopWF
    
    forward = True
    backward = False
    
    # Reset the platform step counter
    RPcounter = 0
    
    # Store the current number of platform rotations
    dummy1 = RProtations
    dummy2 = RProtations
    
    # Ramp function parameters
    UltTargetSpeed = TargetSpeed
    C = TargetSpeed - 0.3
    A = 0.15
    B = (C/2)+0.3
    D = 0.5*np.log(((0.005/C)-2)/(-0.005/C))
    x=0
    oldtime = time.perf_counter()
    # Start loop
    while PrintToggle and not stopALL:
        currenttime = time.perf_counter()
        #print(currenttime)
        if x<50:
            TargetSpeed = (C/2)*((1-np.exp(-2*(A*x-D)))/(1+np.exp(-2*(A*x-D))))+B # Ramp function based on a hyperbolic tangent curve
            x = x+currenttime-oldtime
            #print("Reference signal: " + str(TargetSpeed))
        if RProtations > dummy1: # If the number of rotations increases, step down
            for i in range(stepsDown-1):
                GPIO.output(LPUL, GPIO.HIGH)
                GPIO.output(RPUL, GPIO.HIGH)
                time.sleep(0.0005)
                GPIO.output(LPUL, GPIO.LOW)
                GPIO.output(RPUL, GPIO.LOW)
                time.sleep(0.0005)
                if stopALL:
                    return
            dummy1 = dummy1 + 1 # Update the dummy so that the step down does not trigger until the next rotation is completed
            CurrentHeight = CurrentHeight - DeltaH # Update current height
            ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update the current height label
            if stopALL:
                TargetSpeed = UltTargetSpeed
                return
        time.sleep(0.001) # Delay for stability
        oldtime = currenttime
        if RProtations - dummy2 == numRotations: # If this is the last rotation, go down 4 delta h
            stopWF = True # Stop the wire feeder
            for i in range(4*(stepsDown-1)):
                GPIO.output(LPUL, GPIO.HIGH)
                GPIO.output(RPUL, GPIO.HIGH)
                time.sleep(0.0005)
                GPIO.output(LPUL, GPIO.LOW)
                GPIO.output(RPUL, GPIO.LOW)
                time.sleep(0.0005)
            CurrentHeight = CurrentHeight - 4*   DeltaH # Update current height
            ActuatorSelectionLabel.set("Current Height [mm]: " + str(CurrentHeight)) # Update the current height label
            
            # Turn motors off
            '''GPIO.output(LENA, GPIO.HIGH)
            GPIO.output(RENA, GPIO.HIGH)'''
            break
    return
    

# Define buttons

'''Threads are utilized to run motor functions in the following buttonCommand functions.
Stepper motors use blocking loops to run (while and for loops) that normally
block all other code from running while in those loops. The Raspberry Pi
processor allows for multiple threads to be run at a time. A thread just 
allows for a process to run independently of other processes, so we can run
a blocking loop on one thread without it affecting the rest of the code or
blocking anything else out. This allows us to input commands to a stepper
motor and control other motors while the stepper motor is still moving.'''

def buttonCommand_updateTargetHeight(): # reads the txt entry and update target height
    '''This function reads the input desired height and assigns it to the
    global target height variable. It then starts a thread to run the Target
    function.''' 
    global TargetHeight
    TargetHeight = float(TargetHeightEntry.get())
    # Start the thread
    threadAUTO = threading.Thread(target=Target)
    threadAUTO.start()


def buttonCommand_moveUp():  # manual control for moving up
    '''This function starts a thread that just runs the Up function.'''
    # Start the thread
    threadUP = threading.Thread(target=Up)
    threadUP.start()
    time.sleep(0.002)

def buttonCommand_moveDown():  # manual control for moving down
    '''This function starts a thread that just runs the Down function.'''
    # Start the thread
    threadDOWN = threading.Thread(target=Down)
    threadDOWN.start()
    time.sleep(0.002)
    
def buttonCommand_ZeroActuators():
    '''This function starts a thread that runs the ZeroRActuator and ZeroLActuator functions.'''
    # Start the threads
    threadZR = threading.Thread(target=ZeroRActuator)
    threadZR.start()
    threadZL = threading.Thread(target=ZeroLActuator)
    threadZL.start()

def buttonCommand_FeedWireForward():  # manual control for feeding the wire forward
    '''This function assigns the global forward boolean to True and makes all others False.
    It starts a thread that runs the WFforward function.'''
    global forward
    global backward
    global stop
    
    # Update the direction variables
    forward = True
    backward = False
    stop = False
    
    # Start the thread
    threadWF = threading.Thread(target = WFforward)
    threadWF.start()
        
def buttonCommand_FeedWireBackward():  #manual control for feeding the wire backward
    '''This function assigns the global backward boolean to True and makes all others False.
    It starts a thread that runs the WFbackward function.'''
    global forward
    global backward
    global stop
    
    # Update the direction variables
    forward = False
    backward = True
    stop = False
    
    # Start the thread
    threadWF = threading.Thread(target = WFbackward)
    threadWF.start()

def buttonCommand_STOP(): # manual control for stopping wire feed
    '''This function assigns the global stop boolean to True and makes all others False.
    It starts a thread that runs the WFstop function.'''
    global forward
    global backward
    global stop
    
    # Update the direction variables
    forward = False
    backward = False
    stop = True
    
    # Start the thread
    threadWF = threading.Thread(target = WFstop)
    threadWF.start()
    
def buttonCommand_updateTargetSpeed(): #Reads the txt entry and sends to serial message arduino to update target speed
    '''This function assigns an entered target speed to the TargetSpeed global variable.'''
    global TargetSpeed
    
    TargetSpeed = TargetSpeedEntry.get()
    TargetSpeed = float(TargetSpeed)
 

def buttonCommand_RotateCW(): # manual control for rotating platform clockwise
    '''This function assigns the global rotation direction variable to CW.'''

    global RotateDirectionState
    RotateDirectionState = 1
    RotateDirection.set("Clockwise")

def buttonCommand_RotateCCW(): # manual control for rotating platform counterclockwise
    '''This function assigns the global rotation direction variable to CCW.'''

    global RotateDirectionState
    RotateDirectionState = 0
    RotateDirection.set("Counter-Clockwise")

def buttonCommand_Rotate(): # Control for rotating platform a certain angle
    '''This function assigns a target rotating platform angle to the global 
    variable and starts a thread to run the RPANGLE function.'''

    global TargetAngle
    TargetAngle = float(TargetAngleEntry.get())
    
    # Start the thread
    threadRPangle = threading.Thread(target=RPANGLE)
    threadRPangle.start()

def buttonCommand_updateAutoRPOnOff(): # Toggles between manual and automated control of rotating platform speed
    '''This function toggles the global RPtoggle flag between True and False.
    Depending on the desired direction, it starts a thread that runs either
    the RPCW or RPCCW functtion at an entered target speed. This target
    speed is also stored globally.'''

    global Automated_Controls_stateRP
    global RPtoggle
    global TargetAngleSpeed

    # if we are in automated controls --> set to manual--> else set controls to automatic
    if Automated_Controls_stateRP == 1:
        Automated_Controls_stateRP = 0
        AutoRPLabel.set("Automated Rotating Platform Controls: Off ")
        RPtoggle = False

    else:
        Automated_Controls_stateRP = 1
        AutoRPLabel.set("Automated Rotating Platform Controls: On ")
        RPtoggle = True
        if RotateDirectionState == 1:
            # Start the thread
            threadRPCW = threading.Thread(target=RPCW,args = (TargetAngleSpeed,))
            threadRPCW.start()
        else:
            # Start the thread
            threadRPCCW = threading.Thread(target=RPCCW, args = (TargetAngleSpeed,))
            threadRPCCW.start()
        

def buttonCommand_updateTargetAngleSpeed(): #Reads the txt entry and update target speed
    '''This function updates the global target speed variable with a user
    entered speed.'''

    global TargetAngleSpeed
    TargetAngleSpeed = float(TargetAngleSpeedEntry.get())
    

def buttonCommand_StartRotation(): # Read the txt entry and update the desired number of rotations and the desired delta h
    '''This function updates the WF direction global variables, the number of rotations
    global variable, and the DeltaH global variable. It then starts a thread
    for the PRINT function and one for the WFforward for automatic printing.'''
    global TargetSpeed
    global DeltaH
    global PrintToggle
    global numRotations
    global forward
    global backward
    global stop
    
    # Update the WF direction variables
    forward = True
    backward = False
    stop = False
    
    PrintToggle = True

    numRotations = int(NumberOfTurnsEntry.get())
    DeltaH = float(DecreaseHeightEntry.get())
    
    # Start the wire feeder and print threads
    threadPrint = threading.Thread(target=PRINT)
    threadPrint.start()
    threadWF = threading.Thread(target = WFforward)
    threadWF.start()

 
def buttonCommand_STOPEVERYTHING(): # Update the stop variables
    '''This function assigns the global variable stopALL to True. All
    functions check for this variable to run, so all other threads are
    terminated when this function is called.'''
    global stopALL
    stopALL = True
    
def buttonCommand_STARTEVERYTHING(): # Update the stop variables
    '''This function assignes the global variable stopAll to False, allowing
    all functions to reenable.'''
    global stopALL
    stopALL = False

# declare the automated controls to default at 0 (manual controls)
Automated_Controls_stateWF = 0
Automated_Controls_stateRP = 0

# Platform Rotation Direction default is counterclockwise
RotateDirectionState = 0


# Build GUI-------------------------------------------------------------------------------------------------------------
tkTop = tkinter.Tk()  # Create GUI Box
tkTop.geometry('1600x1080')  # size of GUI
tkTop.title("Test Stand Controller")  # title in top left of window

Title = tkinter.Label(tkTop,text='Test Stand Controls', font=("Courier", 14, 'bold')).grid(row=0, column=0, rowspan=1, columnspan=3)  # Title on top middle of screen


# Fill in the Manual controls Side--------------------------------------------------------------------------------------
ManualFrame = tkinter.Frame(master=tkTop, height=200, width=900) # create frame for the manual controls
ManualLable = tkinter.Label(master=ManualFrame, text='Manual Stand Height Controls',
                            font=("Courier", 12, 'bold')).pack()  # manual controls lable
ManualFrame.grid(row=1, column=0)

LeftButtonsFrame = tkinter.Frame(master=ManualFrame, width=100)
LeftButtonsLable = tkinter.Label(master=LeftButtonsFrame, text='Actuator Selection',
                                 font=("Courier", 12, 'bold')).pack()

RightButtonsFrame = tkinter.Frame(master=ManualFrame, width=100)
RightButtonsLable = tkinter.Label(master=RightButtonsFrame, text='Up/Down Controls',
                                  font=("Courier", 12, 'bold')).pack()

button_left_state = tkinter.Button(LeftButtonsFrame,
                                   text="Zero Actuators",
                                   command=buttonCommand_ZeroActuators,
                                   height=4,
                                   fg="black",
                                   width=8,
                                   bd=5,
                                   activebackground='green'
                                   )
button_left_state.pack(side='top', ipadx=10, padx=10, pady=10)

ActuatorSelectionLabel = tkinter.IntVar()
ActuatorSelection = tkinter.Label(master=LeftButtonsFrame, textvariable=ActuatorSelectionLabel)
ActuatorSelectionLabel.set("")
ActuatorSelection.pack()

button_up_state = tkinter.Button(RightButtonsFrame,
                                 text="Up",
                                 command=buttonCommand_moveUp,
                                 height=4,
                                 fg="black",
                                 width=8,
                                 bd=5,
                                 activebackground='green'
                                 )
button_up_state.pack(side='top', ipadx=10, padx=10, pady=10)

button_down_state = tkinter.Button(RightButtonsFrame,
                                   text="Down",
                                   command=buttonCommand_moveDown,
                                   height=4,
                                   fg="black",
                                   width=8,
                                   bd=5,
                                   activebackground='green'
                                   )
button_down_state.pack(side='top', ipadx=10, padx=10, pady=10)

LeftButtonsFrame.pack(fill=tkinter.BOTH, side=tkinter.LEFT, expand=True)
RightButtonsFrame.pack(fill=tkinter.BOTH, side=tkinter.LEFT, expand=True)

# Fill in the Automated controls Side----------------------------------------------------------------------------------------------------------------------------------------
AutoFrame = tkinter.Frame(master=tkTop, height=200, width=900, bg="gray")
AutoLable = tkinter.Label(master=AutoFrame, text='Automated Stand Height Controls', font=("Courier", 12, 'bold'), bg="gray").pack(
    side='top')  # Automated controls lable

TargetHeightLable = tkinter.Label(master=AutoFrame, text='Enter Target Height: ', font=("Courier", 12), bg="gray").pack(
    side='left', ipadx=10, padx=10, pady=40)  # Automated controls lable
TargetHeightEntry = tkinter.Entry(AutoFrame)
TargetHeightEntry.pack(side='left', ipadx=0, padx=0, pady=0)

button_UpdateTarget = tkinter.Button(AutoFrame,
                                     text="Update Target",
                                     command=buttonCommand_updateTargetHeight,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green'
                                     )
button_UpdateTarget.pack(side='left', ipadx=0, padx=20, pady=10)


AutoFrame.grid(row = 1, column = 1, stick='N')

# Fill in manual wire feed frame
ManualFrameWF = tkinter.Frame(master=tkTop, height=200, width=900)
ManWFLabel = tkinter.Label(master=ManualFrameWF,
                           text='Manual Wire Feed Controls',
                           font=("Courier", 12, 'bold')).grid(row=0, column=0, rowspan = 1, columnspan = 3, pady=20)  # Manual wire feed controls label

button_FeedWireBack = tkinter.Button(ManualFrameWF,
                                     text="Backward",
                                     command=buttonCommand_FeedWireBackward,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green')
button_FeedWireBack.grid(row=1,column=0, columnspan=1, pady=20, padx=10)

button_FeedWireFwd = tkinter.Button(master=ManualFrameWF,
                                     text="Forward",
                                     command=buttonCommand_FeedWireForward,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green')
button_FeedWireFwd.grid(row=1, column=2, columnspan=1, pady=20, padx=10)

button_Stop = tkinter.Button(master=ManualFrameWF,
                                     text="STOP",
                                     command=buttonCommand_STOP,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green')
button_Stop.grid(row=1, column=1, columnspan=1, pady=20, padx=10)

ManualFrameWF.grid(row=2, column=0, pady=20)

# Fill in automatic wire feed frame
AutoFrameWF = tkinter.Frame(master=tkTop, height=200, width=600, bg='gray')
AutoFrameWFLabel = tkinter.Label(master=AutoFrameWF,
                           text='Automatic Wire Feed Controls',
                           font=("Courier", 12, 'bold'),
                           bg="gray").grid(row=0, column=0, columnspan=3, pady=20)  # Automatic wire feed controls label


varLabel1 = tkinter.IntVar()
#AutoWFLabel = tkinter.Label(master=AutoFrameWF, textvariable=varLabel1, bg="gray").grid(row=2, column=1)
varLabel1.set("Automated Wire Feed Controls: Off")

AutoFrameWF.grid(row=2,column=1, pady=20, sticky='N')


TargetSpeedLabel = tkinter.Label(master=AutoFrameWF, text='Enter Target Speed [in/s]: ', font=("Courier", 12), bg="gray").grid(row=2,column=0, padx=10)  # Manual wire feed speed label
TargetSpeedEntry = tkinter.Entry(AutoFrameWF)
TargetSpeedEntry.grid(row=2, column=1, padx=10)

button_UpdateTarget = tkinter.Button(AutoFrameWF,
                                     text="Update Target",
                                     command=buttonCommand_updateTargetSpeed,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green'
                                     )
button_UpdateTarget.grid(row=2, column=2, padx=10)

#Fill in rotating platform manual control
ManualFrameRP = tkinter.Frame(master=tkTop, height=200, width=900)
ManualLabelRP = tkinter.Label(master=ManualFrameRP, text='Manual Platform Controls', font=("Courier", 12, 'bold')).grid(row=0, column=0, columnspan=6)
button_Clockwise = tkinter.Button(ManualFrameRP,
                                  text="Clockwise",
                                  command=buttonCommand_RotateCW,
                                  height=2,
                                  fg="black",
                                  width=15,
                                  bd=5,
                                  activebackground='green')
button_CounterClockwise = tkinter.Button(ManualFrameRP,
                                  text="Counter-Clockwise",
                                  command=buttonCommand_RotateCCW,
                                  height=2,
                                  fg="black",
                                  width=15,
                                  bd=5,
                                  activebackground='green')
button_Clockwise.grid(row=1, column=0, columnspan=3, padx=10, pady=20)
button_CounterClockwise.grid(row=1, column=3, columnspan=3, padx=10, pady=20)

RotateDirection = tkinter.IntVar()
RotateDirectionLabel = tkinter.Label(master=ManualFrameRP, textvariable=RotateDirection, font=("Courier", 12)).grid(row=2, column=0, columnspan=6, padx=10, ipadx=20, pady=5)
RotateDirection.set("Counter-Clockwise")

TargetAngleLabel = tkinter.Label(master=ManualFrameRP, text="Rotate [deg]:", font=("Courier", 12)).grid(row=3, column=0, columnspan=2, padx=10, pady=5)
TargetAngleEntry = tkinter.Entry(ManualFrameRP)
TargetAngleEntry.grid(row=3, column=2, columnspan=2, padx=10, pady=5)

button_TargetAngleUpdate = tkinter.Button(master=ManualFrameRP,
                                          text="Rotate",
                                          command=buttonCommand_Rotate,
                                          height=2,
                                          fg="black",
                                          width=15,
                                          bd=5,
                                          activebackground='green')
button_TargetAngleUpdate.grid(row=3, column=5, columnspan=2, padx=10, pady=5)

ManualFrameRP.grid(row=3, column=0, pady=20, sticky="N")

# Fill in automatic platform control frame
AutoFrameRP = tkinter.Frame(master=tkTop, height=200, width=600, bg='gray')
AutoFrameRPLabel = tkinter.Label(master=AutoFrameRP,
                           text='Automatic Rotating Platform Controls',
                           font=("Courier", 12, 'bold'),
                           bg="gray").grid(row=0, column=0, columnspan=3, pady=20)  # Automatic rotating platform controls label

button_AutoRP_OnOff = tkinter.Button(AutoFrameRP,
                                     text="Automatic Rotating Platform Speed On/Off",
                                     command=buttonCommand_updateAutoRPOnOff,
                                     height=2,
                                     fg="black",
                                     width=40,
                                     bd=5,
                                     activebackground='green'
                                     )
button_AutoRP_OnOff.grid(row=1, column=0, columnspan=3, padx=10)

AutoRPLabel = tkinter.IntVar()
AutoRP = tkinter.Label(master=AutoFrameRP, textvariable=AutoRPLabel, bg="gray").grid(row=2, column=1)
AutoRPLabel.set("Automatic Rotating Platform Controls: Off")

AutoFrameRP.grid(row=2,column=1, pady=20, sticky='N')


TargetAngleSpeedLabel = tkinter.Label(master=AutoFrameRP, text='Enter Target Angle Speed [deg/s]: ', font=("Courier", 12), bg="gray").grid(row=3,column=0, padx=10)
TargetAngleSpeedEntry = tkinter.Entry(AutoFrameRP)
TargetAngleSpeedEntry.grid(row=3, column=1, padx=10)

button_UpdateTargetAngleSpeed = tkinter.Button(AutoFrameRP,
                                     text="Update",
                                     command=buttonCommand_updateTargetAngleSpeed,
                                     height=2,
                                     fg="black",
                                     width=15,
                                     bd=5,
                                     activebackground='green'
                                     )
button_UpdateTargetAngleSpeed.grid(row=3, column=2, padx=10)

AutoFrameRP.grid(row=3, column=1)

# Fill in Rotation plus h mm decrease frame
RotationFrame = tkinter.Frame(master=tkTop, height=200, width=900)
RotationLabel = tkinter.Label(master=RotationFrame,  text='Decrease h [mm] per 360 [deg] Rotation Function', font=("Courier", 12, 'bold')).grid(row=0, column = 0, columnspan=6)

NumberofTurnsLabel = tkinter.Label(master=RotationFrame,  text='Number of rotations:', font=("Courier", 12)).grid(row=1, column = 0, columnspan=2, padx=10, pady=5)
NumberOfTurnsEntry = tkinter.Entry(RotationFrame)
NumberOfTurnsEntry.grid(row=1, column=2, columnspan=2, pady=5, padx=10)

'''button_UpdateNOT = tkinter.Button(master=RotationFrame,
                                  text="Update",
                                  command=buttonCommand_UpdateNOT,
                                  height=2,
                                  fg="black",
                                  width=15,
                                  bd=5,
                                  activebackground='green'
                                  )
button_UpdateNOT.grid(row=1, column=5, columnspan=2, pady=5, padx=10)'''

DecreaseHeightLabel = tkinter.Label(master=RotationFrame, text='Delta h [mm]:', font=("Courier", 12)).grid(row=2, column=0, columnspan=3)
DecreaseHeightEntry = tkinter.Entry(RotationFrame)
DecreaseHeightEntry.grid(row=2, column=3, columnspan=3, pady=5, padx=20)

button_StartFunction = tkinter.Button(master=RotationFrame,
                                      text='START',
                                      command=buttonCommand_StartRotation,
                                      height=2,
                                      fg='black',
                                      width=15,
                                      bd=5,
                                      activebackground='green')
button_StartFunction.grid(row=3, column=0, columnspan=6, pady=5)

RotationFrame.grid(row=4, column=0)

# Stop and start buttons frame
StopStartFrame = tkinter.Frame(master=tkTop, height=200, width=900)
StopStartLabel = tkinter.Label(master=StopStartFrame,  text='STOP and START buttons', font=("Courier", 12, 'bold')).grid(row=0, column = 0, columnspan=6)
button_STOPEVERYTHING = tkinter.Button(master=StopStartFrame,
                                       text='STOP ALL OPERATIONS',
                                       command=buttonCommand_STOPEVERYTHING,
                                       height=10,
                                       fg='white',
                                       width=50,
                                       background='red',
                                       bd=5)
button_STOPEVERYTHING.grid(row=1,column=0, columnspan = 3)

button_STARTEVERYTHING = tkinter.Button(master=StopStartFrame,
                                       text='START ALL OPERATIONS',
                                       command=buttonCommand_STARTEVERYTHING,
                                       height=10,
                                       fg='white',
                                       width=50,
                                       background='green',
                                       bd=5)
button_STARTEVERYTHING.grid(row=1,column=3, columnspan = 3)

StopStartFrame.grid(row=4, column=1)


tkinter.mainloop() # run loop watching for gui interactions


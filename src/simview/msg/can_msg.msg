uint8 throttlePercent
uint8 brakePercent
int16 wheelAngle
float32 vehicleSpeed #  m/s

uint8 curGear
#0 no 1 yes
uint8 hookState
uint8 linkPallet
uint8 controlPanelState
uint8 eabPanelState
uint8 emergencyStop
uint8 batteryPower
uint8 hookButton
uint8 linkButton

#uint8[] ultrasoundDis
#uint8[] timState
#uint8 safeTouch

uint8[] faultCode


#new eps msg zhangyu 20220226
uint8 epsCMD
float32 wheelAngleCMD
uint8 epsMode
float32 epsCurrent
uint8 epsCentring
uint8 epsERR1
uint8 epsERR2
uint8 hookPos
uint8 palletPos
uint8 hookStatus
uint8 palletStatus

uint8[] rawcommand
uint8[] rawfeedback


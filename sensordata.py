import io
import csv
import datetime

def getNextBytes(n,bytes):#Gets the next n bytes
    return bytes[:n]

def getMilliDifference(start_time,end_time):#Calculates difference between current time and initial time
    diff = end_time - start_time
    return int((diff.seconds * 1000) + (diff.microseconds / 1000))

bytes = []
acceleration = []
gyroscope = []
countInitDates = 0
countInitDatesGyro = 0

f = open("data.bin", "rb")#Opens binaty file
nBytes = 0
count = 0
try: #Reads bytes from binary file and stores them
    byte = f.read(1)
    while byte != "":
        nBytes = nBytes + 1
        bytes.append(byte)
        byte = f.read(1)
finally:
    f.close()

accRate = -1
accRange = -1
initialSeconds = 0
initialHundreds = 0
check = 1

while count < nBytes:#Reads all retrieved bytes
    currBytes = getNextBytes(14,bytes)#Gets header of the next piece of data
    bytes = bytes[-(nBytes-count-14):]#Eliminates bytes currently considered from bytes to conider afterwards
    pType = ord(currBytes[1])#Obtains packet type, which corresponds to the 2nd byte, from specs
    count = count + 14#Increases count of bytes considered

    if pType == 0:#Sanity packet, always is 0x02
        bytes = bytes[-(nBytes-count-1):]
        count = count + 1

    elif pType == 1:#Packet of samples of accelerometer data
        initDate = datetime.datetime(ord(currBytes[12]), ord(currBytes[11]), ord(currBytes[10]), ord(currBytes[8]), ord(currBytes[7]), ord(currBytes[6]), ord(currBytes[5])*10000,None)#Set time to packet time
        countInitDates = countInitDates + 1
        if countInitDates == 1:#If no initial time has been set
            prevInitDate = initDate
            currentTime = 0
        else:
            currentTime = getMilliDifference(prevInitDate,initDate)#Compuute difference between initial time and current packet time

        packetLen1 = currBytes[2]#Packet length is spread accross two bytes
        packetLen2 = currBytes[3]
        packetLen = (ord(packetLen1) * 256) + ord(packetLen2)#Shift first length obtained by 8 bits to get total length
        nOfFrames = (packetLen - 14)/6#Total number of sample frames, from specifications
        initialFrames = nOfFrames

        while nOfFrames > 0:
            frameBytes = getNextBytes(6,bytes)#Accelerometer data is in packets of 6 bytes

            signalX = ord(frameBytes[1]) * 256 + ord(frameBytes[0])#Accelerometer values are spread accross two bytes
            signalY = ord(frameBytes[3]) * 256 + ord(frameBytes[2])
            signalZ = ord(frameBytes[5]) * 256 + ord(frameBytes[4])

            select = 0b0000111111111111#Select last 12 bits from acceleration

            xAccel = (signalX & select)*3.91#Accelerometer data is only 12 bits long
            yAccel = (signalY & select)*3.91
            zAccel = (signalZ & select)*3.91
            acceleration.append(tuple((currentTime,tuple(((xAccel),(yAccel),(zAccel))))))#Add acceleration values to table
            currentTime = currentTime + 8#Sampling period is 8ms, from specfications

            bytes = bytes[-(nBytes-count-6):]#Delete current bytes being considered from all
            count = count + 6
            nOfFrames = nOfFrames - 1

    elif pType == 2:#Packet of samples of gyroscope data

        initDateGyro = datetime.datetime(ord(currBytes[12]), ord(currBytes[11]), ord(currBytes[10]), ord(currBytes[8]), ord(currBytes[7]), ord(currBytes[6]), ord(currBytes[5])*10000,None)
        countInitDatesGyro = countInitDatesGyro + 1
        if countInitDatesGyro == 1:
            prevInitDateGyro = initDateGyro
            currentTimeGyro = 0
        else:
            currentTimeGyro = getMilliDifference(prevInitDateGyro,initDateGyro)

        packetLen1 = currBytes[2]
        packetLen2 = currBytes[3]
        packetLen = (ord(packetLen1) * 256) + ord(packetLen2)
        nOfFrames = (packetLen - 14)/6
        initialFrames = nOfFrames

        while nOfFrames > 0:
            frameBytes = getNextBytes(6,bytes)

            signalX = ord(frameBytes[1]) * 256 + ord(frameBytes[0])
            signalY = ord(frameBytes[3]) * 256 + ord(frameBytes[2])
            signalZ = ord(frameBytes[5]) * 256 + ord(frameBytes[4])

            select = 0b0000111111111111

            xGyro = (signalX & select)*3.91
            yGyro = (signalY & select)*3.91
            zGyro = (signalZ & select)*3.91

            gyroscope.append(tuple((currentTimeGyro,tuple(((xGyro),(yGyro),(zGyro))))))
            currentTimeGyro = currentTimeGyro + 8

            bytes = bytes[-(nBytes-count-6):]
            count = count + 6
            nOfFrames = nOfFrames - 1

    elif pType == 3:#Single sample of accelerometer data
        initDate = datetime.datetime(ord(currBytes[12]), ord(currBytes[11]), ord(currBytes[10]), ord(currBytes[8]), ord(currBytes[7]), ord(currBytes[6]), ord(currBytes[5])*10000,None)
        countInitDates = countInitDates + 1
        if countInitDates == 1:
            prevInitDate = initDate
            currentTime = 0
        else:
            currentTime = getMilliDifference(prevInitDate,initDate)

        signalX = ord(frameBytes[1]) * 256 + ord(frameBytes[0])
        signalY = ord(frameBytes[3]) * 256 + ord(frameBytes[2])
        signalZ = ord(frameBytes[5]) * 256 + ord(frameBytes[4])

        select = 0b0000111111111111

        xAccel = (signalX & select)*3.91
        yAccel = (signalY & select)*3.91
        zAccel = (signalZ & select)*3.91

        acceleration.append(tuple((currentTime,tuple(((xAccel),(yAccel),(zAccel))))))

        bytes = bytes[-(nBytes-count-6):]
        count = count + 6

    elif pType == 4:#Single sample of gyroscope data
        initDate = datetime.datetime(ord(currBytes[12]), ord(currBytes[11]), ord(currBytes[10]), ord(currBytes[8]), ord(currBytes[7]), ord(currBytes[6]), ord(currBytes[5])*10000,None)
        countInitDates = countInitDates + 1
        if countInitDates == 1:
            prevInitDate = initDate
            currentTime = 0
        else:
            currentTime = getMilliDifference(prevInitDate,initDate)

        signalX = ord(frameBytes[1]) * 256 + ord(frameBytes[0])
        signalY = ord(frameBytes[3]) * 256 + ord(frameBytes[2])
        signalZ = ord(frameBytes[5]) * 256 + ord(frameBytes[4])

        select = 0b0000111111111111

        xGyro = (signalX & select)*3.91
        yGyro = (signalY & select)*3.91
        zGyro = (signalZ & select)*3.91

        gyroscope.append(tuple((currentTime,tuple(((xGyro),(yGyro),(zGyro))))))

        bytes = bytes[-(nBytes-count-6):]
        count = count + 6

    elif pType == 5:#Magnetometer samples
        bytes = bytes[-(nBytes-count-6):]#Not necessary for the project, so disconsidered
        count = count + 6

    elif pType == 6:#Air pressure samples
        bytes = bytes[-(nBytes-count-4):]
        count = count + 4

    elif pType == 7:#Temperature samples
        bytes = bytes[-(nBytes-count-4):]
        count = count + 4

    elif pType == 8:#Battery level data
        bytes = bytes[-(nBytes-count-2):]
        count = count + 2

    elif pType == 9:#Error bit
        bytes = bytes[-(nBytes-count-1):]
        count = count + 1

    elif pType == 10:#Configuration packets (sample rate, sample range, etc)
        addBytes = getNextBytes(5,bytes)
        accRate = ord(addBytes[0])
        accRange = ord(addBytes[1])
        bytes = bytes[-(nBytes-count-5):]
        count = count + 5

    fd = open('/Users/alvarobejar/Documents/MATLAB//acceleration.csv',"w")#Creates a file in MATLAB directory
    for row in acceleration:
        fd.write(str(row[0])+","+str(row[1][0])+","+str(row[1][1])+","+str(row[1][2])+"\n")#Writes acceleration values to .csv file

    fd.close()

    fd = open('/Users/alvarobejar/Documents/MATLAB//gyroscope.csv',"w")
    #fd.write('time,xAccel,yAccel,zAccel\n')
    for row in gyroscope:
        fd.write(str(row[0])+","+str(row[1][0])+","+str(row[1][1])+","+str(row[1][2])+"\n")#Writes gyroscope values to .csv file

    fd.close()

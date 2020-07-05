import array
import serial
import time

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=100000,
    bytesize=8,
    parity='E',
    stopbits=2,
    timeout=None,
)
count=0
#timeout =1 is in seconds, and only should be used if readline() is used
#readline() uses timeout as end of file (EOF)
#test serial ports in term window: python -m serial.tools.list_ports
print(ser.name)#shows actual port opened
readin = bytearray(25) #'bytearray'init the array bytes to read in
La = list(range(16)) #0 to 15 ; 16 positions
#'I' is unsigned integer, 2 bytes
achannel = array.array('I',La) #initialize a basic python array
Ld = list(range(4)) #0 to 15 ; 16 positions
digichannel = array.array('I',Ld) #initialize a basic python array
sendout = bytearray(25)
#ser.write(b'xyz')#write a string

#---- Function Timer, returns current time
def how_long(start, activity):
    print('%s took %.3fs' % (activity, time.time()-start))
    return time.time()

while (count<100):
    readin=ser.read(25)#readin 25bytes or timeout
    if readin[0]==0x0F:#SBUS message start byte bits are LSB first, 11110000 is F0, but LSB First is 0F
        achannel[0]  = ((readin[1] | readin[2]<<8) & 0x07FF)
        achannel[1]  = ((readin[2]>>3 | readin[3]<<5) & 0x07FF)
        achannel[2]  = ((readin[3]>>6 | readin[4]<<2 | readin[5]<<10) & 0x07FF)
        achannel[3]  = ((readin[5]>>1 | readin[6]<<7) & 0x07FF)
        achannel[4]  = ((readin[6]>>4 | readin[7]<<4) & 0x07FF)
        achannel[5]  = ((readin[7]>>7 | readin[8]<<1 | readin[9]<<9)  & 0x07FF)
        achannel[6]  = ((readin[9]>>2 | readin[10]<<6) & 0x07FF)
        achannel[7]  = ((readin[10]>>5 | readin[11]<<3) & 0x07FF)
        achannel[8]  = ((readin[12] | readin[13]<<8) & 0x07FF)
        achannel[9]  = ((readin[13]>>3 | readin[14]<<5) & 0x07FF)
        achannel[10] = ((readin[14]>>6 | readin[15]<<2 | readin[16]<<10) & 0x07FF)
        achannel[11] = ((readin[16]>>1 | readin[17]<<7) & 0x07FF)
        achannel[12] = ((readin[17]>>4 | readin[18]<<4) & 0x07FF)
        achannel[13] = ((readin[18]>>7 | readin[19]<<1 |readin[20]<<9) & 0x07FF)
        achannel[14] = ((readin[20]>>2 | readin[21]<<6) & 0x07FF)
        achannel[15] = ((readin[21]>>5 | readin[22]<<3) & 0x07FF)
        if (readin[23] & (1<<0)):
            digichannel[0] = 1
        else:
            digichannel[0] = 0
        if(readin[23] & (1<<1)):
            digichannel[1] = 1
        else:
            digichannel[1] = 0
        if(readin[23] & (1<<2)):
            digichannel[2] = 1
        else:#Switch B
            digichannel[2] = 0
        if(readin[23] & (1<<3)):
            digichannel[3] = 1
        else:#Switch F
            digichannel[3] = 0
        count=count+1
        print(achannel[0])
        print(achannel[1])
        print(achannel[2])
        print(achannel[3])
    ser.reset_input_buffer()
    time.sleep(.2)
    #ser.open()
ser.close()#close the serial port


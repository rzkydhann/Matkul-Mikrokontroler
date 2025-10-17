import serial
#arduinoData = serial.Serial('/dev/ttyACM1', 9600)
arduinoData = serial.Serial('COM3', 115200, timeout=0)

while True:
    myData = input('Kirimkan perintah (1/0): ')  
    if myData == "1":
        arduinoData.write(b'1')
    elif myData == "0":  
        arduinoData.write(b'0')
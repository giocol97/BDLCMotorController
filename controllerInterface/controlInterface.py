import serial
import json
import eel


@eel.expose
def stop_drive():
    ser.write(b'stop\n')

@eel.expose
def enable_drive():
    ser.write(b'enable\n')

@eel.expose
def reset_drive():
    ser.write(b'reset\n')

@eel.expose
def send_set_packet(txt):
    ser.write(txt.encode('utf-8')+'\n'.encode('utf-8'))

def serial_thread():
    global ser
    ser = serial.Serial('COM3', 115200, timeout=1)

    while True:
        eel.sleep(0.001)
        line = ser.readline()
        if line:
            jsonData = line.decode('utf-8')
            print(line.decode('utf-8'))
            try:
                y = json.loads(jsonData) #check if valid json
                
                eel.showData(jsonData)
                #plt.plot(xpoints, ypoints)
                # plt.show()

            except:
                # print(jsonData)
                eel.appendToLog(jsonData)


eel.init('web')

eel.spawn(serial_thread)
eel.start('index.html', size=(1536, 800))

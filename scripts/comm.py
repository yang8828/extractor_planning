import serial


class State:
    def __init__(self, x=0.0, y=0.0,workstate=0,azuith=0.0):
        self.x = x
        self.y = y
        self.workstate=workstate
        self.azuith=azuith
    
class Serial:
    def __init__(self, portname='/dev/ttyUSB0', baudrate=115200):
        try:
            self.commport = serial.Serial(portname, baudrate)
            self.flag = True
        except Exception as e:
            self.flag = False
        self.state = State(0, 0,0,0)

    def recv(self):
        if self.commport.isOpen():            
            flag_receive_data = 0
            datalist = []
            while True:
                data = ord(self.commport.read(1))
                if data == 123:
                    flag_receive_data = 1 
                else:
                    continue

                while flag_receive_data == 1:
                    data = ord(self.commport.read(1))
                    datalist.append(data)
                    if data == 125 and datalist.__len__() == 11:
                        flag_receive_data = 2
                        if datalist[0] == 0x01:
                            try:
                                x = (datalist[2] * 0x10000 + datalist[3] * 0x100+datalist[4])/1000.0
                                y = (datalist[6] * 0x10000 + datalist[7] * 0x100+datalist[8])/1000.0
                                workstate=datalist[1]
                                azuith=(datalist[5]*0x100+datalist[9])
                                #print(datalist)
                                self.state = State(x,y,workstate,azuith)
                            except Exception as e:
                                print(e)
                                self.state = State(0, 0)
                break

    def write(self, x, y,finitesate,heading_angle):
        if self.commport.isOpen():
            try:
                datalist_write = []
                datalist_write.append(0x7b)
                datalist_write.append(0x03)
                #datalist_write.append(0x11)
                datalist_write.append(finitesate/0x01)
                datalist_write.append(int(x / 0x10000))
                datalist_write.append(int(x / 0x100) % 0x100)
                datalist_write.append(x % 0x100)
                datalist_write.append(int(y / 0x10000))
                datalist_write.append(int(y / 0x100) % 0x100)
                datalist_write.append(y % 0x100)
                datalist_write.append(int(heading_angle/0x100))
                datalist_write.append(heading_angle % 0x100)
                datalist_write.append(0x7d)
                #print(datalist_write)
                self.commport.write(datalist_write)
            except Exception as e:      
                print(e)
        else:
            print('Comm is not Available!')

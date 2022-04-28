import serial
import threading
import time

class gCodeSender():
    def __init__(self):
        printerPort = "COM4"
        clawPort = "COM3"
        self.link = serial.Serial(printerPort, 115200)
        while True:
            message = self.link.readline()
            print (message)
            if message ==b'init valid: \n':
                break
        for i in range(2):
            self.readresponce()
        #self.readUntillOk()
        print("Init Complete")
        self.link.write(b"G21\n") # millimeters
        self.readUntillOk()
        print("mm set")
        self.link.write(b"G90\n") # Absolute distance mode 
        self.readUntillOk()
        print("Absolute ditance set")
        self.mode = "Absolute"
        #self.link.write(b"M114_REALTIME")
        #self.readUntillOk()
        print("Realtime set")
        self.home()
        print("Home complete")
        self.clawLink = serial.Serial(clawPort, 115200)
        self.cachedpos = (0,0,0,0)
        self.cachedposvalid = False
        self.clawPos = 160
        self.clawWant = 160
        self.clawLink.timeout = 3
        #self.add_next(x=50,y =50,z=100)
    def clawLoop(self):
        while True:
            if not self.clawWant is None:
                command = str(self.clawWant).encode("UTF-8")+b"\n"
                self.clawLink.write(command)
                print("Writtern",command)
                self.clawPos = self.clawWant
                self.clawWant = None
                print(self.clawLink.readline())
                
                #self.clawPos = float(self.clawLink.readline())
            time.sleep(0.01)
    def home(self):
        self.link.write(b"G28\n") # home all
        self.wait_for_command_to_fishish()
    def wait_for_command_to_fishish_with_wait(self):
        self.commandfinished = False
        self.link.write(b"M400")#G4 P0
        message = self.link.readline()
        self.commandfinished = True
    def wait_for_command_to_fishish(self):
        self.commandfinished = False
        self.readUntillOk()
        self.commandfinished = True
    def readresponce(self):
        print(self.link.readline())
    def readUntillOk(self):
        while True:
            message = self.link.readline()
            #print (message)
            if message ==b'ok\n':
                break
    # def command_is_finished(self):
    #     temp = self.link.timeout
    #     self.link.timeout = 1
    #     self.link.send("M400")#G4 P0
    #     try:
    #         message = self.link.readline()
    #         return True
    #     except serial.SerialTimeoutException:
    #         return False
    #     finally:
    #         self.link.timout = temp
    def command_is_finished(self):
        return self.commandfinished
    
    def get_current_pos(self)->tuple:
        if self.cachedposvalid:
            return self.cachedpos +[self.clawPos]
        else:
            self.link.write(b"M114 R\n")
            #self.readUntillOk()
            #while True:
            #    self.readresponce()
            message = self.link.readline()
            if message==b'echo:busy: processing\n':
                self.wait_for_command_to_fishish()
                self.link.write(b"M114 R\n")
                message = self.link.readline()
            self.readresponce()
            message = message.split()
            message = message[:3]#get only xyz
            #print(message)
            message = [(i.split(b":"))for i in message]
            message = [(i[1])for i in message]
            message = [i if i else 0 for i in message]
            pos = [float(i) for i in message]
            if self.command_is_finished():
                self.cachedposvalid = True
                self.cachedpos = pos
            return pos +[self.clawPos]
    def add_next(self,x= None,y = None,z = None,claw = None,mode= "Absolute"):
        #if not(x or y or z or claw):
        #print(x,y,z,claw,mode)
        if (self.command_is_finished() and (x,y,z,claw) ==self.get_current_pos()):
            return
        if not(x is None and y is None and z is None):
            if mode !=self.mode:
                if mode[0].capitalize() == "A":
                    self.link.write(b"G90\n") # Absolute distance mode 
                    self.mode ="Absolute"
                    print("Absolute set:",self.link.readline())
                elif mode[0].capitalize() =="R":
                    self.link.write(b"G91\n") # Reletive distance mode 
                    self.mode ="Relative"
                    print("Relative set:",self.link.readline())
                else:
                    raise "Mode not known"
            cmd = ''
            if x != None:
                if (x>300):
                    return
                cmd += ' X'
                cmd += str(x)
            if y != None:
                if y>300:
                    return
                cmd += ' Y'
                cmd += str(y)
            if z != None:
                cmd += ' Z'
                cmd += str(z)
            cmd += '\n'
            self.link.write(b"G0" + cmd.encode("UTF-8"))
            self.cachedposvalid = False
            #waitforend = threading.Thread(target=self.wait_for_command_to_fishish,daemon=True)
            #waitforend.start()
            self.wait_for_command_to_fishish()
        if not claw is None:
            if mode[0][0].capitalize() == "A":
                self.clawWant = claw
            elif mode[0].capitalize() =="R":
                if self.clawWant is None:
                    self.clawWant = self.clawPos
                self.clawWant += claw
            command = str(self.clawWant).encode("UTF-8")+b"\n"
            self.clawLink.write(command)
            self.clawPos = self.clawWant
            self.clawWant = None
            print(self.clawLink.readline())


    def set_percent_speed(self,percentspeed):
        percentspeed =str(round(percentspeed,2))
        percentspeed = percentspeed.ljust(3,"0")
        self.link.write(b"M220 S"+percentspeed+b"\n")
        
class virtualprinter():
    def __init__(self):
        self.pos = [0,0,50,0]
        self.wants = []
        #self.printerthread = threading.Thread(target = self.simulate_printer,daemon = True) printer has a delay [for now]
        #self.printerthread.start()
        self.speed = 1
        self.mode = "Absolute"
        self.pastpos = [0,0,0,0]
    def simulate_printer_move(self):
        while self.wants:
            print("Moving: ",self.pos)
            self.pastpos = self.pos.copy()
            if self.wants:
                for x,i in enumerate(self.wants[0]):
                    if abs(i-self.pos[x]) <self.speed:
                        self.pos[x] = i
                    if i > self.pos[x]:
                        self.pos[x]+=self.speed
                    elif i< self.pos[x]:
                        self.pos[x]-=self.speed
                    if self.pos[x]>100:
                        self.pos[x]=100
                    if self.pos[x]<0:
                        self.pos[x]=0
                if self.pastpos ==self.pos:
                    self.wants.pop(0)
            time.sleep(0.01)
            #print(self.wants)
    def get_current_pos(self) ->tuple:
        return self.pos.copy()
    def add_next(self,x= None,y = None,z = None,claw = None,mode= "Absolute"):
        if not(x is None and y is None and z is None and claw is None):
            if mode !=self.mode:
                if mode[0].capitalize() == "A":
                    self.mode ="Absolute"
                elif mode[0].capitalize() =="R":
                    self.mode ="Relative"
                else:
                    raise "Mode not known"
            if self.wants:
                temp = self.wants[-1].copy()
            else:
                temp = self.pos.copy()
            for index,value in enumerate((x,y,z,claw)):
                if not(value is None):
                    if self.mode == "Absolute":
                        temp[index] = value
                    else:
                        temp[index]+= value
            if not(self.wants) or self.wants[-1]!=temp:
                self.wants.append(temp)
            self.simulate_printer_move()
                    
    def set_percent_speed(self,percent):
        self.speed = percent/100
    def command_is_finished(self):
        return not self.wants
    def home(self):
        self.add_next(0,0,0)
    def readresponce(self):
        pass

def createPrinter() ->gCodeSender:
    mode = "A"
    if mode =="A":
        try:
            printer = gCodeSender()
        except serial.serialutil.SerialException:
            print("Using Virtual")
            printer = virtualprinter()
    elif mode == "P":
        printer = gCodeSender()
    elif mode =="V":
        printer =virtualprinter()
    return printer
if __name__ == "__main__":
    printer = createPrinter()


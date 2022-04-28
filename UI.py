from tkinter import HORIZONTAL, TclError, messagebox
import cv2
import numpy as np
import tkinter as tk
import PIL.Image, PIL.ImageTk
from os import scandir

import FrameOperations
from GCode import createPrinter
import XYZFrames
CENTERDOTCOLOUR=(0,255,255)
YDOTCOLOUR=(255,0,255)
XDOTCOLOUR=(255,255,0)


BACKGROUNDCOLOUR= "#d94f25"
TEXTBACKGORUNDCOLOUR = "#a63c1c"
ACTIVEBACKGROUNDCOLOUR = "#2585d9"
FOREGROUNDCOLOUR = "White"
SELECTCOLOUR = "Black"


class App():
    def __init__(self):
        self.root = tk.Tk()
        try:
            self.root.iconbitmap("Logo.ico")
        except TclError:
            print("Could not load Icon")
        self.root.title("Robot Controller")
        self.delay = 15
        ##self.maxes = [100,100,100,100]
        ##self.mins = [0,0,22,10]
        self.frame = None
        self.xyz = XYZFrames.XYZ()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.auto_path =None
        self.rendertextcommand = None
        self.numframes = 1
        loading = tk.Label(self.root,text="Loading Printer...")
        loading.grid(sticky = "NSEW")
        self.root.update()
        self.printer = createPrinter()
        loading.grid_forget()
        self.load_tkinter()
        self.update()
        self.root.mainloop()
    def update(self):
        self.loctext.set("X:{0}, Y:{1}, Z:{2}, Claw:{3}".format(*[round(i,2) for i in self.printer.get_current_pos()]))
        if self.cap.isOpened():
            if not(self.paused.get()):
                ret, self.frame = self.cap.read()
            else:
                ret = True
            if ret == True:
                if self.blured.get():
                    self.frame = cv2.medianBlur(self.frame,5)
                if self.mode.get()==0:
                    self.update_pos()
                    if self.xyz.isCalibrated and (self.printer.get_current_pos()[2]):
                        if self.useFrameCenter.get():
                            self.drawCenter()
                        else:
                            self.drawArrows()
                    if (not self.rendertextcommand is None) and self.xyz.isCalibrated:
                        self.rendertextcommand()
                    self.render(self.frame)
                elif self.mode.get()==1:
                    if self.auto_path is None:            
                        self.auto_path =self.xyz.create_auto_path(self.frame,self.autoMaskConfigs,self.autoCenterConfigs)
                        self.frame,pos =next(self.auto_path)
                        self.printer.add_next(*pos)
                    else:
                            if self.printer.command_is_finished():
                                try:
                                    self.frame,pos = self.auto_path.send((self.frame))
                                    self.printer.add_next(*pos)
                                except StopIteration:
                                    print("Auto Complete")
                                    self.mode.set(0)
                    if self.printer.get_current_pos()[2]:
                        self.drawArrows()
                    self.render(self.frame)
                elif self.mode.get() == 2:
                    self.update_pos()
                    if self.calculateYBool.get() and not hasattr(self.xyz,"savedRead"):
                        messagebox.showerror(message='You need to have previously set a "good read"')
                        self.calculateYBool.set(0)
                    validread,masks = self.xyz.configurationModeUpdate(self.frame,self.printer.get_current_pos(),tollerance= self.tollerance.get(),useAverage=self.useAverage.get(),calculateYOffset=self.calculateYBool.get(),returnMasks=True)
                    if validread:
                        self.update_config_readout()
                        pos = self.printer.get_current_pos()
                        dotpos = pos.copy()
                        dotpos[0] -= self.xyz.dot_offset[0]
                        dotpos[1] -= self.xyz.dot_offset[1]
                        cv2.circle(self.frame, self.xyz.coroodinateToPoint(dotpos[:2],printerpos=pos),10,CENTERDOTCOLOUR,-1)
                        cv2.circle(self.frame, self.xyz.coroodinateToPoint((dotpos[0],dotpos[1]+10),printerpos=pos),10,YDOTCOLOUR,-1)
                        cv2.circle(self.frame, self.xyz.coroodinateToPoint((dotpos[0]+10,dotpos[1]),printerpos=pos),10,XDOTCOLOUR,-1)
                        #self.frame= self.xyz.createViewRenderer(*self.xyz.getMaxViewDims(pos[2]),pos[2])(self.frame)
                    #bw = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
                    #bw, center=  readcv2.findCentreOfCirle(bw)
                    #self.configureCameraCalibation((0,0),(3,4),(0,1))
                    self.render(self.frame,*masks)
                elif self.mode.get() ==3:
                    masks = []
                    for i in self.HSVdisplayers.values():
                        if i.render.get():
                            frame,mask =FrameOperations.findMasks(self.frame,i.colour,tolllerance=self.tollerance.get())
                            masks.append(mask[0])
                    self.render(self.frame,*masks)
                elif self.mode.get() ==4:
                    for i in self.autoconfigVars.keys():
                        if i in self.autoMaskConfigs:
                            self.autoMaskConfigs[i] = self.autoconfigVars[i].get()
                        else:
                            self.autoCenterConfigs[i] = self.autoconfigVars[i].get()
                    self.frame,mask = FrameOperations.maskImage(self.frame,**self.autoMaskConfigs)
                    self.frame,centerpoint = FrameOperations.findCentre(self.frame,mask,**self.autoCenterConfigs)
                    self.render(self.frame,mask)
                self.root.after(self.delay,self.update)
            else:
                self.cap.release()
                self.update()
        else:
            self.render()
            self.root.after(self.delay,self.update)

    def get_pixel(self,event:tk.Event):
        if not self.frame is None:
            scale = self.frame.shape[0]/self.vidheight
            if self.numframes>1:
                return int(event.y*scale*2),int(event.x*scale*2)
            else:
                return int(event.y*scale),int(event.x*scale)
    def get_pixel_colour (self,event:tk.Event,colourtype = cv2.COLOR_BGR2HSV,drawcirlce = False):
        y,x = self.get_pixel(event)
        if drawcirlce:
            color =  self.frame[y][x]
            color = ( int (color [ 0 ]), int (color [ 1 ]), int (color [ 2 ]))##this is cursed but for some reason it wroks
            cv2.circle(self.frame, (x,y), 20,color, -1)##so you're sure you've selected the right one
        #print("HSV:",cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)[int(event.y*scale)][int(event.x*scale)] )
        colour = cv2.cvtColor(self.frame, colourtype)[y][x]
        colour = ( int (colour [ 0 ]), int (colour [ 1 ]), int (colour [ 2 ]))
        return colour

    def setSize(self,event:tk.Event):
        if self.numframes==2:
            event.width = event.width*2
        if self.vidwidth== 0 or self.vidheight == 0:
            self.vidwidth= event.width
            self.vidheight = event.height
        else:
            newratio = event.width/event.height
            oldratio = self.vidwidth/self.vidheight
            if newratio>oldratio:#the Relative width is larger than needed
                self.vidwidth = self.vidwidth *(event.height/self.vidheight)# so scale the width the same as the height
                self.vidheight = event.height
            else:
                self.vidheight = self.vidheight *(event.width/self.vidwidth)
                self.vidwidth= event.width
            #self.canvas.configure(width = self.vidwidth, height = self.vidheight)
    def render(self,*frames,numframes = 1):
        if self.cap.isOpened()== False:
            ##ratio = self.vidheight/self.vidwidth
            frame = np.random.randint(0,255,(200, 200,3),np.uint8)
            frame = cv2.resize(frame,(int(self.vidwidth), int(self.vidheight)), interpolation = cv2.INTER_CUBIC)
            self.photo = PIL.ImageTk.PhotoImage(PIL.Image.fromarray(frame))
            self.canvas.create_image(0, 0, image = self.photo, anchor = tk.NW)
        elif len(frames) == 1:
            frame = frames[0]
            width  = self.vidwidth/2 if numframes==2 else self.vidwidth
            self.numframes = numframes
            frame = cv2.resize(frame,(int(width), int(self.vidheight)), interpolation = cv2.INTER_CUBIC)
            frame =cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(frame))
            self.canvas.create_image(0, 0, image = self.photo, anchor = tk.NW)
        elif len(frames) ==2:
            frames = list(frames)
            for x,i in enumerate(frames):
                if len(i.shape) ==2:
                    frames[x]= cv2.cvtColor(i, cv2.COLOR_GRAY2BGR)
            full = np.concatenate((frames[0],frames[1]), axis=0)
            self.halfwidth=True
            self.render(full,numframes= 2)
        else:# defult to 4 for now, i see no reason to go beyond that currenlty
            frames = list(frames)
            #grey = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
            scaleddown = cv2.resize(frames[0],None,fx=1/5, fy=1/5, interpolation = cv2.INTER_CUBIC)
            while len(frames) < 4:
                frames.append(np.random.randint(0,255,scaleddown.shape,np.uint8))
                frames[-1]=cv2.resize(frames[-1],frames[0].shape[1::-1], interpolation = cv2.INTER_CUBIC)
                ##frames.append(np.zeros(frames[0].shape,np.uint8))
            for x,i in enumerate(frames):
                if len(i.shape) ==2:
                    frames[x]= cv2.cvtColor(i, cv2.COLOR_GRAY2BGR)
            top = np.concatenate((frames[0],frames[1]), axis=1)
            bottom = np.concatenate((frames[2],frames[3]), axis=1)
            full = np.concatenate((top,bottom), axis=0)
            self.render(cv2.resize(full,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC),numframes=4)
    def reload_video(self,*args):
        if "webcam" in self.inputvar.get():
            if len(self.inputvar.get()) == len("webcam"):
                name = 0
            else:
                name = int(self.inputvar.get()[len("webcam"):])
        else:
            name = self.inputvar.get()
        if self.cap !=None:
            self.cap.release()
        self.cap = cv2.VideoCapture(name)
        # Check if camera opened successfully
        if (self.cap.isOpened()== False): 
          print("Error opening video stream or file")
          self.vidwidth = 150
          self.vidheight = 100
        else:
            self.vidwidth = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)/2
            self.vidheight = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)/2
            self.xyz.frameDims = [int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
        class holder:
            def __init__(self,width,height):
                self.width = width
                self.height = height
        if self.canvas.winfo_width()+self.canvas.winfo_height()>2:# returns 1 if unupdated
            self.setSize(holder(self.canvas.winfo_width(),self.canvas.winfo_height()))
    def drawArrows(self):
        pos = self.printer.get_current_pos()
        center = self.xyz.coroodinateToPoint(pos[:2],printerpos=pos)
        highx = self.xyz.coroodinateToPoint((pos[0]+10,pos[1]),printerpos=pos)
        highy = self.xyz.coroodinateToPoint((pos[0],pos[1]+10),printerpos=pos)
        cv2.arrowedLine(self.frame,center,highx,(0,0,255),2)
        cv2.arrowedLine(self.frame,center,highy,(0,255,0),2)
    def drawCenter(self):
        pos = self.printer.get_current_pos()
        center = pos[:2]+self.xyz.centerScreenOffset
        center = self.xyz.coroodinateToPoint(center,pos)
        cv2.circle(self.frame,center,10,(0,255,255),-1)
    def getDims(self,frame):
        return self.frame.shape[1::-1]
    def load_tkinter(self):
        self.canvas = tk.Canvas(self.root)
        self.canvas.grid(sticky="NSEW")
        self.root.columnconfigure(0,weight = 5)
        self.root.rowconfigure(0,weight = 1)
        self.canvas.bind("<Configure>",self.setSize)
        self.menu = tk.Frame(self.root)
        self.menu.grid(row = 0,column = 1,sticky = "NSEW")
        #self.root.columnconfigure(1,weight = 1)
        webcamindexes = FrameOperations.getCameraIndexes()
        if len(webcamindexes)<=1:
            inputlist = ["webcam"]
        else:
            inputlist = [f"webcam {i}" for i in webcamindexes]
        for entry in scandir('.'):
            if entry.is_file():
                if entry.name.endswith(".mp4"):
                    inputlist.append(entry.name)
        self.inputvar = tk.StringVar()
        self.inputs = tk.OptionMenu(self.menu,self.inputvar,*inputlist )
        self.inputvar.trace("w", self.reload_video)
        self.cap = None
        self.inputvar.set(inputlist[0])
        self.inputs.grid(row = 0,column = 0,sticky= "NSEW")
        #self.inputs.configure(state='disable')
        self.make_controls()
        self.loctext = tk.StringVar()
        self.loclable =tk.Label(self.menu, textvariable=self.loctext)
        self.loclable.grid(row = 2, sticky = "NSEW")
        self.make_manuals()
        self.make_HSVs()
        self.make_Configurations()
        self.make_speeds()
        self.make_auto_config()
        self.make_modes()
        self.make_camera_controls()
        #self.printreadbutton = tk.Button(self.menu,text= "Print Responce",command = self.printer.readresponce)
        #self.printreadbutton.grid(row = 8,sticky = "NSEW")
        self.configure_colours_recusrive(self.menu)
    
    def make_modes(self):
        self.mode = tk.IntVar()
        self.modesframe = tk.Frame(self.menu)
        for i in range(2):
            self.modesframe.columnconfigure(i,weight = 1)
        self.modesframe.grid(row = 4,column = 0,sticky= "NSEW")
        self.maunalbutton =tk.Radiobutton(self.modesframe,var = self.mode,value = 0,text="Manual Control")
        self.autobutton =  tk.Radiobutton(self.modesframe,var = self.mode,value = 1,text="Auto")
        self.calibatebutton =  tk.Radiobutton(self.modesframe,var = self.mode,value = 2,text="Calibate")
        self.gethsvbutton = tk.Radiobutton(self.modesframe,var = self.mode,value = 3,text="Get HSV")
        self.configautobutton = tk.Radiobutton(self.modesframe,var = self.mode,value = 4,text="Configure automatic mode")
        self.maunalbutton.grid(sticky = "NW")
        self.autobutton.grid(row = 0,column= 1,sticky = "NW")
        self.calibatebutton.grid(row = 1,column= 0,sticky = "NW")
        self.gethsvbutton.grid(row = 1,column= 1,sticky = "NW")
        self.configautobutton.grid(row = 2,column = 0,columnspan = 2,sticky = "NW")
        self.mode.trace("w", self.change_modes)
        self.change_modes()
    def change_modes(self,*args):
        num= self.mode.get() 
        if num ==0:
            self.canvas.bind("<Motion>",self.renderMousePos)
            self.canvas.bind("<Leave>",self.stopRenderText)
            self.canvas.bind("<ButtonPress-1>",self.goToClick)
            self.manualControlsFrame.grid()
            self.speedFrame.grid()
        else:
            self.canvas.unbind("<ButtonPress-1>")
            self.manualControlsFrame.grid_remove()
            self.speedFrame.grid_remove()
            self.canvas.unbind("<Motion>")
        if num ==1:
            self.dissable(self.controls_frame)
        else:
            self.enable(self.controls_frame)
            self.auto_path =None
        if num== 2:
            self.configFrame.grid()
        else:
            self.configFrame.grid_remove()
        if num == 3:
            self.HSVframe.grid()
            self.canvas.bind("<ButtonPress-1>",self.handelHSVs)
            self.canvas.bind("<B1-Motion>",self.handelHSVs)
        else:
            self.HSVframe.grid_remove()
            self.canvas.unbind("<B1-Motion>")
            self.canvas.unbind("<Leave>")
        if num==4:
            self.autoModeConfigFrame.grid()
            self.canvas.bind("<ButtonPress-1>",self.update_auto_mask_colour)
        else:
            self.autoModeConfigFrame.grid_remove()
    def make_auto_config(self):
        self.autoModeConfigFrame =tk.Frame(self.menu)
        self.autoModeConfigFrame.grid(row = 5, column = 0, sticky = "NSEW")
        self.autoMaskConfigs = {"colour":(0, 255, 150),"tollerance":10, "closings":1,"openings":1,"dilateions":0}
        self.autoCenterConfigs = {"minradius":100}
        self.autoconfigWidgets = {}
        self.autoconfigVars= {}
        self.autoconfigWidgets["colour"] = tk.Button(
            self.autoModeConfigFrame,
            text="Colour",
            background='#%02x%02x%02x' % tuple((cv2.cvtColor(np.uint8([[self.autoMaskConfigs["colour"]]]),cv2.COLOR_HSV2RGB))[0][0]),
            command=lambda:print("HSV: "+ str(self.autoMaskConfigs["colour"])))
        self.autoconfigWidgets["colour"].grid(sticky ="NSEW")
        for i in("tollerance","closings","openings","dilateions","minradius"):
            to = 255 if i=="tollerance" else 10
            to = 500 if i=="minradius" else to
            dic = self.autoCenterConfigs if i =="minradius" else self.autoMaskConfigs
            self.autoconfigVars[i] =tk.IntVar(value= dic[i])
            self.autoconfigWidgets[i] = tk.Scale(self.autoModeConfigFrame,variable=self.autoconfigVars[i],to = to,orient= HORIZONTAL,label=i,length=150)
            self.autoconfigWidgets[i].grid(sticky= "NW")
       # self.autoconfigWidgets["Colour Lable"]
    def update_auto_mask_colour(self,event:tk.Event):
        hsv = self.get_pixel_colour(event)
        self.autoMaskConfigs["colour"] =hsv
        self.autoconfigWidgets["colour"].config(bg = '#%02x%02x%02x' % self.get_pixel_colour(event,cv2.COLOR_BGR2RGB))
    def make_manuals(self):
        self.manualControlsFrame =tk.Frame(self.menu)
        self.manualControlsFrame.grid(row = 5,sticky="NSEW")
        self.manualControlsFrame.grid_columnconfigure(0,weight=1)
        self.manualControlsFrame.grid_columnconfigure(1,weight=1)
        self.setboxbutton = tk.Button(self.manualControlsFrame,text ="set box pos",command = self.set_box_pos)
        self.setboxbutton.grid(row = 0,columnspan=2,sticky = "NSEW")
        self.useFrameCenter= tk.IntVar(value = 0)
        self.useClawCenterButton =tk.Radiobutton(self.manualControlsFrame,variable=self.useFrameCenter,value=0,text = "Claw Center")
        self.useFrameCenterButton =tk.Radiobutton(self.manualControlsFrame,variable=self.useFrameCenter,value=1,text = "Frame Center")
        self.useClawCenterButton.grid(row =1,column=0,sticky = "NW")
        self.useFrameCenterButton.grid(row = 1,column=1,sticky = "NW")
    def make_camera_controls(self):
        self.camracontrolframe = tk.Frame(self.menu)
        self.camracontrolframe.grid(row = 10,column = 0,sticky = "NSEW")
        self.paused = tk.IntVar()
        self.pausebox = tk.Checkbutton(self.camracontrolframe,text = "Pause Camera",variable=self.paused)
        self.pausebox.grid(sticky="SW")
        self.blured = tk.IntVar()
        self.blurbox = tk.Checkbutton(self.camracontrolframe,text = "Blur Camera",variable=self.blured)
        self.blurbox.grid(row = 0,column=1,sticky="SW")
    def make_controls(self):
        self.controls_frame = tk.Frame(self.menu)
        self.controls_frame.grid(row = 1,column = 0,sticky = "NSEW")
        self.buttons = {}
        self.buttonstates = {}
        def makeFunctions(i):
            def onpress(event):
                self.buttonstates[i] =True
            def onrelease(event):
                self.buttonstates[i] =False
            return onpress,onrelease
        namestokeys = {"right" :"Right","left":"Left","forward":"Up","backward":"Down","raise":"w","lower":"s","open":"o","close":"c"}
        for i in ("forward","backward","left","right","raise","lower","open","close"):
            onpress,onrelease = makeFunctions(i)
            self.buttons[i] = tk.Button(self.controls_frame,text = i)
            self.buttons[i].bind('<ButtonPress-1>',onpress)
            self.buttons[i].bind('<ButtonRelease-1>',onrelease)
            self.root.bind(f"<KeyPress-{namestokeys[i]}>",onpress)
            self.root.bind(f"<KeyRelease-{namestokeys[i]}>",onrelease)
        self.buttons["forward"].grid(row = 0,column = 1,columnspan = 2,sticky = "NSEW")
        self.buttons["backward"].grid(row = 2,column =1,columnspan = 2,sticky = "NSEW")
        self.buttons["left"].grid(row = 1,column = 0,columnspan = 2,sticky = "NSEW")
        self.buttons["right"].grid(row = 1,column = 2,columnspan = 2,sticky = "NSEW")
        self.buttons["raise"].grid(row = 3,column = 0,sticky = "NSEW")
        self.buttons["lower"].grid(row = 3,column = 1,sticky = "NSEW")
        self.buttons["open"].grid(row = 3,column = 2,sticky = "NSEW")
        self.buttons["close"].grid(row = 3,column = 3,sticky = "NSEW")
        self.homebutton = tk.Button(self.controls_frame,text = "Home All", command = self.printer.home)
        self.homebutton.grid(row = 5,column = 0,columnspan=4,sticky = "NSEW")
        for i in range(4):
            self.controls_frame.grid_columnconfigure(i,weight = 1)
    def update_pos(self):
        #print(self.buttonstates.items())
        plusses = ("right","forward","raise","open")
        minuses = ("left","backward","lower","close")
        key = ("x","y","z","claw")
        temp = {}
        for x,i in self.buttonstates.items():
            if i:
                if x in plusses:
                    temp[key[plusses.index(x)]]=self.speed.get()
                elif x in minuses:
                    temp[key[minuses.index(x)]]=-self.speed.get()
        self.printer.add_next(**temp,mode = "Relative")
                       
    def set_box_pos(self):
        if self.useFrameCenter.get():
            if self.xyz.isCalibrated:
                self.xyz.boxpos = self.xyz.pointToCoroodinate([i/2 for i in self.xyz.frameDims],self.printer.get_current_pos(),True)
            else:
                messagebox.showerror(mesage = "Please Configure first")
        else:
            self.xyz.boxpos = self.printer.get_current_pos()[:2]
        print(f"Set Box Pos: {self.xyz.boxpos}")
    def on_closing(self):
        self.cap.release()
        self.root.destroy()

    def renderMousePos(self,event:tk.Event):
        tup = self.get_pixel(event)
        if tup is None:
            self.rendertextcommand = None
            return
        y,x = tup
        pos = self.xyz.pointToCoroodinate((x,y),self.printer.get_current_pos(),centerofscreen=self.useFrameCenter.get())
        pos = [i.round(2) for i in pos]
        def closeure():
            self.frame =cv2.putText(self.frame, str(pos), (x,y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
        self.rendertextcommand = closeure
    def stopRenderText(self,event:tk.Event):
        self.rendertextcommand = None
    def goToClick(self,event:tk.Event):
        if self.xyz.isCalibrated:
            y,x = self.get_pixel(event)
            coordinate= self.xyz.pointToCoroodinate((x,y),self.printer.get_current_pos(),centerofscreen=self.useFrameCenter.get())
            self.printer.add_next(x= coordinate[0],y=coordinate[1])
        else:
            messagebox.showinfo(message="Please Calibrate the Camera first")
    def make_HSVs(self):
        self.HSVframe = tk.Frame(self.menu)
        self.HSVframe.grid(row = 5,sticky= "NSEW")
        self.tollerance = tk.IntVar(value = 10)
        for i in range(3):
            self.HSVframe.grid_columnconfigure(i,weight = 0)
        self.tolleranceSlider = tk.Scale(self.HSVframe,to=255,variable=self.tollerance,orient=tk.HORIZONTAL,label="Tollerance Value",length=150)
        self.tolleranceSlider.grid(row = 0,columnspan = 3)
        self.HSVselector = tk.StringVar(value="center")
        class HSVdisplayer:
            def __init__(self,master: tk.Frame,name,variable,colour=(0,0,0)):
                self.colour = colour
                self.name =name
                self.selector = tk.Radiobutton(master,variable=variable,value=name,text=name)
                converted =tuple((cv2.cvtColor(np.uint8([[colour]]),cv2.COLOR_HSV2RGB))[0][0])
                self.colourbox = tk.Button(master,background='#%02x%02x%02x' % converted,height=1,width=1,command= lambda: print(f"{self.name}: {self.colour}"))#,highlightthickness=2,highlightcolor="Black")
                self.render = tk.IntVar(value= 0)
                self.renderbox = tk.Checkbutton(master,var= self.render)
        self.HSVdisplayers = {}
        for i in self.xyz.HSVs.keys():
            self.HSVdisplayers[i] = HSVdisplayer(self.HSVframe,i,self.HSVselector,self.xyz.HSVs[i])
            self.HSVdisplayers[i].selector.grid(row= len(self.HSVdisplayers),column = 0,sticky= "NW")
            self.HSVdisplayers[i].colourbox.grid(row= len(self.HSVdisplayers),column = 1)
            self.HSVdisplayers[i].renderbox.grid(row= len(self.HSVdisplayers),column = 2)
        self.calibrationpointstrings = [tk.StringVar(self.HSVframe,value = "0",name="xDotoffset"),tk.StringVar(self.HSVframe,value ="0",name="yDotoffset")]
        for i in self.calibrationpointstrings:
            i.trace("w",self.update_dot_offset)
        self.calibrationboxlables = []
        self.calibrationboxlables.append(tk.Label(self.HSVframe,text="Dot x offset"))
        self.calibrationboxlables.append(tk.Label(self.HSVframe,text="Dot y offset"))
        for x,i in enumerate(self.calibrationboxlables):
            i.grid(row = len(self.HSVdisplayers)+x+1,column = 0,sticky = "NW")
        self.calibrationboxes = []
        self.calibrationboxes.append(tk.Entry(self.HSVframe,textvariable=self.calibrationpointstrings[0],width=5))
        self.calibrationboxes.append(tk.Entry(self.HSVframe,textvariable=self.calibrationpointstrings[1],width=5))
        for x,i in enumerate(self.calibrationboxes):
            i.grid(row = len(self.HSVdisplayers)+x+1,column = 1,sticky = "NW")
        self.root.bind_class("Frame","<Button-1>", lambda event:event.widget.focus_set())# allow for the boxes to be clicked out of
    def update_dot_offset(self,varname,*args):
        try:
            if varname =="xDotoffset":
                string = self.calibrationpointstrings[0].get()
                if string =="":
                    string ="0"
                num = float(string)
                self.xyz.dot_offset[0] = num
            else:
                string = self.calibrationpointstrings[1].get()
                if string =="":
                    string ="0"
                num = float(string)
                self.xyz.dot_offset[1] = num
        except ValueError:
            print("Invalid value entered for "+varname)
        

    def handelHSVs(self,event:tk.Event):
        self.xyz.HSVs[self.HSVselector.get()] = self.get_pixel_colour(event)
        self.HSVdisplayers[self.HSVselector.get()].colour = self.get_pixel_colour(event)
        self.HSVdisplayers[self.HSVselector.get()].colourbox.configure(bg = '#%02x%02x%02x' % self.get_pixel_colour(event,cv2.COLOR_BGR2RGB))
    def make_Configurations(self):
        self.configFrame = tk.Frame(self.menu)
        self.configFrame.grid(row = 5,column = 0,sticky="NSEW")
        self.configReadOut = {}
        for x,i in enumerate(("offset","angle","scalefactor","yoffset","centerScreenOffset")):
            self.configReadOut[i] = tk.Label(self.configFrame,text=i)
            self.configReadOut[i].grid(row = x,column = 0,sticky = "nw",columnspan= 2)
        self.saveConfigButton = tk.Button(self.configFrame,text="Good Read",command=self.xyz.saveRead)
        self.saveConfigButton.grid(row = 5,column = 0,sticky = "nw")
        self.calculateYBool = tk.IntVar()
        self.calculateYBoolBox = tk.Checkbutton(self.configFrame,variable=self.calculateYBool,text="Calc. Y offset")
        self.calculateYBoolBox.grid(row = 5,column=1,sticky="nw")
        self.useAverage = tk.IntVar()
        self.useAverageButton = tk.Checkbutton(self.configFrame,variable=self.useAverage,text = "Calcuate Average Point",command=self.xyz.resetAverage)
        self.useAverageButton.grid(row = 6,column= 0,columnspan=2, sticky="nw")
    def update_config_readout(self):
        for x,i in self.configReadOut.items():
            value = self.xyz.__getattribute__(x)
            try:
                value =[round(i,2) for i in value]
            except TypeError: #not iterable:
                pass
                value = round(value,5)
            i.config(text = f"{x}: {value}")       
    def enable(self,frame):
        for child in frame.winfo_children():
            child.configure(state='normal')
    def dissable(self,frame):
        for child in frame.winfo_children():
            child.configure(state='disabled')
            ##if isinstance(child,tk.Frame,tk.Scale):
            ##    self.disable(child)
            ##else:
    def configure_colours_recusrive(self,widget:tk.Widget):
        background= widget.cget('bg')
        if background == "SystemButtonFace" or background == "#d9d9d9":
            widget.configure (bg = BACKGROUNDCOLOUR)
        elif background=="SystemWindow":
            widget.configure (bg = TEXTBACKGORUNDCOLOUR)
        if isinstance(widget,tk.Frame):
            for child in widget.winfo_children():
               self.configure_colours_recusrive(child)
        else:
            widget.configure(fg =FOREGROUNDCOLOUR,highlightbackground =BACKGROUNDCOLOUR)
            if isinstance(widget,(tk.Checkbutton,tk.Radiobutton)):
                widget.config(selectcolor=SELECTCOLOUR,activebackground=ACTIVEBACKGROUNDCOLOUR)
            elif isinstance(widget,(tk.Button,tk.Scale,tk.OptionMenu)):
                widget.config(activebackground=ACTIVEBACKGROUNDCOLOUR)
            elif isinstance(widget,tk.Entry):
                widget.configure(insertbackground =FOREGROUNDCOLOUR)
    def make_speeds(self):
        self.speedFrame= tk.Frame(self.menu)
        self.speedFrame.grid(row = 9,sticky="NSEW")
        self.speed = tk.DoubleVar(value=1)
        self.speedSlider =tk.Scale(self.speedFrame,to = 20,variable=self.speed,orient=tk.HORIZONTAL,resolution=0.01,label="Control Speed",length=150) 
        self.speedSlider.grid(row =0,column = 0,sticky = "NW")
        self.printerpercentspeed = tk.IntVar(value=100)
        self.printerspeedslider = tk.Scale(self.speedFrame,to = 300,variable=self.printerpercentspeed,orient=tk.HORIZONTAL,label="Printer Speed Percent",length=150)
        self.printerspeedslider.grid(row = 1,column= 0,sticky="NW")
        self.sendspeedbutton = tk.Button(self.speedFrame,text="Override Printer Speed",command = self.overrideSpeed)
        self.sendspeedbutton.grid(row = 2,column = 0,sticky="NW")
    def overrideSpeed(self):
        self.printer.set_percent_speed(self.printerpercentspeed.get())

if __name__ == "__main__":     
    app = App()

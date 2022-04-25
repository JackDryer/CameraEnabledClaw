import re
from typing import List,Generator
import cv2
import numpy as np
import warnings

import FrameOperations
CLAWOPEN=160
CLAWCLOSED = 32
class XYZ():
    def __init__(self):
        self.boxpos = [10,10]
        self.angle = 0
        self.scalefactor = 1
        self.yoffset= 0# seperate from self.offset as its used in different calcuations
        self.offset = [0,0]
        self.dot_offset= [0,0]
        self.frameDims= [100,100]# will be changed 
        self.printerdims = [100,100,100]
        self.HSVs = {"center":(104, 255, 120),"x":(173, 231, 116),"y":(67, 255, 82)}
        self.centerScreenOffset =[0,0]
        self.isCalibrated = False
        self.resetAverage()
    def configurationModeUpdate(self,frame,pos,tollerance = 10,useAverage = False,calculateYOffset =False,returnMasks = False):
        "takes in a frame, if it can find a refernce it will configure the camera offset values and return true"
        centers =[]
        order = self.HSVs["center"],self.HSVs["x"],self.HSVs["y"]
        frame,masks = FrameOperations.findMasks(frame,*order,tolllerance=tollerance)
        for i in masks:
            frame,center = FrameOperations.findCentreLargest(frame,i)
            if not center is None:
                centers.append(center)
        validread = len(centers)==3
        if validread:
            if useAverage:
                self.averageCenter =  self.averageCenter*self.averageCount
                self.averageCenter += centers
                self.averageCount += 1
                self.averageCenter = self.averageCenter/self.averageCount
                centers = self.averageCenter
            if calculateYOffset:
                self.calculateYOffset(*self.savedRead,centers[0],centers[1],pos[2])
            self.configureCameraCalibation(*centers,pos)
            self.pastRead = centers[0],centers[1],pos[2]
            self.centerScreenOffset= np.array(self.pointToCoroodinate([i/2 for i in self.frameDims],pos))-pos[:2]
        if returnMasks:
            return validread ,masks
        else:
            return validread
    def getMaxViewDims(self,height):
        maxx,maxy = np.array(self.frameDims)*self.scalefactor*(height+self.yoffset)
        maxx,maxy = FrameOperations.rotatedRectWithMaxArea(maxx,maxy,self.angle)
        return maxx,maxy
    def createViewRenderer(self,maxx,maxy,height):
        ##create renderer
        # angle = self.angle%(np.pi/2)
        # if (angle>np.pi/4 and self.angle<0):
        #     maxx,maxy = maxy,maxx
        points = [(0,0),(0,maxy),(maxx,maxy),(maxx,0)]
        # points = FrameOperations.rotatePoints(points,angle,(0,maxy))  
        # points = points/(self.scalefactor*(height+self.yoffset))
        # ydiff = points[0][1]
        # points -=[0,ydiff]
        points = [self.coroodinateToPoint(i,(maxx/2,maxy/2,self.printerdims[2])) for i in points]
        def drawer(frame):
            return cv2.polylines(frame,np.int32([points]),True,(255,255,0),3)
        return drawer
    def create_search_path(self,maxx,maxy)->Generator:
        "does not require any thing to be sent into it, genorates a path to march a box accross the printer space"
        yield 0,0,self.printerdims[2],CLAWOPEN
        for y in range(0,self.printerdims[1],int(maxy)):
            for x in range(0,self.printerdims[0],int(maxx)):
                yield x,y ,None,None
            yield self.printerdims[0],y,None,None
        for x in range(0,self.printerdims[0],int(maxx)): #ensue edges have been scanned
            yield x,self.printerdims[1] ,None,None
        yield self.printerdims[0],self.printerdims[1],None,None
    def create_auto_path(self,frame,maskkwargs:dict,centerkwargs:dict) ->Generator:
        "creates a genorator object, takes in frame,mask from the .send fuction retruns frame, [x,y,z,claw]"
        maxx,maxy = self.getMaxViewDims(self.printerdims[2])# wants to have the maximum view for the camera
        dimentionDrawer = self.createViewRenderer(maxx,maxy,self.printerdims[2])
        detected_points = []
        for i in self.create_search_path(maxx,maxy):
            for n in range (5):# scan 5 frames per point
                frame = dimentionDrawer(frame)# draw the available viewing box
                frame =yield frame, i
                frame,mask = FrameOperations.maskImage(frame,**maskkwargs)
                frame, detectedCenterPoint = FrameOperations.findCentre(frame,mask,**centerkwargs)
                if detectedCenterPoint!= None:
                    detected_points.append(detectedCenterPoint)
            if detected_points:
                coordinate = self.calculateAverageCordiante(detected_points,i[:2]+(self.printerdims[2],),centerofscreen=True)
                detected_points = []
                for n in range (10):# 10 tries per "sure" frame
                    frame = yield frame, (coordinate +[None, None])
                    frame,mask = FrameOperations.maskImage(frame,**maskkwargs)
                    frame, detectedCenterPoint = FrameOperations.findCentre(frame,mask,**centerkwargs)#make sure you're centered
                    if detectedCenterPoint!= None:
                        detected_points.append(detectedCenterPoint)
                if detected_points:
                    coordinate = self.calculateAverageCordiante(detected_points,coordinate+[self.printerdims[2]])
                    break
        else:
            print("Path Finished without detecting point")
            return
        frame =yield frame,(coordinate +[None, None])#move to new location
        frame =yield frame,(None, None, 25,None) #lower
        frame = yield frame,(None, None, None, CLAWCLOSED) #close claw
        #yield None,None, 100, None
        frame =yield frame,(self.boxpos +[100,None])
        yield frame,(None,None,None,CLAWOPEN)
    def calculateAverageCordiante(self,detected_points,pos,centerofscreen=False):
        averageCenterPoint = np.array([0,0])
        for i in detected_points:
            averageCenterPoint= averageCenterPoint+ i
        averageCenterPoint = averageCenterPoint/len(detected_points)
        return [round(i) for i in self.pointToCoroodinate(averageCenterPoint,pos,centerofscreen=centerofscreen)]
    def pointToCoroodinate(self,point,printerpos,centerofscreen = False):
        """desighned to work with non scaled points on the image
        
        center of the screen option allows returns the point that would put the center of the screen above the point rather than the claw itself"""
        point = self.flipPoint(point)
        framecenter = [i/2 for i in self.frameDims]
        rotated = FrameOperations.rotatePoints(point,self.angle,framecenter)
        scaled = FrameOperations.scalePoint(rotated,self.scalefactor*(printerpos[2]+self.yoffset),framecenter)
        translated = scaled + printerpos[:2]
        translated = translated + self.offset
        if centerofscreen:
            translated = translated-self.centerScreenOffset
        return list(translated)
    def coroodinateToPoint(self,coordinate,printerpos):
        if len(coordinate) ==3:
            coordinate =coordinate[:2]
        coordinate = np.array(coordinate)
        translated =coordinate-self.offset
        translated =translated - printerpos[:2]
        #print((self.scalefactor*pos[2]))
        frameCenter = [i/2 for i in self.frameDims]
        scaled = FrameOperations.scalePoint(translated,1/(self.scalefactor*(printerpos[2]+self.yoffset)),frameCenter)
        rotated =FrameOperations.rotatePoints(scaled,-self.angle,frameCenter)
        return self.flipPoint(rotated)
    def configureCameraCalibation(self,pointunderclaw,highxpoint,highypoint,pos):
        """from these 3 points calcuate the scale,rotation and offset values required
         
         after calibation self.pointToCoroodinate(pointunderclaw) should == self.pos
         high x point should be 10mm more x, high y shoud be 10mm more y
         
         high y point isn't nesicary but allows for checking ect
         assumes that the camera is at the same y as the gcode
        """
        pointunderclaw = self.flipPoint(pointunderclaw)
        highxpoint = self.flipPoint(highxpoint)
        highypoint = self.flipPoint(highypoint)
        pointunderclaw = np.array(pointunderclaw)
        unaffected = pointunderclaw.copy()
        highxpoint = np.array(highxpoint)
        highypoint = np.array(highypoint)
        distance= FrameOperations.distanceBetween(pointunderclaw,highxpoint)
        #print("x",distance)
        #print("y",np.linalg.norm(np.linalg.norm(pointunderclaw-highypoint)))
        if pos[2] ==0:
            warnings.warn("0 Height")
            return
        expecteddisstance = 10
        if distance==0:
            warnings.warn("0 Point Distance Detected")
            return
        self.scalefactor = (expecteddisstance/distance)/(pos[2]+self.yoffset)
        # set points to be in the correct scale
        frameCenter = [i/2 for i in self.frameDims]
        pointunderclaw = FrameOperations.scalePoint(pointunderclaw,self.scalefactor*(pos[2]+self.yoffset),frameCenter)
        highxpoint = FrameOperations.scalePoint(highxpoint,self.scalefactor*(pos[2]+self.yoffset),frameCenter)
        highypoint = FrameOperations.scalePoint(highypoint,self.scalefactor*(pos[2]+self.yoffset),frameCenter)
        difference = highxpoint-pointunderclaw # now rotated around 0,0
        difference = difference/expecteddisstance #now all values should be between -1 and 1, for sin and cos
        xanlge = np.arcsin(difference[0])
        yanlge = np.arccos(difference[1])
        if xanlge>0:
            self.angle=yanlge
        else:
            self.angle=(2*np.pi)-yanlge
        self.angle = self.angle-(np.pi/2)
        if np.isnan(self.angle):
            warnings.warn("nan detected")
            #print(difference,pointunderclaw,highxpoint,highypoint)
            self.angle =np.pi
        pointunderclaw =FrameOperations.rotatePoints(unaffected,self.angle,frameCenter)
        pointunderclaw = FrameOperations.scalePoint(pointunderclaw,self.scalefactor*(pos[2]+self.yoffset),frameCenter)
        pointunderclaw = pointunderclaw +self.dot_offset#alighn with real claw incase of offset
        self.offset = -pointunderclaw
        self.isCalibrated =True
    def calculateYOffset(self,centerpoint1, offsetpoint1,height1,centerpoint2, offsetpoint2,height2):
        line1 = FrameOperations.distanceBetween(centerpoint1,offsetpoint1)
        line2 =FrameOperations.distanceBetween(centerpoint2,offsetpoint2)
        if (line2-line1) == 0 or height1==height2: #identlical distances, would produce NaN
            warnings.warn("Invalid Y offet reading detected")
            self.yoffset = 0
        else:
            self.yoffset= ((height1*line1)-(height2*line2))/(line2-line1)
    def flipPoint(self,point)->List:
        try:
            point = [int (i) for i in point]# ensure is a list of cords
        except ValueError:
            warnings.warn("Unconvertable value found in Flip point",point)
            return point
        point[1] = self.frameDims[1]-point[1]
        return point
    def resetAverage(self):
        self.averageCenter = np.array([[0,0],[0,0],[0,0]])
        self.averageCount = 0
    def saveRead (self):
        self.savedRead = self.pastRead

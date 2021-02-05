import math 
import matplotlib.pyplot as plt
import numpy as np
import map2d 
import time
class Point:
    def  __init__(self,x,y):
        self.x=x
        self.y=y

 
class aligned_lines:
    def  __init__(self,alinged_distance,line_number):
        self.alinged_distance=alinged_distance
        self .line_number=line_number
        self.origin_line=[]
        self.linelist=[]
        self.best_line=0
        self.breakpoint_List=[]
        self.step_length=0.4

    def Get_Line_theta(self,  i):
        if self.origin_line==[]:
            print('origin line is not initialed!')
        else:            
            self.origin_line_length=len(self.origin_line)
            if(i < self.origin_line_length-1):
                theta= math.atan2(self.origin_line[i+1].y-self.origin_line[i].y, self.origin_line[i+1].x-self.origin_line[i].x)
                if(self.origin_line[i+1].y-self.origin_line[i].y<0.0):
                    theta = theta + np.sign(self.origin_line[i+1].x-self.origin_line[i].x) * math.pi          
            else:
                theta=math.atan2(self.origin_line[self.origin_line_length-1].y-self.origin_line[self.origin_line_length-2].y, self.origin_line[self.origin_line_length-1].x-self.origin_line[self.origin_line_length-2].x)
                if(self.origin_line[self.origin_line_length-1].y-self.origin_line[self.origin_line_length-2].y<0.0):
                    theta = theta + np.sign(self.origin_line[self.origin_line_length-1].x-self.origin_line[self.origin_line_length-2].x) * math.pi   
            return theta
    
    def get_breakpoint(self):
        self.breakpoint_List.append(0)
        for i in range(1,len(self.origin_line)-1):
            if abs(self.Get_Line_theta(i-1)-self.Get_Line_theta(i)) > 45.0*math.pi/180.0 and abs(self.Get_Line_theta(i-1)-self.Get_Line_theta(i))<135.0*math.pi/180.0:
                self.breakpoint_List.append(i)                
        self.breakpoint_List.append(len(self.origin_line)-1)

    def Cal_Aligned_Line(self):
        self.get_breakpoint()
        for i in range( int(self.line_number/2)): 
            line_right=[]
            line_left=[]
            forward_index= int(self.alinged_distance* (i + 1 ) /self.step_length) 
            
            if forward_index<self.breakpoint_List[1]-forward_index:
                min_index=forward_index
            else:
                min_index=self.breakpoint_List[1]-forward_index
            for j in range(min_index):
                new_point_right=Point(0,0)
                point=self.origin_line[j]
                theta=self.Get_Line_theta(j)
                new_point_right.x = point.x + self.alinged_distance * (i+1) * math.cos(theta-math.pi/2)
                new_point_right.y = point.y + self.alinged_distance * (i+1)* math.sin(theta-math.pi/2)
                line_right.append(new_point_right)

            for j in range(len(self.breakpoint_List)-1):
                for k in range(self.breakpoint_List[j]+forward_index,self.breakpoint_List[j+1]-forward_index+1):
                    new_point_right=Point(0,0)
                    point=self.origin_line[k]
                    theta=self.Get_Line_theta(k)
                    new_point_right.x = point.x + self.alinged_distance * (i+1) * math.cos(theta-math.pi/2)
                    new_point_right.y = point.y + self.alinged_distance * (i+1)* math.sin(theta-math.pi/2)
                    line_right.append(new_point_right)

            if self.breakpoint_List[len(self.breakpoint_List)-1]-forward_index<self.breakpoint_List[len(self.breakpoint_List)-2]+forward_index:
                max_index=self.breakpoint_List[len(self.breakpoint_List)-2]+forward_index
            else:
                max_index=self.breakpoint_List[len(self.breakpoint_List)-1]-forward_index
            for j in range(max_index,self.breakpoint_List[len(self.breakpoint_List)-1]+1):
                new_point_right=Point(0,0)
                point=self.origin_line[j]
                theta=self.Get_Line_theta(j)
                new_point_right.x = point.x + self.alinged_distance * (i+1) * math.cos(theta-math.pi/2)
                new_point_right.y = point.y + self.alinged_distance * (i+1)* math.sin(theta-math.pi/2)
                line_right.append(new_point_right)
            

            for j in range(len(self.breakpoint_List)-1):
                for k in range(self.breakpoint_List[j], self.breakpoint_List[j+1]):
                    new_point_left=Point(0,0)
                    point=self.origin_line[k]
                    theta=self.Get_Line_theta(k)
                    new_point_left.x = point.x + self.alinged_distance * (i+1) * math.cos(theta+math.pi/2)
                    new_point_left.y = point.y + self.alinged_distance * (i+1)* math.sin(theta+math.pi/2)
                    line_left.append(new_point_left)

                new_point_left=Point(0,0)
                point=self.origin_line[self.breakpoint_List[j+1]]
                theta=self.Get_Line_theta(self.breakpoint_List[j+1]-1)
                new_point_left.x = point.x + self.alinged_distance * (i+1) * math.cos(theta+math.pi/2)
                new_point_left.y = point.y + self.alinged_distance * (i+1)* math.sin(theta+math.pi/2)
                line_left.append(new_point_left)

                if j<len(self.breakpoint_List)-2:
                    t1=line_left[len(line_left)-1]
                    t2=Point(0,0)
                    theta=self.Get_Line_theta(self.breakpoint_List[j+1]+1)
                    t2.x=self.origin_line[self.breakpoint_List[j+1]].x+ self.alinged_distance * (i+1) * math.cos(theta+math.pi/2)
                    t2.y=self.origin_line[self.breakpoint_List[j+1]].y+ self.alinged_distance * (i+1) * math.sin(theta+math.pi/2)
                    distant = math.sqrt(math.pow(t2.y - t1.y, 2)+math.pow(t2.x-t1.x,2))
                    ratex = (t2.x-t1.x)/distant
                    ratey = (t2.y-t1.y)/distant
                    pointx = t1.x
                    pointy = t1.y
                    while True:
                        pointx += ratex*self.step_length
                        pointy += ratey*self.step_length
                        if ((pointx-t1.x)*(pointx-t2.x)+(pointy-t1.y)*(pointy-t2.y))> 0:                
                            break
                        else:
                            line_left.append(Point(pointx, pointy))
            self.linelist.append(line_right)
            self.linelist.append(line_left)

    def ShowLine(self):
        for line in self.linelist:
            x=[]
            y=[]
            for point in line:
                x.append(point.x)
                y.append(point.y)                               
                plt.plot(x,y, 'b-')
        x=[]
        y=[]
        for point in self.origin_line:           
            x.append(point.x)
            y.append(point.y)
        plt.plot(x,y, 'r-')
        plt.show()

    def local_planning(self,md):
        best_weight=100000.0
        for i in range(self.line_number-1):
            line=self.linelist[i]
            weight=0.0          
            for point in line:
                if(abs(int(point.x/self.step_length))<md.w/2 and abs(int(point.y/self.step_length)-md.h/4)<md.h/2):
                    obs_weight=md.data[int(md.w/2+point.x/self.step_length)][int(md.h/4+point.y/self.step_length)]
                    weight = weight +obs_weight
            #print(weight)
            if(weight<best_weight):
                best_weight=weight
                self.best_line=i
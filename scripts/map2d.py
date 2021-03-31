# coding=utf-8
from numpy import *
import math

x_shift = 0.0
y_shift = 0.0
class map2d:
  
    def __init__(self,x,y,azuith):
        self.h=40
        self.w=40
        self.data =zeros((self.h,self.w)) 
        self.obs_dis=5
        self.x=x
        self.y=y
        self.azuith=azuith
        self.mapx=[]
        self.mapy=[]
        self.vehiclex=[]
        self.vehicley=[]
    
    def cal_mapXY(self, path):
        index=0
        flag=True
        while flag and index <len(path):
            x,y=self.World2Vehicle(path[index].x,path[index].y)
            map_x=int(self.w/2+x/0.4)
            map_y=int(self.h/4+y/0.4)
            flag= not (self.isInMap(map_x,map_y))
            index=index+1
        start_Index=index-1
        if start_Index+40<len(path):
            for i in range(start_Index,start_Index+40):
                x,y=self.World2Vehicle(path[i].x,path[i].y)
                map_x=int(self.w/2+x/0.4)
                map_y=int(self.h/4+y/0.4)
                self.vehiclex.append(x)
                self.vehicley.append(y)
                self.mapx.append(map_x)
                self.mapy.append(map_y)
        else:
            for i in range(start_Index,len(path)):
                x,y=self.World2Vehicle(path[i].x,path[i].y)
                map_x=int(self.w/2+x/0.4)
                map_y=int(self.h/4+y/0.4)
                self.vehiclex.append(x)
                self.vehicley.append(y)
                self.mapx.append(map_x)
                self.mapy.append(map_y)
            for j in range(len(path),start_Index+40):
                x,y=self.World2Vehicle(path[len(path)-1].x,path[len(path)-1].y)
                map_x=int(self.w/2+x/0.4)
                map_y=int(self.h/4+y/0.4)
                self.vehiclex.append(x)
                self.vehicley.append(y)
                self.mapx.append(map_x)
                self.mapy.append(map_y)
        

    def World2Vehicle(self, world_x=0.0, world_y=0.0):
        azuith = self.azuith*math.pi/180.0
        diff_x = world_x - (self.x + x_shift)
        diff_y = world_y - (self.y + x_shift)
        x_ret = diff_x * math.cos(azuith) - diff_y * math.sin(azuith)
        y_ret = diff_x * math.sin(azuith) + diff_y * math.cos(azuith)
        return x_ret, y_ret

    def World2Map(self,world_x,world_y):
        v_x,v_y=self.World2Vehicle(world_x,world_y)
        return (int(self.w/2+v_x/0.4),int(self.h/4+v_y/0.4))

    def setObsWeight(self,x,y):
        for i in range(0, self.w):
                for j in range(0, self.h):
                    if abs(x - i) + abs(y - j) < self.obs_dis:
                        self.data[i][j]=10
        

    def isInMap(self,x,y):
        if (x < 0 or x > self.h - 1) or (y < 0 or y > self.w - 1):
            return False
        else:
            return True

    def isPass(self, point):
        if not self.isInMap(point) :
            return False
        else:
            if self.data[point.x][point.y] == 0:
                return True
            else:
                return False

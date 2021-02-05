import math

class Point:
    def __init__(self, x=0, y=0,point_property=0):
        self.x = x
        self.y = y
        self.point_property=point_property


target_point = []
task_point=[]
path_point = []
x=[]
y=[]


def ReadTarget(path):
    with open(path, "r") as f:
        line = f.readline()
        while line != "":
            lines = line.split(',')
            x = float(lines[0])	    
            y = float(lines[1])
            point_property=float(lines[2])
            p = Point(x, y,point_property)
            target_point.append(p)
            if(p.point_property==1):
                task_point.append(p)
            line = f.readline()


def PathGeneration(target_point, gap):
    for index in range(len(target_point)-1):
        t1,t2 = target_point[index],target_point[index+1]
        distant = math.sqrt(math.pow(t2.y-t1.y,2)+math.pow(t2.x-t1.x,2))
        ratex = (t2.x-t1.x)/distant
        ratey = (t2.y-t1.y)/distant
        pointx = t1.x
        pointy = t1.y
        path_point.append(t1)
        x.append(pointx)
        y.append(pointy)
        while True:
            pointx += ratex*gap
            pointy += ratey*gap
            if ((pointx-t1.x)*(pointx-t2.x)+(pointy-t1.y)*(pointy-t2.y))> 0:                
                break
            else:
                path_point.append(Point(pointx, pointy))
                x.append(pointx)
                y.append(pointy)



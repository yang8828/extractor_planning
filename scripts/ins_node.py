import comm
import path_generation
import math
import map2d
import Astar
import aligned_line
import numpy as np
import matplotlib.pyplot as plt
present_point_index=0
look_forward_index = 8

class VehicleState:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


def calc_target_index(x, y, pathpoint):
    dx = [x - ic.x for ic in pathpoint]
    dy = [y - ic.y for ic in pathpoint]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0
    Lf = look_forward_index * 0.4

    while Lf > L:
        dx = pathpoint[(ind + 1)%len(pathpoint)].x - pathpoint[ind%len(pathpoint)].x
        dy = pathpoint[(ind + 1)%len(pathpoint)].y - pathpoint[ind%len(pathpoint)].y
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind%len(pathpoint)


def cal_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


def stateLoop(x, y, state):
    if cal_distance(x, y, path_generation.task_point[state].x, path_generation.task_point[state].y) < 4.0:
        state = state + 1
    return state % len(path_generation.task_point)

def get_theta(x,y):
    x=x+0.00001
    y=y+0.00001
    theta = math.atan(x/y)*180.0/math.pi
    if(y < 0.0):
        theta = theta + np.sign(x) * 180.0 
    return theta

def Get_Line_theta(i):
    max_number=len(path_generation.path_point)
    if(i < max_number-6):
        vehiclex=path_generation.path_point[i+6].x-path_generation.path_point[i].x
        vehicley=path_generation.path_point[i+6].y-path_generation.path_point[i].y       
    else:         
        vehiclex=path_generation.path_point[i].x-path_generation.path_point[i-6].x
        vehicley=path_generation.path_point[i].y-path_generation.path_point[i-6].y    
    theta = get_theta(vehiclex,vehicley)
    return theta

def ins():

    comm1 = comm.Serial('/dev/ttyUSB0', 115200)
    
    finitesate = 0
    replan_flag=False
    distance_flag=0
    obs_x=0.0
    obs_y=0.0
    while True:
        if comm1.flag:
        #if True:
            comm1.recv()
            x = comm1.state.x-100.0
            y = comm1.state.y-100.0
            azuith = comm1.state.azuith/100.0+90.0
            
           # x = path_generation.path_point[index].x
            #y = path_generation.path_point[index].y
            #azuith=Get_Line_theta(index)

            map= map2d.map2d(x,y,azuith)
            finitesate=stateLoop(x, y, finitesate)
            print(round(x, 3), round(y, 3), round(azuith, 1))
            present_Index = calc_target_index(x,y, path_generation.path_point)
            #map.cal_mapXY(path_generation.path_point)

            '''
            distance = reply() 
            distance_float = float(distance)
            if(distance_float < 6 and distance_float >0):
                distance_flag = 1
                print(distance_flag)
            else:
                distance_flag = 0
                print(distance_flag)
            '''           
            vehicley=0.0
            vehiclex=0.0

            if distance_flag==1:        
                if(replan_flag):#ASTAR
                    pointlist=[]    
                    map.setObsWeight(int(obs_x/0.4+map.w/2),int(obs_y/0.4+map.h/4))   #obs_x,obs_y should in map corrdinate
                    if(cal_distance(x, y, path_generation.task_point[finitesate].x, path_generation.task_point[finitesate].y))<4.0:
                        targetx=path_generation.task_point[(finitesate+1)%len(path_generation.task_point)].x
                        targety=path_generation.task_point[(finitesate+1)%len(path_generation.task_point)].y
                    else:
                        targetx=path_generation.task_point[finitesate].x
                        targety=path_generation.task_point[finitesate].y
                    target_x_map,target_y_map=map.World2Map(targetx,targety)
                    aStar = Astar.AStar(map, Astar.Node(Astar.Point(20,10)), Astar.Node(Astar.Point(target_x_map,target_y_map)))
                    if aStar.start():
                        for node in aStar.pathlist:
                            point_temp=Astar.Point(0,0)
                            point_temp.x=node.point.x
                            point_temp.y=node.point.y
                            pointlist.append(point_temp)            
                        vehiclex =(pointlist[len(pointlist)-5].x-20) *0.4
                        vehicley =(pointlist[len(pointlist)-5].y-10) *0.4
                    else:
                        print("no way") 
                    
                else: #aligned_line
                    map_point=[]
                    map.setObsWeight(obs_x,obs_y)     
                    for i in range(len(map.vehiclex)):
                        point_temp=Astar.Point(map.vehiclex[i],map.vehicley[i])
                        map_point.append(point_temp)
                    aligned_lines=aligned_line.aligned_lines(0.4,21)
                    aligned_lines.origin_line=map_point
                    aligned_lines.Cal_Aligned_Line()
                    #aligned_lines.ShowLine()
                    aligned_lines.local_planning( map)   
                    line=aligned_lines.linelist[aligned_lines.best_line]
                    present_Index_in_vehicle=calc_target_index(0,0,line)      
                    vehiclex=line[present_Index_in_vehicle].x
                    vehicley=line[present_Index_in_vehicle].y
            else:
                target_x = path_generation.path_point[present_Index].x
                target_y = path_generation.path_point[present_Index].y
                vehiclex, vehicley = map.World2Vehicle(target_x, target_y)
                print(target_x,target_y)
            
            heading_angle = get_theta(vehiclex, vehicley)+180.0
            print(round(vehiclex, 3),round(vehicley, 3), finitesate, round(heading_angle, 1))

            send_x = int(vehiclex * 1000) + 1000000
            send_y = int(vehicley * 1000) + 1000000
            send_angle = int(heading_angle * 10)

            comm1.write(send_x, send_y, finitesate, send_angle)
            
            #plt.cla()
            #plt.xlim(0, 40.0)
           # plt.ylim(0, 40.0)
            #plt.plot(map.mapx, map.mapy, "*b")
            #plt.plot(20,10, "go")
            #plt.draw()
            #plt.pause(0.1)
        else:
            print('comm port is not available!')

if __name__ == '__main__':
    '''
    dll = cdll.LoadLibrary("/home/excavator/Desktop/catkin_workspace/devel/lib/libpedestriandetection.so")
    reply = dll.Get_distance
    reply.restype = c_char_p
    '''
    path = "./Target.txt"
    path_generation.ReadTarget(path)
    path_generation.PathGeneration(path_generation.target_point, 0.4)
    ins()

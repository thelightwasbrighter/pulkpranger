#!/usr/bin/python2
Y_STEP=1
X_STEP=0.6545
DISP_SIZE=800
GAGGLE_DIST=1000
MIN_GAGGLE_SIZE=5
KURBEL_DT=20
KURBEL_DX=1000
INITIATOR_SPAN=10
TASK_COL=(100,100,100)
import pygame, pygame.locals
import sys
import copy
import math
import string

class Point(object):
    def __init__(self, t, x, y, ew, ns, h, circling=False, on_task=False):
        self.time=t
        self.x_pos=x
        if ew=='W':
            self.x_pos=-self.x_pos
        self.y_pos=y
        if ns=='S':
            self.y_pos=-self.y_pos
        self.height=int(h)
        self.circling=circling
        self.on_task=on_task

class Turnpoint(object):
    def __init__(self, typ, pos, rad):
        self.typ=typ
        self.pos=pos
        self.rad=rad

class Task(object):
    def __init__(self,igc_file):
        self.waypoints=[]
        for line in open(igc_file, 'r'):
            if line[0:14]=='LSCSDGate open':
                self.gateopen=3600*int(line[15:17])+60*int(line[18:20])
            if line[0:10]=='LSCSRSLINE':
                self.startline_length=float(line[11:])
            if line[0:14]=='LSCSRFCYLINDER':
                self.zielkreis_r=float(line[15:])
            if line[0:6]=='LSCSCS':
                self.low_i=string.find(line,':',7)
                self.high_i=string.find(line,':',self.low_i+1)
                self.start_point=Point(0,float(line[self.high_i+2:self.high_i+5])+(float(line[self.high_i+5:self.high_i+10])/60000),float(line[self.low_i+2:self.low_i+4])+(float(line[self.low_i+4:self.low_i+9])/60000),line[self.low_i+1],line[self.high_i+1],0)

            if line[0:6]=='LSCSCT':
                self.low_i=string.find(line,':',7)
                self.high_i=string.find(line,':',self.low_i+1)
                self.waypoints.append(Point(0, float(line[self.high_i+2:self.high_i+5])+(float(line[self.high_i+5:self.high_i+10])/60000),float(line[self.low_i+2:self.low_i+4])+(float(line[self.low_i+4:self.low_i+9])/60000),line[self.low_i+1],line[self.high_i+1],0))

            if line[0:6]=='LSCSCF':
                self.low_i=string.find(line,':',7)
                self.high_i=string.find(line,':',self.low_i+1)
                self.finish_point=Point(0, float(line[self.high_i+2:self.high_i+5])+(float(line[self.high_i+5:self.high_i+10])/60000),float(line[self.low_i+2:self.low_i+4])+(float(line[self.low_i+4:self.low_i+9])/60000),line[self.low_i+1],line[self.high_i+1],0)

                   
class Trace(object):
    def __init__(self, igc_file, task):
        self.name=igc_file
        self.is_in_gaggle=False
        self.gaggle_counter=0
        self.last_index=0
        self.waypoints=[]
        self.bart_initiator=False
        self.last_time_circling=0
        for line in open(igc_file, 'r'):
            if line.find('B')==0:
                self.time=int(line[1:3])*3600+int(line[3:5])*60+int(line[5:7])
                self.y_pos=line[7:14]
                self.y_ns=line[14]
                self.x_pos=line[15:23]
                self.x_ew=line[23]
                self.height=line[30:35]
                #convert
                self.y_pos=float(self.y_pos[:2])+float(self.y_pos[2:])/60000.0
                self.x_pos=float(self.x_pos[:3])+float(self.x_pos[3:])/60000.0
                self.temp_pos=Point(self.time, self.x_pos, self.y_pos, self.x_ew, self.y_ns ,0,)
                self.waypoints.append(Point(self.time, self.x_pos, self.y_pos, self.x_ew, self.y_ns ,self.height,False,True))
        #circle detection
        for i in range(len(self.waypoints)):
            min_index=i
            while 1:
                if min_index==0:
                    break
                if self.waypoints[i].time-self.waypoints[min_index].time>KURBEL_DT:
                    break
                min_index=min_index-1
            max_index=i
            while 1:
                if max_index==len(self.waypoints)-1:
                    break
                if self.waypoints[max_index].time-self.waypoints[i].time>KURBEL_DT:
                    break
                max_index=max_index+1
            if distance(self.waypoints[min_index],self.waypoints[max_index])<KURBEL_DX:
                self.waypoints[i].circling=True
            
        #for p in self.waypoints:
        #     print p.time
        #     print p.x_pos
        #     print p.y_pos
        #     print ""

    def print_waypoints(self):
        for p in self.waypoints:
            print "time:  ", p.time
            print "lat:   ", p.y_pos
            print "lon:   ", p.x_pos
            print "height:", p.height
            print ""

    def get_start_time(self):
        return self.waypoints[0].time

    def get_finish_time(self):
        return self.waypoints[len(self.waypoints)-1].time

    def get_pos_at_time(self, time):
        old_p=Point(0,0,0,'E','N',0)
        while 1:
            p=self.waypoints[self.last_index]
            if int(p.time)==int(time):
                return p
            elif int(p.time)<int(time):
                if self.last_index==len(self.waypoints)-1:
                    return p
                self.last_index=self.last_index+1
            else:
                if self.last_index==0:
                    return p
                else:
                    old_p=self.waypoints[self.last_index-1]
                    temp_x_pos=float(old_p.x_pos)*(1-(float(time-old_p.time)/(p.time-old_p.time)))+float(p.x_pos)*float(time-old_p.time)/(p.time-old_p.time)
                    temp_y_pos=float(old_p.y_pos)*(1-(float(time-old_p.time)/(p.time-old_p.time)))+float(p.y_pos)*float(time-old_p.time)/(p.time-old_p.time)
                    temp_height=float(old_p.height)*(1-(float(time-old_p.time)/(p.time-old_p.time)))+float(p.height)*float(time-old_p.time)/(p.time-old_p.time)
                    if old_p.circling:
                        return Point(time, temp_x_pos, temp_y_pos, 'E', 'N', temp_height, True)
                    else:
                        return Point(time, temp_x_pos, temp_y_pos, 'E', 'N', temp_height, False)

             
        return old_p
    def get_max_coords(self):
        x_max=-180
        y_max=-90
        h_max=-1000
        for p in self.waypoints:
            if p.x_pos>x_max:
                x_max=p.x_pos
            if p.y_pos>y_max:
                y_max=p.y_pos
            if p.height>h_max:
                h_max=p.height
        return Point(0, x_max, y_max, 'E', 'N', h_max)

    def get_min_coords(self):
        x_min=180
        y_min=90
        h_min=8000
        for p in self.waypoints:
            if p.x_pos<x_min:
                x_min=p.x_pos
            if p.y_pos<y_min:
                y_min=p.y_pos
            if p.height<h_min:
                h_min=p.height
        return Point(0, x_min, y_min, 'E', 'N', h_min)

            
class Trace_batch(object):
    def __init__(self):
        self.tracelist=[]
        
    def add_trace(self, trace):
        self.tracelist.append(trace)

    def get_start_time(self):
        temp_t=self.tracelist[0].get_start_time()
        for tr in self.tracelist:
            if tr.get_start_time()<temp_t:
                temp_t=tr.get_start_time()
        return temp_t

    def get_finish_time(self):
        temp_t=self.tracelist[0].get_finish_time()
        for tr in self.tracelist:
            if tr.get_finish_time()>temp_t:
                temp_t=tr.get_finish_time()
        return temp_t

    def get_max_coords(self):
        x_max=-180
        y_max=-90
        h_max=-1000
        for tr in self.tracelist:
            for p in tr.waypoints:
                if p.x_pos>x_max:
                    x_max=p.x_pos
                if p.y_pos>y_max:
                    y_max=p.y_pos
                if p.height>h_max:
                    h_max=p.height
        return Point(0, x_max, y_max, 'E', 'N', h_max)
                    
    def get_min_coords(self):
        x_min=180
        y_min=90
        h_min=8000
        for tr in self.tracelist:
            for p in tr.waypoints:
                if p.x_pos<x_min:
                    x_min=p.x_pos
                if p.y_pos<y_min:
                    y_min=p.y_pos
                if p.height<h_min:
                    h_min=p.height
        return Point(0, x_min, y_min, 'E', 'N', h_min)

    def get_width_height(self):
        topright=self.get_max_coords()
        bottomleft=self.get_min_coords()
        return [(topright.x_pos-bottomleft.x_pos)*X_STEP,(topright.y_pos-bottomleft.y_pos)*Y_STEP]
    
class Display(object):
    def __init__(self, width, height, scale=1):
        self.background = 0,0,0
        self.width, self.height = width, height
        self.scale = scale
        self.size = (self.width*scale , self.height*scale)
        pygame.init()
        self.screen = pygame.display.set_mode(self.size)
        pygame.display.set_caption("PulkPranger")
        self.text = []
        self.plane_surface=pygame.Surface(self.size)
        self.draft_surface=pygame.Surface(self.size)
        self.task_surface=pygame.Surface(self.size)
        self.plane_surface.set_colorkey(self.background)
        self.task_surface.set_colorkey(self.background)
            
    def set_frame(self, surface):
        self.screen.blit(surface, (0,0))
        pygame.display.flip()        

    def get_event(self):
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                return 'quit'
            elif event.type==pygame.KEYUP:
                #print event.key
                if event.key==pygame.K_q: #q
                    return 'quit'
                elif event.key==pygame.K_SPACE: #SPACE
                    return 'playpause'
                elif event.key==pygame.K_v: #v
                    return 'stop'
                elif event.key==pygame.K_RIGHT: # right arrow
                    return 'right_arrow'
                elif event.key==pygame.K_LEFT: #left arrow
                    return 'left_arrow'
                elif event.key==pygame.K_UP:
                    return 'inc_fps'
                elif event.key==pygame.K_DOWN:
                    return 'dec_fps'


class Player(object):
    def __init__(self, trace_batch, task):
        self.trace_batch=trace_batch
        self.task=task
        self.size=self.trace_batch.get_width_height()
        if self.size[0]>self.size[1]:
            self.size[1]=self.size[1]/self.size[0]*DISP_SIZE
            self.size[0]=DISP_SIZE
        else:
            self.size[0]=self.size[0]/self.size[1]*DISP_SIZE
            self.size[1]=DISP_SIZE
        self.disp=Display(int(self.size[0]), int(self.size[1]))
        
    def draw_task(self):
        #draw task
        self.disp.task_surface.lock()
        self.disp.task_surface.fill(self.disp.background)
        self.disp.task_surface.unlock()
        pygame.draw.line(self.disp.task_surface, TASK_COL, (int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.start_point.x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.start_point.y_pos))),(int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.waypoints[0].x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.waypoints[0].y_pos))))          
        for p_i in range(len(self.task.waypoints)):
            if p_i!=0:
                pygame.draw.line(self.disp.task_surface, TASK_COL, (int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.waypoints[p_i-1].x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.waypoints[p_i-1].y_pos))),(int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.waypoints[p_i].x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.waypoints[p_i].y_pos))))
        
        pygame.draw.line(self.disp.task_surface, TASK_COL, (int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.waypoints[len(self.task.waypoints)-1].x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.waypoints[len(self.task.waypoints)-1].y_pos))),(int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.task.finish_point.x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.task.finish_point.y_pos))))
        
        #start line
        self.x_vect=(self.task.start_point.x_pos-self.task.waypoints[0].x_pos)
        self.y_vect=(self.task.start_point.y_pos-self.task.waypoints[0].y_pos)
        self.zwischen=self.x_vect
        self.x_vect=self.y_vect/X_STEP
        self.y_vect=-self.zwischen*X_STEP
        self.betrag=math.sqrt(self.x_vect*self.x_vect*X_STEP*X_STEP+self.y_vect*self.y_vect)
        self.x_vect=self.x_vect/self.betrag
        self.y_vect=self.y_vect/self.betrag
        self.start_line_a=Point(0,0,0,'N','E',0)
        self.start_line_b=Point(0,0,0,'N','E',0)
        self.start_line_a.x_pos=self.task.start_point.x_pos-self.task.startline_length/220000*self.x_vect
        self.start_line_b.x_pos=self.task.start_point.x_pos+self.task.startline_length/220000*self.x_vect
        self.start_line_a.y_pos=self.task.start_point.y_pos-self.task.startline_length/220000*self.y_vect
        self.start_line_b.y_pos=self.task.start_point.y_pos+self.task.startline_length/220000*self.y_vect
        pygame.draw.line(self.disp.task_surface, TASK_COL, (int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.start_line_a.x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.start_line_a.y_pos))),(int(map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2], self.x_map_parameter[3], self.start_line_b.x_pos)), int(self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2], self.y_map_parameter[3], self.start_line_b.y_pos))))


    def play(self):
        self.start_time=self.task.gateopen #EDDELself.trace_batch.get_start_time()
        self.duration=self.trace_batch.get_finish_time()-self.start_time
        self.x_map_parameter=[self.trace_batch.get_min_coords().x_pos,self.trace_batch.get_max_coords().x_pos, 0, self.size[0]]
        self.y_map_parameter=[self.trace_batch.get_min_coords().y_pos,self.trace_batch.get_max_coords().y_pos, 0, self.size[1]]
        self.temp_pos_mapped=Point(0,0,0,'N','E',0)
        self.draw_task()
        for t in range(self.duration):
            self.disp.draft_surface.lock()
            self.disp.draft_surface.fill(self.disp.background)
            self.disp.draft_surface.unlock()
            self.disp.plane_surface.lock()
            self.disp.plane_surface.fill(self.disp.background)
            self.disp.plane_surface.unlock()
            for tr in self.trace_batch.tracelist:
                self.cnt=0
                for tr2 in self.trace_batch.tracelist:
                    if tr!=tr2:
                        if distance(tr.waypoints[tr.last_index],tr2.waypoints[tr2.last_index])<GAGGLE_DIST:
                            self.cnt=self.cnt+1
                if tr.last_index>0 and tr.last_index<len(tr.waypoints)-1:
                    if self.cnt>MIN_GAGGLE_SIZE-1 and distance(tr.waypoints[tr.last_index],tr.waypoints[tr.last_index-1])>10  and tr.waypoints[tr.last_index].circling:
                        if not tr.bart_initiator:
                            tr.is_in_gaggle=True
                            tr.gaggle_counter=tr.gaggle_counter+1
                        else:
                            tr.is_in_gaggle=False
                            tr.last_time_circling=t
                    elif tr.waypoints[tr.last_index].circling:
                        tr.is_in_gaggle=False
                        tr.bart_initiator=True
                        tr.last_time_circling=t
                    else:
                        tr.is_in_gaggle=False
                        if tr.bart_initiator and t-tr.last_time_circling>INITIATOR_SPAN:
                            tr.bart_initiator=False
                else:
                    tr.is_in_gaggle=False
                    tr.bart_initiator=False
            for tr in self.trace_batch.tracelist:
                self.temp_pos=tr.get_pos_at_time(t+self.start_time)
                self.temp_pos_mapped.x_pos=map(self.x_map_parameter[0],self.x_map_parameter[1],self.x_map_parameter[2],self.x_map_parameter[3],self.temp_pos.x_pos)
                self.temp_pos_mapped.y_pos=self.y_map_parameter[3]-map(self.y_map_parameter[0],self.y_map_parameter[1],self.y_map_parameter[2],self.y_map_parameter[3],self.temp_pos.y_pos)
                self.disp.plane_surface.lock()
                if tr.is_in_gaggle:
                    self.disp.plane_surface.set_at((int(self.temp_pos_mapped.x_pos),int(self.temp_pos_mapped.y_pos)), (255,0,0))
                elif tr.bart_initiator:
                    self.disp.plane_surface.set_at((int(self.temp_pos_mapped.x_pos),int(self.temp_pos_mapped.y_pos)), (0,255,0))
                else:
                    self.disp.plane_surface.set_at((int(self.temp_pos_mapped.x_pos),int(self.temp_pos_mapped.y_pos)), (255,255,255))
                self.disp.plane_surface.unlock()
                        
            self.disp.draft_surface.blit(self.disp.task_surface, (0,0))
            self.disp.draft_surface.blit(self.disp.plane_surface, (0,0))
            self.disp.set_frame(self.disp.draft_surface)
                
                
def map(a,b,x,y,e):
    return ((float(e)-float(a))/(b-a)*(float(y)-float(x)))

    
def distance(a,b):
    return math.sqrt(pow((a.x_pos-b.x_pos)*X_STEP*110000,2)+pow((a.y_pos-b.y_pos)*Y_STEP*110000,2)+pow(a.height-b.height,2))

def distance_horizontal(a,b):
    return math.sqrt(pow((a.x_pos-b.x_pos)*X_STEP*110000,2)+pow((a.y_pos-b.y_pos)*Y_STEP*110000,2))

mytask=Task(sys.argv[1])
mybatch=Trace_batch()
for igc in sys.argv[1:]:
    mybatch.add_trace(Trace(igc,mytask))

myplayer=Player(mybatch,mytask)
myplayer.play()

sorted_tracelist=sorted(mybatch.tracelist, key=lambda x: x.gaggle_counter, reverse=True)
for tr in sorted_tracelist:
    print tr.name, tr.gaggle_counter

#mytrace.print_waypoints()
# print mybatch.get_start_time()
# print mybatch.get_finish_time()

# max=mybatch.get_max_coords()
# min=mybatch.get_min_coords()
# print "max lat:   ", max.y_pos
# print "max lon:   ", max.x_pos
# print "min lat:   ", min.y_pos
# print "min lon:   ", min.x_pos


# pos=mytrace.get_pos_at_time(151357)
# print "time:  ", pos.time
# print "lat:   ", pos.y_pos, pos.y_ns
# print "lon:   ", pos.x_pos, pos.x_ew
# print "height:", pos.height
# print ""

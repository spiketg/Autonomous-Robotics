#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  6 11:59:58 2019

@author: s1983630

"""

from time import time
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(linewidth=200)
margin = 0.15
x_bound = 4.25
y_bound = 3.2

obst = [(0.,0.,3.95,0.),
        (3.95,0.,4.25,0.3),
        (4.25,0.3,4.25,2.15),
        (3.20,2.15,4.25,2.15),
        (3.20,2.15,3.20,3.2),
        (0.6,3.20,3.20,3.20),
        (0.0,2.50,0.6,3.20),
        (0.0,0.0,0.0,2.50),
        (1.05,0.75,1.05,0.91),
        (1.05,0.91,3.20,0.91),
        (1.05,0.75,3.20,0.75),
        (3.20,0.75,3.20,0.91),
        (1.10,2.20,1.85,2.20),
        (1.10,2.20,1.475,1.55),
        (1.475,1.55,1.85,2.20),
        (0.75,0.67,0.91,0.67),
        (0.75,0.83,0.91,0.83),
        (0.75,0.67,0.75,0.83),
        (0.91,0.67,0.91,0.83)] # list of walls in format X_start, Y_start, X_end, Y_end.
obst = np.array(obst)
class Grid():
    
	# Set the size of grid cells and the number of allowed orientations
    cell_size = 0.2
    angle_res = 12
	# Set resolution for sensor readings simulation
	# Set the number of rays used when simulating sensor readings
	# Specify arena data
    arena_size = np.array([4.25, 3.2])
    arena_size_idx = np.ceil(arena_size / cell_size)
    arena_size_idx = (int(arena_size_idx[0]), int(arena_size_idx[1]))
	# Write equations of walls - a set of constraints making a point (x,y)
	# lie OUTSIDE of available spate. 
    not_in_arena = ['x < 0+margin or x > 4.25-margin or y < 0+margin or y > 3.2-margin', # external boundaries of the arenas
   				'y > 2.05 and x > (x_bound-1.15)', # the sq corner
   				'x > y  +3.85', # small cut corner
   				'y > x  +2.4', # large cut corner
    			#'x > 1 and y > 0.5 * x + 1.9 and y < -0.5 * x + 3.65', 
                #' x>1.375 and x+1.375<0.5*y +2.3 and y<2.3 and y>1.5 ',
                '(x>1 and x<1.85 and y <2.3 and y>1.45) ',# the triangle island
    			'x > .95 and x <3.30  and y < 1. and y > 0.65',
                'x >3.8 and x<4.0 and y> 0.26 and y<0.46',
                'x <.91 and x>.75 and y>0.755-.08 and y<0.755+.08']
    
    '''
    (1.10,2.20,1.85,2.20),
    (1.10,2.20,1.475,1.55),
    (1.475,1.55,1.85,2.20)
    '''
    
    start_point = np.array([.40, 0.41])
    	

    def __init__(self):
        self.in_arena = np.empty(Grid.arena_size_idx, dtype=np.uint)
		# Check if centre of each grid cell is in the arena
        it= np.nditer(self.in_arena, flags=['multi_index'], op_flags=['writeonly'])
        while not it.finished:
            it[0]= Grid.is_in_arena(it.multi_index[0], it.multi_index[1])
            it.iternext()

    @staticmethod
    def is_in_arena(x_idx, y_idx):
        '''
        Evaluates if the centre of a cell of the given index lies within the arena
        '''
        (x, y) = Grid.grid_to_xy((x_idx, y_idx))
        for excluding_condition in Grid.not_in_arena:
            if eval(excluding_condition):
                return 1
        return 0

    @staticmethod
    def grid_to_xy(grid_coord):
        '''
        Gives the (x,y) coordinates of the centre of a grid cell indexed by grid_coord.
        '''
        return (np.array(grid_coord)*Grid.cell_size)+np.array([Grid.cell_size/2, Grid.cell_size/2])
    @staticmethod
    def xy_togrid(x,y):
        return np.array([int(y//Grid.cell_size), int(x//Grid.cell_size)])
    
    def to_array(self):
        '''defined
        Displays a human-readable representatnion of the arena
        '''
        retval=np.zeros((self.in_arena.shape[1],self.in_arena.shape[0]), dtype=int)
        for y in (range(self.in_arena.shape[1])):
            for x in (range(self.in_arena.shape[0])):
                retval[y,x]= 0 if not self.in_arena[x,y] else 1
            return retval
class CustomTile:
	def __init__(self, _worldPos=[0,0], _cellPos=[0,0], _isObstacle=False):
		self.worldPos = _worldPos
		self.cellPos = _cellPos
		self.isObstacle = _isObstacle
		self.prevTile = None
		self.G = 0
		self.H = 0
		
	def GetPrev(self):
		return self.prevTile
		
	def SetPrev(self,_prev):
		self.prevTile = _prev
		
	def SetG(self,_g):
		self.G = _g
		
	def SetH(self,_h):
		self.H = _h
		
	def GetG(self):
		return self.G
		
	def GetH(self):
		return self.H
		
	def GetF(self):
		return self.G + self.H
		
	def GetIsObstacle(self):
		return self.isObstacle
		
	def GetCellPos(self):
		return self.cellPos
	
#Behaves like a linked list manager and stores world occupancy grid(map) and the dictionary of coords	
class BotPathData:
	def __init__(self, _map):
		self.map = _map
		self.tileDictionary = {}

		for n in range(len(_map)):
			for p in range(len(_map[n])):
				localPlace = [n, p]
				key = str(n) + "," + str(p)
				if key not in self.tileDictionary:
					self.tileDictionary[key] = CustomTile(localPlace, localPlace, False if self.map[n][p] == 0 else True)
		
	#Semi-legacy, the algo was initially designed to work with grid + real world positions
	#from which tile of the grid was located
	def TileFromCellPos(self, _x, _y):
		key = str(_x) + "," + str(_y)
		if key in self.tileDictionary:
			return self.tileDictionary[key]

		return None;

	#Getting neighbours of front, back, left, right of a tile
	def GetNeighbours(self, _cur):
		neighbours = []

		xShift = _cur.GetCellPos()[0] + 1
		yShift = _cur.GetCellPos()[1]
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0]
		yShift = _cur.GetCellPos()[1] + 1
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0] - 1
		yShift = _cur.GetCellPos()[1]
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0]
		yShift = _cur.GetCellPos()[1] - 1
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		return neighbours

#Main class that runs the A* algo and retrieves a path
class Pathfinding:
    def __init__(self, _map):
        self.botData = BotPathData(_map)
        self.pathToDestination = []
		
    def GetPathToDestination(self):
        return self.pathToDestination
    
    def CalculateFinalPath(self, startCoords, destCoords):
        discoveredTiles = set()
        discardedTiles = set()
        current = self.botData.TileFromCellPos(startCoords[0], startCoords[1])
        dest = self.botData.TileFromCellPos(destCoords[0], destCoords[1])
		
        discoveredTiles.add(current)

        while len(discoveredTiles) > 0:
			presentTile = CustomTile()
			init = True
			for it in discoveredTiles:
				if init == True:
					presentTile = it
					init = False
				else:
					itTile = it
					if itTile.GetF() < presentTile.GetF() or itTile.GetF() == presentTile.GetF() and itTile.GetH() < presentTile.GetH():
						presentTile = itTile
			discoveredTiles.remove(presentTile)
			discardedTiles.add(presentTile)
			
			if presentTile.GetCellPos()[0] == dest.GetCellPos()[0] and presentTile.GetCellPos()[1] == dest.GetCellPos()[1]:
				self.GetSequence(current, dest)
			
			for curNeighbour in self.botData.GetNeighbours(presentTile):
				if curNeighbour.GetIsObstacle() == True or curNeighbour in discardedTiles:
					continue
					
				curCellPos = current.GetCellPos()
				neighCellPos = curNeighbour.GetCellPos()
				
				cost = current.GetG() + abs(curCellPos[0] - neighCellPos[0]) + abs(curCellPos[1] - neighCellPos[1])
				if cost < curNeighbour.GetG() or curNeighbour not in discoveredTiles:
					destCellPos = dest.GetCellPos()
					curNeighbour.SetG(cost)
					curNeighbour.SetH(abs(neighCellPos[0] - destCellPos[0]) + abs(neighCellPos[1] - destCellPos[1]))
					
					curNeighbour.SetPrev(presentTile)
					
					if curNeighbour not in discoveredTiles:
						discoveredTiles.add(curNeighbour)
                        
    def GetSequence(self, _cur, _dest):
        path = []
        current = _dest
        while (current != _cur):
            path.append(current)
            current = current.GetPrev()
			
        path.append(_cur)
        path = list(reversed(path))
        self.pathToDestination = path
        
def computeGridPathPOI(map, startPos, endPos):
        simulation = Pathfinding(map)
        simulation.CalculateFinalPath(startPos, endPos)
        listPOI = []
        fullPath = simulation.GetPathToDestination()
        
        prevX = []
        prevY = []
        prevDiffX = 0
        prevDiffY = 0
        i = 0
        for tile in simulation.GetPathToDestination():
            if i == 0:
                prevX = tile.GetCellPos()[0]
                prevY = tile.GetCellPos()[1]
                listPOI.append([prevX,prevY])
            elif i == 1:
                prevDiffX = tile.GetCellPos()[0] - prevX
                prevDiffY = tile.GetCellPos()[1] - prevY
                prevX = tile.GetCellPos()[0]
                prevY = tile.GetCellPos()[1]
            else:
                curX = tile.GetCellPos()[0]
                curY = tile.GetCellPos()[1]

                diffX = curX - prevX
                diffY = curY - prevY

                if diffX == prevDiffX and diffY == prevDiffY:
                    prevDiffX = diffX
                    prevDiffY = diffY
                    prevX = curX
                    prevY = curY
                else:
                    listPOI.append([prevX, prevY])
                    prevDiffX = diffX
                    prevDiffY = diffY
                    prevX = curX
                    prevY = curY
		  
            i += 1
        listPOI.append([prevX, prevY])
        
        return listPOI
    
def perp( a ) :
    b = np.full_like(a,a[:,::-1])
    b[:,0] = -a[:,1]
   
    #b[:,1] = a[:,0]
    return b
def distance_matrix(a,b):
    return np.sqrt((a[:,0] - b[:,0])**2 + (a[:,1] - b[:,1])**2)
def is_between(a,c,b):
    #print distance(a,b), distance(a,c)+distance(c,b)
    return np.isclose(distance_matrix(a,b),(distance_matrix(a,c) + distance_matrix(c,b)))

def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
   
    db = b2-b1
   
    dp = a1-b1
 
    dap = perp(da)
    #print dap.T
    denom =np.dot( dap, db.T)
 
    num = np.dot( dap, dp.T )
    
    if np.all(denom) == 0:
        inters_final = None
        
    else:
        inters  = (num/denom)*np.identity(len(num/denom))
        inters_final = (np.dot(inters,db))+b1
  
    return inters_final
def slope(p1,p2):
    return(p1[:,0]-p2[:,0])/(p1[:,1]-p2[:,1])
def check_angle(p1,p2):
    return np.arctan2((p2[:,1]-p1[:,1]),(p2[:,0]-p1[:,0]))
def get_shortest(points):
    reached =False
    i=0
    k=-1
    short = []
    short.append(points[-1])
    while reached ==False:
        inter =False
        pointa = points[k]
        pointb = points[i]
        pos_reads = np.full_like((obst),(points[i][0],points[i][1],points[k][0],points[k][1]))
        '''print (pos_reads)'''
        obst1= obst[:,0:2]
        obst2= obst[:,2:]
        p1 = pos_reads[:,0:2]
        p2 = pos_reads[:,2:]
        
        
        inter = seg_intersect(obst1,obst2,p1,p2)
        if np.all(inter) != None:
            #print 'intersection = ' + str(inter)
            true = is_between(obst1,inter,obst2)
            te = np.argwhere(true)
            pos = distance_matrix(p1,inter) 
            test = inter[te]
          
            f2  = np.sign(p1[te]-test)==np.sign(p1[te]-p2[te])
            
            
            f1 = inter[te]
          
            f4 = np.argwhere(np.all([f1,f2],axis=0))[:,0]
            
            f5= np.expand_dims(f4, axis=1)
            f6 = np.unique(f5)
            f7 = te[f6].flatten()
            
            final2= inter[f7]

            test=0
            
            pos2 = distance_matrix(p1,p2)
            pos = distance_matrix(p1,inter)  

            ins=pos[f7]
            if (np.any(pos2[f7]-pos[f7]>=0))==True:
                inter = True
            else:
                inter = False
        else:
            inter = False
            
        if inter == False:
            short.append(pointb)
            if i==0:
            
                reached=True
            else:
                print('i is :'+str(i))
                print('k is :' +str(k))
                print('inter is :' +str(inter))
                print('list is :' + str (short))
                k=i
                i=0
                
        elif inter ==True :
            i+=1
            print('i is :'+str(i))
            print('k is :' +str(k))
            print('inter is :' +str(inter))
            print('list is :' + str (short))
    return short
    
tl = time()
map2 = Grid()
map = map2.to_array()
print(map2.to_array())
tl2 = time()
maxMapX = 22
maxMapY = 16
simulation = Pathfinding(map)
poi1_wc = (3.9,0.5)
poi2_wc = (0.7,0.6)
poi3_wc =(0.5,2.5)
base = (3.9,1.8)
init = Grid.xy_togrid(base[0],base[1])
poi1 = Grid.xy_togrid(poi1_wc[0],poi1_wc[1])
poi2 = Grid.xy_togrid(poi2_wc[0],poi2_wc[1])
poi3 = Grid.xy_togrid(poi3_wc[0],poi3_wc[1])
#poi2 = Grid.xy_togrid(3.0,3.0)
simulation.CalculateFinalPath([init[0],init[1]], [poi1[0],poi1[1]])
points = []
for tile in simulation.GetPathToDestination():
    points.append(((tile.GetCellPos()[1]),(tile.GetCellPos()[0])))
    
print(points)
print("Path processing completed!")
tl3=time()
f_maps = map
listPOI = computeGridPathPOI(map, [init[0],init[1]], [poi1[0],poi1[1]])
w_points = []
tl4=time()
for i,point in enumerate(listPOI):
    f_maps[point[0],point[1]] = '8'
    w_points.append(map2.grid_to_xy((point[1],point[0])))

    
u,ind =  np.unique(w_points,axis=0,return_index=True)

wp=np.array(w_points)
wp=wp[ind]

print(f_maps)
print(w_points)
print(listPOI)

short=get_shortest(w_points)
final_path = list(reversed(short))
print final_path
e = time()
print('m: '+str(tl2 -tl))
print('A*: '+str(tl3-tl2))
print('get rot: '+str(tl4-tl3))
print('rt: ' + str((e-tl4)))
print('total: '+str(e-tl))

print('A* len: ' +str(len(points)))
print('rot len: ' + str(len(listPOI)))
print('short len: ' +str(len(short)-1 ))
                
                

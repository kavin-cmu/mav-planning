import csv
from time import sleep
from typing import Any
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import tf.transformations as tf_utils
import networkx as nx
import itertools


class Geometry():
    
    def __init__(self):
        
        self.type = None
        self.shape = []
        self.position = []
        self.orientation = []
        
          

class PlanDataVisualizer():

    def __init__(self):
         
        self.files_dir = "/home/kyouma/dev/main/office_ws/src/mav_planning/data/"
        self.path = None
        self.obstacles = []
        self.map_min_bounds = None
        self.map_max_bounds = None
        self.graph = None
        self.robot_model = Geometry()
        
        self.read_data()
        self.plot_data()


    def read_data(self):
        # Read Path Data
        with open(str(self.files_dir+"path.csv")) as csv_file:
                rows = csv.reader(csv_file, delimiter=',')
                path = []
                cnt=0
                for row in rows:
                    path.append([float(i) for i in row])
                    cnt+=1

                self.path = np.array(path)

                # np.transpose(self.path)

                # print(self.path)
        
        # Read Robot Data
        with open(str(self.files_dir+"robot_model.csv")) as csv_file:
                rows = csv.reader(csv_file, delimiter='|')

                # print(list(rows))

                for row in rows:
                    
                    elems = [e.split(',') for e in row]
                    self.robot_model.position = [float(i) for i in elems[0]]
                    self.robot_model.orientation = [float(i) for i in elems[1]]
                    self.robot_model.type = int(elems[2][0])
                    self.robot_model.shape = [float(i) for i in elems[3]]
                    # print(self.robot_model.position)
                    # print(self.robot_model.orientation)
                    # print(self.robot_model.type)
                    # print(self.robot_model.shape)
                    # row_list = [float(i) for i in ll]
                    # print(row_list)
                    break

        
        # Read shapetacle Data
        with open(str(self.files_dir+"map.csv")) as csv_file:
                rows = csv.reader(csv_file, delimiter='|')

                cnt=0
                for row in rows:
                    elems = [e.split(',') for e in row]

                    if cnt==0 and len(elems)==1:
                        self.map_min_bounds = np.array([float(i) for i in elems[0]])
                    if cnt==1 and len(elems)==1:
                        self.map_max_bounds = np.array([float(i) for i in elems[0]])
                    elif cnt > 1 and len(elems)==4:
                        shape = Geometry()
                        shape.position = [float(i) for i in elems[0]]
                        shape.orientation = [float(i) for i in elems[1]]
                        shape.type = int(elems[2][0])
                        shape.shape = [float(i) for i in elems[3]]
                                
                        self.obstacles.append(shape)
                    cnt+=1

        # Read Graph Data
        self.graph = nx.read_graphml(self.files_dir+"data.graphml")
    
    def get_pos_from_node(self, node):
        pos_str = node['coords']
        tmp = pos_str.split(",")
        pos = [float(i) for i in tmp]

        return pos[:3]        

    def plot_shape(self, obj, ax, clr='black', alpha=1.0):        
        
        if not isinstance(obj, Geometry):
            raise TypeError()

        center = obj.position
        rot = tf_utils.quaternion_matrix(obj.orientation)[:3,:3]
        type = obj.type
        shape = obj.shape
        
        # Sphere
        if type==0:
            rad = shape[0]
            phi, theta = np.mgrid[0.0:math.pi:15j, 0.0:2.0*math.pi:15j]

            x = center[0] + rad*np.sin(phi)*np.cos(theta)
            y = center[1] + rad*np.sin(phi)*np.sin(theta)
            z = center[2] + rad*np.cos(phi)

            ax.plot_surface(x, y, z,  rstride=1, cstride=1, color=clr, alpha=alpha, linewidth=1)

        # Box type
        if type==1:

            dr = [0.5*i for i in shape]

            dx = [-dr[0], dr[0]]
            dy = [-dr[1], dr[1]]
            dz = [-dr[2], dr[2]]

            vert_list_lf = np.array(list(itertools.product(dx, dy, dz)))
            vert_list_ff = np.empty_like(vert_list_lf)
            num_verts = range(len(vert_list_lf))

            for i in num_verts:
                # rotate to fixed frame
                vert_list_ff[i] = center + np.matmul(vert_list_lf[i],rot)

            verts = list(itertools.combinations(vert_list_ff, 3))

            ax.add_collection3d(Poly3DCollection(verts, facecolors=clr, linewidths=1, alpha=alpha))
        
        # Cylinder type
        if type==2:
            rad = shape[0]
            hgt = shape[1]

            theta = np.linspace(0.0, 2*math.pi, 100)

            x = [rad*np.cos(t) for t in theta]
            y = [rad*np.sin(t) for t in theta]

            circ_top = list(zip(x,y,[ hgt/2 for i in range(len(theta))]))

            circ_top_ff = []
            circ_bot_ff = []
            side_surf = []

            c1 = np.array(circ_top)

            for i in range(len(c1)):
                # rotate to fixed frame
                #top circle
                tmp = center + np.matmul(c1[i], rot)
                circ_top_ff.append(tmp)
                # get bottom circle from top circle (z-offset)
                tmp = center + np.matmul(c1[i]-np.array([0,0,hgt]), rot)
                circ_bot_ff.append(tmp)
            

            for i in range(len(circ_top)-1):
                tmp = []

                tmp.append(circ_top_ff[i])
                tmp.append(circ_top_ff[i+1])
                tmp.append(circ_bot_ff[i+1])
                tmp.append(circ_bot_ff[i])

                side_surf.append(tmp)                  

            ax.add_collection3d(Poly3DCollection([circ_top_ff, circ_bot_ff], facecolors=clr, linewidths=1, alpha=alpha))
            ax.add_collection3d(Poly3DCollection(side_surf, facecolors=clr, linewidths=0, alpha=alpha))
 
    def plot_data(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.grid()

        # Plot Graph
        for e in self.graph.edges(data=True):
            src_vert_id = e[0]
            tgt_vert_id = e[1]

            src_vert_pos = self.get_pos_from_node(self.graph.nodes[src_vert_id])
            tgt_vert_pos = self.get_pos_from_node(self.graph.nodes[tgt_vert_id])

            line = list(zip(src_vert_pos, tgt_vert_pos))

            ax.plot3D(line[0], line[1], line[2], color='black', linewidth=0.2, marker = '.', markersize=1)
        
        # Plot obstacles
        for shape in self.obstacles:
            v = self.plot_shape(shape, ax, 'magenta', alpha=0.30)

        # Plot path
        ax.plot3D(self.path[:,0], self.path[:,1], self.path[:,2], color='red', linewidth=0.75, marker = 'x', markersize=5)

        # Plot robot shape along path
        for pt in self.path:
            q = tf_utils.quaternion_from_euler(pt[3],0,0,'szyx')
            self.robot_model.position = pt[:3]
            self.robot_model.orientation = q
            self.plot_shape(self.robot_model, ax, 'red', 1.0)

        
        ax.set_title('3D Planning Demo')
        # self.set_axes_equal(ax)
        # Set axes label
        ax.set_xlabel('x', labelpad=20)
        ax.set_ylabel('y', labelpad=20)
        ax.set_zlabel('z', labelpad=20)

        # ax_lim_min = min(self.map_min_bounds)
        # ax_lim_max = max(self.map_max_bounds)
        
        # ax.set_xlim(ax_lim_min, ax_lim_max)
        # ax.set_ylim(ax_lim_min, ax_lim_max)
        # ax.set_zlim(ax_lim_min, ax_lim_max)

        # ax.set_xlim(self.map_min_bounds[0], self.map_max_bounds[0])
        # ax.set_ylim(self.map_min_bounds[1], self.map_max_bounds[1])
        # ax.set_zlim(self.map_min_bounds[2], self.map_max_bounds[2])


        plt.show()


data_viz = PlanDataVisualizer()
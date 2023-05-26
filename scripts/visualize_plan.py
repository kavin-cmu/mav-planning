import csv
import numpy as np
import matplotlib.pyplot as plt
import math
import tf.transformations as tf_utils
import networkx as nx


class PlanDataVisualizer():

    def __init__(self):
         
        self.files_dir = "/home/kyouma/dev/main/office_ws/src/mav_planning/data/"
        self.path = None
        self.circ_obstacles = []
        self.rect_obstacles = []
        self.graph = None
        
        self.read_data_se2()
        self.plot_data()


    def read_data_se2(self):
        with open(str(self.files_dir+"path.csv")) as csv_file:
                rows = csv.reader(csv_file, delimiter=',')
                path = []
                cnt=0
                for row in rows:
                    path.append([float(row[0]),float(row[1]),float(row[2])])
                    cnt+=1

                self.path = np.array(path)

                # np.transpose(self.path)

                # print(self.path)
        
        with open(str(self.files_dir+"obstacles.csv")) as csv_file:
                rows = csv.reader(csv_file, delimiter=',')

                cnt=0
                for row in rows:
                    # print(len(row))
                    if len(row)==3:
                        self.circ_obstacles.append([float(row[0]),float(row[1]),float(row[2])])
                    elif len(row)==5:
                        self.rect_obstacles.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4])])
                    cnt+=1

                self.circ_obstacles = np.array(self.circ_obstacles)
                self.rect_obstacles = np.array(self.rect_obstacles)

                # print(self.rect_obstacles)
        
        self.graph = nx.read_graphml(self.files_dir+"data.graphml")
            
        # self.graph.get  
        # nx.draw_networkx_nodes(self.graph, pos=nx.spring_layout(self.graph))
        # plt.show()
    
    def get_pos_from_node(self, node):
        pos_str = node['coords']
        tmp = pos_str.split(",")
        pos = [float(tmp[0]), float(tmp[1])]

        return pos        

    def plot_data(self):
         
        fig, ax = plt.subplots()

        # nx.draw(self.graph)
        plt.xlim([-15, 15])
        plt.ylim([-15, 15])

        # Constructed Tree
        for e in self.graph.edges(data=True):
            # print()
            src = e[0]
            tgt = e[1]

            p1 = self.get_pos_from_node(self.graph.nodes[src])
            p2 = self.get_pos_from_node(self.graph.nodes[tgt])

            plt.plot([p1[0],p2[0]],[p1[1],p2[1]], color='gray', linewidth=0.2, marker = '.', markersize=1)

        # Planned Path
        plt.plot(self.path[:,0], self.path[:,1], color='red', linewidth=0.75, marker = 'x', markersize=5)

        # Robot Footprint
        for pts in self.path:
            l = 2*1.50
            h = 2*0.75

            pt_3d = np.array([pts[0], pts[1], 0.0])
            rect_pts = []

            rot_mat = tf_utils.euler_matrix(pts[2],0,0,'szyx')[:3,:3]

            rect_pts.append(pt_3d+np.matmul(np.array([-l/2,-h/2,0.0]),rot_mat))
            rect_pts.append(pt_3d+np.matmul(np.array([l/2,-h/2,0.0]),rot_mat))
            rect_pts.append(pt_3d+np.matmul(np.array([l/2,h/2,0.0]),rot_mat))
            rect_pts.append(pt_3d+np.matmul(np.array([-l/2,h/2,0.0]),rot_mat))
            rect_pts.append(pt_3d+np.matmul(np.array([-l/2,-h/2,0.0]),rot_mat))

            for i in range(4):
                clr = 'red'
                if(i==1):
                    clr = 'blue'
                plt.plot([rect_pts[i][0],rect_pts[i+1][0]],[rect_pts[i][1],rect_pts[i+1][1]], color=clr, linewidth=0.75)

        # Circular Obstacles
        for obs in self.circ_obstacles:
            ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2], color='blue'))
        
        # Rectangular Obstacles
        for obs in self.rect_obstacles:
            ax.add_patch(plt.Rectangle((obs[0]-obs[2]/2, obs[1]-obs[3]/2), obs[2], obs[3], angle=np.degrees(obs[4]), color='black'))

        
        
        plt.show()


lmao = PlanDataVisualizer()
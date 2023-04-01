import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl
import sys
from time import perf_counter

class Space(object):

    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.setGeometry(0,110,1920,1080)
        self.w.show()
        self.w.setWindowTitle('Orbit')
        self.w.setCameraPosition(distance=30, elevation=8)

        self.bodies = []
        self.satellites = []
        self.elapsed_time = 0
        self.time = 1
        self.dt = 0.1
        self.refresh_rate = int((1000/60))


    def addGrid(self):
        
        grid = gl.GLGridItem()
        grid.scale(2, 2, 2)
        self.w.addItem(grid)

    
    def set_dt(self, dt):

        self.dt = dt


    def addBodies(self, models):

        self.bodies.append(models)

    
    def addSatellites(self, models):

        self.satellites.append(models)

    
    def update(self):
        
        self.elapsed_time = self.elapsed_time + self.time
        for b in self.bodies:
            
            b.updateSphere()
            
        for s in self.satellites:
            
            s.updateCube()
            
            
    def display(self):
            
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtWidgets.QApplication.instance().exec()
            
    
    def simulateDynamics(self):

        if (self.time<1000):
            for b in self.bodies:
                b.newRotation([3, np.sin(np.deg2rad(self.time)),np.cos(np.deg2rad(self.time)),0])
                b.moveObject([0,0,0])
            for s in self.satellites:
                #euler_dynamics
                s.newRotation([1*self.time, 1,0,0])
                s.moveObject([5*np.sin(np.deg2rad(self.time)),5*np.cos(np.deg2rad(self.time)),0])
            self.time = self.time+1

        else:
            self.time = 0



    def animation(self):
        
        for b in self.bodies:
            self.w.addItem(b.modelw)
        for s in self.satellites:
            self.w.addItem(s.modelw)
            pass

        timer = QtCore.QTimer()
        timer.timeout.connect(self.simulateDynamics)
        timer.timeout.connect(self.update)
        timer.start(self.refresh_rate)
        self.display()



class Sphere(object):
    
    def __init__(self, ):
    
        self.modeld = gl.MeshData.sphere(rows=10,cols=10)
        self.modelw = gl.GLMeshItem(meshdata=self.modeld, drawEdges=True)

        self.modelw.scale(2, 2, 2)


        self.pos = np.array([0,0,0], ndmin=2)
        self.rot = np.array([0,0,0,0], ndmin=2)
        

    def updateSphere(self):
        
        self.modelw.resetTransform()
        self.modelw.translate(self.pos[-1][0],self.pos[-1][1],self.pos[-1][2])
        self.modelw.rotate(self.rot[-1][0],self.rot[-1][1],self.rot[-1][2],
                           self.rot[-1][3], local=True)

    def scaleObject(self):

        self.modelw.resetTransform()
        self.modelw.scale()

    def moveObject(self, new_pos):

        self.pos = np.append(self.pos, [new_pos], axis = 0)
    
    def newRotation(self, new_rot):
        
        self.rot = np.append(self.rot, [new_rot], axis = 0)
        

class Cubesat(object):

    def __init__(self):

        self.Itensor = np.array([4, 4, 1])
        self.verticies_base = np.reshape(np.mgrid[-1:2:2, -1:2:2, -1:2:2].T, (8, 3))
        

        # np.diag(self.Itensor) is a transformation matrix for the verticies
        self.verticies = np.matmul(self.verticies_base, 
                                   (np.divide(np.linalg.inv(np.diag(self.Itensor)),
                                              np.linalg.norm(np.linalg.inv(np.diag(self.Itensor))))))
        
        # Connections between the verticies(like connect the dots)
        self.faces = np.array([[0,3,1],[0,2,3],[0,1,4],
                      [0,2,4],[1,4,5],[1,5,7],
                      [5,6,4],[5,6,7],[3,6,7],
                      [3,2,6],[6,2,4],[0,2,6]])
        
        self.modeld = gl.MeshData(vertexes=self.verticies, faces=self.faces)
        self.modelw = gl.GLMeshItem(meshdata=self.modeld,drawEdges=True)

        self.w_dot = np.array([0, 0, 0],ndmin=2)
        self.rot = np.array([1, 0, 0, 0],ndmin=2)
        self.pos = np.array([0,0,0], ndmin=2)
        

    def updateCube(self):
        
        self.modelw.resetTransform()
        self.modelw.translate(self.pos[-1][0],self.pos[-1][1],self.pos[-1][2])
        self.modelw.rotate(self.rot[-1][0],self.rot[-1][1],self.rot[-1][2],
                           self.rot[-1][3], local=True)

    def scaleObject(self):

        self.modelw.resetTransform()
        self.modelw.scale()

    def moveObject(self, new_pos):

        self.pos = np.append(self.pos, [new_pos], axis = 0)
    
    def newRotation(self, new_rot):
        
        self.rot = np.append(self.rot, [new_rot], axis = 0)


'''class Spacecraft:

    def __init__(self, init_W_dot=[0,0,0], init_attitude=[1,0,0,0]):

        # Initial physical conditions to generate the verticies
        self.Itensor = np.array([3, 3, 6])
        self.verticies_base = np.reshape(np.mgrid[-1:2:2, -1:2:2, -1:2:2].T, (8, 3))

        # np.diag(self.Itensor) is a transformation matrix for the verticies
        self.verticies = np.matmul(self.verticies_base, 
                                   (np.divide(np.linalg.inv(np.diag(self.Itensor)),
                                              np.linalg.norm(np.linalg.inv(np.diag(self.Itensor))))))

        # Connections between the verticies(like connect the dots)
        self.edges = (
                        (0, 1), (0, 2), (0, 4), (3, 2),
                        (3, 7), (3, 1), (5, 7), (5, 4),
                        (5, 1), (6, 4), (6, 7), (6, 2),
                        (8, 9), (8, 10), (8, 11)
                     )
        
        self.angular_velocity = init_W_dot
        self.rot = init_attitude
        self.moments = [0, 0, 0]
        
    def set_inertia(self, Itensor):
        
        self.Itensor = Itensor
        self.verticies = np.matmul(self.verticies_base, 
                                   (np.divide(np.linalg.inv(np.diag(self.Itensor)), np.linalg.norm(np.linalg.inv(np.diag(self.Itensor))))))

    def add_elemetns(self, elements):
        
        self.verticies = np.append(self.verticies, elements, axis=0)

    def reset_moment(self):
        
        self.moments = [0, 0, 0]
    
    def draw_space_craft(self):

        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex in edge:
                glVertex3fv(self.verticies[vertex])
        glEnd()


    def draw_misc_lines(self, verts, eds):

        glBegin(GL_LINES)
        for ed in eds:
            for vert in ed:
                glVertex3fv(verts[vert])
        glEnd()


    def axis_of_rotation_line(self, vector, scale=3):

        self.draw_misc_lines([[0, 0, 0], (np.multiply(vector, scale))],
                        ((0, 1), (0, 0)))


        pass'''



if __name__ == '__main__':

    z = np.sin(np.arange(0, 100, 0.1))


    window = Space()
    moon = Sphere()
    cubesat = Cubesat()
    cubesat1 = Cubesat()
    cubesat2 = Cubesat()
    cubesat3 = Cubesat()
    cubesat4 = Cubesat()



    window.addBodies(moon)
    window.addSatellites(cubesat)
    window.addSatellites(cubesat1)
    window.addSatellites(cubesat2)
    window.addSatellites(cubesat3)
    window.addSatellites(cubesat4)
    cubesat1.modelw.translate(0,0,2)
    cubesat2.modelw.translate(0,0,3)
    cubesat3.modelw.translate(0,0,4)
    cubesat4.modelw.translate(0,0,5)
    window.addGrid()
    
    window.animation()

    

    
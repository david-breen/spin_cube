import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl
import sys
from time import perf_counter
from euler_dynamics import *
from gimbal_commander import *


class Space(object):

    def __init__(self, parent=None):

        self.w = gl.GLViewWidget(parent=parent)
        self.w.setGeometry(0, 110, 1920, 1080)
        self.w.setWindowTitle('Orbit')
        self.w.setCameraPosition(distance=30, elevation=8)

        self.w.show()

        self.bodies = []
        self.satellites = []
        self.inputs = []
        self.elapsed_time = 0
        self.time = 1
        self.dt = 0.01
        self.refresh_rate = int((1000/60))
        self.run_time = 0.1

    def addGrid(self):
        
        grid = gl.GLGridItem()
        grid.scale(2, 2, 2)
        self.w.addItem(grid)

    def set_dt(self, dt):

        self.dt = dt

    def setCameraPos(self):

        self.w.cameraPosition()

    def addBodies(self, models):

        self.bodies.append(models)

    def addSatellites(self, models):

        self.satellites.append(models)

    def addInputs(self, widget):

        self.inputs.append(widget)

    def update(self):
        
        for i in self.inputs:
            
            i.update() 

    def display(self):

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec()

    def simulateDynamics(self):

        if (self.time < 1000):
            for b in self.bodies:
                b.simulateNext(self.time)
                b.updateSphere()
            for s in self.satellites:
                s.simulateNext(self.dt, self.run_time)
                s.updateCube()
            self.time = self.time+self.dt
        else:
            self.time = 0

    def animation(self):
        
        # Here all of the bodies are added to the space
        # it is important that all bodies have an addItem function if they
        # are added using this method
        for b in self.bodies:
            self.w.addItem(b.modelw)
        for s in self.satellites:
            self.w.addItem(s.modelw)
            if(s.camera):
                self.w.addItem(s.modelcw)
            pass

        timer = QtCore.QTimer()
        timer.timeout.connect(self.simulateDynamics)
        timer.timeout.connect(self.update)
        timer.start(self.refresh_rate)
        self.display()


class Sphere(object):
    
    def __init__(self, name=''):
        
        self.name = name
        self.modeld = gl.MeshData.sphere(rows=20, cols=20)
        self.modelw = gl.GLMeshItem(meshdata=self.modeld, drawEdges=True,
                                    smooth=True, color=(1, 0, 0, 1),
                                    shader='shaded')

        self.modelw.scale(20, 20, 20)


        self.pos = np.array([0,0,0], ndmin=2)
        self.rot = np.array([0,0,0,0], ndmin=2)
        self.scale = np.array([0,0,0], ndmin=2)

    def updateSphere(self):
        
        self.modelw.resetTransform()
        self.modelw.scale(self.scale[-1][0],self.scale[-1][1],self.scale[-1][2])
        self.modelw.translate(self.pos[-1][0],self.pos[-1][1],self.pos[-1][2])
        self.modelw.rotate(self.rot[-1][0],self.rot[-1][1],self.rot[-1][2],
                           self.rot[-1][3], local=True)

    def scaleObject(self, new_scale):

        self.scale = np.append(self.scale, [new_scale], axis = 0)

    def moveObject(self, new_pos):

        self.pos = np.append(self.pos, [new_pos], axis = 0)
    
    def newRotation(self, new_rot):
        
        self.rot = np.append(self.rot, [new_rot], axis=0)

    def simulateNext(self, time):

        newRotation = ([90*np.sin(np.deg2rad(time)), 0, 0, 1])
        self.rot = np.append(self.rot, [newRotation], axis=0)


class Cubesat(object):

    def __init__(self, name='', controller=None, orbit=None, camera=True,
                 magnetorquers=True):

        self.name = name
        self.controller = controller
        self.camera = camera
        self.magnetorquers = magnetorquers
        self.Itensor = np.array([4, 4, 1])
        self.verticies_base = np.reshape(np.mgrid[-1:2:2, -1:2:2, -1:2:2].T,
                                         (8, 3))

        # np.diag(self.Itensor) is a transformation matrix for the verticies
        self.verticies = np.matmul(self.verticies_base,
                                   (np.divide(np.linalg.inv(np.diag(self.Itensor)),
                                              np.linalg.norm(np.linalg.inv(np.diag(self.Itensor))))))

        # Connections between the verticies(like connect the dots)
        self.faces = np.array([[0, 3, 1], [0, 2, 3], [0, 1, 4],
                               [0, 2, 4], [1, 4, 5], [1, 5, 7],
                               [5, 6, 4], [5, 6, 7], [3, 6, 7],
                               [3, 2, 6], [6, 2, 4], [0, 2, 6]])

        self.modeld = gl.MeshData(vertexes=self.verticies, faces=self.faces)
        self.modelw = gl.GLMeshItem(meshdata=self.modeld, drawEdges=True)
        self.modelc = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.1, 3.2],
                                           length=5.)

        self.colors = np.ones((self.modelc.faceCount(), 4), dtype=float)
        self.colors[::2, 0] = 0
        self.colors[:, 1] = np.linspace(0, 1, self.colors.shape[0])
        self.modelc.setFaceColors(self.colors)

        self.modelcw = gl.GLMeshItem(meshdata=self.modelc, smooth=True,
                                     drawEdges=True, edgeColor=(1, 0, 0, 1),
                                     shader='balloon')

        self.modell = gl.GLLinePlotItem(pos=[0, 0, 0])
        self.modell.setParentItem(self.modelw)

        self.angular_velocity = np.array([0.3, 0.25, 0.1], ndmin=2)
        self.rot = np.array([1, 0, 0, 0], float, ndmin=2)
        self.pos = np.array([0, 0, 0], ndmin=2)
        self.moments = np.array([0, 0, 0])
        self.setPointECEF = np.array([0, 1, 0])  # defined in ECEF
        self.setPointBF = np.array([0, 1, 0])

        self.kp = 0.1
        self.kd = 1
        self.km = -10
        self.error = 0
        self.derivative = 0
        self.lastError = 0
        self.BFieldECEF = np.array([0, 0, 1])
        self.BField = np.array([0, 0, 1])
        self.BDot = np.array([0, 0, 0])
        self.antennaVector = np.array([0, 0, 1])

    def updateCube(self):

        self.modelw.resetTransform()
        self.modelw.translate(self.pos[-1][0],self.pos[-1][1],self.pos[-1][2])
        new_rot = quaternion_2_euler_aa(self.rot[-1])
        self.modelw.rotate(new_rot[0], new_rot[1], new_rot[2], new_rot[3],
                           local=True)
        if (self.camera):
            self.modelcw.resetTransform()
            # self.modelcw.rotate(90,1,0,0)
            self.modelcw.rotate(new_rot[0], new_rot[1], new_rot[2], new_rot[3], 
                                local=False)
        if (self.magnetorquers):
            lastRot = quaternion_hamilton(self.rot[-2], self.rot[-1])

            self.BField = quaternion_rotation_matrix((self.rot[-1]),
                                                     self.BFieldECEF)
            self.setPointBF = quaternion_rotation_matrix((self.rot[-1]),
                                                          self.setPointECEF)

    def scaleObject(self):

        self.modelw.resetTransform()
        self.modelw.scale()

    def moveObject(self, new_pos):

        self.pos = np.append(self.pos, [new_pos], axis=0)

    def newRotation(self, new_rot):

        self.rot = np.append(self.rot, [new_rot], axis=0)

    def actuatorMoment(self, dutyCycle):
        # dutyCycle is a 1x3 array of the duty cycle of the acturators
        # add check for duty cycle shape in future
        saturationField = np.array([0.1, 0.1, 0.1])  # In Tesla I think
        # Unit vectors for the magnetorquer forces maths
        # used to be setPointBF and BField. Changing to antenna Vector and 
        # setpointBF
        '''quatVector = np.add(np.cross(self.antennaVector, self.setPointBF)*self.kp,
                            np.cross(self.angular_velocity[-1], self.BField)*100)
        print(self.angular_velocity[-1])
        if(np.linalg.norm(quatVector) != 0):
            quatVector = np.divide(quatVector, np.linalg.norm(quatVector))'''
        dipoleVector = np.add(np.cross(self.antennaVector, self.setPointBF)* dutyCycle,
                              np.cross(self.angular_velocity[-1], self.BField)*self.km)
        if(np.linalg.norm(dipoleVector) != 0):
            dipoleVector = np.divide(dipoleVector, np.linalg.norm(dipoleVector))
        # signVector = np.sign(self.setPointBF-self.antennaVector)
        # dipoleVector = np.multiply(dipoleVector, signVector)
        # Multiply the unit vectors by the field strength to get the
        # saturation force
        saturationForce = np.multiply(saturationField, dipoleVector)
        magneticTorque = np.cross(self.BField, saturationForce)
        # This can then be controlled by the controller by duty cycling
        self.moments = magneticTorque
        self.modell.setData(pos=[[0, 0, 0], self.setPointBF*10,
                                 [0, 0, 0], self.BField*5,
                                 [0, 0, 0], dipoleVector*5,
                                 [0, 0, 0], self.antennaVector*5])
        #print(dutyCycle)

    # THIS NEEDS WORK!!! ITS HARD CODED FOR ONLY ONE INPUT
    # THIS NEEDS TO BE IN A QUATERNION FORMAT
    def controllerInput(self):

        Setpoint = [self.controller.x,
                    self.controller.y,
                    1]
        Setpoint = np.divide(Setpoint, np.linalg.norm(Setpoint))
        # remember this***
        self.BFieldECEF = Setpoint

    def controllerPD(self):

        self.lastError = self.error

        dotP = np.dot(self.setPointBF, self.antennaVector)
        normA = np.linalg.norm(self.antennaVector)
        normB = np.linalg.norm(self.setPointBF)
        divNorm = dotP / (normA * normB)
        self.error = np.arccos(divNorm)
        # This has a singularity at 0, but should be fine
        #print(self.error)

        self.derivative = (self.error - self.lastError)
        adjust = (self.kp*self.error + self.kd*self.derivative)
        #adjust = np.clip(adjust, -1, 1)

        return adjust

    def simulateNext(self, dt, run_time):

        # THIS IS A TEMPORARY BODGE
        # FIND A CLEANER MORE FLEXIBLE WAY
        self.controllerInput()
        self.actuatorMoment(self.controllerPD())

        new_angular_velocity, new_rot = simulate_dynamics(self.Itensor,
                                                       self.angular_velocity[-1],
                                                       self.rot[-1],
                                                       self.moments,
                                                       run_time,
                                                       dt,
                                                       just_last=True)

        self.rot = np.append(self.rot, [new_rot], axis=0)
        self.angular_velocity = np.append(self.angular_velocity,
                                          [new_angular_velocity], axis=0)


class Satjoystick(object):

    def __init__(self):

        self.w = pg.JoystickButton()
        self.w.show()
        self.x = 0
        self.y = 0
        self.z = 0

    def update(self):

        self.x, self.y = self.w.getState()


class Textbox(object):

    def __init__(self):
        pass


class Axies(object):

    def __init__(self, body) -> None:

        self.axies = body.rot[-1]
        self.modelw = gl.GLLinePlotItem(pos=self.axies)
        pass

    def transform(self, q1):
        pass

    def scale(self, scale):
        self.axies = self.axies * scale

    def difference():
        pass


class Actuator(object):
    
    def __init__(self):

        #params
        pass


if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    win.setWindowTitle('pyqtgraph example: Flowchart')
    cw = QtWidgets.QWidget()
    win.setCentralWidget(cw)
    layout = QtWidgets.QGridLayout()
    cw.setLayout(layout)

    sim_space = Space()
    layout.addWidget(sim_space.w, 0, 0, 4, 4)
    controls = Satjoystick()
    layout.addWidget(controls.w)

    moon = Sphere()
    earth = Sphere()
    cubesat = Cubesat(controller=controls)

    win.show()

    sim_space.addBodies(moon)
    sim_space.addBodies(earth)
    sim_space.addSatellites(cubesat)
    sim_space.addInputs(controls)
    sim_space.w.addItem(cubesat.modell)

    moon.scaleObject([2, 2, 2])
    moon.moveObject([20, -23, 5])
    earth.scaleObject([20, 20, 20])
    earth.moveObject([0, 25, 0])

    sim_space.addGrid()
    sim_space.animation()

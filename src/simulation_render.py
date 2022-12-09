import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
from euler_dynamics import *
from gimbal_commander import *


# Initial physical conditions to generate the verticies
pygame.init()

font = pygame.font.SysFont('arial', 24)
ui_elements = ["Mi = {:.3e}", "Mj = {:.3e}", "Mk = {:.3e}", 
               "wi = {:.3e}", "wj = {:.3e}", "wk = {:.3e}"]





class Spacecraft:
    
    def __init__(self, Itensor):
        
        # Initial physical conditions to generate the verticies
        #Itensor = np.array([5, 3, 1])  # kg*m^2
        self.Itensor = Itensor
        trans_mat = np.diag(Itensor)
        verticies = np.reshape(np.mgrid[-1:2:2, -1:2:2, -1:2:2].T, (8, 3))

        # Instate principal axies at origin
        verticies = np.append(verticies, [[0, 0, 0], [0.5, 0, 0],
                              [0, 0.5, 0], [0, 0, 0.5]], axis=0)

        self.verticies = np.matmul(verticies, trans_mat)
        # Connections between the verticies(like connect the dots)
        self.edges = (
            (0, 1), (0, 2), (0, 4), (3, 2),
            (3, 7), (3, 1), (5, 7), (5, 4),
            (5, 1), (6, 4), (6, 7), (6, 2),
            (8, 9), (8, 10), (8, 11)
        )


# spacecraft function for drawing the new spacecraft
def draw_space_craft(spacecraft):

    glBegin(GL_LINES)
    for edge in spacecraft.edges:
        for vertex in edge:
            glVertex3fv(spacecraft.verticies[vertex])

    glEnd()


def draw_misc_lines(verts, eds):

    glBegin(GL_LINES)
    for ed in eds:
        for vert in ed:
            glVertex3fv(verts[vert])
    glEnd()


def axis_of_rotation_line(vector, scale = 3):

    draw_misc_lines([[0, 0, 0], (np.multiply(vector, scale))],
                    ((0, 1), (0, 0)))

# Draws the dashboard and UI on the screen
def drawText(x, y, text):                                                

    textSurface = font.render(text, True, (255, 255, 66, 255), (0, 66, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA,
                 GL_UNSIGNED_BYTE, textData)

def renderText():
    
    drawText(0, 100, ui_elements[0].format(10))
    #drawText(0, 75, ui_elements[1].format(moments[1]))
    #drawText(0, 50, ui_elements[2].format(moments[2]))
    #drawText(0, 300, ui_elements[3].format(velocity_mat[0]))
    #drawText(0, 275, ui_elements[4].format(velocity_mat[1]))
    #drawText(0, 250, ui_elements[5].format(velocity_mat[2]))


def check_button_input():

    moments = [0, 0, 0]

    if pygame.key.get_pressed()[K_w]:
        moments[1] = 1
    if pygame.key.get_pressed()[K_s]:
        moments[1] = -1
    if pygame.key.get_pressed()[K_q]:
        moments[0] = 1
    if pygame.key.get_pressed()[K_a]:
        moments[0] = -1
    if pygame.key.get_pressed()[K_e]:
        moments[2] = 1
    if pygame.key.get_pressed()[K_d]:
        moments[2] = -1
    
    return moments


def menu():

    while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                
                if event.type == pygame.MOUSEBUTTONDOWN:
                    break

            
            pygame.display.flip()
            pygame.time.wait(10)


def run_simulation():

    pygame.init()
    display = (1600,1200)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    s1 = Spacecraft(np.array([5, 3, 1]))

    # Initialize the display
    gluPerspective(20, (display[0]/display[1]), 0.1, 70.0)
    glTranslate(0.0, 0.0, -60)
    glRotatef(0, 0, 0, 0)

    # Time stuff
    run_time = 0.01
    dt = 0.001

    # initial conditions
    initial_velocity = [0.0, 0, 1]    # rad/s
    initial_attitude = [1, 0, 0, 0] # Unit quaternion orientation [s, i, j, k]
    moments = [0, 0, 0]     # Moments applied to the spacecraft in the B frame
    
    init_gimbals()

    # Keyboard inputs for forces on the spacecraft
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                pass

        #running simulation for a single frame
        velocity_mat, attitude_mat = simulate_dynamics(s1.Itensor,
                                                       initial_velocity,
                                                       initial_attitude,
                                                       moments,
                                                       run_time,
                                                       dt,
                                                       just_last=True)
                
        O, x, y, z = quaternion_rotation(initial_attitude, attitude_mat)
        initial_attitude = attitude_mat
        initial_velocity = velocity_mat
        #quaternion_rotation([0, 0, 0, 1], attitude_mat )
        z_axis = quaternion_rotation_matrix(attitude_mat, [0, 0, 1])
        y_axis = quaternion_rotation_matrix(attitude_mat, [0, 1, 0])
        x_axis = quaternion_rotation_matrix(attitude_mat, [0, 0, 0])
        
        glRotatef(O, x, y, z)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        renderText()
        gimbal_movement(z_axis, velocity_mat)

        # Drawing all of the vectors
        draw_space_craft(s1)
        #axis_of_rotation_line(rot_line)
        
        pygame.display.flip()
        pygame.time.wait(10)

run_simulation()
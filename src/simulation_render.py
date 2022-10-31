import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as npg
from euler_dynamics import *


# Initial physical conditions to generate the verticies
pygame.init()
Itensor = np.array([2, 5, 2])  # kg*m^2
trans_mat = np.diag(Itensor)
verticies = np.reshape(np.mgrid[-1:2:2,-1:2:2,-1:2:2].T, (8,3))
verticies = np.matmul(verticies, trans_mat)

font = pygame.font.SysFont('arial', 24)
ui_elements = ["Mi = {:.3e}", "Mj = {:.3e}", "Mk = {:.3e}", 
               "wi = {:.3e}", "wj = {:.3e}", "wk = {:.3e}"]

# Connections between the verticies
edges = (
    (0,1), (0,2), (0,4), (3,2),
    (3,7), (3,1), (5,7), (5,4),
    (5,1), (6,4), (6,7), (6,2)
)

# spacecraft function for drawing the new spacecraft
def draw_space_craft():

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])

    glEnd()


def drawText(x, y, text):                                                

    textSurface = font.render(text, True, (255, 255, 66, 255), (0, 66, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA,
                 GL_UNSIGNED_BYTE, textData)


def main():

    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    # Initialize the display
    gluPerspective(20, (display[0]/display[1]), 0.1, 70.0)
    glTranslate(0.0, 0.0, -60)
    glRotatef(0, 0, 0, 0)

    # Time stuff
    run_time = 0.1
    dt = 0.001

    # initial conditions
    initial_velocity = [0, 0, 0]    # rad/s
    initial_attitude = [1, 0, 0, 0] # Unit quaternion orientation [s, i, j, k]
    moments = [0, 0, 0]     # Moments applied to the spacecraft in the B frame

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
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

        velocity_mat, attitude_mat = simulate_dynamics(Itensor,
                                                       initial_velocity,
                                                       initial_attitude,
                                                       moments,
                                                       run_time,
                                                       dt,
                                                       just_last=True)
                
        O, x, y, z = quaternion_rotation(initial_attitude, attitude_mat)
        initial_attitude = attitude_mat
        initial_velocity = velocity_mat
        
        glRotatef(O, x, y, z)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        draw_space_craft()

        drawText(0, 100, ui_elements[0].format(moments[0]))
        drawText(0, 75, ui_elements[1].format(moments[1]))
        drawText(0, 50, ui_elements[2].format(moments[2]))
        drawText(0, 300, ui_elements[3].format(velocity_mat[0]))
        drawText(0, 275, ui_elements[4].format(velocity_mat[1]))
        drawText(0, 250, ui_elements[5].format(velocity_mat[2]))
        pygame.display.flip()
        pygame.time.wait(10)

main()
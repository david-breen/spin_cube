from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

line1 = np.array([[-0.5, 0.0], [0.5, 0.0]]) # Coordinates for line 1
line2 = np.array([[0.0, -0.5], [0.0, 0.5]]) # Coordinates for line 2

def display():
    glClear(GL_COLOR_BUFFER_BIT)

    glColor3f(1.0, 1.0, 1.0) # Set the color for the lines

    glBegin(GL_LINES) # Begin drawing lines
    glVertex2f(line1[0][0], line1[0][1])
    glVertex2f(line1[1][0], line1[1][1])
    glEnd() # End drawing line 1

    glBegin(GL_LINES) # Begin drawing line 2
    glVertex2f(line2[0][0], line2[0][1])
    glVertex2f(line2[1][0], line2[1][1])
    glEnd() # End drawing line 2

    glutSwapBuffers() # Swap the front and back buffers

def rotate():
    global line1, line2

    glClear(GL_COLOR_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)

    # Rotate line 1 clockwise
    glLoadIdentity()
    glRotatef(0.5, 0.0, 0.0, 1.0)
    line1 = np.dot(line1, glMatrixMode(GL_MODELVIEW_MATRIX))

    # Rotate line 2 counterclockwise
    glLoadIdentity()
    glRotatef(-0.5, 0.0, 0.0, 1.0)
    line2 = np.dot(line2, glMatrixMode(GL_MODELVIEW_MATRIX))

    glutPostRedisplay() # Mark the current window as needing to be redisplayed
    glutTimerFunc(30, rotate, 0) # Set a timer to call the rotate function again after 30 milliseconds

def main():
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
    glutInitWindowSize(500, 500)
    glutCreateWindow("Rotating Lines")
    glutDisplayFunc(display)
    glutTimerFunc(30, rotate, 0)
    glutMainLoop()

if __name__ == '__main__':
    main()

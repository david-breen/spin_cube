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
pygame.display.set_caption('MS. spaceflight simulator 95')

font = pygame.font.SysFont('arial', 24)
ui_elements = ["Mi = {:.3e}", "Mj = {:.3e}", "Mk = {:.3e}", 
               "wi = {:.3e}", "wj = {:.3e}", "wk = {:.3e}"]

dt = 0.1
run_time = 0.5
com_port  = "COM6"


class Spacecraft:

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
        self.attitude = init_attitude
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


        pass


class Button:

    def __init__(self, x, y, image):
        self.image = image
        self.rect = self.image.get_rect()
        self.rect.topleft = (x, y)
        self.clicked = False

    def draw(self, screen):
        screen.blit(self.image, (self.rect.x, self.rect.y))

    def handle_event(self, mouse_pos):
        
        if self.rect.collidepoint(mouse_pos):
            if pygame.mouse.get_pressed()[0] == 1:
                self.clicked = True

        return self.clicked


class Slider:
    
    def __init__(self, x, y, color, scale_x=1, scale_y=1 ):
        
        self.color = color
        self.rect = pygame.Rect(x, y, 10, 10)
        #self.rect.scale_by(scale_x, scale_y)

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)
        #screen.blit(self.rect)


class SliderBar:
    def __init__(self, x, y, width, height, min_value, 
                 max_value, initial_value):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.min_value = min_value
        self.max_value = max_value
        self.value = initial_value
        self.slider_rect = pygame.Rect(self.x, self.y, self.width, self.height)

    def draw(self, screen):
        pygame.draw.rect(screen, (255, 255, 255), 
                         self.slider_rect, border_radius=10)
        slider_width = int((self.value - self.min_value) / 
                           (self.max_value - self.min_value) * 
                           (self.width))
        slider_rect = pygame.Rect(self.x + (self.x/2), self.y + (self.y/2), 
                                  slider_width, self.height/2)
        pygame.draw.rect(screen, (0, 0, 255), slider_rect, border_radius=10)

    def update(self, mouse_pos):

        if self.slider_rect.collidepoint(mouse_pos):
            if pygame.mouse.get_pressed()[0] == 1:

                self.value = int((((mouse_pos[0] - self.x) * 
                                   (self.max_value - self.min_value)) / 
                                   self.width) + self.min_value)

    def get_value(self):
        return self.value


class TextBox:
    def __init__(self, x, y, width, height, font_size=32, text_color=(255,0,0), 
                 box_color=(255,255,255), initial_text='1'):
        self.rect = pygame.Rect(x, y, width, height)
        self.font = pygame.font.Font(None, font_size)
        self.text_color = text_color
        self.box_color = box_color
        self.text = initial_text
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = True
            else:
                self.active = False
            return self.active
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_RETURN:
                    self.active = False
                elif event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    self.text += event.unicode
                    
        

    def draw(self, screen):
        
        pygame.draw.rect(screen, self.box_color, self.rect, 1)
        text_surface = self.font.render(self.text, True, self.text_color)
        screen.blit(text_surface, (self.rect.x+5, self.rect.y+5))


class TextPG:

    def __init__(self, x, y, font_size=32, text_color=(255,0,0), text=''):

        self.x = x
        self.y = y
        self.fontsize = font_size
        self.font = pygame.font.Font(None, font_size)
        self. text_color = text_color
        self.text = text
    
    def new_text(self, text):

        self.text = text

    def draw(self, screen):
        text_surface = self.font.render(self.text, True, self.text_color)
        screen.blit(text_surface, (self.x, self.y))


#class Sphere:






# Draws the dashboard and UI on the screen
def drawTextGL(x, y, text):                                                

    textSurface = font.render(text, True, (255, 255, 66, 255), (0, 66, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA,
                 GL_UNSIGNED_BYTE, textData)


def renderTextGL(moments, velocity_mat):

    drawTextGL(0, 100, ui_elements[0].format(moments[0]))
    drawTextGL(0, 75, ui_elements[1].format(moments[1]))
    drawTextGL(0, 50, ui_elements[2].format(moments[2]))
    drawTextGL(0, 300, ui_elements[3].format(velocity_mat[0]))
    drawTextGL(0, 275, ui_elements[4].format(velocity_mat[1]))
    drawTextGL(0, 250, ui_elements[5].format(velocity_mat[2]))


def menu(spacecraft):

    pygame.init()
    display = (800, 400)
    screen = pygame.display.set_mode(display)

    start_img = pygame.image.load('images/RunSimulationButtonBlack.png')
    menu_img = pygame.image.load('images/ParametersButtonBlack.png')
    
    start_button = Button(100, 200, start_img)
    menu_button = Button(500, 200, menu_img)

    s1 = spacecraft
    pygame.time.wait(100)
    while True:
        mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if start_button.handle_event(mouse_pos):
                run_simulation(s1)
            if menu_button.handle_event(mouse_pos):
                s1 = settings_menu(s1)
        start_button.draw(screen)
        menu_button.draw(screen)        

        pygame.display.flip()
        pygame.time.wait(10)


def settings_menu(spacecraft):

    pygame.init()
    display = (800, 400)
    screen = pygame.display.set_mode(display)

    return_img = pygame.image.load('images/ReturnButtonBlack.png')
    return_button = Button(100, 300, return_img)

    global dt
    global run_time

    s1 = spacecraft
    
    textbox1 = TextBox(100, 100, 100, 30, initial_text=str(s1.Itensor[0]))
    textbox2 = TextBox(100, 150, 100, 30, initial_text=str(s1.Itensor[1]))
    textbox3 = TextBox(100, 200, 100, 30, initial_text=str(s1.Itensor[2]))
    textbox4 = TextBox(300, 100, 50, 30, initial_text=str(dt))
    textbox5 = TextBox(350, 200, 50, 30, initial_text=str(run_time))
    label1 = TextPG(50,100,text='Ixx')
    label2 = TextPG(50,150,text='Iyy')
    label3 = TextPG(50,200,text='Izz')
    label4 = TextPG(270,100,text='dt')
    label5 = TextPG(250,200,text='runtime')

    pygame.time.wait(100)
    while True:
        screen.fill((0, 0, 0))
        mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if return_button.handle_event(mouse_pos): 
                menu(spacecraft)
            if (textbox1.active or textbox2.active or textbox3.active):
                if(len(textbox1.text)>0 and 
                   len(textbox2.text)>0 and 
                   len(textbox3.text)>0):
                    spacecraft.set_inertia([int(textbox1.text),
                                            int(textbox2.text),
                                            int(textbox3.text)])
            if (textbox4.active or textbox5.active):
                if(len(textbox4.text)>0 and 
                   len(textbox5.text)>0):
                    print("activated")
                    dt = float(textbox4.text)
                    run_time = float(textbox5.text)
                                            
            textbox1.handle_event(event)
            textbox2.handle_event(event)
            textbox3.handle_event(event)
            textbox4.handle_event(event)
            textbox5.handle_event(event)
            
        
        return_button.draw(screen)
        textbox1.draw(screen)
        textbox2.draw(screen)
        textbox3.draw(screen)
        textbox4.draw(screen)
        textbox5.draw(screen)
        label1.draw(screen)
        label2.draw(screen)
        label3.draw(screen)
        label4.draw(screen)
        label5.draw(screen)
                    
        pygame.display.flip()
        pygame.time.wait(50)

def run_simulation(spacecraft):

    pygame.init()
    display = (1800, 1000)
    screen = pygame.display.set_mode(display, HWSURFACE|DOUBLEBUF|OPENGL)

    s1 = spacecraft
    s1.add_elemetns([[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])


    # Initialize the display
    gluPerspective(20, (display[0]/display[1]), 0.1, 70.0)
    glTranslate(0.0, 0.0, -20)


    # initial conditions
    initial_attitude = s1.attitude  # Unit quaternion orientation [s, i, j, k]

    init_gimbals(com_port)

    # Keyboard inputs for forces on the spacecraft
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                pass
            if pygame.key.get_pressed()[K_w]:
                s1.moments[1] = 0.1
            if pygame.key.get_pressed()[K_s]:
                s1.moments[1] = -0.1
            if pygame.key.get_pressed()[K_q]:
                s1.moments[0] = 0.1
            if pygame.key.get_pressed()[K_a]:
                s1.moments[0] = -0.1
            if pygame.key.get_pressed()[K_e]:
                s1.moments[2] = 0.1
            if pygame.key.get_pressed()[K_d]:
                s1.moments[2] = -0.1
            
        
            


        # running simulation for a single frame
        s1.angular_velocity, s1.attitude = simulate_dynamics(s1.Itensor,
                                                       s1.angular_velocity,
                                                       s1.attitude,
                                                       s1.moments,
                                                       run_time,
                                                       dt,
                                                       just_last=True)

        O, x, y, z = quaternion_rotation(initial_attitude, s1.attitude)
        initial_attitude = s1.attitude
        
        
        # quaternion_rotation([0, 0, 0, 1], s1.attitude )
        z_axis = quaternion_rotation_matrix(s1.attitude, [0, 0, 1])
        x_axis = quaternion_rotation_matrix(s1.attitude, [1, 0, 0])



        glRotatef(O, x, y, z)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        renderTextGL(s1.moments, s1.angular_velocity)
        gimbal_movement(z_axis, x_axis, s1.angular_velocity)
        s1.reset_moment()
        # Drawing all of the vectors
        s1.draw_space_craft()
        # axis_of_rotation_line(z_axis)
        #axis_of_rotation_line(axis_of_rotation(s1.angular_velocity), scale=8)
        pygame.display.flip()
        pygame.time.wait(1)


if __name__ == "__main__":

    s1 = Spacecraft()
    menu(s1)
    '''print(simulate_dynamics([4,4,1],
                                                       [0,0,0],
                                                       [1,0,0,0],
                                                       [0,0,1],
                                                       run_time,
                                                       dt,
                                                       just_last=True))'''

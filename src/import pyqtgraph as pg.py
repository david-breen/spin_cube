import pyqtgraph as pg
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton


mesh_size = 10
x = np.linspace(-10, 10, mesh_size)
y = np.linspace(-10, 10, mesh_size)
z = np.linspace(-10, 10, mesh_size)
xx, yy, zz = np.meshgrid(x, y, z)
pts = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

scalar_field = np.random.rand(mesh_size**3)

window = new QWidget

mesh = pg.GLMeshItem(vertexes=pts, faces=pg.makeMeshFaces(mesh_size, mesh_size, mesh_size), smooth=True)
mesh.setData(vertexColors=scalar_field, shader='shaded', glOptions='translucent')

button1 = QPushButton('Button 1')
button2 = QPushButton('Button 2')
button_layout = QHBoxLayout()
button_layout.addWidget(button1)
button_layout.addWidget(button2)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mesh with Buttons')
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        layout.addWidget(mesh.widget)
        layout.addLayout(button_layout)
        central_widget.setLayout(layout)

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()

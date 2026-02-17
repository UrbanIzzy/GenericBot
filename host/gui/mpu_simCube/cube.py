import socket
import threading
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from collections import deque

# --- Configuration ---
WINDOW_SIZE = (800, 600)
GRAPH_HEIGHT = 200
TARGET =("127.0.0.1", 5000)

# CUBE RATIO DIMENSIONS: 1 (Z) : 3 (Y) : 5 (X)
# Width = 5 (-2.5 to 2.5), Height = 3 (0 to 3), Depth = 1 (-0.5 to 0.5)
# Set ROT_POINT to (0, 1.5, 0) if you want to rotate around the exact center.
ROT_POINT = (0, 0, 0) 

def draw_grid(size=20, step=1):
    glLineWidth(1)
    glColor3f(0.2, 0.2, 0.2)
    glBegin(GL_LINES)
    for i in range(-size, size + 1, step):
        glVertex3f(-size, 0, i); glVertex3f(size, 0, i)
        glVertex3f(i, 0, -size); glVertex3f(i, 0, size)
    glEnd()

def draw_cube():
    """Draws a rectangular prism with a 1:3:5 ratio."""
    glBegin(GL_QUADS)
    # Proportions: X=5 (Width), Y=3 (Height), Z=1 (Depth)
    faces = [
        # Front face (Red)
        ((1,0,0), [(2.5, 1, 1.5), (-2.5, 1, 1.5), (-2.5, 0, 1.5), (2.5, 0, 1.5)]),
        # Back face (Green)
        ((0,1,0), [(2.5, 1,-1.5), (-2.5, 1,-1.5), (-2.5, 0,-1.5), (2.5, 0,-1.5)]),
        # Left face (Blue)
        ((0,0,1), [(-2.5, 1, 1.5), (-2.5, 1,-1.5), (-2.5, 0,-1.5), (-2.5, 0, 1.5)]),
        # Right face (Yellow)
        ((1,1,0), [(2.5, 1, 1.5), (2.5, 1,-1.5), (2.5, 0,-1.5), (2.5, 0, 1.5)]),
        # Top face (Cyan)
        ((0,1,1), [(2.5, 1, 1.5), (-2.5, 1, 1.5), (-2.5, 1,-1.5), (2.5, 1,-1.5)]),
        # Bottom face (Magenta)
        ((1,0,1), [(2.5, 0, 1.5), (-2.5, 0, 1.5), (-2.5, 0,-1.5), (2.5, 0,-1.5)])
        ]
    for color, verts in faces:
        glColor3fv(color)
        for v in verts: glVertex3fv(v)
    glEnd()

def draw_ui_text(x, y, text, color):
    """Renders text at screen coordinates with transparency."""
    font = pygame.font.SysFont('Consolas', 22, bold=True)
    surface = font.render(text, True, color)
    text_data = pygame.image.tostring(surface, "RGBA", True)
    
    # glWindowPos coordinates start from bottom-left
    glWindowPos2d(x, WINDOW_SIZE[1] - y - surface.get_height())
    glDrawPixels(surface.get_width(), surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

def draw_angle_graph(h_x, h_y, h_z, h_len):
    glDisable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, h_len, -180, 180, -1, 1)

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    # 1. Background
    glColor4f(0.07, 0.07, 0.07, 1.0)
    glBegin(GL_QUADS)
    glVertex2f(0, -180); glVertex2f(h_len, -180)
    glVertex2f(h_len, 180); glVertex2f(0, 180)
    glEnd()

    # 2. Fine Grid with Thinner Lines
    glLineWidth(0.5)
    glColor4f(0.15, 0.15, 0.15, 1.0)
    glBegin(GL_LINES)
    # Vertical lines
    for x in range(0, h_len, 20):
        glVertex2f(x, -180); glVertex2f(x, 180)
    # Horizontal lines (angles)
    for y in range(-180, 181, 20):
        glVertex2f(0, y); glVertex2f(h_len, y)
    glEnd()

    # 3. Data Lines
    glLineWidth(2)
    for data, color in [(h_x, (255, 60, 60)), (h_y, (60, 255, 60)), (h_z, (80, 80, 255))]:
        glColor3ub(*color)
        glBegin(GL_LINE_STRIP)
        for i, val in enumerate(data):
            glVertex2f(i, val)
        glEnd()
    
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glEnable(GL_DEPTH_TEST)

# -----------------------------
# Networking & Threading
# -----------------------------
angles = {"x": 0.0, "y": 0.0, "z": 0.0}

def socket_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(TARGET)
    s.listen(1)
    try:
        conn, _ = s.accept()
        while True:
            data = conn.recv(64)
            if not data: break
            parts = data.decode().strip().split()
            if len(parts) == 3:
                angles["x"], angles["y"], angles["z"] = map(float, parts)
    except: pass
    finally: s.close()

def main():
    pygame.init()
    pygame.display.set_mode(WINDOW_SIZE, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Telemetry Tool - 1:3:5 Ratio")
    
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    threading.Thread(target=socket_thread, daemon=True).start()

    history_len = 300
    h_x, h_y, h_z = [deque([0]*history_len, maxlen=history_len) for _ in range(3)]
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); return

        h_x.append(angles["x"]); h_y.append(angles["y"]); h_z.append(angles["z"])
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # --- 3D VIEWPORT ---
        glViewport(0, GRAPH_HEIGHT, WINDOW_SIZE[0], WINDOW_SIZE[1] - GRAPH_HEIGHT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, WINDOW_SIZE[0] / (WINDOW_SIZE[1] - GRAPH_HEIGHT), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, -3, -20) # Moved back further to fit the larger cube
        
        draw_grid()

        glPushMatrix()
        glTranslatef(ROT_POINT[0], ROT_POINT[1], ROT_POINT[2])
        glRotatef(angles["x"], 1, 0, 0)
        glRotatef(angles["y"], 0, 1, 0)
        glRotatef(angles["z"], 0, 0, 1)
        glTranslatef(-ROT_POINT[0], -ROT_POINT[1], -ROT_POINT[2])
        draw_cube()
        glPopMatrix()

        # --- GRAPH VIEWPORT ---
        glViewport(0, 0, WINDOW_SIZE[0], GRAPH_HEIGHT)
        draw_angle_graph(h_x, h_y, h_z, history_len)

        # --- TOP-LEFT INFO BOX (Absolute Window Space) ---
        draw_ui_text(20, 20, f"x: {int(angles['x'])}", (255, 60, 60))
        draw_ui_text(20, 50, f"y: {int(angles['y'])}", (60, 255, 60))
        draw_ui_text(20, 80, f"z: {int(angles['z'])}", (80, 80, 255))

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
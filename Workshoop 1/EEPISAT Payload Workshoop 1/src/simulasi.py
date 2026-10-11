
import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import numpy as np
import math
import time

# ============================================================
# SERIAL
# ============================================================

SERIAL_PORT = "COM5"       # GANTI SESUAI PORT ESP32
BAUD_RATE = 115200

ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=0.01
)

time.sleep(2)

# ============================================================
# QUATERNION
# ============================================================

qw = 1.0
qx = 0.0
qy = 0.0
qz = 0.0


def normalize_quaternion(q):

    norm = np.linalg.norm(q)

    if norm == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


# ============================================================
# QUATERNION -> ROTATION MATRIX
# ============================================================

def quaternion_to_matrix(q):

    w, x, y, z = q

    return np.array([
        [
            1 - 2*y*y - 2*z*z,
            2*x*y - 2*z*w,
            2*x*z + 2*y*w,
            0
        ],

        [
            2*x*y + 2*z*w,
            1 - 2*x*x - 2*z*z,
            2*y*z - 2*x*w,
            0
        ],

        [
            2*x*z - 2*y*w,
            2*y*z + 2*x*w,
            1 - 2*x*x - 2*y*y,
            0
        ],

        [
            0,
            0,
            0,
            1
        ]
    ], dtype=np.float32)


# ============================================================
# SERIAL READER
# ============================================================

def read_serial():

    global qw, qx, qy, qz

    try:

        while ser.in_waiting:

            line = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            # Mencari baris:
            #
            # W: 0.998532 | X: 0.012341 |
            # Y: -0.034521 | Z: 0.041223

            if (
                line.startswith("W:") and
                "| X:" in line and
                "| Y:" in line and
                "| Z:" in line
            ):

                parts = line.split("|")

                if len(parts) >= 4:

                    w = float(parts[0].replace("W:", "").strip())
                    x = float(parts[1].replace("X:", "").strip())
                    y = float(parts[2].replace("Y:", "").strip())
                    z = float(parts[3].replace("Z:", "").strip())

                    q = normalize_quaternion(
                        np.array([w, x, y, z])
                    )

                    qw = q[0]
                    qx = q[1]
                    qy = q[2]
                    qz = q[3]

    except Exception:
        pass


# ============================================================
# OPENGL
# ============================================================

def init_opengl(width, height):

    glEnable(GL_DEPTH_TEST)

    glEnable(GL_COLOR_MATERIAL)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    glLightfv(
        GL_LIGHT0,
        GL_POSITION,
        (5, 5, 10, 1)
    )

    glLightfv(
        GL_LIGHT0,
        GL_DIFFUSE,
        (1, 1, 1, 1)
    )

    glClearColor(
        0.03,
        0.03,
        0.05,
        1
    )

    glMatrixMode(GL_PROJECTION)

    gluPerspective(
        45,
        width / height,
        0.1,
        100
    )

    glMatrixMode(GL_MODELVIEW)


# ============================================================
# CYLINDER
# ============================================================

def draw_cylinder(radius, height, slices=32):

    quad = gluNewQuadric()

    # Cylinder
    gluCylinder(
        quad,
        radius,
        radius,
        height,
        slices,
        1
    )

    # Bottom
    glPushMatrix()

    glRotatef(
        180,
        1,
        0,
        0
    )

    gluDisk(
        quad,
        0,
        radius,
        slices,
        1
    )

    glPopMatrix()

    # Top
    glPushMatrix()

    glTranslatef(
        0,
        0,
        height
    )

    gluDisk(
        quad,
        0,
        radius,
        slices,
        1
    )

    glPopMatrix()

    gluDeleteQuadric(quad)


# ============================================================
# CONE
# ============================================================

def draw_cone(
    radius,
    height,
    slices=32
):

    quad = gluNewQuadric()

    gluCylinder(
        quad,
        radius,
        0,
        height,
        slices,
        1
    )

    gluDeleteQuadric(quad)


# ============================================================
# FIN
# ============================================================

def draw_fin():

    glBegin(GL_TRIANGLES)

    glVertex3f(0, 0, 0)

    glVertex3f(0.7, 0, 0.0)

    glVertex3f(0.15, 0, 1.2)

    glEnd()


# ============================================================
# ROCKET
# ============================================================

def draw_rocket():

    # --------------------------------------------------------
    # BODY
    # --------------------------------------------------------

    glColor3f(
        0.75,
        0.78,
        0.82
    )

    draw_cylinder(
        0.45,
        2.5
    )

    # --------------------------------------------------------
    # NOSE CONE
    # --------------------------------------------------------

    glPushMatrix()

    glTranslatef(
        0,
        0,
        2.5
    )

    glColor3f(
        0.9,
        0.1,
        0.1
    )

    draw_cone(
        0.45,
        0.9
    )

    glPopMatrix()

    # --------------------------------------------------------
    # BOTTOM ENGINE
    # --------------------------------------------------------

    glColor3f(
        0.15,
        0.15,
        0.15
    )

    glPushMatrix()

    glTranslatef(
        0,
        0,
        -0.4
    )

    draw_cylinder(
        0.30,
        0.4
    )

    glPopMatrix()

    # --------------------------------------------------------
    # FIN 1
    # --------------------------------------------------------

    glColor3f(
        0.9,
        0.15,
        0.05
    )

    glPushMatrix()

    glTranslatef(
        0.42,
        0,
        0.1
    )

    glRotatef(
        90,
        0,
        1,
        0
    )

    draw_fin()

    glPopMatrix()

    # --------------------------------------------------------
    # FIN 2
    # --------------------------------------------------------

    glPushMatrix()

    glTranslatef(
        -0.42,
        0,
        0.1
    )

    glRotatef(
        -90,
        0,
        1,
        0
    )

    draw_fin()

    glPopMatrix()

    # --------------------------------------------------------
    # FIN 3
    # --------------------------------------------------------

    glPushMatrix()

    glTranslatef(
        0,
        0.42,
        0.1
    )

    glRotatef(
        90,
        1,
        0,
        0
    )

    draw_fin()

    glPopMatrix()

    # --------------------------------------------------------
    # FIN 4
    # --------------------------------------------------------

    glPushMatrix()

    glTranslatef(
        0,
        -0.42,
        0.1
    )

    glRotatef(
        -90,
        1,
        0,
        0
    )

    draw_fin()

    glPopMatrix()


# ============================================================
# AXIS
# ============================================================

def draw_axis():

    glDisable(GL_LIGHTING)

    # X = RED
    glColor3f(1, 0, 0)

    glBegin(GL_LINES)

    glVertex3f(0, 0, 0)
    glVertex3f(3, 0, 0)

    glEnd()

    # Y = GREEN
    glColor3f(0, 1, 0)

    glBegin(GL_LINES)

    glVertex3f(0, 0, 0)
    glVertex3f(0, 3, 0)

    glEnd()

    # Z = BLUE
    glColor3f(0, 0.5, 1)

    glBegin(GL_LINES)

    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 3)

    glEnd()

    glEnable(GL_LIGHTING)


# ============================================================
# GRID
# ============================================================

def draw_grid():

    glDisable(GL_LIGHTING)

    glColor3f(
        0.15,
        0.15,
        0.18
    )

    glBegin(GL_LINES)

    for i in range(-10, 11):

        glVertex3f(
            i,
            -10,
            0
        )

        glVertex3f(
            i,
            10,
            0
        )

        glVertex3f(
            -10,
            i,
            0
        )

        glVertex3f(
            10,
            i,
            0
        )

    glEnd()

    glEnable(GL_LIGHTING)


# ============================================================
# TEXT
# ============================================================

def draw_text(
    text,
    x,
    y,
    font
):

    surface = font.render(
        text,
        True,
        (255, 255, 255)
    )

    texture = pygame.image.tostring(
        surface,
        "RGBA",
        True
    )

    width = surface.get_width()
    height = surface.get_height()

    glDisable(GL_LIGHTING)

    glWindowPos2d(
        x,
        y
    )

    glDrawPixels(
        width,
        height,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        texture
    )

    glEnable(GL_LIGHTING)


# ============================================================
# MAIN
# ============================================================

def main():

    pygame.init()

    width = 1200
    height = 800

    pygame.display.set_mode(
        (
            width,
            height
        ),
        DOUBLEBUF | OPENGL
    )

    pygame.display.set_caption(
        "ESP32 Rocket Quaternion Visualizer"
    )

    init_opengl(
        width,
        height
    )

    font = pygame.font.SysFont(
        "consolas",
        20
    )

    clock = pygame.time.Clock()

    running = True

    camera_distance = 8.0

    while running:

        # ----------------------------------------------------
        # EVENTS
        # ----------------------------------------------------

        for event in pygame.event.get():

            if event.type == QUIT:

                running = False

        # ----------------------------------------------------
        # SERIAL
        # ----------------------------------------------------

        read_serial()

        # ----------------------------------------------------
        # CLEAR
        # ----------------------------------------------------

        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT
        )

        # ----------------------------------------------------
        # CAMERA
        # ----------------------------------------------------

        glMatrixMode(
            GL_MODELVIEW
        )

        glLoadIdentity()

        gluLookAt(
            camera_distance,
            camera_distance,
            camera_distance,
            0,
            0,
            1,
            0,
            0,
            1
        )

        # ----------------------------------------------------
        # GRID
        # ----------------------------------------------------

        draw_grid()

        # ----------------------------------------------------
        # AXIS
        # ----------------------------------------------------

        draw_axis()

        # ----------------------------------------------------
        # ROCKET
        # ----------------------------------------------------

        glPushMatrix()

        # Quaternion rotation
        rotation_matrix = quaternion_to_matrix(
            np.array([
                qw,
                qx,
                qy,
                qz
            ])
        )

        glMultMatrixf(
            rotation_matrix.T
        )

        draw_rocket()

        glPopMatrix()

        # ----------------------------------------------------
        # INFO
        # ----------------------------------------------------

        draw_text(
            f"Quaternion",
            20,
            height - 30,
            font
        )

        draw_text(
            f"W : {qw: .5f}",
            20,
            height - 55,
            font
        )

        draw_text(
            f"X : {qx: .5f}",
            20,
            height - 80,
            font
        )

        draw_text(
            f"Y : {qy: .5f}",
            20,
            height - 105,
            font
        )

        draw_text(
            f"Z : {qz: .5f}",
            20,
            height - 130,
            font
        )

        # ----------------------------------------------------
        # FPS
        # ----------------------------------------------------

        pygame.display.flip()

        clock.tick(60)

    # --------------------------------------------------------
    # CLOSE
    # --------------------------------------------------------

    ser.close()

    pygame.quit()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()

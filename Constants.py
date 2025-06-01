class Constants:

    DXF_FILE = r"C:/Users/msgaf/Desktop/Arduino summer 2025/m.dxf"    
    SERIAL_PORT = "COM3"
    BAUD_RATE = 9600

    # Link lengths
    l1, l2, l3, l4, l5, a = 3.0, 6.0, 7.5, 7.5, 6.0, 1.125

    # Initial marker position
    START_XY = (0.0, 0.0)
    START_X = START_XY[0]
    START_Y = START_XY[1]

    # Motor values
    MAX_RPM = 3.0
    STEPS_PER_REV = 4096.0
    MAX_STEPS_PER_SEC = MAX_RPM * STEPS_PER_REV / 60.0
    DEGREES_PER_STEP = 360.0 / STEPS_PER_REV

    TOTAL_POINT_COUNT = 600
    INITIAL_SPACING = 0.02

    # Drawing Area
    DRAWABLE_WIDTH = 10
    DRAWABLE_HEIGHT = 5
    ASPECT_RATIO = DRAWABLE_HEIGHT / DRAWABLE_WIDTH

    # Translation adjustment Board -> Robot
    OFFSET_X = -3.5
    OFFSET_Y = 6.5

    DRAWABLE_CENTER_X = OFFSET_X + DRAWABLE_WIDTH/2
    DRAWABLE_CENTER_Y = OFFSET_Y + DRAWABLE_HEIGHT/2
import ezdxf
import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# === CONFIGURATION ===
DXF_FILE = r"C:/Users/msgaf/Desktop/Arduino summer 2025/wpi_text.dxf"
SERIAL_PORT = "COM3"
BAUD_RATE = 9600

# Link lengths
l1, l2, l3, l4, l5, a = 3.0, 6.0, 7.5, 7.5, 6.0, 1.125

# Initial marker position
start_xy = (0.0, 0.0)

start_x = start_xy[0]
start_y = start_xy[1]

# Motor values
MAX_RPM = 3.0
STEPS_PER_REV = 4096.0
MAX_STEPS_PER_SEC = MAX_RPM * STEPS_PER_REV / 60.0
DEGREES_PER_STEP = 360.0 / STEPS_PER_REV

TOTAL_POINT_COUNT = 600
INITIAL_SPACING = 0.1

# Drawing Area
DRAWABLE_WIDTH = 10
DRAWABLE_HEIGHT = 7
ASPECT_RATIO = DRAWABLE_HEIGHT / DRAWABLE_WIDTH

# Translation adjustment Board -> Robot
OFFSET_X = -3.5
OFFSET_Y = 4.5

DRAWABLE_CENTER_X = OFFSET_X + DRAWABLE_WIDTH/2
DRAWABLE_CENTER_Y = OFFSET_Y + DRAWABLE_HEIGHT/2

# Corners of smallest rectangle containing every point in a set
def get_bounding_box(points):
    xs, ys = zip(*points)
    return min(xs), max(xs), min(ys), max(ys)

# Prints the X and Y ranges of input point set
def print_bounding_box(points):
    min_x, max_x, min_y, max_y = get_bounding_box(points)
    print(f"X: {min_x:.2f} to {max_x:.2f}")
    print(f"Y: {min_y:.2f} to {max_y:.2f}")
    return

def line_to_points(start, end, spacing):
    x1, y1 = start
    x2, y2 = end
    length = math.hypot(x2 - x1, y2 - y1)
    num_segments = max(int(length / spacing), 1)
    points = []
    for i in range(num_segments + 1):
        t = i / num_segments
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        points.append((x, y))
    return points

def arc_to_points(center, radius, start_angle, end_angle, spacing):
    points = []
    start_rad = math.radians(start_angle)
    end_rad = math.radians(end_angle)
    
    if end_rad < start_rad:
        end_rad += 2 * math.pi
    arc_length = radius * (end_rad - start_rad)
    num_segments = max(int(arc_length / spacing), 1)
    step = (end_rad - start_rad) / num_segments

    for i in range(num_segments + 1):
        angle = start_rad + i * step
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y))

    return points

def polyline_to_points(vertices, spacing):
    points = []
    for i in range(len(vertices) - 1):
        start = vertices[i]
        end = vertices[i + 1]
        points.extend(line_to_points(start, end, spacing=spacing))
    return points

# Inverse kinematics for this system
def ik(x, y):
    global targetTheta1, targetTheta2
    x += OFFSET_X
    y += OFFSET_Y

    d = math.hypot(x - l1, y)
    theta2_rad = math.atan2(y, x - l1) - math.acos((-d**2 - l2**2 + (l3 + a)**2) / (-2.0 * l2 * d))

    x3 = (a / (l3 + a)) * l2 * math.cos(theta2_rad) + (a / (l3 + a)) * l1 + (l3 / (l3 + a)) * x
    y3 = (a / (l3 + a)) * l2 * math.sin(theta2_rad) + (l3 / (l3 + a)) * y

    r = math.hypot(x3, y3)
    theta1_rad = math.atan2(y3, x3) + math.acos((-r**2 - l5**2 + l4**2) / (-2.0 * l5 * r))

    theta1_deg = math.degrees(theta1_rad)
    theta2_deg = math.degrees(theta2_rad)
    return theta1_deg, theta2_deg

def calc_steps(x, y, currentTheta1, currentTheta2, targetTheta1, targetTheta2):
    deltaTheta1 = targetTheta1 - currentTheta1
    deltaTheta2 = targetTheta2 - currentTheta2

    steps1 = int(round(deltaTheta1 / DEGREES_PER_STEP))
    steps2 = int(round(deltaTheta2 / DEGREES_PER_STEP))

    max_steps = max(abs(steps1), abs(steps2))
    if max_steps == 0:
        interval = 0
    else:
        interval = (int) (1000 / MAX_STEPS_PER_SEC)

    return -steps1, -steps2, interval

# Send steps thru serial to arduino
def send_steps(ser, steps1, steps2, interval):
    cmd = f"{steps1} {steps2} {interval}\n"
    ser.write(cmd.encode())

    while True:
        line = ser.readline().decode().strip()
        if line == "DONE":
            break

# Returns list of geometries
def extract_geometries(doc, spacing):
    msp = doc.modelspace()
    geometries = []
    lines, circles, arcs, polylines = 0, 0, 0, 0

    for e in msp:
        if e.dxftype() == 'LINE':
            start = (e.dxf.start.x, e.dxf.start.y)
            end = (e.dxf.end.x, e.dxf.end.y)
            geometries.append(line_to_points(start, end, spacing))
            lines += 1

        elif e.dxftype() == 'CIRCLE':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, 0, 360, spacing*2))
            circles += 1

        elif e.dxftype() == 'ARC':
            center = (e.dxf.center.x, e.dxf.center.y)
            radius = e.dxf.radius
            geometries.append(arc_to_points(center, radius, e.dxf.start_angle, e.dxf.end_angle, spacing * 2))
            arcs += 1

        elif e.dxftype() in ['LWPOLYLINE', 'POLYLINE']:
            vertices = []
            if e.dxftype() == 'LWPOLYLINE':
                vertices = [(pt[0], pt[1]) for pt in e]
            else:  # POLYLINE
                for v in e.vertices:
                    loc = v.dxf.location
                    vertices.append((loc.x, loc.y))
            geometries.append(polyline_to_points(vertices, spacing * 3))
            polylines += 1

    return geometries, lines, circles, arcs, polylines

# Center and scale drawing to whiteboard
def scale_and_translate_geometries(geometries):
    all_points = [pt for geometry in geometries for pt in geometry]
    min_x, max_x, min_y, max_y = get_bounding_box(all_points)
    width = max_x - min_x
    height = max_y - min_y
    if height < ASPECT_RATIO * width:
        scale_factor = DRAWABLE_WIDTH / width
    else:
        scale_factor = DRAWABLE_HEIGHT / height
    tx = DRAWABLE_WIDTH/2 - ((min_x + max_x) / 2) * scale_factor
    ty = DRAWABLE_HEIGHT/2 - ((min_y + max_y) / 2) * scale_factor

    scaled_geometries = []
    for geometry in geometries:
        scaled_geometry = [(x * scale_factor + tx, y * scale_factor + ty) for (x, y) in geometry]
        scaled_geometries.append(scaled_geometry)
    return scaled_geometries

def dist_points(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def geometry_dist_to_start(p):
    return min(dist_points(start_xy, p[0]), dist_points(start_xy, p[-1]))

def geometry_distance(g1, g2):
    candidates = [
        (dist_points(g1[-1], g2[0]), False, False),    # g1 normal end -> g2 normal start
        (dist_points(g1[-1], g2[-1]), False, True),    # g1 normal end -> g2 reversed start
        (dist_points(g1[0], g2[0]), True, False),      # g1 reversed end -> g2 normal start
        (dist_points(g1[0], g2[-1]), True, True),      # g1 reversed end -> g2 reversed start
    ]
    return min(candidates, key=lambda x: x[0])  # returns (distance, reverse_g1, reverse_g2)

def tsp_geometry_order(geometries):
    n = len(geometries)
    visited = [False] * n
    path = []
    
    current_index = min(range(n), key=lambda i: geometry_dist_to_start(geometries[i]))
    current_geometry = geometries[current_index]
    visited[current_index] = True

    if dist_points(start_xy, current_geometry[0]) > dist_points(start_xy, current_geometry[-1]):
        current_geometry = list(reversed(current_geometry))
        
    path.append(current_geometry)

    for _ in range(n - 1):
        best_dist = float('inf')
        best_index = None
        best_reverse = False
        best_geom_reversed = False
        current_end = path[-1][-1]

        for i in range(n):
            if visited[i]:
                continue
            dist, rev_curr, rev_next = geometry_distance(path[-1], geometries[i])
            if dist < best_dist:
                best_dist = dist
                best_index = i
                best_geom_reversed = rev_next

        next_geom = geometries[best_index]
        
        if best_geom_reversed:
            next_geom = list(reversed(next_geom))
            
        path.append(next_geom)
        visited[best_index] = True

    return path

# Plot set of points
def plot_points(ax, filtered_points):
    if filtered_points:
        fx, fy = zip(*filtered_points)
        ax.scatter(fx, fy, color='gray', s=30, label="Filtered Points")

    ax.set_xlim(0, DRAWABLE_WIDTH)
    ax.set_ylim(0, DRAWABLE_HEIGHT)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title("Filtered Points (Press Enter to Overlay Path)")
    ax.legend()

# Plot path given ordered set of points
def plot_path(ax, path_points):
    if path_points:
        px, py = zip(*path_points)
        ax.plot(px, py, 'r-', linewidth=2, label="Planned Path")
        ax.scatter(px, py, color='blue', s=30)
        ax.legend()
        plt.draw()

def reduce_points(points, n):
    if n >= len(points):
        return points

    step = (len(points) - 1) / (n - 1)
    indices = [round(i * step) for i in range(n - 1)]
    indices.append(len(points) - 1)

    indices = sorted(set(indices))

    return [points[i] for i in indices]


# Filter out points that within threshold motor steps of previous point
def filter_low_step_points(points, currentTheta1, currentTheta2, min_steps_threshold):
    filtered_points = []
    prev_theta1 = currentTheta1
    prev_theta2 = currentTheta2

    for x, y in points:
        t1, t2 = ik(x, y)
        delta1 = t1 - prev_theta1
        delta2 = t2 - prev_theta2
        steps1 = int(round(delta1 / DEGREES_PER_STEP))
        steps2 = int(round(delta2 / DEGREES_PER_STEP))

        if abs(steps1) + abs(steps2) >= min_steps_threshold:
            filtered_points.append((x, y))
            prev_theta1 = t1
            prev_theta2 = t2

    return filtered_points

# Repeat filter_zero_step_points until point set is within a reasonable size
def filter_until_target_point_qty(points, theta1, theta2, max_size, initial_threshold=0, increment=1):
    threshold = initial_threshold
    filtered = filter_low_step_points(points, theta1, theta2, threshold)
    while len(filtered) > max_size:
        threshold += increment
        filtered = filter_low_step_points(points, theta1, theta2, threshold)
    print(f"Final min_steps_threshold: {threshold}")
    return filtered

def save_points(points, filename="saved_points.txt"):
    with open("C:\\Users\\msgaf\\Desktop\\saved_points.txt", "w") as f:
        f.write("x\ty\n")
        for x, y in points:
            f.write(f"{x:.3f}\t{y:.3f}\n")

def main():
    # Define start config
    currentTheta1, currentTheta2 = ik(start_x, start_y)

    # Load DXF and extract geometries
    doc = ezdxf.readfile(DXF_FILE)
    geometries, lines, circles, arcs, polylines = extract_geometries(doc, INITIAL_SPACING)
    print(f"\nExtracted {len(geometries)} geometries: {lines} lines, {circles} circles, {arcs} arcs, {polylines} polylines")

    all_points = [pt for geometry in geometries for pt in geometry]
    print("\nBounding box BEFORE scaling & translation:")
    print_bounding_box(all_points)

    # Show bounding box before scaling
    scaled_geometries = scale_and_translate_geometries(geometries)
    scaled_all_points = [pt for geometries in scaled_geometries for pt in geometries]
    
    # Show bounding box after scaling
    print("\nBounding box AFTER scaling and translation:")
    print_bounding_box(scaled_all_points)

    # Order geometries
    ordered_geometries = tsp_geometry_order(scaled_geometries)

    # Turn ordered geometries into points
    ordered_all_points = []
    for geometry in ordered_geometries:
        ordered_all_points.extend(geometry)

    # Filter points to reduce total
    print(f"\nTotal points before filtering: {len(ordered_all_points)}")
    filtered_points = reduce_points(ordered_all_points, TOTAL_POINT_COUNT)
    print(f"Total points after filtering: {len(filtered_points)}")

    # Save coords to txt
    save_points(filtered_points)

    # Set up plot
    fig, ax = plt.subplots()
    plt.ion()
    fig.set_size_inches(DRAWABLE_WIDTH, DRAWABLE_HEIGHT)
    ax.set_xlim(0, DRAWABLE_WIDTH)
    ax.set_ylim(0, DRAWABLE_HEIGHT)   
    ax.set_aspect('equal', adjustable='box')

    # Plot points
    points_plot, = ax.plot(*zip(*filtered_points), 'bo', label='Points')
    path_plot, = ax.plot([],[], 'r-', linewidth=2, label='Path')
    plt.legend()
    plt.show()

    # Plot path
    input("\nPress Enter to show path...")
    path_plot.set_data(*zip(*([(0, 0)] + filtered_points)))
    points_plot.set_visible(False)
    fig.canvas.draw()

    input("Press Enter to start...")

    # Setup serial communication
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    time.sleep(2)
    ser.flush()

    for i, (x, y) in enumerate(filtered_points):
        targetTheta1, targetTheta2 = ik(x, y)
        steps1, steps2, interval = calc_steps(x,y,currentTheta1,currentTheta2,targetTheta1,targetTheta2)
        send_steps(ser, steps1, steps2, interval)

        print(f"Point {i}/{len(filtered_points)} | Sent: {steps1:>4} {steps2:>4}  |  Moved to: x={x:.2f}, y={y:.2f}, t1={targetTheta1:.2f}, t2={targetTheta2:.2f}")

        currentTheta1 = targetTheta1
        currentTheta2 = targetTheta2        

    print("DONE!")

    plt.ioff()

if __name__ == "__main__":
    main()

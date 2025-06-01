import math
from Constants import Constants as c

class PathProcessor:
    def __init__(self, dxf_file, spacing):
        self.dxf_file = dxf_file
        self.spacing = spacing
        self.geometries = []

    @staticmethod
    def get_bounding_box(points):
        xs, ys = zip(*points)
        return min(xs), max(xs), min(ys), max(ys)

    @staticmethod
    def print_bounding_box(points):
        min_x, max_x, min_y, max_y = PathProcessor.get_bounding_box(points)
        print(f"X: {min_x:.2f} to {max_x:.2f}")
        print(f"Y: {min_y:.2f} to {max_y:.2f}")

    @staticmethod
    def line_to_points(start, end, spacing):
        x1, y1 = start
        x2, y2 = end
        length = math.hypot(x2 - x1, y2 - y1)
        num_segments = max(int(length / spacing), 1)
        return [(x1 + t * (x2 - x1), y1 + t * (y2 - y1)) for t in [i / num_segments for i in range(num_segments + 1)]]

    @staticmethod
    def arc_to_points(center, radius, start_angle, end_angle, spacing):
        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        if end_rad < start_rad:
            end_rad += 2 * math.pi
        arc_length = radius * (end_rad - start_rad)
        num_segments = max(int(arc_length / spacing), 1)
        step = (end_rad - start_rad) / num_segments
        return [(center[0] + radius * math.cos(start_rad + i * step),
                 center[1] + radius * math.sin(start_rad + i * step)) for i in range(num_segments + 1)]

    @staticmethod
    def polyline_to_points(vertices, spacing):
        points = []
        for i in range(len(vertices) - 1):
            points.extend(PathProcessor.line_to_points(vertices[i], vertices[i + 1], spacing=spacing))
        return points

    @staticmethod
    def extract_geometries(doc, spacing):
        msp = doc.modelspace()
        geometries = []
        lines, circles, arcs, polylines = 0, 0, 0, 0

        for e in msp:
            if e.dxftype() == 'LINE':
                start = (e.dxf.start.x, e.dxf.start.y)
                end = (e.dxf.end.x, e.dxf.end.y)
                geometries.append(PathProcessor.line_to_points(start, end, spacing))
                lines += 1

            elif e.dxftype() == 'CIRCLE':
                center = (e.dxf.center.x, e.dxf.center.y)
                radius = e.dxf.radius
                geometries.append(PathProcessor.arc_to_points(center, radius, 0, 360, spacing*2))
                circles += 1

            elif e.dxftype() == 'ARC':
                center = (e.dxf.center.x, e.dxf.center.y)
                radius = e.dxf.radius
                geometries.append(PathProcessor.arc_to_points(center, radius, e.dxf.start_angle, e.dxf.end_angle, spacing * 2))
                arcs += 1

            elif e.dxftype() in ['LWPOLYLINE', 'POLYLINE']:
                vertices = []
                if e.dxftype() == 'LWPOLYLINE':
                    vertices = [(pt[0], pt[1]) for pt in e]
                else:  # POLYLINE
                    for v in e.vertices:
                        loc = v.dxf.location
                        vertices.append((loc.x, loc.y))
                geometries.append(PathProcessor.polyline_to_points(vertices, spacing * 3))
                polylines += 1

        return geometries, lines, circles, arcs, polylines

    @staticmethod
    def scale_and_translate_geometries(geometries):
        all_points = [pt for geometry in geometries for pt in geometry]
        min_x, max_x, min_y, max_y = PathProcessor.get_bounding_box(all_points)
        width = max_x - min_x
        height = max_y - min_y
        scale_factor = c.DRAWABLE_WIDTH / width if height < c.ASPECT_RATIO * width else c.DRAWABLE_HEIGHT / height
        tx = c.DRAWABLE_WIDTH / 2 - ((min_x + max_x) / 2) * scale_factor
        ty = c.DRAWABLE_HEIGHT / 2 - ((min_y + max_y) / 2) * scale_factor

        return [[(x * scale_factor + tx, y * scale_factor + ty) for (x, y) in geometry] for geometry in geometries]

    @staticmethod
    def dist_points(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    @staticmethod
    def geometry_dist_to_start(p):
        return min(PathProcessor.dist_points(c.START_XY, p[0]), PathProcessor.dist_points(c.START_XY, p[-1]))

    @staticmethod
    def geometry_distance(g1, g2):
        candidates = [
            (PathProcessor.dist_points(g1[-1], g2[0]), False, False),
            (PathProcessor.dist_points(g1[-1], g2[-1]), False, True),
            (PathProcessor.dist_points(g1[0], g2[0]), True, False),
            (PathProcessor.dist_points(g1[0], g2[-1]), True, True),
        ]
        return min(candidates, key=lambda x: x[0])

    @staticmethod
    def tsp_geometry_order(geometries):
        n = len(geometries)
        visited = [False] * n
        path = []

        current_index = min(range(n), key=lambda i: PathProcessor.geometry_dist_to_start(geometries[i]))
        current_geometry = geometries[current_index]
        visited[current_index] = True

        if PathProcessor.dist_points(c.START_XY, current_geometry[0]) > PathProcessor.dist_points(c.START_XY, current_geometry[-1]):
            current_geometry = list(reversed(current_geometry))

        path.append(current_geometry)

        for _ in range(n - 1):
            best_dist = float('inf')
            best_index = None
            best_geom_reversed = False

            for i in range(n):
                if visited[i]:
                    continue
                dist, _, rev_next = PathProcessor.geometry_distance(path[-1], geometries[i])
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
    
    @staticmethod
    def reduce_points(points, n):
        if n >= len(points):
            return points

        step = (len(points) - 1) / (n - 1)
        indices = [round(i * step) for i in range(n - 1)]
        indices.append(len(points) - 1)

        indices = sorted(set(indices))

        return [points[i] for i in indices]
    
    @staticmethod
    def save_points(points, filename="saved_points.txt"):
        with open("C:\\Users\\msgaf\\Desktop\\saved_points.txt", "w") as f:
            f.write("x\ty\n")
            for x, y in points:
                f.write(f"{x:.3f}\t{y:.3f}\n")

"""

OLD STUFF

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
"""
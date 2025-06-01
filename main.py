import argparse
import ezdxf
import time
import serial
import matplotlib.pyplot as plt
from Path import PathProcessor as p
from IK import InverseKinematics as IK
from SerialCommunicator import SerialCommunicator
from Constants import Constants as c

def main():
    dxf_file = c.DXF_FILE
    com_port = c.SERIAL_PORT
    baud_rate = c.BAUD_RATE
    print(f"Using DXF file: {dxf_file}")

    # Define start config
    currentTheta1, currentTheta2 = IK.ik(c.START_X, c.START_Y)

    # Load DXF and extract geometries
    doc = ezdxf.readfile(c.DXF_FILE)
    geometries, lines, circles, arcs, polylines = p.extract_geometries(doc, c.INITIAL_SPACING)
    print(f"\nExtracted {len(geometries)} geometries: {lines} lines, {circles} circles, {arcs} arcs, {polylines} polylines")

    all_points = [pt for geometry in geometries for pt in geometry]
    print("\nBounding box BEFORE scaling & translation:")
    p.print_bounding_box(all_points)

    # Show bounding box before scaling
    scaled_geometries = p.scale_and_translate_geometries(geometries)
    scaled_all_points = [pt for geometries in scaled_geometries for pt in geometries]
    
    # Show bounding box after scaling
    print("\nBounding box AFTER scaling and translation:")
    p.print_bounding_box(scaled_all_points)

    # Order geometries
    ordered_geometries = p.tsp_geometry_order(scaled_geometries)

    # Turn ordered geometries into points
    ordered_all_points = []
    for geometry in ordered_geometries:
        ordered_all_points.extend(geometry)

    # Filter points to reduce total
    print(f"\nTotal points before filtering: {len(ordered_all_points)}")
    filtered_points = p.reduce_points(ordered_all_points, c.TOTAL_POINT_COUNT)
    print(f"Total points after filtering: {len(filtered_points)}")

    # Save coords to txt
    p.save_points(filtered_points)

    # Set up plot
    fig, ax = plt.subplots()
    plt.ion()
    fig.set_size_inches(c.DRAWABLE_WIDTH, c.DRAWABLE_HEIGHT)
    ax.set_xlim(0, c.DRAWABLE_WIDTH)
    ax.set_ylim(0, c.DRAWABLE_HEIGHT)   
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
    serialcomm = SerialCommunicator(c.SERIAL_PORT, c.BAUD_RATE)
    time.sleep(2)
    serialcomm.ser.flush()

    for i, (x, y) in enumerate(filtered_points):
        targetTheta1, targetTheta2 = IK.ik(x, y)
        steps1, steps2, interval = IK.calc_steps(x,y,currentTheta1,currentTheta2,targetTheta1,targetTheta2)
        serialcomm.send_steps(steps1, steps2, interval)

        print(f"Point {i}/{len(filtered_points)} | Sent: {steps1:>4} {steps2:>4}  |  Moved to: x={x:.2f}, y={y:.2f}, t1={targetTheta1:.2f}, t2={targetTheta2:.2f}")

        currentTheta1 = targetTheta1
        currentTheta2 = targetTheta2        

    print("DONE!")

    plt.ioff()

if __name__ == "__main__":
    main()

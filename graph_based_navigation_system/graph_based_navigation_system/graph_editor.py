import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

# ===== Configuration =====
MAP_NAME = 'test_TER_1'  # Base name without extension
GRAPH_FILENAME = f'{MAP_NAME}_graph.yaml'  # Auto-generate graph filename
PACKAGE_NAME = 'graph_based_navigation_system'

# ===== Directory Setup =====
def get_maps_directory():
    """Get and ensure maps directory exists"""
    map_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'maps')
    os.makedirs(map_dir, exist_ok=True)
    return map_dir

# ===== Main Program =====
def main():
    # Initialize directories
    maps_dir = get_maps_directory()
    
    # === Map Loading ===
    map_yaml_path = os.path.join(maps_dir, f'{MAP_NAME}.yaml')
    if not os.path.exists(map_yaml_path):
        raise FileNotFoundError(f"Map file not found at {map_yaml_path}")
    
    with open(map_yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    # === Graph Setup ===
    graph_save_path = os.path.join(maps_dir, GRAPH_FILENAME)
    
    # Load existing graph if it exists
    if os.path.exists(graph_save_path):
        with open(graph_save_path, 'r') as f:
            graph_data = yaml.safe_load(f) or {}
        nodes = graph_data.get('nodes', {})
        edges = graph_data.get('edges', [])
        node_counter = max(nodes.keys()) + 1 if nodes else 0
    else:
        nodes = {}
        edges = []
        node_counter = 0

    # === Map Data Processing ===
    resolution = map_data['resolution']
    origin = map_data['origin']
    image_path = os.path.join(maps_dir, map_data['image'])
    img = mpimg.imread(image_path)
    height, width = img.shape[:2]

    # === Coordinate Conversions ===
    def pixel_to_world(x_pix, y_pix):
        x = x_pix * resolution + origin[0]
        y = (height - y_pix) * resolution + origin[1]
        return round(x, 3), round(y, 3)

    def world_to_pixel(x, y):
        x_pix = int((x - origin[0]) / resolution)
        y_pix = int(height - (y - origin[1]) / resolution)
        return x_pix, y_pix

    # === Global State ===
    clicked_for_edge = []
    last_node_id = None
    history = []
    fig, ax = plt.subplots()

    # === Drawing Functions ===
    def redraw():
        ax.clear()
        ax.imshow(img, cmap='gray')

        # Draw nodes
        for nid, props in nodes.items():
            x_pix, y_pix = world_to_pixel(props['x'], props['y'])
            ax.plot(x_pix, y_pix, 'ro', markersize=8)
            ax.text(x_pix + 5, y_pix - 10, str(nid), color='cyan', fontsize=10)
            if 'yaw' in props:
                dx = 20 * math.cos(props['yaw'])
                dy = -20 * math.sin(props['yaw'])
                ax.arrow(x_pix, y_pix, dx, dy, head_width=5, color='yellow')

        # Draw edges
        for edge in edges:
            a = nodes[edge['from']]
            b = nodes[edge['to']]
            ax.plot(
                [world_to_pixel(a['x'], a['y'])[0], world_to_pixel(b['x'], b['y'])[0]],
                [world_to_pixel(a['x'], a['y'])[1], world_to_pixel(b['x'], b['y'])[1]],
                'g-', linewidth=1.5
            )

        plt.draw()

    # === Event Handlers ===
    def on_click(event):
        nonlocal node_counter, last_node_id, clicked_for_edge

        if event.inaxes != ax:
            return

        x_pix, y_pix = int(event.xdata), int(event.ydata)
        x_world, y_world = pixel_to_world(x_pix, y_pix)

        if event.button == 1 and not event.key:  # Left-click
            nodes[node_counter] = {'x': x_world, 'y': y_world, 'yaw': 0.0}
            history.append(('add_node', node_counter))
            print(f"Added node {node_counter} at ({x_world:.2f}, {y_world:.2f})")
            last_node_id = node_counter
            node_counter += 1
            redraw()

        elif event.button == 3:  # Right-click
            nearest_id = find_nearest_node(x_pix, y_pix)
            if nearest_id is not None:
                clicked_for_edge.append(nearest_id)
                print(f"Selected node {nearest_id} for edge")
                if len(clicked_for_edge) == 2:
                    a, b = clicked_for_edge
                    if a != b and not edge_exists(a, b):
                        edges.append({'from': a, 'to': b})
                        history.append(('add_edge', (a, b)))
                        print(f"Edge added: {a} <-> {b}")
                    clicked_for_edge = []
                    redraw()

    def find_nearest_node(x_pix, y_pix, threshold=15):
        for nid, props in nodes.items():
            xp, yp = world_to_pixel(props['x'], props['y'])
            if math.dist((x_pix, y_pix), (xp, yp)) < threshold:
                return nid
        return None

    def edge_exists(a, b):
        return any(e for e in edges if {e['from'], e['to']} == {a, b})

    def on_motion(event):
        nonlocal last_node_id
        if event.key != 'shift' or last_node_id is None or event.inaxes != ax:
            return

        node = nodes[last_node_id]
        x0, y0 = world_to_pixel(node['x'], node['y'])
        x1, y1 = event.xdata, event.ydata
        if None in (x1, y1):
            return

        dx = x1 - x0
        dy = y0 - y1  # Flip y
        old_yaw = node.get('yaw', 0.0)
        yaw = math.atan2(dy, dx)
        node['yaw'] = round(yaw, 3)
        history.append(('set_yaw', last_node_id, old_yaw))
        redraw()

    def on_key(event):
        if event.key == 'z':
            undo_last()

    def undo_last():
        nonlocal node_counter, last_node_id
        if not history:
            print("Nothing to undo.")
            return

        action = history.pop()
        if action[0] == 'add_node':
            nid = action[1]
            if nid in nodes:
                del nodes[nid]
                node_counter -= 1
                print(f"Undid node {nid}")
            last_node_id = node_counter - 1 if node_counter > 0 else None
        elif action[0] == 'add_edge':
            edge = action[1]
            if {'from': edge[0], 'to': edge[1]} in edges:
                edges.remove({'from': edge[0], 'to': edge[1]})
                print(f"Undid edge: {edge[0]} <-> {edge[1]}")
        elif action[0] == 'set_yaw':
            nid, old_yaw = action[1], action[2]
            if nid in nodes:
                nodes[nid]['yaw'] = old_yaw
                print(f"Reset yaw of node {nid} to {old_yaw}")

        redraw()

    def on_close(event):
        with open(graph_save_path, 'w') as f:
            yaml.dump({'nodes': nodes, 'edges': edges}, f, sort_keys=False)
        print(f"\nGraph saved to: {graph_save_path}")

    # === UI Setup ===
    ax.imshow(img, cmap='gray')
    ax.set_title(f"Map: {MAP_NAME}")
    
    # Print comprehensive instructions to terminal
    print("\n" + "="*60)
    print("GRAPH EDITOR INSTRUCTIONS".center(60))
    print("="*60)
    print("Left-click: Add new node at cursor position")
    print("Right-click two nodes: Create edge between them")
    print("Shift + Drag from node: Set orientation (yaw)")
    print("Z key: Undo last action (node/edge/yaw change)")
    print("\nAdditional Info:")
    print("- Nodes are automatically numbered")
    print("- Edges require selecting two different nodes")
    print("- Node orientation shown with yellow arrow")
    print("- Graph auto-saves when closing the window")
    print(f"\nWorking on map: {MAP_NAME}")
    print(f"Graph will be saved to: {graph_save_path}")
    print("="*60 + "\n")

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('close_event', on_close)

    redraw()
    plt.show()

if __name__ == "__main__":
    main()
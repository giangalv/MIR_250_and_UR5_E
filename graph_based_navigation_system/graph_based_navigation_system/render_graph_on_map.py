import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import math
from ament_index_python.packages import get_package_share_directory

# ===== Configuration =====
MAP_NAME = 'test_TER_1'  # Base name without extension
GRAPH_FILENAME = f'{MAP_NAME}_graph.yaml'  # Graph file associated with map
OUTPUT_IMAGE_NAME = f'{MAP_NAME}_with_nodes_edges.png'
PACKAGE_NAME = 'graph_based_navigation_system'

# ===== Directory Setup =====
def get_maps_directory():
    """Get and ensure the maps directory exists."""
    map_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), 'maps')
    os.makedirs(map_dir, exist_ok=True)
    return map_dir

# ===== Coordinate Conversion =====
def world_to_pixel(x, y, origin, resolution, image_height):
    x_pix = int((x - origin[0]) / resolution)
    y_pix = int(image_height - (y - origin[1]) / resolution)
    return x_pix, y_pix

# ===== Main Program =====
def main():
    maps_dir = get_maps_directory()

    # --- Load map data ---
    map_yaml_path = os.path.join(maps_dir, f'{MAP_NAME}.yaml')
    if not os.path.exists(map_yaml_path):
        raise FileNotFoundError(f"Map YAML not found: {map_yaml_path}")

    with open(map_yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    resolution = map_data['resolution']
    origin = map_data['origin']
    image_path = os.path.join(maps_dir, map_data['image'])

    img = mpimg.imread(image_path)
    height, width = img.shape[:2]

    # --- Load graph data ---
    graph_yaml_path = os.path.join(maps_dir, GRAPH_FILENAME)
    if not os.path.exists(graph_yaml_path):
        raise FileNotFoundError(f"Graph YAML not found: {graph_yaml_path}")

    with open(graph_yaml_path, 'r') as f:
        graph = yaml.safe_load(f)

    nodes = {str(k): v for k, v in graph.get('nodes', {}).items()}
    edges = graph.get('edges', [])

    # --- Draw the map, nodes, and edges ---
    dpi = 100 # Set DPI for better resolution
    fig, ax = plt.subplots(figsize=(width / dpi, height / dpi), dpi=dpi)
    ax.imshow(img, cmap='gray')

    for edge in edges:
        a = nodes[str(edge['from'])]
        b = nodes[str(edge['to'])]
        x1, y1 = world_to_pixel(a['x'], a['y'], origin, resolution, height)
        x2, y2 = world_to_pixel(b['x'], b['y'], origin, resolution, height)
        ax.plot([x1, x2], [y1, y2], 'b-', linewidth=0.7)

    for nid, props in nodes.items():
        x_pix, y_pix = world_to_pixel(props['x'], props['y'], origin, resolution, height)
        ax.plot(x_pix, y_pix, 'ro', markersize=5)
        ax.text(x_pix, y_pix, nid, color='white', fontsize=5, ha='center', va='center', fontweight='bold')

        if 'yaw' in props:
            dx = 10 * math.cos(props['yaw'])
            dy = -10 * math.sin(props['yaw'])
            ax.arrow(x_pix, y_pix, dx, -dy, head_width=2, color='orange')  # Flip Y

    # --- Save annotated map ---
    output_image_path = os.path.join(maps_dir, OUTPUT_IMAGE_NAME)
    
    plt.axis('off')
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    plt.savefig(output_image_path, dpi=dpi, bbox_inches='tight', pad_inches=0)

    print(f"✅ Annotated map saved to: {output_image_path}")

    # --- Save new YAML map with updated image name ---
    new_yaml_path = output_image_path.replace('.png', '.yaml')
    new_map_data = map_data.copy()
    new_map_data['image'] = os.path.basename(output_image_path)

    with open(new_yaml_path, 'w') as f:
        yaml.dump(new_map_data, f, default_flow_style=False)

    print(f"✅ Updated YAML map saved to: {new_yaml_path}")
    plt.show()

if __name__ == "__main__":
    main()

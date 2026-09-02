import json
import argparse
import psycopg2
from pathlib import Path

# Database Credentials
DB_CONFIG = {
    'dbname': 'ros2',
    'user': 'admin',
    'password': 'Robotlab2019',
    'host': '192.168.130.87',
    'port': '5433'
}

def export_db_to_graph_json(output_json: str):
    vertices = []
    edges = []

    try:
        # Connect to the database
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()

        # 1. Fetch Vertices (Nodes)
        # Using theta AS yaw to match the expected JSON format
        cursor.execute("SELECT id, x, y, theta, node_type, obj_id FROM graph_node ORDER BY id;")
        for row in cursor.fetchall():
            vertices.append({
                "id": int(row[0]),
                "x": float(row[1]),
                "y": float(row[2]),
                "yaw": float(row[3]),      # Mapped from 'theta'
                "node_type": row[4],
                "obj_id": row[5]
            })

        # 2. Fetch Edges
        cursor.execute("SELECT source_id, target_id, weight, edge_type FROM graph_edge;")
        for row in cursor.fetchall():
            edges.append({
                "source": int(row[0]),
                "target": int(row[1]),
                "weight": float(row[2]),
                "edge_type": row[3]
            })

        cursor.close()
        conn.close()

    except Exception as e:
        print(f"Database error: {e}")
        return

    # Combine into graph structure
    graph_data = {
        "vertices": vertices,
        "edges": edges
    }

    # Ensure parent directory exists and write JSON
    output_path = Path(output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, mode='w', encoding='utf-8') as f:
        json.dump(graph_data, f, indent=2)

    print(f"Successfully exported graph data from Database!")
    print(f" - Vertices: {len(vertices)}")
    print(f" - Edges:    {len(edges)}")
    print(f" - Output:   {output_path.resolve()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export graph nodes and edges from PostgreSQL to JSON.")
    parser.add_argument("--output", default="graph_data.json", help="Path for output JSON file")
    
    args = parser.parse_args()
    
    print(f"Generating graph JSON with arguments: {args}")
    export_db_to_graph_json(args.output)
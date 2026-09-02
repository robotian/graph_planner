import csv
import json
import argparse
from pathlib import Path


def convert_csv_to_graph_json(vertices_csv: str, edges_csv: str, output_json: str):
    vertices = []
    edges = []

    # Read Vertices CSV
    with open(vertices_csv, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            vertices.append({
                "id": int(row["id"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"])
            })

    # Read Edges CSV
    with open(edges_csv, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            edges.append({
                "source": int(row["source"]),
                "target": int(row["target"]),
                "weight": float(row["weight"])
            })

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

    print(f"Successfully converted graph data!")
    print(f" - Vertices: {len(vertices)}")
    print(f" - Edges:    {len(edges)}")
    print(f" - Output:   {output_path.resolve()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert vertex and edge CSV files into graph JSON.")
    parser.add_argument("--vertices", default="vertex_set1.csv", help="Path to vertex CSV file")
    parser.add_argument("--edges", default="edge_set1.csv", help="Path to edge CSV file")
    parser.add_argument("--output", default="graph_data.json", help="Path for output JSON file")
    
    args = parser.parse_args()
    convert_csv_to_graph_json(args.vertices, args.edges, args.output)
    print(f"Graph generated with arguments: {parser.parse_args()}")
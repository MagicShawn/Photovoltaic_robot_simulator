import csv
import sys
import os

def normalize_elapsed_time(csv_path, output_path=None):
    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        return

    with open(csv_path, 'r') as f:
        reader = list(csv.reader(f))
        header = reader[0]
        data = reader[1:]

    if len(data) == 0:
        print("No data to process.")
        return

    try:
        first_time = float(data[0][0])
        for row in data:
            row[0] = f"{float(row[0]) - first_time:.3f}"
    except ValueError:
        print("The first column must contain float timestamps.")
        return

    if output_path is None:
        output_path = csv_path

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(data)

    print(f"Elapsed time normalized. Output saved to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python normalize_elapsed_time.py path/to/input.csv [path/to/output.csv]")
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else None
        normalize_elapsed_time(input_file, output_file)

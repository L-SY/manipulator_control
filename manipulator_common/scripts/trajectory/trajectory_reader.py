import csv
import yaml

def read_trajectory_from_csv(file_path):
    joint_names = []
    trajectory_points = []

    with open(file_path, 'r') as csvfile:
        csvreader = csv.DictReader(csvfile)
        fieldnames = csvreader.fieldnames
        # Extract joint names from the header
        joint_names = [name.split('_position')[0] for name in fieldnames if '_position' in name]

        for row in csvreader:
            point = {
                'time_from_start': float(row['time']),
                'positions': [
                    float(row[f'{joint}_position']) for joint in joint_names
                ],
                'velocities': [
                    float(row[f'{joint}_velocity']) for joint in joint_names
                ] if f'{joint_names[0]}_velocity' in row else []
            }
            trajectory_points.append(point)

    return joint_names, trajectory_points

def read_trajectory_from_yaml(file_path):
    with open(file_path, 'r') as yamlfile:
        data = yaml.safe_load(yamlfile)

    joint_names = []
    trajectory_points = []

    # Ensure the 'file_path' key exists and is a list
    if 'file_path' in data and isinstance(data['file_path'], list):
        # Skip the first entry which is the file path
        trajectory_entries = data['file_path'][1:]
        for entry in trajectory_entries:
            if isinstance(entry, dict):
                print(f"Processing entry: {entry}")  # Debugging line

                time_from_start = entry.get('time_from_start')
                positions = entry.get('positions', [])
                velocities = entry.get('velocities', [])

                if not joint_names:
                    joint_names = [f'joint{i+1}' for i in range(len(positions))]

                point = {
                    'time_from_start': time_from_start,
                    'positions': positions,
                    'velocities': velocities
                }
                trajectory_points.append(point)
            else:
                print(f"Skipping invalid entry: {entry}")  # Debugging line
    else:
        print("No valid 'file_path' key found in YAML data")

    return joint_names, trajectory_points



# Example usage:
if __name__ == '__main__':
    # file_path = '/home/lsy/manipulator_control/src/manipulator_python/trajectory/recorded_trajectory.csv'
    # joint_names, trajectory_points = read_trajectory_from_csv(file_path)
    # print("Joint Names:", joint_names)
    # print("Trajectory Points:", trajectory_points)
    yaml_file_path = "/home/lsy/manipulator_control/src/manipulator_python/trajectory/recorded_trajectory.yaml"
    joint_names, trajectory_points = read_trajectory_from_yaml(yaml_file_path)
    print(f"Joint Names: {joint_names}")
    for point in trajectory_points:
        print(point)

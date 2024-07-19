import csv

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

# Example usage:
if __name__ == '__main__':
    file_path = '/home/lsy/manipulator_control/src/manipulator_python/trajectory/recorded_trajectory.csv'
    joint_names, trajectory_points = read_trajectory_from_csv(file_path)
    print("Joint Names:", joint_names)
    print("Trajectory Points:", trajectory_points)

#!/usr/bin/env python3
import argparse, argcomplete
import os
import sys
import yaml


def float_representer(dumper, value):
    text = '{0:.3f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)


def generate_even_grid(grid_size: tuple[int, int], distance: tuple[float,
                                                                   float],
                       offset: tuple[float, float, float]):
    data = {}
    data['loop'] = False
    data['waypoints'] = []
    for row in range(grid_size[0]):
        for col in range(grid_size[1]):
            x = col * distance[0] + offset[0]
            y = row * distance[1] + offset[1]
            z = offset[2]

            data['waypoints'].append({'x': x, 'y': y, 'z': z})
    return data


def main():
    parser = argparse.ArgumentParser(description='Grid waypoint generator')
    parser.add_argument('--distance',
                        nargs=2,
                        default=[0.1, 0.1],
                        type=float,
                        help='Distance between waypoints in x- and y-direction')
    parser.add_argument('--grid-size',
                        nargs=2,
                        default=[13, 7],
                        type=int,
                        help='Number of rows and columns')
    parser.add_argument(
        '--offset',
        nargs=3,
        default=[0.0, 0.0, -0.5],
        type=float,
        help='Offset of the grid relative to origin of gantry coordinate system'
    )
    parser.add_argument('--out-dir',
                        required=True,
                        help='Output directory of the generated files')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    yaml.add_representer(float, float_representer)

    print(f'Generating a 2D grid of waypoints with the following parameters: ')
    print(f'-- dx: {args.distance[0]}m,\n-- dy: {args.distance[1]}m')
    print(f'-- Number of points in x-direction: {args.grid_size[0]}')
    print(f'-- Number of points in y-direction: {args.grid_size[1]}')
    print(
        f'-- x_0 = {args.offset[0]}m,\n-- y_0 = {args.offset[1]}m,\n-- z = {args.offset[2]}m for all waypoints'
    )
    print('\n')
    print(f'Resulting in the following maximum positions: ')
    print(f'-- x_max = {args.distance[0]*args.grid_size[0] +  args.offset[0]}')
    print(f'-- y_max = {args.distance[1]*args.grid_size[1] +  args.offset[1]}')
    print('\n')
    print(f'Be sure to check that the grid size does not violate the maximum' +
          ' x-, y- and z-position of the gantry!')
    print(f'The safe area inside the tank might be even smaller due to' +
          ' additional obstacles!')
    print('\n')

    # Generate some data!
    waypoint_data = generate_even_grid(args.grid_size, args.distance,
                                       args.offset)

    if args.out_dir == '-':
        yaml.dump(waypoint_data,
                  sys.stdout,
                  default_flow_style=True,
                  width=float('inf'))
    else:
        filename = 'new_waypoint_grid.yaml'
        filepath = os.path.join(args.out_dir, filename)
        with open(filepath, 'w') as f:
            yaml.dump(waypoint_data, f)
            print(f'Created file [{filepath}]')


if __name__ == '__main__':
    main()

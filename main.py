from src.HeatMapCreator import HeatMapCreator


if __name__ == '__main__':
    test_list_1_data = [1, 0, 0, 1, 1, 0, 0, 0, 0]
    test_list_2_data = [1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0]
    Hpc = HeatMapCreator()
    Hpc.callback(test_list_1_data,
                 3,
                 3,
                 1.0,
                 2,
                 2,)
    print(f'{Hpc.raw_grid_map_width_pixel, Hpc.raw_grid_map_height_pixel}')
    Hpc.callback(test_list_2_data,
                 3,
                 4,
                 1.0,
                 2,
                 2)
    print(f'{Hpc.raw_grid_map_width_pixel, Hpc.raw_grid_map_height_pixel}')

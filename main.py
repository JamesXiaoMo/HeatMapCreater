from src.HeatMapCreator import HeatMapCreator


if __name__ == '__main__':
    test_list_1_data = [0] * 25
    Hpc = HeatMapCreator(heat_map_interval=2)
    Hpc.callback(test_list_1_data,
                 5,
                 5,
                 1.0,
                 -2,
                 -2)

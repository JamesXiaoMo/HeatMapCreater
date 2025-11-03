from src.HeatMapCreator import HeatMapCreator


if __name__ == '__main__':
    test_list_1 = [1, 0, 0, 1, 1, 0, 0, 0, 0]
    Hpc = HeatMapCreator()
    Hpc.callback(test_list_1,
                 3,
                 3,
                 1.0,
                 2,
                 2,)

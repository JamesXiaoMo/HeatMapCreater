from src.HeatMapCreator import HeatMapCreator


if __name__ == '__main__':
    test_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    Hpc = HeatMapCreator()
    Hpc.update(test_list, 5, 2, 1.0, 2)
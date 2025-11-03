import numpy as np


class HeatMapCreator:
    """
    由ROS2发布的/map话题数据生成热力图
    """
    def __init__(self, heat_map_interval: float):
        self.raw_grid_map_width_pixel = 0           # 原始地图的宽度 [pixel]
        self.raw_grid_map_height_pixel = 0          # 原始地图的高度 [pixel]
        self.raw_grid_map_width = 0                 # 原始地图的宽度 [m]
        self.raw_grid_map_height = 0                # 原始地图的高度 [m]
        self.raw_grid_map_resolution = 0            # 原始地图的分辨率 [m/pixel]
        self.raw_grid_map_data = []                 # 原始地图的数据
        self.raw_grid_map_origin_x = 0.0            # 原始地图原点x轴坐标 [pixel]
        self.raw_grid_map_origin_y = 0.0            # 原始地图原点y轴坐标 [pixel]

        self.raw_grid_map_data_2d = []              # 原始地图的2D数据

        self.heat_map_width_pixel = 0               # 热力图的宽度 [pixel]
        self.heat_map_height_pixel = 0              # 热力图的高度 [pixel]
        self.heat_map_width = 0                     # 热力图的宽度 [m]
        self.heat_map_height = 0                    # 热力图的高度 [m]
        self.heat_map_resolution = 0                # 热力图的分辨率 [m/pixel]
        self.heat_map_interval = heat_map_interval  # 热力图的测量间隔 [m]
        self.heat_map_data_2d = []                  # 热力图的2D数据

        self.counter = 0                            # 获取原始数据的计数器

    def callback(
        self,
        raw_grid_map_data: list,
        raw_grid_map_width_pixel: int,
        raw_grid_map_height_pixel: int,
        raw_grid_map_resolution: float,
        raw_grid_map_origin_x: float,
        raw_grid_map_origin_y: float,
    ):
        """
        用于在订阅ROS2的/map时的回调函数
        :param raw_grid_map_data: 原始地图的数据
        :param raw_grid_map_width_pixel: 原始地图的宽度 [pixel]
        :param raw_grid_map_height_pixel: 原始地图的高度 [pixel]
        :param raw_grid_map_resolution: 原始地图的分辨率 [m/pixel]
        :param raw_grid_map_origin_x: 原始地图原点x轴坐标 [pixel]
        :param raw_grid_map_origin_y: 原始地图原点y轴坐标 [pixel]
        :return:
        """
        # 初始化
        self.raw_grid_map_width_pixel = raw_grid_map_width_pixel
        self.raw_grid_map_height_pixel = raw_grid_map_height_pixel
        self.raw_grid_map_resolution = raw_grid_map_resolution
        self.raw_grid_map_width = raw_grid_map_width_pixel * raw_grid_map_resolution
        self.raw_grid_map_height = raw_grid_map_height_pixel * raw_grid_map_resolution
        self.raw_grid_map_data = raw_grid_map_data
        self.raw_grid_map_origin_x = raw_grid_map_origin_x
        self.raw_grid_map_origin_y = raw_grid_map_origin_y

        # 创建空数组
        self.raw_grid_map_data_2d = np.full((2, self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel), -1)
        # 将原始地图的数据转化为2D形式(完整变换，仅进行一次)
        index = 0
        for i in range(self.raw_grid_map_height_pixel):
            for j in range(self.raw_grid_map_width_pixel):
                self.raw_grid_map_data_2d[0][self.raw_grid_map_height_pixel - 1 - i][j] = self.raw_grid_map_data[index]
                index += 1
        # debug: 检查转换是否正确
        print(f'原始地图2D数据:\n{self.raw_grid_map_data_2d}')

        # 绝对值原点坐标
        true_origin_point = (abs(self.raw_grid_map_origin_x), self.raw_grid_map_height_pixel - abs(self.raw_grid_map_origin_y))
        explore_origin_points = []
        explore_results = []

        print(true_origin_point)





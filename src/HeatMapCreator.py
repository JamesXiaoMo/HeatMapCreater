import numpy as np


class HeatMapCreator:
    """
    由ROS2发布的/map话题数据生成热力图
    """
    def __init__(self):
        self.raw_grid_map_width_pixel = 0         # 原始地图的宽度 [pixel]
        self.raw_grid_map_height_pixel = 0        # 原始地图的高度 [pixel]
        self.raw_grid_map_width = 0               # 原始地图的宽度 [m]
        self.raw_grid_map_height = 0              # 原始地图的高度 [m]
        self.raw_grid_map_resolution = 0          # 原始地图的分辨率 [m/pixel]
        self.raw_grid_map_data = []               # 原始地图的数据

        self.raw_grid_map_data_2d = []            # 原始地图的2D数据

        self.heat_map_width_pixel = 0             # 热力图的宽度 [pixel]
        self.heat_map_height_pixel = 0            # 热力图的高度 [pixel]
        self.heat_map_width = 0                   # 热力图的宽度 [m]
        self.heat_map_height = 0                  # 热力图的高度 [m]
        self.heat_map_resolution = 0              # 热力图的分辨率 [m/pixel]
        self.heat_map_data_2d = []                # 热力图的2D数据

    def update(
        self,
        raw_grid_map_data: list,
        raw_grid_map_width_pixel: int,
        raw_grid_map_height_pixel: int,
        raw_grid_map_resolution: float,
        heat_map_resolution: float,
    ):
        """
        用于在订阅ROS2的/map时的回调函数
        :param raw_grid_map_data: 原始地图的数据
        :param raw_grid_map_width_pixel: 原始地图的宽度 [pixel]
        :param raw_grid_map_height_pixel: 原始地图的高度 [pixel]
        :param raw_grid_map_resolution: 原始地图的分辨率 [m/pixel]
        :param heat_map_resolution: 热力图的分辨率 [m/pixel]
        :return:
        """
        self.raw_grid_map_width_pixel = raw_grid_map_width_pixel
        self.raw_grid_map_height_pixel = raw_grid_map_height_pixel
        self.raw_grid_map_resolution = raw_grid_map_resolution
        self.raw_grid_map_width = raw_grid_map_width_pixel * raw_grid_map_resolution
        self.raw_grid_map_height = raw_grid_map_height_pixel * raw_grid_map_resolution
        self.raw_grid_map_data = raw_grid_map_data

        self.raw_grid_map_data_2d = np.full((self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel), -1)

        # 将原始地图的数据转化为2D形式
        index = 0
        for i in range(raw_grid_map_height_pixel):
            for j in range(raw_grid_map_width_pixel):
                self.raw_grid_map_data_2d[self.raw_grid_map_height_pixel - 1 - i][j] = self.raw_grid_map_data[index]
                index += 1

        # print(self.raw_grid_map_data_2d)

        self.heat_map_resolution = heat_map_resolution
        self.heat_map_width = self.raw_grid_map_width
        self.heat_map_height = self.raw_grid_map_height
        self.heat_map_width_pixel = self.heat_map_width // self.heat_map_resolution
        self.heat_map_height_pixel = self.heat_map_height // self.heat_map_resolution


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
        self.raw_grid_map_origin_x = 0.0          # 原始地图原点x轴坐标 [pixel]
        self.raw_grid_map_origin_y = 0.0          # 原始地图原点y轴坐标 [pixel]
        self.raw_grid_map_delta_x_positive = 0    # 原始地图数据x轴正方向到原点距离 [pixel]
        self.raw_grid_map_delta_x_negative = 0    # 原始地图数据x轴负方向到原点距离 [pixel]
        self.raw_grid_map_delta_y_positive = 0    # 原始地图数据y轴正方向到原点距离 [pixel]
        self.raw_grid_map_delta_y_negative = 0    # 原始地图数据y轴负方向到原点距离 [pixel]

        self.raw_grid_map_data_2d = []            # 原始地图的2D数据

        self.heat_map_width_pixel = 0             # 热力图的宽度 [pixel]
        self.heat_map_height_pixel = 0            # 热力图的高度 [pixel]
        self.heat_map_width = 0                   # 热力图的宽度 [m]
        self.heat_map_height = 0                  # 热力图的高度 [m]
        self.heat_map_resolution = 0              # 热力图的分辨率 [m/pixel]
        self.heat_map_data_2d = []                # 热力图的2D数据

        self.counter = 0                          # 获取原始数据的计数器

    def callback(
        self,
        raw_grid_map_data: list,
        raw_grid_map_width_pixel: int,
        raw_grid_map_height_pixel: int,
        raw_grid_map_resolution: float,
        raw_grid_map_origin_x: float,
        raw_grid_map_origin_y: float,
        heat_map_resolution: float,
    ):
        """
        用于在订阅ROS2的/map时的回调函数
        :param raw_grid_map_data: 原始地图的数据
        :param raw_grid_map_width_pixel: 原始地图的宽度 [pixel]
        :param raw_grid_map_height_pixel: 原始地图的高度 [pixel]
        :param raw_grid_map_resolution: 原始地图的分辨率 [m/pixel]
        :param raw_grid_map_origin_x: 原始地图原点x轴坐标 [pixel]
        :param raw_grid_map_origin_y: 原始地图原点y轴坐标 [pixel]
        :param heat_map_resolution: 热力图的分辨率 [m/pixel]
        :return:
        """
        self.raw_grid_map_width_pixel = raw_grid_map_width_pixel
        self.raw_grid_map_height_pixel = raw_grid_map_height_pixel
        self.raw_grid_map_resolution = raw_grid_map_resolution
        self.raw_grid_map_width = raw_grid_map_width_pixel * raw_grid_map_resolution
        self.raw_grid_map_height = raw_grid_map_height_pixel * raw_grid_map_resolution
        self.raw_grid_map_data = raw_grid_map_data

        # 判断是否为第一次传入数据
        if self.counter == 0 and len(self.raw_grid_map_data_2d) == 0:
            # 第一次传入数据
            self.raw_grid_map_full_update()
        else:
            # 非第一次传入数据
            self.raw_grid_map_part_update()

        # 获取原始数据的计数器自增
        self.counter += 1

        # self.heat_map_resolution = heat_map_resolution
        # self.heat_map_width = self.raw_grid_map_width
        # self.heat_map_height = self.raw_grid_map_height
        # self.heat_map_width_pixel = self.heat_map_width // self.heat_map_resolution
        # self.heat_map_height_pixel = self.heat_map_height // self.heat_map_resolution

    def raw_grid_map_full_update(self):
        """
        由原始地图数据完整转换为2D数据
        仅在第一次传入原始数据时调用
        :return:
        """
        # 创建原点到各边的距离
        self.raw_grid_map_delta_x_positive = raw_grid_map_width_pixel - raw_grid_map_origin_x
        self.raw_grid_map_delta_x_negative = raw_grid_map_origin_x
        self.raw_grid_map_delta_y_positive = raw_grid_map_height_pixel - raw_grid_map_origin_y
        self.raw_grid_map_delta_y_negative = raw_grid_map_origin_y
        # 创建空数组
        self.raw_grid_map_data_2d = np.full((self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel), -1)
        # 传入原始地图的坐标原点（仅进行一次）
        self.raw_grid_map_origin_x = raw_grid_map_origin_x
        self.raw_grid_map_origin_y = raw_grid_map_origin_y
        # 将原始地图的数据转化为2D形式(完整变换，仅进行一次)
        index = 0
        for i in range(raw_grid_map_height_pixel):
            for j in range(raw_grid_map_width_pixel):
                self.raw_grid_map_data_2d[self.raw_grid_map_height_pixel - 1 - i][j] = self.raw_grid_map_data[index]
                index += 1
        # debug: 检查转换是否正确
        print(f'原始地图2D数据:\n{self.raw_grid_map_data_2d}')

    def raw_grid_map_part_update(self):
        """
        由原始地图数据新增部分加入现有的2D数据
        除了第一次以外传入原始数据时调用
        :return:
        """


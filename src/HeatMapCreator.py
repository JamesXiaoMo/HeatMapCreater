import numpy as np


class HeatMapCreator:
    """
    由ROS2发布的/map话题数据生成热力图
    """

    def __init__(self, heat_map_interval: int):
        self.raw_grid_map_width_pixel = 0  # 原始地图的宽度 [pixel]
        self.raw_grid_map_height_pixel = 0  # 原始地图的高度 [pixel]
        self.raw_grid_map_resolution = 0  # 原始地图的分辨率 [m/pixel]
        self.raw_grid_map_data = []  # 原始地图的数据

        self.raw_grid_map_data_2d = []  # 原始地图的2D数据

        self.heat_map_width_pixel = 0  # 热力图的宽度 [pixel]
        self.heat_map_height_pixel = 0  # 热力图的高度 [pixel]
        self.heat_map_resolution = 0  # 热力图的分辨率 [m/pixel]
        self.heat_map_interval = heat_map_interval  # 热力图的测量间隔 [m]
        self.heat_map_interval_pixel = 0  # 热力图的测量间隔 [pixel]

        self.robot_radius = 0.16  # 机器人半径 [m]
        self.robot_radius_pixel = 0  # 机器人半径 [pixel]

        self.available_measurement_points = []  # 实际可用测量点坐标

    def callback(
            self,
            raw_grid_map_data: list,
            raw_grid_map_width_pixel: int,
            raw_grid_map_height_pixel: int,
            raw_grid_map_resolution: float,
            raw_grid_map_origin_x: int,
            raw_grid_map_origin_y: int,
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
        self.raw_grid_map_data = raw_grid_map_data
        self.raw_grid_map_width_pixel = raw_grid_map_width_pixel
        self.raw_grid_map_height_pixel = raw_grid_map_height_pixel
        self.raw_grid_map_resolution = raw_grid_map_resolution

        self.heat_map_interval_pixel = int(self.heat_map_interval / self.raw_grid_map_resolution)

        self.robot_radius_pixel = int(self.robot_radius / self.raw_grid_map_resolution)

        # 创建空数组
        self.raw_grid_map_data_2d = np.full((self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel), -1)
        # 将原始地图的数据转化为2D形式(完整变换，仅进行一次)
        index = 0
        for i in range(self.raw_grid_map_height_pixel):
            for j in range(self.raw_grid_map_width_pixel):
                self.raw_grid_map_data_2d[self.raw_grid_map_height_pixel - 1 - i][j] = self.raw_grid_map_data[index]
                index += 1
        # world坐标 -> map坐标
        world_origin = [-raw_grid_map_origin_x, self.raw_grid_map_height_pixel - 1 - raw_grid_map_origin_y]

        self.raw_grid_map_data_2d[world_origin[1]][world_origin[0]] = 1

        explore_points = [world_origin]
        explore_results = []

        while len(explore_points) != 0:
            for i in explore_points:
                # 上
                if (
                        0
                        <= i[1] - self.heat_map_interval_pixel
                        <= self.raw_grid_map_height_pixel - 1
                ):
                    explore_results.append([i[0], i[1] - self.heat_map_interval_pixel])
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[i[1] - self.heat_map_interval_pixel][i[0]] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if (((-self.robot_radius_pixel + x) ** 2 + (-self.robot_radius_pixel + y) ** 2) ** 0.5
                                        <= self.robot_radius_pixel):
                                    if self.raw_grid_map_data_2d[i[1] - self.heat_map_interval_pixel - self.robot_radius_pixel + y][i[0] - self.robot_radius_pixel + x] == 100:
                                        is_point_available = False
                        # 2025.11.7 0:07
                        if is_point_available:
                            for j in self.available_measurement_points:
                                if j[0] - 10 <= int(i[0] - true_origin_point[0]) <= j[0] + 10 and j[1] - 10 <= int(i[1] - self.heat_map_interval_pixel - true_origin_point[1]) <= j[1] + 10:
                                    is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([int(i[0] - true_origin_point[0]), int(i[1] - self.heat_map_interval_pixel - true_origin_point[1])])
                        self.raw_grid_map_data_2d[0][i[1] - self.heat_map_interval_pixel][i[0]] = 1

                # 下
                if (
                        0
                        <= i[1] + self.heat_map_interval_pixel
                        <= self.raw_grid_map_height_pixel - 1
                        and self.raw_grid_map_data_2d[i[1] + self.heat_map_interval_pixel][i[0]] != 1
                ):
                    explore_results.append([i[0], i[1] + self.heat_map_interval_pixel])
                    self.raw_grid_map_data_2d[1][i[1] + self.heat_map_interval_pixel][i[0]] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][i[1] + self.heat_map_interval_pixel][i[0]] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (
                                        -self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][i[1] + self.heat_map_interval_pixel - self.robot_radius_pixel + y][i[0] - self.robot_radius_pixel + x] == 100:
                                        is_point_available = False
                        if is_point_available:
                            for j in self.available_measurement_points:
                                if j[0] - 10 <= int(i[0] - true_origin_point[0]) <= j[0] + 10 and j[1] - 10 <= int(i[1] + self.heat_map_interval_pixel - true_origin_point[1]) <= j[1] + 10:
                                    is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([int(i[0] - true_origin_point[0]), int(i[1] + self.heat_map_interval_pixel - true_origin_point[1])])
                        self.raw_grid_map_data_2d[0][i[1] + self.heat_map_interval_pixel][i[0]] = 1

                # 左
                if (
                        0
                        <= i[0] - self.heat_map_interval_pixel
                        <= self.raw_grid_map_width_pixel - 1
                        and self.raw_grid_map_data_2d[i[1]][i[0] - self.heat_map_interval_pixel]!= 1
                ):
                    explore_results.append([i[0] - self.heat_map_interval_pixel, i[1]])
                    self.raw_grid_map_data_2d[1][i[1]][i[0] - self.heat_map_interval_pixel] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][i[1]][i[0] - self.heat_map_interval_pixel] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (
                                        -self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][i[1] - self.robot_radius_pixel + y][i[0] - self.heat_map_interval_pixel - self.robot_radius_pixel + x] == 100:
                                        is_point_available = False
                        if is_point_available:
                            for j in self.available_measurement_points:
                                if j[0] - 10 <= int(i[0] - self.heat_map_interval_pixel - true_origin_point[0]) <= j[0] + 10 and j[1] - 10 <= int(i[1] - true_origin_point[1]) <= j[1] + 10:
                                    is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([int(i[0] - self.heat_map_interval_pixel - true_origin_point[0]), int(i[1] - true_origin_point[1])])
                        self.raw_grid_map_data_2d[0][i[1]][i[0] - self.heat_map_interval_pixel] = 1

                # 右
                if (
                        0
                        <= i[0] + self.heat_map_interval_pixel
                        <= self.raw_grid_map_width_pixel - 1
                        and self.raw_grid_map_data_2d[i[1]][i[0] + self.heat_map_interval_pixel] != 1
                ):
                    explore_results.append([i[0] + self.heat_map_interval_pixel, i[1]])
                    self.raw_grid_map_data_2d[1][i[1]][i[0] + self.heat_map_interval_pixel] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][i[1]][i[0] + self.heat_map_interval_pixel] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (
                                        -self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][i[1] - self.robot_radius_pixel + y][i[0] + self.heat_map_interval_pixel - self.robot_radius_pixel + x] == 100:
                                        is_point_available = False
                        if is_point_available:
                            for j in self.available_measurement_points:
                                if j[0] - 10 <= int(i[0] + self.heat_map_interval_pixel - true_origin_point[0]) <= j[0] + 10 and j[1] - 10 <= int(i[1] - true_origin_point[1]) <= j[1] + 10:
                                    is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([int(i[0] + self.heat_map_interval_pixel - true_origin_point[0]), int(i[1] - true_origin_point[1])])
                        self.raw_grid_map_data_2d[0][i[1]][i[0] + self.heat_map_interval_pixel] = 1

            explore_points = explore_results
            explore_results = []
        print(self.available_measurement_points)

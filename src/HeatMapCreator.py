import numpy as np


def value_to_color(val):
    """
    val: 0~1
    return: [B,G,R] (0~255)
    """
    val = np.clip(val, 0.0, 1.0)
    b, g, r = 0, 0, 0
    if val <= 0.33:
        b = 255 * ((0.33 - val) / 0.33)
        g = 255 * (val / 0.33)
        r = 0
    elif 0.33 < val <= 0.66:
        b = 0
        g = 255
        r = 255 * ((val - 0.33) / 0.33)
    else:
        b = 0
        g = 255 * ((1.0 - val) / 0.33)
        r = 255
    return [b, g, r]  # OpenCV/ROS 用 BGR 顺序


class HeatMapCreator:
    """
    由ROS2发布的/map话题数据生成热力图
    """

    def __init__(self, heat_map_interval: float):
        self.raw_grid_map_width_pixel = 0  # 原始地图的宽度 [pixel]
        self.raw_grid_map_height_pixel = 0  # 原始地图的高度 [pixel]
        self.raw_grid_map_width = 0  # 原始地图的宽度 [m]
        self.raw_grid_map_height = 0  # 原始地图的高度 [m]
        self.raw_grid_map_resolution = 0  # 原始地图的分辨率 [m/pixel]
        self.raw_grid_map_data = []  # 原始地图的数据
        self.raw_grid_map_origin_x = 0  # 原始地图原点x轴坐标 [m]
        self.raw_grid_map_origin_x_pixel = 0  # 原始地图原点x轴坐标 [pixel]
        self.raw_grid_map_origin_y = 0  # 原始地图原点y轴坐标 [m]
        self.raw_grid_map_origin_y_pixel = 0  # 原始地图原点y轴坐标 [pixel]

        self.raw_grid_map_data_2d = []  # 原始地图的2D数据

        self.heat_map_width_pixel = 0  # 热力图的宽度 [pixel]
        self.heat_map_height_pixel = 0  # 热力图的高度 [pixel]
        self.heat_map_width = 0  # 热力图的宽度 [m]
        self.heat_map_height = 0  # 热力图的高度 [m]
        self.heat_map_resolution = 0  # 热力图的分辨率 [m/pixel]
        self.heat_map_interval = heat_map_interval  # 热力图的测量间隔 [m]
        self.heat_map_interval_pixel = 0  # 热力图的测量间隔 [pixel]
        self.heat_map_data_2d = []  # 热力图的2D数据

        self.robot_radius = 0.16  # 机器人半径 [m]
        self.robot_radius_pixel = 0  # 机器人半径 [pixel]

        self.available_measurement_points = []  # 实际可用测量点坐标 [pixel]
        self.available_measurement_points_world = []  # 世界坐标系中实际可用测量点坐标 [m]

    def map_callback(
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
        :param raw_grid_map_origin_x: 原始地图原点x轴坐标 [m]
        :param raw_grid_map_origin_y: 原始地图原点y轴坐标 [m]
        :return:
        """
        # 初始化
        self.raw_grid_map_data = raw_grid_map_data
        self.raw_grid_map_width_pixel = raw_grid_map_width_pixel
        self.raw_grid_map_height_pixel = raw_grid_map_height_pixel
        self.raw_grid_map_resolution = raw_grid_map_resolution
        self.raw_grid_map_origin_x = raw_grid_map_origin_x
        self.raw_grid_map_origin_x_pixel = int(raw_grid_map_origin_x / raw_grid_map_resolution)
        self.raw_grid_map_origin_y = raw_grid_map_origin_y
        self.raw_grid_map_origin_y_pixel = int(raw_grid_map_origin_y / raw_grid_map_resolution)

        self.heat_map_interval_pixel = int(self.heat_map_interval / self.raw_grid_map_resolution)

        self.robot_radius_pixel = int(self.robot_radius / self.raw_grid_map_resolution)

        # 创建空数组
        self.raw_grid_map_data_2d = np.full((2, self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel), -1)
        # 将原始地图的数据转化为2D形式(完整变换，仅进行一次)
        index = 0
        for i in range(self.raw_grid_map_height_pixel):
            for j in range(self.raw_grid_map_width_pixel):
                self.raw_grid_map_data_2d[0][self.raw_grid_map_height_pixel - 1 - i][j] = self.raw_grid_map_data[index]
                index += 1
        # debug: 检查转换是否正确
        # print(f'原始地图2D数据:\n{self.raw_grid_map_data_2d}')

        # 像素坐标系中的世界坐标系原点的坐标
        true_origin_point = [
            -self.raw_grid_map_origin_x_pixel,
            self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel
        ]
        # debug: 像素坐标系中的世界坐标系原点的坐标
        print(f'像素坐标系中的世界坐标系原点的坐标: {true_origin_point}')

        self.raw_grid_map_data_2d[1][true_origin_point[1]][true_origin_point[0]] = 1

        explore_origin_points = [true_origin_point]
        explore_results = []

        self.available_measurement_points.clear()
        self.available_measurement_points_world.clear()

        while len(explore_origin_points) != 0:
            for i in explore_origin_points:
                # 上
                new_x = i[0]
                new_y = i[1] - self.heat_map_interval_pixel
                if (
                        0
                        <= new_y
                        <= self.raw_grid_map_height_pixel - 1
                        and self.raw_grid_map_data_2d[1][new_y][new_x] != 1
                ):
                    explore_results.append([new_x, new_y])
                    self.raw_grid_map_data_2d[1][new_y][new_x] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][new_y][new_x] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (-self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][new_y - self.robot_radius_pixel + y][new_x - self.robot_radius_pixel + x] != 0:
                                        is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([new_x, new_y])
                        self.raw_grid_map_data_2d[0][new_y][new_x] = 1

                # 下
                new_x = i[0]
                new_y = i[1] + self.heat_map_interval_pixel
                if (
                        0
                        <= new_y
                        <= self.raw_grid_map_height_pixel - 1
                        and self.raw_grid_map_data_2d[1][new_y][new_x] != 1
                ):
                    explore_results.append([new_x, new_y])
                    self.raw_grid_map_data_2d[1][new_y][new_x] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][new_y][new_x] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (-self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][new_y - self.robot_radius_pixel + y][new_x - self.robot_radius_pixel + x] != 0:
                                        is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([new_x, new_y])
                        self.raw_grid_map_data_2d[0][new_y][new_x] = 1

                # 左
                new_x = i[0] - self.heat_map_interval_pixel
                new_y = i[1]
                if (
                        0
                        <= new_x
                        <= self.raw_grid_map_width_pixel - 1
                        and self.raw_grid_map_data_2d[1][new_y][new_x] != 1
                ):
                    explore_results.append([new_x, new_y])
                    self.raw_grid_map_data_2d[1][new_y][new_x] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][new_y][new_x] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (-self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][new_y - self.robot_radius_pixel + y][new_x - self.robot_radius_pixel + x] != 0:
                                        is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([new_x, new_y])
                        self.raw_grid_map_data_2d[0][new_y][new_x] = 1

                # 右
                new_x = i[0] + self.heat_map_interval_pixel
                new_y = i[1]
                if (
                        0
                        <= new_x
                        <= self.raw_grid_map_width_pixel - 1
                        and self.raw_grid_map_data_2d[1][new_y][new_x] != 1
                ):
                    explore_results.append([new_x, new_y])
                    self.raw_grid_map_data_2d[1][new_y][new_x] = 1
                    is_point_available = True
                    if not self.raw_grid_map_data_2d[0][new_y][new_x] == 0:
                        is_point_available = False
                    else:
                        for y in range(self.robot_radius_pixel * 2):
                            for x in range(self.robot_radius_pixel * 2):
                                if ((-self.robot_radius_pixel + x) ** 2 + (-self.robot_radius_pixel + y) ** 2) ** 0.5 <= self.robot_radius_pixel:
                                    if self.raw_grid_map_data_2d[0][new_y - self.robot_radius_pixel + y][new_x - self.robot_radius_pixel + x] != 0:
                                        is_point_available = False
                    if is_point_available:
                        self.available_measurement_points.append([new_x, new_y])
                        self.raw_grid_map_data_2d[0][new_y][new_x] = 1

            explore_origin_points = explore_results
            explore_results = []
        for n in self.available_measurement_points:
            self.available_measurement_points_world.append([
                (n[0] - true_origin_point[0]) * self.raw_grid_map_resolution,
                -(n[1] - true_origin_point[1]) * self.raw_grid_map_resolution
            ])
        print(f'{len(self.available_measurement_points_world)}个测量点\n{self.available_measurement_points_world}')

    def heatmap_callback(self):
        """
        生成亮度热力图
        :return:
        """
        img = np.ones(
            (self.raw_grid_map_height_pixel, self.raw_grid_map_width_pixel, 3),
            dtype=np.uint8
        ) * 150

        # debug:创造假亮度数据
        for i in self.available_measurement_points:
            i.append(np.random.randint(0, 100) * 0.01)

        brightness = []
        for i in self.available_measurement_points:
            brightness.append(i[2])

        # 生成热力图
        # origin_x = -self.raw_grid_map_origin_x_pixel
        # origin_y = self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel
        # img[origin_y + 1:origin_y + self.heat_map_interval_pixel, origin_x + 1:origin_x + self.heat_map_interval_pixel] = value_to_color(0.9)

        for i in self.available_measurement_points:
            brightness_1 = [i[2]]
            brightness_2 = [i[2]]
            brightness_3 = [i[2]]
            brightness_4 = [i[2]]
            for j in self.available_measurement_points:
                if i[0] + self.heat_map_interval_pixel - 3 <= j[0] <= i[0] + self.heat_map_interval_pixel + 3 and i[1] - 3 <= j[0] <= i[1] + 3:
                    brightness_1.append(j[2])
                    brightness_4.append(j[2])
                elif i[0] + self.heat_map_interval_pixel - 3 <= j[0] <= i[0] + self.heat_map_interval_pixel + 3 and i[1] - self.heat_map_interval_pixel - 3 <= j[1] <= i[1] - self.heat_map_interval_pixel + 3:
                    brightness_1.append(j[2])
                elif i[0] - 3 <= j[0] <= i[0] + 3 and i[1] - self.heat_map_interval_pixel - 3 <= j[0] <= i[1] - self.heat_map_interval_pixel + 3:
                    brightness_1.append(j[2])
                    brightness_2.append(j[2])
                elif i[0] - self.heat_map_interval_pixel - 3 <= j[0] <= i[0] - self.heat_map_interval_pixel + 3 and i[1] - self.heat_map_interval_pixel - 3 <= j[1] <= i[1] - self.heat_map_interval_pixel + 3:
                    brightness_2.append(j[2])
                elif i[0] - self.heat_map_interval_pixel - 3 <= j[0] <= i[0] - self.heat_map_interval_pixel + 3 and i[1] - 3 <= j[1] <= i[1] + 3:
                    brightness_2.append(j[2])
                    brightness_3.append(j[2])
                elif i[0] - self.heat_map_interval_pixel - 3 <= j[0] <= i[0] - self.heat_map_interval_pixel + 3 and i[1] + self.heat_map_interval_pixel - 3 <= j[1] <= i[1] + self.heat_map_interval_pixel + 3:
                    brightness_3.append(j[2])
                elif i[0] - 3 <= j[0] <= i[0] + 3 and i[1] + self.heat_map_interval_pixel - 3 <= j[0] <= i[1] + self.heat_map_interval_pixel + 3:
                    brightness_3.append(j[2])
                    brightness_4.append(j[2])
                elif i[0] + self.heat_map_interval_pixel - 3 <= j[0] <= i[0] + self.heat_map_interval_pixel + 3 and i[1] + self.heat_map_interval_pixel - 3 <= j[1] <= i[1] + self.heat_map_interval_pixel + 3:
                    brightness_4.append(j[2])
            value_1 = ((sum(brightness_1) / len(brightness_1)) - min(brightness))/(max(brightness) - min(brightness))
            img[i[1] - self.heat_map_interval_pixel:i[1], i[0] + 1:i[0] + self.heat_map_interval_pixel] = value_to_color(value_1)
            value_2 = ((sum(brightness_2) / len(brightness_2)) - min(brightness)) / (max(brightness) - min(brightness))
            img[i[1] - self.heat_map_interval_pixel:i[1], i[0] - self.heat_map_interval_pixel:i[0]] = value_to_color(value_2)
            value_3 = ((sum(brightness_3) / len(brightness_3)) - min(brightness)) / (max(brightness) - min(brightness))
            img[i[1] + 1:i[1] + self.heat_map_interval_pixel, i[0] - self.heat_map_interval_pixel:i[0]] = value_to_color(value_3)
            value_4 = ((sum(brightness_4) / len(brightness_4)) - min(brightness)) / (max(brightness) - min(brightness))
            img[i[1] + 1:i[1] + self.heat_map_interval_pixel, i[0] + 1:i[0] + self.heat_map_interval_pixel] = value_to_color(value_4)

        # 生成网格线
        for y_positive in range(int((self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel) / self.heat_map_interval_pixel) + 1):
            y = int(self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel - y_positive * self.heat_map_interval_pixel)
            img[y, :] = [50, 50, 50]

        for y_negative in range(int((self.raw_grid_map_height_pixel - (self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel)) / self.heat_map_interval_pixel) + 1):
            y = int(self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel + y_negative * self.heat_map_interval_pixel)
            img[y, :] = [50, 50, 50]

        for x_positive in range(int((self.raw_grid_map_width_pixel - -self.raw_grid_map_origin_x_pixel) / self.heat_map_interval_pixel) + 1):
            x = int(-self.raw_grid_map_origin_x_pixel + x_positive * self.heat_map_interval_pixel)
            img[:, x] = [50, 50, 50]

        for x_negative in range(int(-self.raw_grid_map_origin_x_pixel / self.heat_map_interval_pixel) + 1):
            x = int(-self.raw_grid_map_origin_x_pixel - x_negative * self.heat_map_interval_pixel)
            img[:, x] = [50, 50, 50]

        index = 0
        for i in range(self.raw_grid_map_height_pixel):
            for j in range(self.raw_grid_map_width_pixel):
                if self.raw_grid_map_data_2d[0][self.raw_grid_map_height_pixel - 1 - i][j] > 1:
                    img[-i, j] = [0, 0, 0]
                index += 1

        # 生成原点坐标
        origin_x = -self.raw_grid_map_origin_x_pixel
        origin_y = self.raw_grid_map_height_pixel - 1 - -self.raw_grid_map_origin_y_pixel
        img[origin_y - 1:origin_y + 2, origin_x - 1:origin_x + 2] = [0, 0, 255]

        # 生成测量点坐标
        for i in self.available_measurement_points:
            img[i[1] - 1:i[1] + 2, i[0] - 1:i[0] + 2] = [255, 0, 0]
        return img


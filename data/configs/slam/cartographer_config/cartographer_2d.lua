include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- название фрейма карты
  tracking_frame = "base_link",  -- название отслеживаемого фрейма
  published_frame = "base_link",  -- название публикуемого фрейма
  odom_frame = "odom",  -- название фрейма одометрии
  provide_odom_frame = true,  -- предоставлять ли фрейм одометрии
  publish_frame_projected_to_2d = false,  -- публиковать ли 2D позу
  use_pose_extrapolator = false,
  use_odometry = false,  -- использовать ли одометрию
  use_nav_sat = false,  -- использовать ли навигационный спутник
  use_landmarks = false,  -- использовать ли ориентиры
  num_laser_scans = 1,  -- количество лидаров
  num_multi_echo_laser_scans = 0,  -- количество многоканальных лидаров
  num_subdivisions_per_laser_scan = 1,  -- количество подразделений для каждого сканирования
  num_point_clouds = 0,  -- количество облаков точек
  lookup_transform_timeout_sec = 0.2,  -- таймаут поиска преобразований (секунды)
  submap_publish_period_sec = 0.3,  -- период публикации вспомогательных карт
  pose_publish_period_sec = 5e-3,  -- период публикации позы (секунды)
  trajectory_publish_period_sec = 30e-3,  -- период публикации траектории (секунды)
  rangefinder_sampling_ratio = 1.,  -- коэффициент выборки дальномера
  odometry_sampling_ratio = 1.,  -- коэффициент выборки одометрии
  fixed_frame_pose_sampling_ratio = 1.,  -- коэффициент выборки позы фиксированного фрейма
  imu_sampling_ratio = 1.,  -- коэффициент выборки IMU
  landmarks_sampling_ratio = 1.,  -- коэффициент выборки ориентиров
}
 
MAP_BUILDER.use_trajectory_builder_2d = true  -- использовать ли 2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- Количество данных диапазона для вспомогательных карт в 2D построителе траекторий
TRAJECTORY_BUILDER_2D.min_range = 0.5  -- минимальный диапазон сканирования лидара
TRAJECTORY_BUILDER_2D.max_range = 15.5  -- максимальный диапазон сканирования лидара
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- ограничено максимальным диапазоном сканирования лидара
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- использовать ли данные IMU
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- использовать ли сопоставление сканов с реальным временем для обнаружения петель

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Изменено с 1.0 на 0.1, увеличена чувствительность к движению
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Изменено с 0.55 на 0.65, минимальный score Fast csm, можно оптимизировать выше этого значения
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Изменено с 0.6 на 0.7, минимальный score глобальной позиции, ниже которого глобальная позиция считается неточной

return options
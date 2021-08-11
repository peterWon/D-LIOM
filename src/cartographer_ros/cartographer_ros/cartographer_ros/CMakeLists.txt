# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

google_binary(cartographer_assets_writer
  SRCS
    assets_writer_main.cc
    ros_map_writing_points_processor.h
    ros_map_writing_points_processor.cc
)

install(TARGETS cartographer_assets_writer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_node
  SRCS
    node_main.cc
)

install(TARGETS cartographer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_offline_node
  SRCS
    offline_node_main.cc
)

install(TARGETS cartographer_offline_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_start_trajectory
  SRCS
    start_trajectory_main.cc
)

install(TARGETS cartographer_start_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_occupancy_grid_node
  SRCS
    occupancy_grid_node_main.cc
)

install(TARGETS cartographer_occupancy_grid_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_rosbag_validate
  SRCS
    rosbag_validate_main.cc
)

install(TARGETS cartographer_rosbag_validate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_kitti_trajectory_from_pbstream
  SRCS
  kitti_trajectory_from_pbstream.cc
)

install(TARGETS cartographer_kitti_trajectory_from_pbstream
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_read_constraints_from_pbsteam
  SRCS
  read_constraints_from_pbsteam.cc
)

install(TARGETS cartographer_read_constraints_from_pbsteam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_kaist_trajectory_from_pbstream
  SRCS
  kaist_trajectory_from_pbstream.cc
)

install(TARGETS cartographer_kaist_trajectory_from_pbstream
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_kitti_urdf_param_caculator
  SRCS
  kitti_urdf_param_caculator.cc
)

install(TARGETS cartographer_kitti_urdf_param_caculator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_dlio_eval_node
  SRCS
  dlio_eval_node.cc
)

install(TARGETS cartographer_dlio_eval_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_trajectory_info_from_pbstream
  SRCS
  trajectory_info_from_pbstream.cc
)

install(TARGETS cartographer_trajectory_info_from_pbstream
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_gen_ground_truth_by_ndt_match
  SRCS
  gen_ground_truth_by_ndt_match.cc
)

install(TARGETS cartographer_gen_ground_truth_by_ndt_match
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


google_binary(cartographer_pbstream_3d_map_publisher_main
  SRCS
  pbstream_3d_map_publisher_main.cc
)

install(TARGETS cartographer_pbstream_3d_map_publisher_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_pbstream_to_ros_map
  SRCS
    pbstream_to_ros_map_main.cc
)

install(TARGETS cartographer_pbstream_to_ros_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_pb_range_data_to_ros_cloud_main
  SRCS
    pb_range_data_to_ros_cloud_main.cc
)

install(TARGETS cartographer_pb_range_data_to_ros_cloud_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_pbstream_map_publisher
  SRCS
    pbstream_map_publisher_main.cc
)

install(TARGETS cartographer_pbstream_map_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_dev_rosbag_publisher
  SRCS
  dev/rosbag_publisher_main.cc
)

install(TARGETS cartographer_dev_rosbag_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_dev_trajectory_comparison
  SRCS
  dev/trajectory_comparison_main.cc
)

install(TARGETS cartographer_dev_trajectory_comparison
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO(cschuet): Add support for shared library case.
if (${BUILD_GRPC})
  google_binary(cartographer_grpc_node
    SRCS
      cartographer_grpc/node_grpc_main.cc
  )

  install(TARGETS cartographer_grpc_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  google_binary(cartographer_grpc_offline_node
    SRCS
      cartographer_grpc/offline_node_grpc_main.cc
  )

  install(TARGETS cartographer_grpc_offline_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(PROGRAMS
    ../scripts/cartographer_grpc_server.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

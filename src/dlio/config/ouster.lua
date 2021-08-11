-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "basic_config_3d.lua"

options.tracking_frame = "os1_imu"
options.sensor_type = "ouster"

TRAJECTORY_BUILDER_3D.scan_period = 0.1
TRAJECTORY_BUILDER_3D.eable_mannually_discrew = true
TRAJECTORY_BUILDER_3D.frames_for_static_initialization = 7
TRAJECTORY_BUILDER_3D.frames_for_dynamic_initialization = 7
TRAJECTORY_BUILDER_3D.enable_ndt_initialization = true

TRAJECTORY_BUILDER_3D.imu.acc_noise= 1.249*1e2
TRAJECTORY_BUILDER_3D.imu.gyr_noise= 2.08*1e-1
TRAJECTORY_BUILDER_3D.imu.acc_bias_noise= 0.000106
TRAJECTORY_BUILDER_3D.imu.gyr_bias_noise= 0.000004
TRAJECTORY_BUILDER_3D.imu.gravity= 9.80511

return options
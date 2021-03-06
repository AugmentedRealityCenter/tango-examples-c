/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TANGO_DATA_H
#define TANGO_DATA_H
#define GLM_FORCE_RADIANS

#include <pthread.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <tango_client_api.h>
#include "tango-gl-renderer/gl_util.h"

const int kMeterToMillimeter = 1000;
const int kVersionStringLength = 27;
const float kSecondToMillisecond = 1000.0f;

// Opengl camera to color camera matrix.
const glm::mat4 oc_2_c_mat =
    glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
              -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// Start service to opengl world matrix.
const glm::mat4 ss_2_ow_mat =
    glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

using namespace std;

class TangoData {
 public:
  static TangoData& GetInstance() {
    static TangoData instance;
    return instance;
  }
  TangoData();
  ~TangoData();

  TangoErrorType Initialize(JNIEnv* env, jobject activity);
  bool SetConfig();
  TangoErrorType Connect();
  bool ConnectCallbacks();
  void Disconnect();
  bool GetIntrinsics();
  bool SetupExtrinsicsMatrices();
  glm::mat4 GetOC2OWMat(bool is_depth, bool already_locked = false);

  void UpdatePoseData();
  void UpdateXYZijData();

  pthread_mutex_t pose_mutex;
  pthread_mutex_t xyzij_mutex;
  pthread_mutex_t event_mutex;
  pthread_mutex_t image_buffer_mutex;

  float* depth_buffer;
  float* color_buffer;
  uint32_t depth_buffer_size; //Really stores size of depth_buffer, not number of vertices
  bool is_xyzij_dirty;

  float* points_in_world;
  uint32_t piw_size;
  uint32_t piw_front;
  uint32_t piw_num_items;

  TangoPoseData cur_pose_data;
  TangoPoseData prev_pose_data;
  bool is_pose_dirty;

  int pose_status_count;
  float pose_frame_delta_time;

  float depth_average_length;
  float depth_frame_delta_time;

  uint32_t max_vertex_count;

  // Tango Intrinsic for color camera.
  int cc_width;
  int cc_height;
  double cc_fx;
  double cc_fy;
  double cc_cx;
  double cc_cy;
  double cc_distortion[5];

  const TangoImageBuffer* last_image_buffer;

private:
  // Device to start service matrix.
  glm::mat4 d_2_ss_mat_depth;
  // Device to start service matrix.
  glm::mat4 d_2_ss_mat_motion;
  // Device to IMU matrix. -- set once, in SetupExtrinsicsMatrices
  glm::mat4 d_2_imu_mat;
  // Color camera to IMU matrix. -- set once, in SetupExtrinsicsMatrices
  glm::mat4 c_2_imu_mat;
public:
  string event_string;
  string lib_version_string;
  string pose_string;
private:
  TangoConfig config_;
};

#endif  // TANGO_DATA_H

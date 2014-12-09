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

#include "pointcloud.h"

static const char kVertexShader[] =
	"precision highp float;\n"
	"precision highp int;\n"
	"attribute vec4 vertex;\n"
	"attribute vec2 tcs;\n"
	"varying vec2 f_textureCoords;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "  gl_PointSize = 5.0;\n"
    "  gl_Position = mvp*vertex;\n"
	"  f_textureCoords = tcs;\n"
    "}\n";

static const char kFragmentShader[] =
		"#extension GL_OES_EGL_image_external : require\n"
		"precision highp float;\n"
		"precision highp int;\n"
		"uniform samplerExternalOES texture;\n"
		"varying vec2 f_textureCoords;\n"
		"void main() {\n"
		"  gl_FragColor = texture2D(texture, f_textureCoords);\n"
		"}\n";

static const glm::mat4 inverse_z_mat = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
                                                 0.0f, -1.0f, 0.0f, 0.0f,
                                                 0.0f, 0.0f, -1.0f, 0.0f,
                                                 0.0f, 0.0f, 0.0f, 1.0f);

Pointcloud::Pointcloud() {
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  shader_program_ = GlUtil::CreateProgram(kVertexShader, kFragmentShader);
  if (!shader_program_) {
    LOGE("Could not create program.");
  }

  glUseProgram(shader_program_); //Needed so we can use glUniform1i below
  glEnable (GL_TEXTURE_EXTERNAL_OES);
  glGenTextures(1, &texture_id_);
  TangoData::GetInstance().ConnectTexture(texture_id_);

  glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  uniform_texture_ = glGetUniformLocation(shader_program_, "texture");
  glUniform1i(uniform_texture_, texture_id_);
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);

  uniform_mvp_mat_ = glGetUniformLocation(shader_program_, "mvp");
  attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");
  attrib_texCoords_ = glGetAttribLocation(shader_program_, "tcs");

  glGenBuffers(1, &vertex_buffers_);
  glGenBuffers(1, &texCoord_buffers_);
  glUseProgram(0);
}

void Pointcloud::Render(glm::mat4 projection_mat, glm::mat4 view_mat, glm::mat4 model_mat,
                        int depth_buffer_size, float *depth_data_buffer, int texCoord_buffer_size, float *texCoord_data_buffer) {
  glUseProgram(shader_program_);
  // Lock xyz_ij mutex.
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);

  // Calculate model view projection matrix.
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat * inverse_z_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  // Bind vertex buffer.
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * depth_buffer_size,
               depth_data_buffer, GL_STATIC_DRAW);
  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  glBindBuffer(GL_ARRAY_BUFFER, texCoord_buffers_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoord_buffer_size,
                 texCoord_data_buffer, GL_STATIC_DRAW);
  glEnableVertexAttribArray(attrib_texCoords_);
  glVertexAttribPointer(attrib_texCoords_, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDrawArrays(GL_POINTS, 0, depth_buffer_size/3);

  // Unlock xyz_ij mutex.
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);

  GlUtil::CheckGlError("glDrawArray()");
  glUseProgram(0);
  GlUtil::CheckGlError("glUseProgram()");
}

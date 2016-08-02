/*
 * Copyright 2016 The Cartographer Authors
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

#version 120

varying vec2 out_texture_coordinate;

uniform sampler2D u_texture;

void main()
{
  vec2 texture_color = texture2D(u_texture, out_texture_coordinate).rg;
  float opacity = texture_color.g;
  float value = texture_color.r;
  gl_FragColor = vec4(value, value, value, opacity);
}

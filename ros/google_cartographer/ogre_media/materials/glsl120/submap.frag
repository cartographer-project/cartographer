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

varying vec2 out_submap_texture_coordinate;

uniform sampler2D u_submap;
uniform float u_alpha;

void main()
{
  vec2 texture_value = texture2D(u_submap, out_submap_texture_coordinate).rg;
  float value = texture_value.r;
  float alpha = texture_value.g;
  float is_known = step(1e-3, alpha + value);
  gl_FragColor = vec4(u_alpha * value, u_alpha * is_known, 0., u_alpha * alpha);
}

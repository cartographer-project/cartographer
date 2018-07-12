.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

==============
Cost functions
==============

Relative Transform Error 2D
===========================

Given two poses
:math:`\mathbf{p}_i = [\mathbf{x}_i; \theta_i] = [x_i, y_i, \theta_i]^T`
and :math:`\mathbf{p}_j = [\mathbf{x}_j; \theta_j] = [x_j, y_j, \theta_j]^T`
the transformation :math:`\mathbf T` from the coordinate frame :math:`j` to the
coordinate frame :math:`i` has the following form

.. math::
 \mathbf{T}( \mathbf{p}_i,\mathbf{p}_j) =
 \left[
   \begin{array}{c}
        R(\theta_i)^T (\mathbf x_j - \mathbf x_i) \\
        \theta_j-\theta_i
   \end{array}
 \right]

where :math:`R(\theta_i)^T` is the rotation matrix of :math:`\theta_i`.

The weighted error :math:`f:\mathbb R^6 \mapsto \mathbb R^3` between
:math:`\mathbf T` and the measured transformation :math:`\mathbf T_{ij}^m =
[\mathbf x_{ij}^m; \theta_j^m]` from the coordinate frame :math:`j` to the
coordinate frame :math:`i` can be computed as

.. math::
 \mathbf f_{\text{relative}}( \mathbf{p}_i,\mathbf{p}_j) =
 \left[
   w_{\text{t}} \; w_{\text{r}}
 \right]
 \left(
   \mathbf T_{ij}^m - \mathbf T( \mathbf{p}_i,\mathbf{p}_j)
 \right) =
 \left[
   \begin{array}{c}
      w_{\text{t}}\left(
        \mathbf x_{ij}^m - R(\theta_i)^T (\mathbf x_j - \mathbf x_i)
      \right) \\
      w_{\text{r}}\left(
        \mathrm{clamp}(\theta_{ij}^m - (\theta_j-\theta_i))
      \right)
   \end{array}
 \right]

where :math:`w_t` and :math:`w_r` are weights for translation and rotation
respectively and :math:`\mathrm{clamp}: \mathbb R \mapsto [-\pi, \pi]`
normalizes the angle difference.

Jacobian matrix  :math:`J_f` is given by:

.. math::
 \begin{align}
   J_f( \mathbf{p}_i,\mathbf{p}_j) &=
   \left[
     \frac{\partial\mathbf f}{\partial x_i} \quad
     \frac{\partial\mathbf f}{\partial y_i} \quad
     \frac{\partial\mathbf f}{\partial \theta_i} \quad
     \frac{\partial\mathbf f}{\partial x_j} \quad
     \frac{\partial\mathbf f}{\partial y_j} \quad
     \frac{\partial\mathbf f}{\partial \theta_j}
   \right] \\
   &\mathstrut \\
   &=
   \left[
     \begin{array}{cccc}
         w_{\text{t}} R^T(\theta_i)
           & -w_{\text{t}} {\frac{\mathrm d R^T(\theta_i)}{\mathrm d \theta}}(\mathbf x_j - \mathbf x_i)
           & -w_{\text{t}} R^T(\theta_i)
           & \mathbf{0} \\
        \mathbf{0}^T
         & w_{\text{r}}
         & \mathbf{0}^T
         & -w_{\text{r}}
     \end{array}
   \right]
 \end{align}

Landmark Cost Function
======================

Let :math:`\mathbf{p}_o` denote the global pose of the SLAM tracking frame at
which a landmark with the global pose :math:`\mathbf{p}_l` is observed.
The landmark observation itself is the measured transformation
:math:`\mathbf{T}^m_{ol}` that was observed at time :math:`t_o`.

As the landmark can be observed asynchronously, the pose of observation 
:math:`\mathbf{p}_o` is modeled in between two regular, consecutive trajectory
nodes :math:`\mathbf{p}_i, \mathbf{p}_j`.
It is interpolated between :math:`\mathbf{p}_i` and
:math:`\mathbf{p}_j` at the observation time :math:`t_o` using a linear
interpolation for the translation and a quaternion SLERP for the rotation:

.. math::
  \mathbf{p}_o = \text{interpolate}(\mathbf{p}_i, \mathbf{p}_j, t_o)

Then, the full weighted landmark cost function can be written as:

.. math::
  \begin{align}
    \mathbf f_{\text{landmark}}(\mathbf{p}_l, \mathbf{p}_i, \mathbf{p}_j) &= 
      \mathbf f_{\text{relative}}(\mathbf{p}_l, \mathbf{p}_o) \\ 
    &= 
    \left[
      w_{\text{t}} \; w_{\text{r}}
    \right]
    \left(
      \mathbf T_{ol}^m - \mathbf T( \mathbf{p}_o,\mathbf{p}_l)
    \right)
  \end{align}

The translation and rotation weights :math:`w_{\text{t}}, w_{\text{r}}` are
part of the landmark observation data that is fed into Cartographer.

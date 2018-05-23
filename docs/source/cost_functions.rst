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
:math:`\mathbf{p_i} = [\mathbf{x_i}; \theta_i] = [x_i, y_i, \theta_i]^T`
and :math:`\mathbf{p_j} = [\mathbf{x_j}; \theta_j] = [x_j, y_j, \theta_j]^T`
the transformation :math:`\mathbf T` from the coordinate frame :math:`j` to the
coordinate frame :math:`i` has the following form

.. math::
 \mathbf{T}( \mathbf{p_i},\mathbf{p_j}) =
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
 \mathbf f( \mathbf{p_i},\mathbf{p_j}) =
 \left[
   w_{\text{t}} \; w_{\text{r}}
 \right]
 \left(
   \mathbf T_{ij}^m - \mathbf T( \mathbf{p_i},\mathbf{p_j})
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
   J_f( \mathbf{p_i},\mathbf{p_j}) &=
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

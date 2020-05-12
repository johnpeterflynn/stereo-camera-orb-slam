/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <sophus/se3.hpp>
#include "common_types.h"

// Implement exp for SO(3)
template <class T>
Eigen::Matrix<T, 3, 3> user_implemented_expmap(
    const Eigen::Matrix<T, 3, 1>& xi) {
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> res;

  T theta = T(xi.norm());

  // TODO: Need a more robust way of handling small theta. What to do it theta
  // is almost but not exactly 0?
  if (theta != T(0)) {
    Eigen::Matrix<T, 3, 1> a = xi / theta;
    Eigen::Matrix<T, 3, 3> a_hat;

    a_hat << T(0), -a(2), a(1), a(2), T(0), -a(0), -a(1), a(0), T(0);

    res = cos(theta) * I + (T(1) - cos(theta)) * a * a.transpose() +
          sin(theta) * a_hat;
  } else {
    res = I;
  }

  return res;
}

// Implement log for SO(3)
template <class T>
Eigen::Matrix<T, 3, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 3, 3>& mat) {
  T theta = acos((mat.trace() - T(1)) / T(2));

  Eigen::Matrix<T, 3, 1> a;
  a << mat(2, 1) - mat(1, 2), mat(0, 2) - mat(2, 0), mat(1, 0) - mat(0, 1);

  // TODO: Same here. What is a better way to handle small theta?
  if (theta != T(0)) {
    a *= T(1) / (T(2) * sin(theta));
  }

  Eigen::Matrix<T, 3, 1> res = theta * a;

  return res;
}

// Implement exp for SE(3)
template <class T>
Eigen::Matrix<T, 4, 4> user_implemented_expmap(
    const Eigen::Matrix<T, 6, 1>& xi) {
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 1> p(xi(0), xi(1), xi(2));
  Eigen::Matrix<T, 3, 1> phi(xi(3), xi(4), xi(5));

  Eigen::Matrix<T, 3, 3> J;
  T theta = T(phi.norm());

  Eigen::Matrix<T, 3, 3> R;  // = user_implemented_expmap(phi);

  // TODO: Same here. What is a better way to handle small theta?
  if (theta != T(0)) {
    Eigen::Matrix<T, 3, 1> a = phi / theta;
    Eigen::Matrix<T, 3, 3> a_hat;

    a_hat << T(0), -a(2), a(1), a(2), T(0), -a(0), -a(1), a(0), T(0);

    R = cos(theta) * I + (T(1) - cos(theta)) * a * a.transpose() +
        sin(theta) * a_hat;

    J = (sin(theta) / theta) * I +
        (1 - sin(theta) / theta) * a * a.transpose() +
        ((1 - cos(theta)) / theta) * a_hat;
  } else {
    R = I;
    J = I;
  }

  Eigen::Matrix<T, 4, 4> res = Eigen::Matrix<T, 4, 4>::Zero();

  res.block(0, 0, 3, 3) = R;
  res.block(0, 3, 3, 1) = J * p;
  res(3, 3) = T(1);

  return res;
}

// Implement log for SE(3)
template <class T>
Eigen::Matrix<T, 6, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 4, 4>& mat) {
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> R = mat.block(0, 0, 3, 3);
  Eigen::Matrix<T, 3, 1> t = mat.block(0, 3, 3, 1);
  Eigen::Matrix<T, 3, 3> J;
  Eigen::Matrix<T, 3, 1> phi = Eigen::Matrix<T, 3, 1>::Zero();

  T theta = acos((R.trace() - T(1)) / T(2));

  Eigen::Matrix<T, 3, 1> a;
  a << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);

  // TODO: Same here. What is a better way to handle small theta?
  if (theta != T(0)) {
    a *= T(1) / (T(2) * sin(theta));

    phi = theta * a;

    Eigen::Matrix<T, 3, 3> a_hat;
    a_hat << T(0), -a(2), a(1), a(2), T(0), -a(0), -a(1), a(0), T(0);

    J = (sin(theta) / theta) * I +
        (1 - sin(theta) / theta) * a * a.transpose() +
        ((1 - cos(theta)) / theta) * a_hat;
  } else {
    J = I;
  }

  Eigen::Matrix<T, 3, 1> rho = J.inverse() * t;

  Eigen::Matrix<T, 6, 1> res;
  res << rho, phi;

  return res;
}

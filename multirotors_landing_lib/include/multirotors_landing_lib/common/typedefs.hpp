/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Luca Bartolomei
 *********************************************************************/

#ifndef __TYPEDEFS_HPP__
#define __TYPEDEFS_HPP__

#include <eigen3/Eigen/Eigen>

namespace ml {

// Define the scalar type used.
using Scalar = float;  // numpy float32

// Define `Dynamic` matrix size.
static constexpr int Dynamic = Eigen::Dynamic;

// Using shorthand for `Matrix<ros, cols>` with scalar type.
template <int rows = Dynamic, int cols = Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
using Matrix4 = Matrix<4, 4>;

// Using shorthand for `Matrix<ros, cols>` with scalar type.
template <int rows = Dynamic, int cols = Dynamic>
using MatrixRowMajor = Eigen::Matrix<Scalar, rows, cols, Eigen::RowMajor>;

// Using shorthand for `Vector<row>` with scalar type.
template <int rows = Dynamic>
using Vector = Matrix<rows, 1>;
using Vector3 = Vector<3>;

// Vector bool
template <int rows = Dynamic>
using BoolVector = Eigen::Matrix<bool, -1, 1>;

// Using shorthand for `Array<rows, cols>` with scalar type.
template <int rows = Dynamic, int cols = rows>
using Array = Eigen::Array<Scalar, rows, cols>;

// Using `SparseMatrix` with type.
using SparseMatrix = Eigen::SparseMatrix<Scalar>;

// Using SparseTriplet with type.
using SparseTriplet = Eigen::Triplet<Scalar>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<Scalar>;

// Using `Ref` for modifier references.
template <class Derived>
using Ref = Eigen::Ref<Derived>;

// // Using `ConstRef` for constant references.
// template<class Derived>
// using ConstRef = const Eigen::Ref<const Derived>;

// Using `Map`.
template <class Derived>
using Map = Eigen::Map<Derived>;

static constexpr Scalar Gz = -9.81;
const Vector<3> GVEC{0.0, 0.0, Gz};

// Utility functions
template <typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x) {
  return ((x - x).array() == (x - x).array()).all();
}

template <typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x) {
  return ((x.array() != x.array())).all();
}

}  // namespace ml

#endif  // __TYPEDEFS_HPP__

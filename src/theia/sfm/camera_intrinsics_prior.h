// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_SFM_CAMERA_INTRINSICS_PRIOR_H_
#define THEIA_SFM_CAMERA_INTRINSICS_PRIOR_H_

#include <stdint.h>

#include "theia/sfm/camera/camera_intrinsics_model_type.h"

namespace theia {

// Weak calibration is not always available, so we need this helper struct to
// keep track of which data fields have been set.
template <int N>
class Prior {
 public:
  bool is_set = false;
  double value[N] = {0.0};

 private:
};

// Prior information about a View. This is typically gathered from EXIF or
// sensor data that provides weak calibration.
struct CameraIntrinsicsPrior {
 public:
  // The image size *should* always be set, so we don't have to worry about
  // making it an Prior type.
  int image_width = 0;
  int image_height = 0;

  // The camera intrinsics model type. Pinhole by default.
  std::string camera_intrinsics_model_type = "PINHOLE";

  // Camera intrinsics parameters.
  Prior<1> focal_length;
  Prior<2> principal_point;
  Prior<1> aspect_ratio;
  Prior<1> skew;
  // Up to 4 radial distortion parameters. For fisheye cameras, the fisheye
  // distortion parameters would be set as radial_distortion.
  Prior<4> radial_distortion;
  Prior<2> tangential_distortion;

  // Extrinsics that may be available from EXIF or elsewhere. Position is
  // specified as x,y,z and orientation is specified as the angle-axis rotation.
  Prior<3> position;
  Prior<3> orientation;

  // GPS priors. The altitude is measured as meters above sea level.
  Prior<1> latitude;
  Prior<1> longitude;
  Prior<1> altitude;

 private:
};

}  // namespace theia

#endif  // THEIA_SFM_CAMERA_INTRINSICS_PRIOR_H_

/*!
  \file        vo_5points.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/12/27

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */
#include "feature_tracker.h"
#include "5point/5point.h"
#include <iostream>

class VisualOdometry5Points : public FeatureTracker {
public:
  typedef cv::Point3d Pt3d;

  VisualOdometry5Points() {
    K = (cv::Mat1d(3, 3) << 6.5282368244472104e+02, 0., 2.8512945251750091e+02, 0.,
         6.5831840521080176e+02, 2.3929204792676040e+02, 0., 0., 1.);
    Kinv = K.inv();
    K11 = Kinv(0, 0); K12 = Kinv(0, 1); K13 = Kinv(0, 2); // row, col
    K21 = Kinv(1, 0); K22 = Kinv(1, 1); K23 = Kinv(1, 2);
    K31 = Kinv(2, 0); K32 = Kinv(2, 1); K33 = Kinv(2, 2);
    printf("%g\t%g\t%g\n%g\t%g\t%g\n%g\t%g\t%g\n", K11, K12, K13, K21, K22, K23, K31, K32, K33);
    _cam_pos_homo = (cv::Mat1d(4, 1) << 0, 0, 0, 1);
  }

  // handle the currently tracked points
  void handleTrackedPoints(cv:: Mat &frame,
                           cv:: Mat &output) {
    FeatureTracker::handleTrackedPoints(frame, output);

    // K, the 3×3 intrinsic camera matrix
    //ptn1 = inv(K)*pt1
    //ptn2 = inv(K)*pt2

    // 3. try and determine the transform [R, t]
    // old_features actually contains the features found in the previous frame,
    // new_features in the current frame and so are less numerous
    unsigned int npts = new_features.size(), arr_size = npts * 2;
    if (old_features.size() != npts) {
      printf("Solve5PointEssential() inconsistent data!\n", npts, old_features.size());
      return;
    }
    if (npts < 5) {
      printf("Solve5PointEssential() returned an error!\n");
      return;
    }
    _pts0.resize(arr_size);
    _pts1.resize(arr_size);
    for (unsigned int i = 0; i < npts; ++i) {
      normalize_pt(old_features[i].x, old_features[i].y, _pts0[2*i], _pts0[2*i+1]);
      normalize_pt(new_features[i].x, new_features[i].y, _pts1[2*i], _pts1[2*i+1]);
      // printf("(%g, %g) -> (%g, %g)\n", old_features[i].x, old_features[i].y, _pts0[2*i], _pts0[2*i+1]);
    } // end for i
    std::vector <cv::Mat> E, P; // essential matrix, 3x4 projection matrix
    std::vector <int> inliers;
    if (!Solve5PointEssential(_pts0.data(), _pts1.data(), npts, E, P, inliers)) {
      printf("Solve5PointEssential() returned an error!\n");
      return;
    } // end if (ret)
    printf("Solve5PointEssential() found %i solutions, first sol:%g\%% inliers of %i pts\n",
           E.size(), 100. * inliers[0] / npts, npts);
    std::cout << "E:" << E[0] << std::endl;
    std::cout << "P:" << P[0] << std::endl;

    // 4. extract R and t
    cv::Mat1d R = P[0](cv::Range(0,3), cv::Range(0,3)), t = P[0].col(3);
    if (cv::determinant(R) < 0) { // reflection
      R = R * -1;
      t = t * -1;
    }

    // 5. compute relative scale
    double scale = 1;
//    cv::Mat Pinv = P[0].inv();
//    int i = 0, j = 1;
//    scale = dist3(pixel2world(_pts0[2*i], _pts0[2*i+1], Pinv),
//        pixel2world(_pts0[2*j], _pts0[2*j+1], Pinv))
//        /
//        dist3(pixel2world(_pts1[2*i], _pts1[2*i+1], Pinv),
//                pixel2world(_pts1[2*j], _pts1[2*j+1], Pinv));
//    printf("scale:%g\n", scale);

    // 5bis. rescale t
    t = t * scale;

    // 4bis. form T_k = [ R    t]
    //                  [ 0    1]
    cv::Mat1d T(4, 4);
    T.setTo(0);
    T(3, 3) = 1;
    R.copyTo(T(cv::Range(0,3), cv::Range(0,3)));
    t.copyTo(T(cv::Range(0,3), cv::Range(3,4)));
    printf("T:%ix%i\n", T.cols, T.rows);
    std::cout << "T:" << T << std::endl;
    printf("_cam_pos_homo:%ix%i\n", _cam_pos_homo.cols, _cam_pos_homo.rows);
    std::cout << "_cam_pos_homo:" << _cam_pos_homo << std::endl;

    // 6. concatenate transformation by computing C_k = C_k-1 * T_k
    _cam_pos_homo = T * _cam_pos_homo;
    _cam_pos.x = _cam_pos_homo(0, 0) / _cam_pos_homo(3, 0);
    _cam_pos.y = _cam_pos_homo(1, 0) / _cam_pos_homo(3, 0);
    _cam_pos.z = _cam_pos_homo(2, 0) / _cam_pos_homo(3, 0);
    printf("cam_pos:(%g,%g,%g)\n", _cam_pos.x, _cam_pos.y, _cam_pos.z);
  } // end handleTrackedPoints();

  // determine which tracked point should be accepted
  virtual bool acceptTrackedPoint(int i) {
    return status[i]; // accept non moving points
  }
protected:

  // P: 3 rows × 4 columns projection matrix transforms a 3D point
  // so that it’s relative to the camera’s view
  // Pinv: 4 rows * 3 columns
  template<class T>
  inline Pt3d pixel2world(const T & xn, const T & yn,
                          const cv::Mat1f & Pinv) { // 4 rows, 3 columns
    cv::Mat1f pt3d_mat = Pinv * (cv::Mat3d (3, 1) << xn, yn, 1); // rows, cols
    double zinv = 1. * pt3d_mat(3,0);
    return Pt3d(pt3d_mat(0,0) * zinv, pt3d_mat(1,0) * zinv, pt3d_mat(2,0) * zinv);
  }

  template<class Pt3>
  inline double dist3(const Pt3 & A, const Pt3 & B) {
    return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
  }

  //! normalize a 2D point
  template<class T>
  inline void normalize_pt(const T & x, const T & y, double & xn, double & yn) {
    double z_inv = 1. / (K31 *x + K32 * y + K33);
    xn = (K11 *x + K12 * y + K13) * z_inv;
    yn = (K21 *x + K22 * y + K23) * z_inv;
  }

  cv::Mat1d _cam_pos_homo;
  Pt3d _cam_pos;
  std::vector<double> _pts0, _pts1;
  cv::Mat1d K, Kinv;
  double K11, K12, K13, K21, K22, K23, K31, K32, K33;
}; // end class VisualOdometry5Points

////////////////////////////////////////////////////////////////////////////////

int main() {
  // Create feature tracker instance
  VisualOdometry5Points tracker;
  cv::VideoCapture src (0);
  cv::Mat frame, output;
  while (true) {
    src >> frame;
    if (frame.empty()) {
      printf("capture function failed.\n");
      exit(-1);
    }
    tracker.process(frame, output);
    cv::imshow("output", output);
    cv::waitKey(25);
  }
} // end main()

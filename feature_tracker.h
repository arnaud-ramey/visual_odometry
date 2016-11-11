/*!
  \file        feature_tracker.h
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

5-point algorithm:
http://www.vis.uky.edy/~stewe/FIVEPOINT
http://nghiaho.com/?p=1675

8-point
  function F = sevenp(x1,x2)
  Q1 = x1';
  Q2 = x2';
  Q = [Q1(:,1).*Q2(:,1) , ...
       Q1(:,2).*Q2(:,1) , ...
       Q1(:,3).*Q2(:,1) , ...
       Q1(:,1).*Q2(:,2) , ...
       Q1(:,2).*Q2(:,2) , ...
       Q1(:,3).*Q2(:,2) , ...
       Q1(:,1).*Q2(:,3) , ...
       Q1(:,2).*Q2(:,3) , ...
       Q1(:,3).*Q2(:,3) ] ;
  %EE = null( Q);
  [U,S,V] = svd(Q,0);
  F = V(:,9);
  F = F/norm(F);
 */
#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdio.h>

class FeatureTracker {
protected:
  cv::Mat gray;  // current gray-level image
  cv::Mat gray_prev; // previous gray-level image
  std::vector<cv::Point2f> old_features, new_features; // tracked features from 0->1
  std::vector<cv::Point2f> initial; // initial position of tracked points
  std::vector<cv::Point2f> features; // detected features
  int max_count; // maximum number of features to detect
  double qlevel; // quality level for feature detection
  double minDist; // min distance between two points
  std::vector<uchar> status; // status of tracked features
  std::vector<float> err; // error in tracking
public:
  FeatureTracker() : max_count(100),
    qlevel(0.01), minDist(10.) {}

  void process(cv:: Mat &frame, cv:: Mat &output) {
    // convert to gray-level image
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);
    // 1. if new feature points must be added
    if(addNewPoints())
    {
      // detect feature points
      detectFeaturePoints();
      // add the detected features to
      // the currently tracked features
      old_features.insert(old_features.end(),
                          features.begin(),features.end());
      initial.insert(initial.end(),
                     features.begin(),features.end());
    }
    // for first image of the sequence
    if(gray_prev.empty())
      gray.copyTo(gray_prev);
    // 2. track features
    cv::calcOpticalFlowPyrLK(
          gray_prev, gray, // 2 consecutive images
          old_features, // input point positions in first image
          new_features, // output point positions in the 2nd image
          status,
          // tracking success
          err);
    // tracking error
    // 2. loop over the tracked points to reject some
    int k=0;
    for( int i= 0; i < new_features.size(); i++ ) {
      if (!acceptTrackedPoint(i)) // do we keep this point?
        continue;
      // keep this point in vector
      initial[k]= initial[i];
      new_features[k] = new_features[i];
      old_features[k] = old_features[i];
      ++k;
    }
    // eliminate unsuccesful points
    new_features.resize(k);
    old_features.resize(k);
    initial.resize(k);
    // 3. handle the accepted tracked points
    handleTrackedPoints(frame, output);
    // 4. current points and image become previous ones
    std::swap(new_features, old_features);
    cv::swap(gray_prev, gray);
  } // end process

  // feature point detection
  void detectFeaturePoints() {
    // detect the features
    cv::goodFeaturesToTrack(gray, // the image
                            features,
                            // the output detected features
                            max_count, // the maximum number of features
                            qlevel,
                            // quality level
                            minDist);
    // min distance between two features
  }

  // determine if new points should be added
  bool addNewPoints() {
    // if too few points
    return old_features.size()<=10;
  }

  // determine which tracked point should be accepted
  virtual bool acceptTrackedPoint(int i) {
    return status[i] &&
        // if point has moved
        (abs(old_features[i].x-new_features[i].x)+
         (abs(old_features[i].y-new_features[i].y))>2);
  }

  // handle the currently tracked points
  virtual void handleTrackedPoints(cv:: Mat &frame,
                           cv:: Mat &output) {
    // draw
    cv::Scalar color = CV_RGB(0, 255, 0);
    // for all tracked points
    for(int i= 0; i < new_features.size(); i++ ) {
      // draw line and circle
      cv::line(output, old_features[i], // initial position
               new_features[i],// new position
               CV_RGB(255, 0, 0));
      cv::line(output, initial[i], // initial position
               new_features[i],// new position
               color);
      cv::circle(output, new_features[i], 3, color,-1);
    }
  } // end handleTrackedPoints();
}; // end class FeatureTracker

#endif // FEATURE_TRACKER_H

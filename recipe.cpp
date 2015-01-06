/*!
  \file        recipe.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/12/24

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
 */
#include "feature_tracker.h"
int main() {
  // Create feature tracker instance
  FeatureTracker tracker;
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

/*
This my own implementation of the 5 point algorithm from the paper
H. Li and R. Hartley, "Five-point motion estimation made easy", icpr, 2006
I do not use anything from their GPL code.
*/

#include <vector>
#include <iostream>
#include <assert.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hpp>

#include "5point.h"

using namespace std;

int main()
{
    // x,y pairing
    double pts1[] = {0.4964 ,1.0577,
                    0.3650, -0.0919,
                    -0.5412, 0.0159,
                    -0.5239, 0.9467,
                    0.3467, 0.5301,
                    0.2797, 0.0012,
                    -0.1986, 0.0460,
                    -0.1622, 0.5347,
                    0.0796, 0.2379,
                    -0.3946, 0.7969};

    double pts2[] = {0.7570, 2.7340,
                    0.3961, 0.6981,
                    -0.6014, 0.7110,
                    -0.7385, 2.2712,
                    0.4177, 1.2132,
                    0.3052, 0.4835,
                    -0.2171, 0.5057,
                    -0.2059, 1.1583,
                    0.0946, 0.7013,
                    -0.6236, 3.0253};

    int num_pts = 10;

    vector <cv::Mat> E; // essential matrix
    vector <cv::Mat> P; // 3x4 projection matrix
    vector <int> inliers;
    bool ret;

    // Do some timing test
    double start = cv::getTickCount();

    for(int i=0; i < 1000; i++) {
        ret = Solve5PointEssential(pts1, pts2, num_pts, E, P, inliers);
    }

    double end = cv::getTickCount();
    double elapse = (end - start)/cv::getTickFrequency();

    double avg_time_us = (elapse/1000)*1000000;

    cout << "Average execution time: " << avg_time_us << " us" << endl;
    cout << endl;

    if(ret) {
        cout << "Success! Found " <<  E.size() << " possible solutions" << endl;
        cout << "The best one has the highest inliers. An inlier is a point that is in front of both cameras." << endl;
        cout << endl;

        for(size_t i=0; i < E.size(); i++) {
            cout << "Solution " << (i+1) << "/" << E.size() << endl;
            cout << endl;
            cout << "E = " << E[i] << endl;
            cout << endl;

            if(cv::determinant(P[i](cv::Range(0,3), cv::Range(0,3))) < 0) {
                cout << "Detected a reflection in P. Multiplying all values by -1." << endl;
                cout << "P = " << (P[i] * -1) << endl;
            }
            else {
                cout << "P = " << P[i] << endl;
            }

            cout << endl;
            cout << "inliers = " << inliers[i] << "/" << num_pts << endl;
            cout << "=========================================" << endl;
            cout << endl;
        }
    }
    else {
        cout << "Could not find a valid essential matrix" << endl;
    }

    return 0;
}

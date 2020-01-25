/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    vector<PerformanceMeasures> perfMeas;

    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    vector<string> detectors = {"SHITOMASI","HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptors = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    for(const string& det : detectors)
    {
        for(const string& des : descriptors)
        {


            // misc
            const int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
            RingBuffer<DataFrame,dataBufferSize> frameBuffer;

            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                if(det != "AKAZE" && des == "AKAZE")
                {
                    PerformanceMeasures newMeas;
                    newMeas.detType = det;
                    newMeas.desType = des;
                    newMeas.FrameNum =imgIndex;
                    perfMeas.push_back(newMeas);
                    continue;
                }
                if(det == "SIFT" && des == "ORB")
                {
                    PerformanceMeasures newMeas;
                    newMeas.detType = det;
                    newMeas.desType = des;
                    newMeas.FrameNum =imgIndex;
                    perfMeas.push_back(newMeas);
                    continue;
                }


                PerformanceMeasures newMeas;
                newMeas.detType = det;
                newMeas.desType = des;
                newMeas.FrameNum =imgIndex;

                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath;
                imgFullFilename += imgPrefix;
                imgFullFilename += imgNumber.str();
                imgFullFilename += imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                frameBuffer.add(frame);

                //// EOF STUDENT ASSIGNMENT
                //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                string detectorType = det;

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

                if (detectorType == "SHITOMASI")
                {
                    newMeas.detTime = detKeypointsShiTomasi(keypoints, imgGray, false);
                }
                else if (detectorType == "HARRIS")
                {
                    newMeas.detTime = detKeypointsHarris(keypoints, imgGray, false);
                }
                else if (detectorType == "FAST")
                {
                    newMeas.detTime = detKeypointsFAST(keypoints, imgGray, false);
                }
                else if (detectorType == "BRISK")
                {
                    newMeas.detTime = detKeypointsBRISK(keypoints, imgGray, false);
                }
                else if (detectorType == "ORB")
                {
                    newMeas.detTime = detKeypointsORB(keypoints, imgGray, false);
                }
                else if (detectorType == "AKAZE")
                {
                    newMeas.detTime = detKeypointsAKAZE(keypoints, imgGray);
                }
                else if (detectorType == "SIFT")
                {
                    newMeas.detTime = detKeypointsSIFT(keypoints, imgGray, false);
                }
                else
                {
                    //...
                }
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                newMeas.KeyPointPerFrame = keypoints.size();

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    auto kp = keypoints.begin();
                    while( kp != keypoints.end())
                    {
                        if(kp->pt.x < vehicleRect.x || kp->pt.x > vehicleRect.x+vehicleRect.width
                           || kp->pt.y < vehicleRect.y || kp->pt.y > vehicleRect.y + vehicleRect.height)
                        {
                            kp = keypoints.erase(kp);
                        }
                        else
                        {
                            ++kp;
                        }
                    }

                }
                newMeas.KeyPointPerROI = keypoints.size();
                cout << "KeyPoints in ROI: " << keypoints.size() << endl;

                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType == "SHITOMASI")
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                frameBuffer.getLatest()->keypoints = keypoints;
                //cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                string descriptor = des; //BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
                newMeas.desTime = descKeypoints(frameBuffer.getLatest()->keypoints, frameBuffer.getLatest()->cameraImg, descriptors, descriptor);
                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                frameBuffer.getLatest()->descriptors = descriptors;

                //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
                if (frameBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                    newMeas.matchingTime = matchDescriptors(frameBuffer.getSecondLatest()->keypoints, frameBuffer.getLatest()->keypoints,
                                     frameBuffer.getSecondLatest()->descriptors, frameBuffer.getLatest()->descriptors,
                                     matches, descriptorType, matcherType, selectorType, descriptor);

                    //// EOF STUDENT ASSIGNMENT
                    newMeas.matchedPoints = matches.size();
                    // store matches in current data frame
                    frameBuffer.getLatest()->kptMatches = matches;

                    //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = (frameBuffer.getLatest()->cameraImg).clone();
                        cv::drawMatches(frameBuffer.getSecondLatest()->cameraImg, frameBuffer.getSecondLatest()->keypoints,
                                        frameBuffer.getLatest()->cameraImg, frameBuffer.getLatest()->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                        rectangle(matchImg,vehicleRect, cv::Scalar(255,0,0),1,8,0);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        //cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
                perfMeas.push_back(newMeas);
            } // eof loop over all images

        }
    }
    std::ofstream myfile;
    myfile.open ("example.csv");
    myfile << "This is the first cell in the first column.\n";
    for(const PerformanceMeasures& perfM : perfMeas)
    {
        myfile << perfM.detType << "," << perfM.desType << "," << perfM.FrameNum << "," << perfM.KeyPointPerFrame << "," << perfM.KeyPointPerROI << "," << perfM.detTime << "," << perfM.desTime << "," << perfM.matchedPoints << "," << perfM.matchingTime << ",\n";
    }
    myfile.close();

    return 0;
}

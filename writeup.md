# Mid Term Writeup

## MP.1 Data Buffer Optimization
Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

Solution:
Solved by adding a vector based RingBuffer data structure using templates. Details can be found in dataStructures.h.

## MP.2 Keypoint Detection
Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

Solution:
Given structures have been extended by adding the given detectors using standard parameters.

## MP.3 Keypoint Removal
Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

Solution:
Iterating through the keypoints while checking if they are outside the given rectangle coordinates. If they are outside the given boundaries, they are getting removed from the vector. This saves memory by avoiding storing duplicates in an additional variable.

## MP.4 Keypoint Descriptors
Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

Solution:
Given structures have been extended by adding the given descriptors.

## MP.5 Descriptor Matching
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

Solution:
Extended given structure by FLANN and k-nearest neighbor selection, while taking care of the type issue in case of FLANN.

## MP.6 Descriptor Distance Ratio
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.


## MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

## MP.8 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

## MP.9 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

#Performance Evaluation

* 	Detector Type, Descriptor Type, Frame Number, Keypoints in Frame, Keypoints in ROI, detection time, Description time, Matched keypoints, Matching time

## SHITOMASI & BRISK
* 	Detector Type	 Descriptor Type	 Frame Number	 Keypoints in Frame	 Keypoints in ROI	 detection time	 Description time	 Matched keypoints	 Matching time
* 	SHITOMASI	BRISK	0	1370	127	504.066	143.716	0	0.000
* 	SHITOMASI	BRISK	1	1301	120	183.366	541.225	97	171.838
* 	SHITOMASI	BRISK	2	1361	123	21.356	309.821	88	105.028
* 	SHITOMASI	BRISK	3	1358	120	18.885	296.556	80	0.312281
* 	SHITOMASI	BRISK	4	1333	120	18.794	579.785	90	0.530778
* 	SHITOMASI	BRISK	5	1284	115	190.422	308.962	82	0.569074
* 	SHITOMASI	BRISK	6	1322	114	18.116	297.767	79	0.432803
* 	SHITOMASI	BRISK	7	1366	125	191.532	645.593	86	0.596328
* 	SHITOMASI	BRISK	8	1389	112	20.161	286.036	86	0.309264
* 	SHITOMASI	BRISK	9	1339	113	44.061	289.269	83	0.541688
* 				1342,3	118,9	121.076	369.873	77,1	27.687

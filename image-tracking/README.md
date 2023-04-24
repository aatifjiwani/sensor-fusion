# 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## Basic Build Instructions
```bash
mkdir build && cd build
cmake .. && make
./2D_feature_tracking
```

## Implementation Overview

### Data Buffer Optimization
In order to build a data buffer that does not exceed a limit _and_ does not incur a large cost, we resorted to using the C++ STL `deque`. 
A `deque` is essentially a two-sided vector that allows us to insert and remove from either end of the list with constant runtime. 
When adding new elements to the buffer, add on the right/back. When the buffer is at the size limit, remove from the left/front. 

### Keypoint Detection
In `matching2D_Student.cpp`, we implemented the following Keypoint detectors using OpenCV: HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT. 
All the detectors can be used using their respective API: `detKeypoints<TYPE>`. 

#### Keypoint Filtering
In order to focus only on the preceding vehicle, we filter the keypoints produced by the above OpenCV `FeatureDetector`s. All we need to do
is check if each keypoint is contained within a bounding box over the preceding vehicle, and only keep the ones that are. 

### Describing Keypoints
In `matching2D_Student.cpp`, we implement an API `descKeypoints` that uses OpenCV to use the following feature descriptors: BRIEF, ORB, FREAK, AKAZE, and SIFT. 
All descriptors can be used by specifying the `descriptorType` argument. 

### Descriptor Matching
Once we have keypoints and their descriptions for every frame, we need to match the keypoints between each pair of sequential frames. To do this, we use OpenCV's 
implementation of Brute-Force matching and FLANN, a fast approximation of nearest neighbors. We implement these matchers in `matchDescriptors`

#### Descriptor Distance Ratio
In order to find the best matches and keypoints that aren't worth using, we can use K-nearest neighbors and perform a distance ratio test. For K=2, if the ratio 
of the distances between the best and second-best match is within a threshold, then keep it. 

## Performance Evaluations
1. Number of Keypoints detected on preceding vehicle for each detector

| Detector | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | Avg. Neighborhood |
| :---    | :---  | :---  | :---  |  :--- | :---  | :---  | :---  | :---  | :---  | :---  | :---: |
| SHI-TOMASI | 128 | 120 | 106 | 118 | 125 | 131 | 130 | 131 | 140 | 142 | 4 |
| HARRIS | 50 | 54 | 53 | 55 | 56 | 58 | 57 | 61 | 59 | 57 | 4 |
| FAST | 149 | 152 | 150 | 155 | 149 | 149 | 156 | 150 | 138 | 143 | 7 |
| BRISK | 124 | 121 | 120 | 129 | 121 | 123 | 125 | 118 | 122 | 105 | 19 |
| ORB | 57 | 64 | 62 | 62 | 65 | 74 | 62 | 66 | 65 | 62 | 31 |
| AKAZE | 114 | 110 | 113 | 106 | 108 | 110 | 113 | 113 | 119 | 114 | 7 |
| SIFT | 128 | 120 | 106 | 118 | 125 | 131 | 130 | 131 | 140 | 142 | 6 |


2. Number of Matched Keypoints for all 10 images

| Detector / Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| ---                 | :---  | :---  |:--- |:---   |:---   |:---  |
| SHI-TOMASI           | 767   | 944   | 907 | 766   | n/a     | 927  |
| HARRIS              | 393   | 460   | 449 | 403   | n/a     | 459  |
| FAST                | 899   | 1099   | 1081 | 881   | n/a     | 1046  |
| BRISK               | 734  | 757  | 711| 774  | n/a     | 778 |
| ORB                 | 464   | 481   | 425 | 445   | n/a     | 453  |
| AKAZE               | 809  | 903  | 816| 801  | 812  |815  |
| SIFT                | 612   | 721   | n/a   | 620   | n/a     |775   |

3. Time it took to detect and describe keypoints, average for all 10 images. All numbers are in milliseconds. 

| Detector / Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| ---                 | :---  | :---  |:--- |:---   |:---   |:---  |
| SHI-TOMASI           | 34.31   | 13.58   | 12.45 | 32.17   | n/a     | 25.52  |
| HARRIS              | 34.18   | 11.40   | 11.98 | 32.28   | n/a     | 25.71  |
| FAST                | 19.83   | 1.72   | 2.09 | 25.46   | n/a     | 16.04  |
| BRISK               | 50.29  | 32.52  | 36.44| 58.33  | n/a     | 49.47 |
| ORB                 | 34.89   | 16.42   | 16.37 | 40.10   | n/a     | 35.82  |
| AKAZE               | 62.44  | 31.86  | 38.89| 57.17  | 89.73  |55.16  |
| SIFT                | 95.18   | 71.15   | n/a   | 100.58   | n/a     |132.58   |

In terms of time and number of matched keypoints, the Top 3 Detector/Descriptor pairs are:
1. FAST/BRIEF with a runtime of 1.72 ms per image and a total of 1099 matched features
2. FAST/ORB with a runtime of 2.09 ms per image and a total of 1081 matched features
3. SHI-TOMASI/ORB with a runtime of 12.45 ms per image and a total of 907 matched features. 

It seems based on these that the FAST detector and the ORB descriptor are generally the best. 
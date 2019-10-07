# 3D-Object-Tracking

## 1. Match 3D objects
*Lines 224-284 in camFusion_Student.cpp*
As suggested, I used a `std::multimap<int,int>` to track pairs of bounding box IDs. I then counted the keypoint correspondences per box pair to determine the best matches between frames.

// Count the greatest number of matches in the multimap, where each element is {key=currBoxID, val=prevBoxID}
## 2. Compute lidar-based TTC
*Lines 202-221 in camFusion_Student.cpp*
In each frame, I took the median x-distance to reduce the impact of outlier lidar points on my TTC estimate. With the constant velocity model, the key equation is as follows.

TTC = d1 * (1.0 / frameRate) / (d0 - d1);
Lines 192-199 in camFusion_Student.cpp
To calculate the median, I built a helper function to sort the vector of lidar points.

void sortLidarPointsX(std::vector<LidarPoint> &lidarPoints)
{
    // This std::sort with a lambda mutates lidarPoints, a vector of LidarPoint
    std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;  // Sort ascending on the x coordinate only
    });
}
                         
## 3. Associate keypoint matches with bounding boxes
*Lines 133-142 in camFusion_Student.cpp*
This function is called for each bounding box, and it loops through every matched keypoint pair in an image. If the keypoint falls within the bounding box region-of-interest (ROI) in the current frame, the keypoint match is associated with the current BoundingBox data structure.

## 4. Compute mono camera-based TTC
*Lines 145-189 in camFusion_Student.cpp*
The code for this function computeTTCCamera draws heavily on the example provided in an earlier lesson. It uses distance ratios on keypoints matched between frames to determine the rate of scale change within an image. This rate of scale change can be used to estimate the TTC.

TTC = (-1.0 / frameRate) / (1 - medianDistRatio);
Like the lidar TTC estimation, this function uses the median distance ratio to avoid the impact of outliers. Unfortunately this approach is still vulnerable to wild miscalculations (-inf, NaN, etc.) if there are too many mismatched keypoints. Note also that this algorithm calculates the Euclidean distance for every paired combination of keypoints within the bounding box, O(n^2) on the number of keypoints.

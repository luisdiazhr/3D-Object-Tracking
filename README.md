# 3D-Object-Tracking

## 1. Match 3D objects
*Lines 224-284 in camFusion_Student.cpp*

As suggested, I used a `std::multimap<int,int>` to track pairs of bounding box IDs. I then counted the keypoint correspondences per box pair to determine the best matches between frames. Count the greatest number of matches in the multimap, where each element is `{key=currBoxID, val=prevBoxID}`

## 2. Compute lidar-based TTC
*Lines 202-221 in camFusion_Student.cpp*

In each frame, I took the median x-distance to reduce the impact of outlier lidar points on my TTC estimate. With the constant velocity model, the key equation is as follows.
```
TTC = d1 * (1.0 / frameRate) / (d0 - d1);
```
*Lines 192-199 in camFusion_Student.cpp*

To calculate the median, I built a helper function to sort the vector of lidar points.
```
void sortLidarPointsX(std::vector<LidarPoint> &lidarPoints)
{
    // This std::sort with a lambda mutates lidarPoints, a vector of LidarPoint
    std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;  // Sort ascending on the x coordinate only
    });
}
```                      
## 3. Associate keypoint matches with bounding boxes
*Lines 133-142 in camFusion_Student.cpp*

This function is called for each bounding box, and it loops through every matched keypoint pair in an image. If the keypoint falls within the bounding box region-of-interest (ROI) in the current frame, the keypoint match is associated with the current BoundingBox data structure.

## 4. Compute mono camera-based TTC
*Lines 145-189 in camFusion_Student.cpp*

The code for this function computeTTCCamera draws heavily on the example provided in an earlier lesson. It uses distance ratios on keypoints matched between frames to determine the rate of scale change within an image. This rate of scale change can be used to estimate the TTC.

`TTC = (-1.0 / frameRate) / (1 - medianDistRatio);`
Like the lidar TTC estimation, this function uses the median distance ratio to avoid the impact of outliers. Unfortunately this approach is still vulnerable to wild miscalculations (-inf, NaN, etc.) if there are too many mismatched keypoints. Note also that this algorithm calculates the Euclidean distance for every paired combination of keypoints within the bounding box, O(n^2) on the number of keypoints.

## 5. 

## 6. Performance Evaluation
Different combinations of detector-descriptor are tested. Results are stored in the file `build/Performance.csv`. One of the most accurate combinations is the AKAZE-AKAZE combination. The difference between the TTCLidar and TTCCamera in average is 1.15 seconds which shows a pretty accurate model. The next table shows results for other combinations.

| Detector Type | Descriptor Type | Frame index | TTC Lidar | TTC Camera | Difference |
|---------------|-----------------|-------------|-----------|------------|------------|
| SIFT          | BRIEF           | 1           | 12.5156   | 12.0794    | 0.43623    |
| SIFT          | BRIEF           | 2           | 12.6142   | 14.2363    | 1.62204    |
| SIFT          | BRIEF           | 3           | 14.091    | 15.0722    | 0.981234   |
| SIFT          | BRIEF           | 4           | 16.6894   | 21.3098    | 4.62038    |
| SIFT          | BRIEF           | 5           | 15.9082   | 13.6004    | 2.30785    |
| SIFT          | BRIEF           | 6           | 12.6787   | 12.5451    | 0.133622   |
| SIFT          | BRIEF           | 7           | 11.9844   | 14.5792    | 2.59484    |
| SIFT          | BRIEF           | 8           | 13.1241   | 15.2578    | 2.1337     |
| SIFT          | BRIEF           | 9           | 13.0241   | 13.0446    | 0.020501   |
| SIFT          | BRIEF           | 10          | 11.1746   | 10.3097    | 0.8649     |
| SIFT          | BRIEF           | 11          | 12.8086   | 13.0488    | 0.240237   |
| SIFT          | BRIEF           | 12          | 8.95978   | 9.86362    | 0.903838   |
| SIFT          | BRIEF           | 13          | 9.96439   | 10.1282    | 0.163821   |
| SIFT          | BRIEF           | 14          | 9.59863   | 9.26825    | 0.330384   |
| SIFT          | BRIEF           | 15          | 8.57352   | 9.99448    | 1.42095    |
| SIFT          | BRIEF           | 16          | 9.51617   | 9.06875    | 0.447425   |
| SIFT          | BRIEF           | 17          | 9.54658   | 8.94616    | 0.600421   |
| SIFT          | BRIEF           | 18          | 8.3988    | 9.23255    | 0.833749   |
| SIFT          | BRISK           | 1           | 12.5156   | 11.6775    | 0.838124   |
| SIFT          | BRISK           | 2           | 12.6142   | 13.3302    | 0.715973   |
| SIFT          | BRISK           | 3           | 14.091    | 14.4771    | 0.386061   |
| SIFT          | BRISK           | 4           | 16.6894   | 18.9432    | 2.25386    |
| SIFT          | BRISK           | 5           | 15.9082   | 14.9067    | 1.00153    |
| SIFT          | BRISK           | 6           | 12.6787   | 11.5499    | 1.12881    |
| SIFT          | BRISK           | 7           | 11.9844   | 14.3801    | 2.3958     |
| SIFT          | BRISK           | 8           | 13.1241   | 15.5638    | 2.43968    |
| SIFT          | BRISK           | 9           | 13.0241   | 13.0591    | 0.0349901  |
| SIFT          | BRISK           | 10          | 11.1746   | 10.8439    | 0.330747   |
| SIFT          | BRISK           | 11          | 12.8086   | 13.2349    | 0.426301   |
| SIFT          | BRISK           | 12          | 8.95978   | 10.296     | 1.33619    |
| SIFT          | BRISK           | 13          | 9.96439   | 9.98874    | 0.0243465  |
| SIFT          | BRISK           | 14          | 9.59863   | 9.55221    | 0.0464218  |
| SIFT          | BRISK           | 15          | 8.57352   | 9.4644     | 0.890871   |
| SIFT          | BRISK           | 16          | 9.51617   | 8.77179    | 0.744376   |
| SIFT          | BRISK           | 17          | 9.54658   | 9.52357    | 0.0230142  |
| SIFT          | BRISK           | 18          | 8.3988    | 9.3265     | 0.927693   |
| SIFT          | FREAK           | 1           | 12.5156   | 11.3069    | 1.20868    |
| SIFT          | FREAK           | 2           | 12.6142   | 13.8353    | 1.22108    |
| SIFT          | FREAK           | 3           | 14.091    | 13.6135    | 0.477498   |
| SIFT          | FREAK           | 4           | 16.6894   | 21.1569    | 4.46748    |
| SIFT          | FREAK           | 5           | 15.9082   | 13.5634    | 2.34484    |
| SIFT          | FREAK           | 6           | 12.6787   | 12.2423    | 0.436415   |
| SIFT          | FREAK           | 7           | 11.9844   | 14.8139    | 2.82958    |
| SIFT          | FREAK           | 8           | 13.1241   | 15.2346    | 2.11046    |
| SIFT          | FREAK           | 9           | 13.0241   | 14.3892    | 1.36506    |
| SIFT          | FREAK           | 10          | 11.1746   | 11.5716    | 0.397001   |
| SIFT          | FREAK           | 11          | 12.8086   | 12.06      | 0.748574   |
| SIFT          | FREAK           | 12          | 8.95978   | 10.0932    | 1.13341    |
| SIFT          | FREAK           | 13          | 9.96439   | 10.2522    | 0.287834   |
| SIFT          | FREAK           | 14          | 9.59863   | 9.89631    | 0.297681   |
| SIFT          | FREAK           | 15          | 8.57352   | 9.19662    | 0.623095   |
| SIFT          | FREAK           | 16          | 9.51617   | 9.49771    | 0.0184577  |
| SIFT          | FREAK           | 17          | 9.54658   | 9.18097    | 0.365609   |
| SIFT          | FREAK           | 18          | 8.3988    | 9.93234    | 1.53354    |
| SIFT          | SIFT            | 1           | 12.5156   | 11.9768    | 0.538849   |
| SIFT          | SIFT            | 2           | 12.6142   | 12.5597    | 0.0545167  |
| SIFT          | SIFT            | 3           | 14.091    | 13.1757    | 0.915288   |
| SIFT          | SIFT            | 4           | 16.6894   | 19.6447    | 2.95532    |
| SIFT          | SIFT            | 5           | 15.9082   | 15.27      | 0.638206   |
| SIFT          | SIFT            | 6           | 12.6787   | 11.1096    | 1.5691     |
| SIFT          | SIFT            | 7           | 11.9844   | 13.9094    | 1.92504    |
| SIFT          | SIFT            | 8           | 13.1241   | 15.6287    | 2.50454    |
| SIFT          | SIFT            | 9           | 13.0241   | 15.0377    | 2.01354    |
| SIFT          | SIFT            | 10          | 11.1746   | 10.5587    | 0.615959   |
| SIFT          | SIFT            | 11          | 12.8086   | 11.7094    | 1.09921    |
| SIFT          | SIFT            | 12          | 8.95978   | 11.6298    | 2.67001    |
| SIFT          | SIFT            | 13          | 9.96439   | 9.24521    | 0.719181   |
| SIFT          | SIFT            | 14          | 9.59863   | 10.6436    | 1.04495    |
| SIFT          | SIFT            | 15          | 8.57352   | 10.4387    | 1.86517    |
| SIFT          | SIFT            | 16          | 9.51617   | 9.57997    | 0.0638051  |
| SIFT          | SIFT            | 17          | 9.54658   | 8.88465    | 0.661932   |
| SIFT          | SIFT            | 18          | 8.3988    | 9.01231    | 0.61351    |
| SHITOMASI     | BRIEF           | 1           | 12.5156   | 14.0747    | 1.55912    |
| SHITOMASI     | BRIEF           | 2           | 12.6142   | 13.8102    | 1.19596    |
| SHITOMASI     | BRIEF           | 3           | 14.091    | 11.712     | 2.379      |
| SHITOMASI     | BRIEF           | 4           | 16.6894   | 13.6007    | 3.08865    |
| SHITOMASI     | BRIEF           | 5           | 15.9082   | 12.3       | 3.60823    |
| SHITOMASI     | BRIEF           | 6           | 12.6787   | 13.2703    | 0.591609   |
| SHITOMASI     | BRIEF           | 7           | 11.9844   | 21.3301    | 9.34575    |
| SHITOMASI     | BRIEF           | 8           | 13.1241   | 12.5199    | 0.604197   |
| SHITOMASI     | BRIEF           | 9           | 13.0241   | 11.6077    | 1.41639    |
| SHITOMASI     | BRIEF           | 10          | 11.1746   | 13.6717    | 2.49701    |
| SHITOMASI     | BRIEF           | 11          | 12.8086   | 11.7805    | 1.02808    |
| SHITOMASI     | BRIEF           | 12          | 8.95978   | 11.7604    | 2.80058    |
| SHITOMASI     | BRIEF           | 13          | 9.96439   | 11.7506    | 1.78625    |
| SHITOMASI     | BRIEF           | 14          | 9.59863   | 11.6524    | 2.0538     |
| SHITOMASI     | BRIEF           | 15          | 8.57352   | 11.2953    | 2.72181    |
| SHITOMASI     | BRIEF           | 16          | 9.51617   | 11.5174    | 2.00119    |
| SHITOMASI     | BRIEF           | 17          | 9.54658   | 11.1869    | 1.64029    |
| SHITOMASI     | BRIEF           | 18          | 8.3988    | 7.96101    | 0.437791   |
| SHITOMASI     | BRISK           | 1           | 12.5156   | 13.9019    | 1.38632    |
| SHITOMASI     | BRISK           | 2           | 12.6142   | 12.9876    | 0.373362   |
| SHITOMASI     | BRISK           | 3           | 14.091    | 13.2474    | 0.843567   |
| SHITOMASI     | BRISK           | 4           | 16.6894   | 12.8675    | 3.82185    |
| SHITOMASI     | BRISK           | 5           | 15.9082   | 13.4476    | 2.46065    |
| SHITOMASI     | BRISK           | 6           | 12.6787   | 13.5266    | 0.847853   |
| SHITOMASI     | BRISK           | 7           | 11.9844   | 12.7172    | 0.732857   |
| SHITOMASI     | BRISK           | 8           | 13.1241   | 13.712     | 0.587852   |
| SHITOMASI     | BRISK           | 9           | 13.0241   | 11.5126    | 1.5115     |
| SHITOMASI     | BRISK           | 10          | 11.1746   | 13.456     | 2.28139    |
| SHITOMASI     | BRISK           | 11          | 12.8086   | 11.9816    | 0.827026   |
| SHITOMASI     | BRISK           | 12          | 8.95978   | 11.8135    | 2.85372    |
| SHITOMASI     | BRISK           | 13          | 9.96439   | 12.0367    | 2.0723     |
| SHITOMASI     | BRISK           | 14          | 9.59863   | 11.342     | 1.74338    |
| SHITOMASI     | BRISK           | 15          | 8.57352   | 9.0531     | 0.479579   |
| SHITOMASI     | BRISK           | 16          | 9.51617   | 10.4612    | 0.945076   |
| SHITOMASI     | BRISK           | 17          | 9.54658   | 11.6823    | 2.13575    |
| SHITOMASI     | BRISK           | 18          | 8.3988    | 9.73863    | 1.33983    |
| SHITOMASI     | ORB             | 1           | 12.5156   | 14.2952    | 1.7796     |
| SHITOMASI     | ORB             | 2           | 12.6142   | 12.439     | 0.175234   |
| SHITOMASI     | ORB             | 3           | 14.091    | 12.2131    | 1.87795    |
| SHITOMASI     | ORB             | 4           | 16.6894   | 12.8595    | 3.82988    |
| SHITOMASI     | ORB             | 5           | 15.9082   | 12.1368    | 3.77144    |
| SHITOMASI     | ORB             | 6           | 12.6787   | 13.6733    | 0.994545   |
| SHITOMASI     | ORB             | 7           | 11.9844   | 11.9741    | 0.010265   |
| SHITOMASI     | ORB             | 8           | 13.1241   | 12.3793    | 0.744804   |
| SHITOMASI     | ORB             | 9           | 13.0241   | 11.3267    | 1.69741    |
| SHITOMASI     | ORB             | 10          | 11.1746   | 13.6447    | 2.47001    |
| SHITOMASI     | ORB             | 11          | 12.8086   | 11.3027    | 1.50592    |
| SHITOMASI     | ORB             | 12          | 8.95978   | 11.3196    | 2.35986    |
| SHITOMASI     | ORB             | 13          | 9.96439   | 12.3765    | 2.41211    |
| SHITOMASI     | ORB             | 14          | 9.59863   | 11.5476    | 1.94897    |
| SHITOMASI     | ORB             | 15          | 8.57352   | 10.4388    | 1.86529    |
| SHITOMASI     | ORB             | 16          | 9.51617   | 11.6681    | 2.15188    |
| SHITOMASI     | ORB             | 17          | 9.54658   | 9.69202    | 0.145439   |
| SHITOMASI     | ORB             | 18          | 8.3988    | 7.60063    | 0.798178   |
| SHITOMASI     | FREAK           | 1           | 12.5156   | 13.6733    | 1.15766    |
| SHITOMASI     | FREAK           | 2           | 12.6142   | 13.4468    | 0.832573   |
| SHITOMASI     | FREAK           | 3           | 14.091    | 11.4627    | 2.62833    |
| SHITOMASI     | FREAK           | 4           | 16.6894   | 12.561     | 4.12838    |
| SHITOMASI     | FREAK           | 5           | 15.9082   | 12.6318    | 3.27639    |
| SHITOMASI     | FREAK           | 6           | 12.6787   | 15.1537    | 2.47495    |
| SHITOMASI     | FREAK           | 7           | 11.9844   | 13.3422    | 1.35784    |
| SHITOMASI     | FREAK           | 8           | 13.1241   | 12.9348    | 0.18929    |
| SHITOMASI     | FREAK           | 9           | 13.0241   | 12.1911    | 0.833005   |
| SHITOMASI     | FREAK           | 10          | 11.1746   | 13.1951    | 2.02042    |
| SHITOMASI     | FREAK           | 11          | 12.8086   | 11.3267    | 1.4819     |
| SHITOMASI     | FREAK           | 12          | 8.95978   | 12.269     | 3.30926    |
| SHITOMASI     | FREAK           | 13          | 9.96439   | 12.6681    | 2.70368    |
| SHITOMASI     | FREAK           | 14          | 9.59863   | 11.5384    | 1.93978    |
| SHITOMASI     | FREAK           | 15          | 8.57352   | 10.2478    | 1.67428    |
| SHITOMASI     | FREAK           | 16          | 9.51617   | 11.3079    | 1.79176    |
| SHITOMASI     | FREAK           | 17          | 9.54658   | 10.8743    | 1.3277     |
| SHITOMASI     | FREAK           | 18          | 8.3988    | 7.87897    | 0.519833   |
| SHITOMASI     | SIFT            | 1           | 12.5156   | 14.2804    | 1.76482    |
| SHITOMASI     | SIFT            | 2           | 12.6142   | 12.9001    | 0.285861   |
| SHITOMASI     | SIFT            | 3           | 14.091    | 11.6087    | 2.48236    |
| SHITOMASI     | SIFT            | 4           | 16.6894   | 12.9209    | 3.76847    |
| SHITOMASI     | SIFT            | 5           | 15.9082   | 12.1886    | 3.71964    |
| SHITOMASI     | SIFT            | 6           | 12.6787   | 13.0553    | 0.376627   |
| SHITOMASI     | SIFT            | 7           | 11.9844   | 12.6612    | 0.676806   |
| SHITOMASI     | SIFT            | 8           | 13.1241   | 12.1376    | 0.986508   |
| SHITOMASI     | SIFT            | 9           | 13.0241   | 11.9899    | 1.03424    |
| SHITOMASI     | SIFT            | 10          | 11.1746   | 13.5009    | 2.32624    |
| SHITOMASI     | SIFT            | 11          | 12.8086   | 12.1033    | 0.705309   |
| SHITOMASI     | SIFT            | 12          | 8.95978   | 11.8967    | 2.93693    |
| SHITOMASI     | SIFT            | 13          | 9.96439   | 11.6227    | 1.65834    |
| SHITOMASI     | SIFT            | 14          | 9.59863   | 10.9335    | 1.33491    |
| SHITOMASI     | SIFT            | 15          | 8.57352   | 12.5287    | 3.95516    |
| SHITOMASI     | SIFT            | 16          | 9.51617   | 10.68      | 1.16387    |
| SHITOMASI     | SIFT            | 17          | 9.54658   | 10.7642    | 1.21763    |
| SHITOMASI     | SIFT            | 18          | 8.3988    | 9.52513    | 1.12633    |
| BRISK         | BRIEF           | 1           | 12.5156   | 12.8448    | 0.329151   |
| BRISK         | BRIEF           | 2           | 12.6142   | 16.8618    | 4.2476     |
| BRISK         | BRIEF           | 3           | 14.091    | 11.6672    | 2.42379    |
| BRISK         | BRIEF           | 4           | 16.6894   | 20.5725    | 3.88311    |
| BRISK         | BRIEF           | 5           | 15.9082   | 19.7052    | 3.79695    |
| BRISK         | BRIEF           | 6           | 12.6787   | 17.5642    | 4.88549    |
| BRISK         | BRIEF           | 7           | 11.9844   | 15.5809    | 3.5965     |
| BRISK         | BRIEF           | 8           | 13.1241   | 18.7451    | 5.62094    |
| BRISK         | BRIEF           | 9           | 13.0241   | 15.4585    | 2.43438    |
| BRISK         | BRIEF           | 10          | 11.1746   | 11.4809    | 0.306308   |
| BRISK         | BRIEF           | 11          | 12.8086   | 13.3636    | 0.555024   |
| BRISK         | BRIEF           | 12          | 8.95978   | 14.3071    | 5.34729    |
| BRISK         | BRIEF           | 13          | 9.96439   | 12.3266    | 2.36225    |
| BRISK         | BRIEF           | 14          | 9.59863   | 10.9457    | 1.34706    |
| BRISK         | BRIEF           | 15          | 8.57352   | 11.6289    | 3.05541    |
| BRISK         | BRIEF           | 16          | 9.51617   | 12.1739    | 2.65772    |
| BRISK         | BRIEF           | 17          | 9.54658   | 11.3792    | 1.83263    |
| BRISK         | BRIEF           | 18          | 8.3988    | 10.7022    | 2.30337    |
| BRISK         | BRISK           | 1           | 12.5156   | 13.4086    | 0.893008   |
| BRISK         | BRISK           | 2           | 12.6142   | 21.5276    | 8.9134     |
| BRISK         | BRISK           | 3           | 14.091    | 12.625     | 1.46601    |
| BRISK         | BRISK           | 4           | 16.6894   | 15.2037    | 1.48564    |
| BRISK         | BRISK           | 5           | 15.9082   | 27.6679    | 11.7597    |
| BRISK         | BRISK           | 6           | 12.6787   | 18.3114    | 5.63273    |
| BRISK         | BRISK           | 7           | 11.9844   | 17.1427    | 5.15834    |
| BRISK         | BRISK           | 8           | 13.1241   | 16.0991    | 2.97495    |
| BRISK         | BRISK           | 9           | 13.0241   | 14.801     | 1.77691    |
| BRISK         | BRISK           | 10          | 11.1746   | 13.9297    | 2.75505    |
| BRISK         | BRISK           | 11          | 12.8086   | 13.1359    | 0.327275   |
| BRISK         | BRISK           | 12          | 8.95978   | 11.3409    | 2.38113    |
| BRISK         | BRISK           | 13          | 9.96439   | 11.8565    | 1.89211    |
| BRISK         | BRISK           | 14          | 9.59863   | 12.3978    | 2.79915    |
| BRISK         | BRISK           | 15          | 8.57352   | 12.7089    | 4.13539    |
| BRISK         | BRISK           | 16          | 9.51617   | 11.503     | 1.98687    |
| BRISK         | BRISK           | 17          | 9.54658   | 9.29379    | 0.252789   |
| BRISK         | BRISK           | 18          | 8.3988    | 10.7753    | 2.37649    |
| BRISK         | ORB             | 1           | 12.5156   | 14.8431    | 2.3275     |
| BRISK         | ORB             | 2           | 12.6142   | 18.7802    | 6.166      |
| BRISK         | ORB             | 3           | 14.091    | 13.2755    | 0.815493   |
| BRISK         | ORB             | 4           | 16.6894   | 16.3966    | 0.292827   |
| BRISK         | ORB             | 5           | 15.9082   | 20.1467    | 4.23848    |
| BRISK         | ORB             | 6           | 12.6787   | 19.9254    | 7.24672    |
| BRISK         | ORB             | 7           | 11.9844   | 18.2074    | 6.22308    |
| BRISK         | ORB             | 8           | 13.1241   | 16.9607    | 3.83656    |
| BRISK         | ORB             | 9           | 13.0241   | 14.5576    | 1.53346    |
| BRISK         | ORB             | 10          | 11.1746   | 11.3267    | 0.15201    |
| BRISK         | ORB             | 11          | 12.8086   | 13.3173    | 0.508659   |
| BRISK         | ORB             | 12          | 8.95978   | 11.3844    | 2.42461    |
| BRISK         | ORB             | 13          | 9.96439   | 11.8035    | 1.83914    |
| BRISK         | ORB             | 14          | 9.59863   | 12.7655    | 3.16692    |
| BRISK         | ORB             | 15          | 8.57352   | 11.2164    | 2.64286    |
| BRISK         | ORB             | 16          | 9.51617   | 11.5484    | 2.03224    |
| BRISK         | ORB             | 17          | 9.54658   | 10.2454    | 0.698814   |
| BRISK         | ORB             | 18          | 8.3988    | 11.2957    | 2.89694    |
| BRISK         | FREAK           | 1           | 12.5156   | 12.7574    | 0.241844   |
| BRISK         | FREAK           | 2           | 12.6142   | 24.137     | 11.5228    |
| BRISK         | FREAK           | 3           | 14.091    | 13.9605    | 0.130522   |
| BRISK         | FREAK           | 4           | 16.6894   | 13.9445    | 2.74486    |
| BRISK         | FREAK           | 5           | 15.9082   | 23.0655    | 7.15728    |
| BRISK         | FREAK           | 6           | 12.6787   | 16.8726    | 4.19391    |
| BRISK         | FREAK           | 7           | 11.9844   | 15.5199    | 3.53557    |
| BRISK         | FREAK           | 8           | 13.1241   | 18.7996    | 5.67552    |
| BRISK         | FREAK           | 9           | 13.0241   | 14.9704    | 1.94626    |
| BRISK         | FREAK           | 10          | 11.1746   | 13.7428    | 2.56811    |
| BRISK         | FREAK           | 11          | 12.8086   | 12.8798    | 0.0712148  |
| BRISK         | FREAK           | 12          | 8.95978   | 11.4488    | 2.48906    |
| BRISK         | FREAK           | 13          | 9.96439   | 11.4362    | 1.4718     |
| BRISK         | FREAK           | 14          | 9.59863   | 11.2969    | 1.69826    |
| BRISK         | FREAK           | 15          | 8.57352   | 13.3364    | 4.76292    |
| BRISK         | FREAK           | 16          | 9.51617   | 10.3718    | 0.855642   |
| BRISK         | FREAK           | 17          | 9.54658   | 9.33041    | 0.216168   |
| BRISK         | FREAK           | 18          | 8.3988    | 10.9372    | 2.53843    |
| BRISK         | SIFT            | 1           | 12.5156   | 12.9834    | 0.467761   |
| BRISK         | SIFT            | 2           | 12.6142   | 16.0654    | 3.45117    |
| BRISK         | SIFT            | 3           | 14.091    | 16.6949    | 2.60387    |
| BRISK         | SIFT            | 4           | 16.6894   | 12.5648    | 4.12457    |
| BRISK         | SIFT            | 5           | 15.9082   | 26.3307    | 10.4225    |
| BRISK         | SIFT            | 6           | 12.6787   | 14.504     | 1.82532    |
| BRISK         | SIFT            | 7           | 11.9844   | 13.9956    | 2.01125    |
| BRISK         | SIFT            | 8           | 13.1241   | 17.0214    | 3.89726    |
| BRISK         | SIFT            | 9           | 13.0241   | 17.8988    | 4.87469    |
| BRISK         | SIFT            | 10          | 11.1746   | 14.6452    | 3.47054    |
| BRISK         | SIFT            | 11          | 12.8086   | 15.9755    | 3.16693    |
| BRISK         | SIFT            | 12          | 8.95978   | 11.7122    | 2.7524     |
| BRISK         | SIFT            | 13          | 9.96439   | 13.3127    | 3.34831    |
| BRISK         | SIFT            | 14          | 9.59863   | 10.5612    | 0.962616   |
| BRISK         | SIFT            | 15          | 8.57352   | 14.2108    | 5.63726    |
| BRISK         | SIFT            | 16          | 9.51617   | 11.0325    | 1.51631    |
| BRISK         | SIFT            | 17          | 9.54658   | 10.1474    | 0.600803   |
| BRISK         | SIFT            | 18          | 8.3988    | 12.3189    | 3.92008    |
| FAST          | BRIEF           | 1           | 12.5156   | 11.6181    | 0.897532   |
| FAST          | BRIEF           | 2           | 12.6142   | 10.9298    | 1.68447    |
| FAST          | BRIEF           | 3           | 14.091    | 15.5521    | 1.46112    |
| FAST          | BRIEF           | 4           | 16.6894   | 12.7493    | 3.94004    |
| FAST          | BRIEF           | 5           | 15.9082   | 15.1979    | 0.710382   |
| FAST          | BRIEF           | 6           | 12.6787   | 12.6506    | 0.028119   |
| FAST          | BRIEF           | 7           | 11.9844   | 13.6193    | 1.63497    |
| FAST          | BRIEF           | 8           | 13.1241   | 12.6071    | 0.517003   |
| FAST          | BRIEF           | 9           | 13.0241   | 12.5606    | 0.463494   |
| FAST          | BRIEF           | 10          | 11.1746   | 12.1008    | 0.926182   |
| FAST          | BRIEF           | 11          | 12.8086   | 12.2522    | 0.556355   |
| FAST          | BRIEF           | 12          | 8.95978   | 11.4605    | 2.50074    |
| FAST          | BRIEF           | 13          | 9.96439   | 11.8703    | 1.90591    |
| FAST          | BRIEF           | 14          | 9.59863   | 10.9228    | 1.3242     |
| FAST          | BRIEF           | 15          | 8.57352   | 11.179     | 2.60546    |
| FAST          | BRIEF           | 16          | 9.51617   | 11.9352    | 2.41905    |
| FAST          | BRIEF           | 17          | 9.54658   | 8.2058     | 1.34078    |
| FAST          | BRIEF           | 18          | 8.3988    | 11.6293    | 3.23046    |
| FAST          | BRISK           | 1           | 12.5156   | 12.3974    | 0.11824    |
| FAST          | BRISK           | 2           | 12.6142   | 12.3176    | 0.296692   |
| FAST          | BRISK           | 3           | 14.091    | 13.813     | 0.277964   |
| FAST          | BRISK           | 4           | 16.6894   | 12.7138    | 3.97555    |
| FAST          | BRISK           | 5           | 15.9082   | 22.6027    | 6.69447    |
| FAST          | BRISK           | 6           | 12.6787   | 12.9554    | 0.27672    |
| FAST          | BRISK           | 7           | 11.9844   | 12.511     | 0.526637   |
| FAST          | BRISK           | 8           | 13.1241   | 12.0122    | 1.11194    |
| FAST          | BRISK           | 9           | 13.0241   | 13.5598    | 0.535661   |
| FAST          | BRISK           | 10          | 11.1746   | 14.0994    | 2.92477    |
| FAST          | BRISK           | 11          | 12.8086   | 12.0299    | 0.778725   |
| FAST          | BRISK           | 12          | 8.95978   | 12.56      | 3.60023    |
| FAST          | BRISK           | 13          | 9.96439   | 11.6249    | 1.66054    |
| FAST          | BRISK           | 14          | 9.59863   | 11.4982    | 1.89961    |
| FAST          | BRISK           | 15          | 8.57352   | 11.8       | 3.22648    |
| FAST          | BRISK           | 16          | 9.51617   | 12.4833    | 2.96717    |
| FAST          | BRISK           | 17          | 9.54658   | 9.7903     | 0.24372    |
| FAST          | BRISK           | 18          | 8.3988    | 12.2277    | 3.82891    |
| FAST          | ORB             | 1           | 12.5156   | 11.9536    | 0.562043   |
| FAST          | ORB             | 2           | 12.6142   | 11.5558    | 1.05844    |
| FAST          | ORB             | 3           | 14.091    | 14.4138    | 0.322749   |
| FAST          | ORB             | 4           | 16.6894   | 12.8765    | 3.81291    |
| FAST          | ORB             | 5           | 15.9082   | 19.7021    | 3.79387    |
| FAST          | ORB             | 6           | 12.6787   | 12.5506    | 0.128127   |
| FAST          | ORB             | 7           | 11.9844   | 13.1068    | 1.12249    |
| FAST          | ORB             | 8           | 13.1241   | 12.1318    | 0.992347   |
| FAST          | ORB             | 9           | 13.0241   | 12.7526    | 0.271536   |
| FAST          | ORB             | 10          | 11.1746   | 13.1274    | 1.95274    |
| FAST          | ORB             | 11          | 12.8086   | 12.5219    | 0.286746   |
| FAST          | ORB             | 12          | 8.95978   | 11.7136    | 2.75384    |
| FAST          | ORB             | 13          | 9.96439   | 11.981     | 2.01656    |
| FAST          | ORB             | 14          | 9.59863   | 10.8193    | 1.22062    |
| FAST          | ORB             | 15          | 8.57352   | 9.90596    | 1.33243    |
| FAST          | ORB             | 16          | 9.51617   | 11.5965    | 2.08034    |
| FAST          | ORB             | 17          | 9.54658   | 8.69538    | 0.851198   |
| FAST          | ORB             | 18          | 8.3988    | 12.1729    | 3.77407    |
| FAST          | FREAK           | 1           | 12.5156   | 12.0855    | 0.430137   |
| FAST          | FREAK           | 2           | 12.6142   | 28.6859    | 16.0716    |
| FAST          | FREAK           | 3           | 14.091    | 15.0691    | 0.978127   |
| FAST          | FREAK           | 4           | 16.6894   | 13.6141    | 3.07531    |
| FAST          | FREAK           | 5           | 15.9082   | 32.168     | 16.2597    |
| FAST          | FREAK           | 6           | 12.6787   | 12.08      | 0.598706   |
| FAST          | FREAK           | 7           | 11.9844   | 13.33      | 1.34566    |
| FAST          | FREAK           | 8           | 13.1241   | 12.5196    | 0.604559   |
| FAST          | FREAK           | 9           | 13.0241   | 13.2483    | 0.224161   |
| FAST          | FREAK           | 10          | 11.1746   | 13.3599    | 2.18525    |
| FAST          | FREAK           | 11          | 12.8086   | 12.5452    | 0.263368   |
| FAST          | FREAK           | 12          | 8.95978   | 11.616     | 2.65622    |
| FAST          | FREAK           | 13          | 9.96439   | 11.9483    | 1.98388    |
| FAST          | FREAK           | 14          | 9.59863   | 11.4       | 1.80137    |
| FAST          | FREAK           | 15          | 8.57352   | 11.3289    | 2.75533    |
| FAST          | FREAK           | 16          | 9.51617   | 12.1074    | 2.59124    |
| FAST          | FREAK           | 17          | 9.54658   | 9.23191    | 0.314667   |
| FAST          | FREAK           | 18          | 8.3988    | 12.382     | 3.98321    |
| FAST          | SIFT            | 1           | 12.5156   | 12.0531    | 0.462487   |
| FAST          | SIFT            | 2           | 12.6142   | 11.0789    | 1.53539    |
| FAST          | SIFT            | 3           | 14.091    | 22.5166    | 8.42554    |
| FAST          | SIFT            | 4           | 16.6894   | 12.5139    | 4.17547    |
| FAST          | SIFT            | 5           | 15.9082   | 17.6305    | 1.72227    |
| FAST          | SIFT            | 6           | 12.6787   | 12.6299    | 0.0487711  |
| FAST          | SIFT            | 7           | 11.9844   | 13.0534    | 1.06905    |
| FAST          | SIFT            | 8           | 13.1241   | 12.8337    | 0.290419   |
| FAST          | SIFT            | 9           | 13.0241   | 12.3176    | 0.706565   |
| FAST          | SIFT            | 10          | 11.1746   | 12.4284    | 1.25377    |
| FAST          | SIFT            | 11          | 12.8086   | 12.5009    | 0.307718   |
| FAST          | SIFT            | 12          | 8.95978   | 11.3317    | 2.37194    |
| FAST          | SIFT            | 13          | 9.96439   | 12.1502    | 2.18583    |
| FAST          | SIFT            | 14          | 9.59863   | 11.2847    | 1.6861     |
| FAST          | SIFT            | 15          | 8.57352   | 9.9049     | 1.33138    |
| FAST          | SIFT            | 16          | 9.51617   | 11.5658    | 2.0496     |
| FAST          | SIFT            | 17          | 9.54658   | 7.78794    | 1.75865    |
| FAST          | SIFT            | 18          | 8.3988    | 11.266     | 2.86722    |
| AKAZE         | AKAZE           | 1           | 12.5156   | 12.3444    | 0.171208   |
| AKAZE         | AKAZE           | 2           | 12.6142   | 14.0983    | 1.48408    |
| AKAZE         | AKAZE           | 3           | 14.091    | 12.8828    | 1.20818    |
| AKAZE         | AKAZE           | 4           | 16.6894   | 14.4781    | 2.21132    |
| AKAZE         | AKAZE           | 5           | 15.9082   | 16.7468    | 0.838524   |
| AKAZE         | AKAZE           | 6           | 12.6787   | 13.8675    | 1.18879    |
| AKAZE         | AKAZE           | 7           | 11.9844   | 15.3448    | 3.36044    |
| AKAZE         | AKAZE           | 8           | 13.1241   | 14.1379    | 1.01381    |
| AKAZE         | AKAZE           | 9           | 13.0241   | 13.8398    | 0.815652   |
| AKAZE         | AKAZE           | 10          | 11.1746   | 11.5483    | 0.373644   |
| AKAZE         | AKAZE           | 11          | 12.8086   | 12.1341    | 0.674506   |
| AKAZE         | AKAZE           | 12          | 8.95978   | 11.0554    | 2.09557    |
| AKAZE         | AKAZE           | 13          | 9.96439   | 11.2852    | 1.32083    |
| AKAZE         | AKAZE           | 14          | 9.59863   | 10.5845    | 0.985867   |
| AKAZE         | AKAZE           | 15          | 8.57352   | 10.1989    | 1.62535    |
| AKAZE         | AKAZE           | 16          | 9.51617   | 9.81256    | 0.296389   |
| AKAZE         | AKAZE           | 17          | 9.54658   | 9.06452    | 0.482057   |
| AKAZE         | AKAZE           | 18          | 8.3988    | 8.9739     | 0.575094   |

# Things

## TRANSFORMERS

* Parallel lines meet at infinity
* Images are 3D projections onto 2D space
    * Homogeneous coordintaes represent 2d point as a 3d vector (x, y, 1)
* Transformation matrix p' = (T * R * S) * p
* Translation (T), rigid (T + R), similarity (T + R + S), affine (Warping, 12 dof), projective (Warping, 15 dof) transformations
* Map 3D to 2D coordinates with projection matrix
    * Map onto an image plane d away from optical center
    * Removes z coordinate by mapping (x, y, z)  ->  (x, y) using the projection matrix and homogenous coordinates
    * Projection loses angles, distances from camera, all points along a ray map to same pixel
* Vanishing point = where parallel lines meet, can be in image or outside of image, can be many vanishing points
    * Provides info about camera orientation
    * Find straight lines and calculate their intersection point, if multiple lines intersect at same point -> Vanishing point
* P = K * PROJ * R * T * [X Y Z 1]
* K Matrix has focal length and center pixel aka principal point, skew
* Big focal length = more zoom (smaller field of view)

## IMG PROCESSING

* Point operators
    * RGB Values
* Neighborhood processing
    * Filters with convolution
    * Dot product over area of image
    * Box, gaussian, sobel filters ...
* Global operators
* Fourier transform helps understanding difference between continuous and discrete images
    * Can be used to find the orientation of an image of text for example
* Image resize
    * Too low sampling rate causes aliasing in image
    * Alternate between blurring and subsampling for example with gaussian filter
    * When upsampling we need to estimate a continuous signal with a discrete one
        * Use a smoother function for a visually better result than nearest neighbour filter

## Feature detection

* Find feature
    * Represent feature
    * Match / Track feature between images
    * Should be invariant to transform/rotation/scale, brightness/exposure
* Local features
    * Robust to occlusion
    * Many unique features
    * Distinctive
    * Efficient to compute
* Corners are good features as the region changes no matter the direction of movement
* Harris corners:
    * Calculate sum of squared differences where error should be high, i.e. identifying features to translation
    * H Matrix captures the changes in the image (Eigenvectors and eigenvalues describe the feature)
    * How to use
        * Gradient at each point in image
        * H matrix with gradient
        * Compute Eigens
        * Find points with large response
        * Pick points where lambda min is local maximum as good features
* Invariance: corner locations do not change if image is transformed
* Equivariance: two transformed versions of the image find features in corresponding locations
* SIFT Features :^)
    * Take 16x16 window around a feature, calculate histogram of gradient angles, clipping weak gardients
    * Split window into 4x4 grid and save 8 dimensional descriptor for each cell
* Feature distance
    * Use ratio distance to hopefully find correct matches
    * Use RoC curve and true/false positive rate to evaluate performance
* Edges specify rapid change in image intensity
    * Noise makes it difficult to find edges
        * SMOOTHING
* Find gradients with filters
    * Non-max suppression to find clean edges
    * Threshold edges to keep only strong edges
        * Weak edges can connect strong edges but dont stand on their own
            * Canny edge detector
                * Parameter for how fine edges to detect
* Hough transform
    * Detect straight lines from images
    * Transform edges into hough space and accumulate lines of (r, theta)
    * Vote for these lines in the accumulator array
    * Draw lines above some threshold where they are local maxima

## Stitches

* Use projective transformations to combine two projective planes of the same subject
* Homography can be used when scene is planar, little depth variation, camera is only rotated
* Need at least 4 point correspondences to find Homography matrix
    * Find H that minimizes a cost function
        * Least squares, RANSAC, Hough Transform, EM
    * Create 2n x 9 matrix A
        * Compute SVD of A
            * Singular vector responding to smallest singular value is H
                * Reshape to H
    * Linear least squares not robust to outliers
    * RANSAC
        * Sample number of points required to fit model
        * Solve parameters with sample
        * Score by inliers
        * Repeat until found answer

## Camera calibration and epipolar constraint

* Camera calibration
    * Given points, find P (pose) from PX
        * Create linear equations, minimize error with smallest SVD vector
            * Decompose Projection matrix into K * [R | t]
            * To find center C, do SVD on P and c is smallest Eigenvector
                * Since PC = 0 because C is the center obviously
            * M = KR -> To get K and R do QR Decompostition on the matrix as K is upper triangular matrix =)
* Camera calibration from known geometry
    * Minimize reprojection error between point in image and real world coordinates
    * Add radial distortion variable to correct for radial distortion in the image
    * Multi-plane calibration
        * Take multiple images of a plane and use ready made tools :)
        * Non-linear optimization problemm
* Triangulation
    * Cross product (x, PX) = 0
        * Stack the two systems and S * X = 0
            * SVD and all that good stuff again
* Epipolar geometry
    * Baseline between the image centers, epipolar plane formed by the rays
    * Epipolar line is intersection of the epipolar plane and image plane
    * Epipoles are the points that are on the baseline
    * Epipolar constraint allows limiting finding the matching point in left and right images

## Fundamental and essential matrices

* Essential matrix
    * Maps a point into the epipolar line in the second view
    * Assumes points are aligned to camera coordinate axis
* Fundamental matrix
    * Can be estimated from two images
    * 8 point algorithm
* Finding scale is possible directly with special cameras
    * Lidar, RGB-D cameras, lasers
* Minimize stereo image error
    * Energy minimization
        * Combine match cost and smoothness cost

## Optical flow

* Given two image frames, estimate motion for each pixel
    * Small motion, brightness remains constant
    * Find spatial and temporal derivatives (Filter and frame difference)
    * Lucas-Kanade
        * Constant flow for all pixels (local method)
        * Make patch, solve for minimum error (u, v)
    * Horn-Schunk
        * Smooth flow, can vary pixel to pixel
        * Global method
        * Minimize smoothenss and brightness constancy at each pixel
--README

General SLAM Algorithm Guidelines

--Prerequisite
+ Odometry data
+ Sensors
+ Robot

--Introduction

Vocabulary
+ SLAM - acronym for Simultaneous Localization and Mapping which is the challenge of constructing and updating a map while having the entity within an environment
+ Odometry - the use of sensors to estimate changes in position over time based on the controls given to the robot
+ Jacobian - the matrix of all partial derivatives of a vector-valued function. When the matrix is a square matrix, the matrix and the determinant of the matrix is referred to as Jacobian
+ EKF - acronym for Extended Kalman Filter which is the nonlinear version of the Kalman filter that linearlizes estimates of the mean and covariance

More information is available on Wikipedia

The Problem

The problem can of SLAM can be separated into different parts
1. Landmark extraction
2. Data association
3. State estimation
4. State update
5. Landmark update

Equipment Notes
Range Measurement Device
+ laser scanners
	generally expensive
	are very accurate and do not take much computational power
	have difficulty measuring distances when there are light distortive surfaces such as glass, water, etc.
+ Sonar
	very cheap
	fairly accurate but can have significantly inaccurate readings depending on environment
	best used underwater
+ Vision
	computationally intensive
	error prone if there are changes in light
	
--Simultaneous Localization and Mapping
Landmarks
+ are features that can easily be observed and re-observed and distinguished from the environment
+ used with the range measurement to "correct" the prediction of the odometry data
+ different dependent on the environment
+ need to be easily re-observable from different angles
+ need to be fairly unique otherwise the robot might associate the wrong landmark
+ need many landmarks or the robot will spend too much time trying to find the few landmarks
+ must be stationary or the robot will overcompensate the distance it traveled with the extraction data

Landmark Extraction
+ once the landmarks are chosen, we need to reliably extract them from the robot's sensory inputs
+ there are a few algorithms that can be used for Landmark Extraction: Spike, RANSAC, SURF, SIFT, Scan matching, and ORB
+ Spike
	- uses extrema to find landmarks
	- works by finding big changes in laser scans
	- or works by having three laser scan measurements X, Y, and Z. (Y - X) + (Z - Y) is the measurement used
	- fails in smooth environments
+ RANSAC (Random Sampling Consensus)
	- used to extract lines from laser scans
	- lines can be used as landmarks
	- uses least square approximation of the random samples of laser readings to find a best fit line
	- checks how many laser readings are close to the best fit line
	- if number is above threshold then we can say we saw a landmark
	- EKF implementation
+ More information on the rest of the algorithms can be read
	SURF: http://www.vision.ee.ethz.ch/~surf/eccv06.pdf
	SIFT: https://www.inf.fu-berlin.de/lehre/SS09/CV/uebungen/uebung09/SIFT.pdf
	Scan Matching: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa11/slides/scan-matching.pdf
	ORB: http://www.vision.cs.chubu.ac.jp/CV-R/pdf/Rublee_iccv2011.pdf

Data Association
+ matching observed landmarks from different laser scans with each other
+ the robot has to re-observe landmarks
+ the landmark may "disappear" from the robot's sensors
+ a devastating issue of incorrectly associating landmarks
	- the robot will re-localize itself to a different part of the map and move accordingly to its incorrect position
+ nearest-neighbor approach
	- calculating the euclidean (or Mahalanobis) distance from the nearest landmark
+ validation gate
	- checking if the landmark lies within an area of uncertainty

EKF
+ the EKF is an algorithm that uses measurements that contain statistical noise to produce more accurate measurements
+ the EKF is basically a standard for GPS and navigational systems
+ prerequisites: landmark extraction and data association
+ updates the current state estimate using the odometry data
+ updates the estimated state from re-observing landmarks
+ add new landmarks to the current state
+ more of the math can be read here: https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf

Other Considerations
+ closing the loop
	- what happens when the robot returns to a place it has already seen before
+ occupation grid
	- creating a human-readable map so path planning can be done
+ A* and D* Search Algorithms
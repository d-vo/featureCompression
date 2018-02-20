/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*
* This part contains code from ORB-SLAM
* Copyright of ORB-SLAM2:
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once


#include <opencv2/opencv.hpp>


class ImgBufferEntry
{
public:
	ImgBufferEntry(int width = -1, int height = -1, int nLevels = 8);

	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<unsigned int> visualWords;

	int mnWidth;
	int mnHeight;
	int mnLevels;

	long long mnImageId;

	int mnCols = 32;
	int mnRows = 24;
	float mWidthInv;
	float mHeightInv;
	std::vector<std::vector<std::vector<unsigned int> > > mGrid;
	std::vector<std::vector<std::vector<unsigned int> > > mvRowIndices;

	void addFeatures( const std::vector<cv::KeyPoint> &kpts, const cv::Mat &descriptor);
	void addFeature( const cv::KeyPoint &kp, const cv::Mat &descriptor);
	void addFeature( const cv::KeyPoint &kp, const cv::Mat &descriptor, const unsigned int visualWord );
	bool PositionInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
	void AssignFeatures();
	std::vector<unsigned int> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const;
	std::vector<unsigned int> GetStereoFeaturesInLine(const float  &yL, const int &octave) const;
};

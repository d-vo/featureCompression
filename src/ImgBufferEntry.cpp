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



#include "ImgBufferEntry.h"
#include <chrono>
#include <iostream>

ImgBufferEntry::ImgBufferEntry(int width, int height, int nLevels)

{
	mnImageId = 0;
	mnWidth = width;
	mnHeight = height;
	mnLevels = nLevels;

	mWidthInv = ((float) mnCols) / mnWidth;
	mHeightInv = ((float) mnRows) / mnHeight;

	const int res = 1200.0f / (mnCols*mnRows);

	mGrid.resize(mnCols);
	for(int i=0; i<mnCols;i++)
	{
		mGrid[i].resize(mnRows);
		for(int j=0; j<mnRows;j++)
			mGrid[i][j].reserve(res);
	}


	keypoints.reserve(1200);
	visualWords.reserve(1200);
}

void ImgBufferEntry::addFeatures( const std::vector<cv::KeyPoint> &kpts, const cv::Mat &descriptor)
{
	for( auto &kp : kpts )
	{
		int nGridPosX, nGridPosY;
		if(PositionInGrid(kp,nGridPosX,nGridPosY))
			mGrid[nGridPosX][nGridPosY].push_back(keypoints.size());

		keypoints.push_back(kp);
	}

	descriptors.push_back(descriptor);
}


void ImgBufferEntry::addFeature( const cv::KeyPoint &kp, const cv::Mat &descriptor)
{
	int nGridPosX, nGridPosY;
	if(PositionInGrid(kp,nGridPosX,nGridPosY))
		mGrid[nGridPosX][nGridPosY].push_back(keypoints.size());

	keypoints.push_back(kp);
	descriptors.push_back(descriptor);
}

void ImgBufferEntry::addFeature( const cv::KeyPoint &kp, const cv::Mat &descriptor, const unsigned int visualWord)
{
	int nGridPosX, nGridPosY;
	if(PositionInGrid(kp,nGridPosX,nGridPosY))
		mGrid[nGridPosX][nGridPosY].push_back(keypoints.size());

	keypoints.push_back(kp);
	descriptors.push_back(descriptor);
	visualWords.push_back(visualWord);
}

bool ImgBufferEntry::PositionInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
	posX = round(kp.pt.x*mWidthInv);
	posY = round(kp.pt.y*mHeightInv);

	if( posX < 0 || posY < 0 || posX >= mnCols || posY >= mnRows )
		return false;

	return true;
}


std::vector<unsigned int> ImgBufferEntry::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
	std::vector<unsigned int> vIndices;

	const int nMinCellX = std::max(0,(int)floor((x-r)*mWidthInv));
	if(nMinCellX>=mnCols)
		return vIndices;

	const int nMaxCellX = std::min((int)mnCols-1,(int)ceil((x+r)*mWidthInv));
	if(nMaxCellX<0)
		return vIndices;

	const int nMinCellY = std::max(0,(int)floor((y-r)*mHeightInv));
	if(nMinCellY>=mnRows)
		return vIndices;

	const int nMaxCellY = std::min((int)mnRows-1,(int)ceil((y+r)*mHeightInv));
	if(nMaxCellY<0)
		return vIndices;

	const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

	for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
	{
		for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
		{
			const std::vector<unsigned int> &vCell = mGrid[ix][iy];
			if(vCell.empty())
				continue;

			// loop through all keypoints in the current grid cell
			for(size_t j=0, jend=vCell.size(); j<jend; j++)
			{
				const cv::KeyPoint &kpUn = keypoints[vCell[j]];
				if(bCheckLevels)
				{
					if(kpUn.octave<minLevel)
						continue;
					if(maxLevel>=0)
						if(kpUn.octave>maxLevel)
							continue;
				}

				const float distx = kpUn.pt.x-x;
				const float disty = kpUn.pt.y-y;

				// save the keypoint when it is within the search radius
				if(fabs(distx)<=r && fabs(disty)<=r)
					vIndices.push_back(vCell[j]);
			}
		}
	}
	return vIndices;
}


void ImgBufferEntry::AssignFeatures()
{
	//Assign keypoints to row table
	mvRowIndices.resize(mnLevels);
	for( int o=0; o < mnLevels; o++)
	{
		mvRowIndices[o].resize(mnHeight);
		for(int i=0; i<mnHeight; i++)
			mvRowIndices[o][i].reserve(200);
	}


	const int Nr = keypoints.size();

	for(int iR=0; iR<Nr; iR++)
	{
		const cv::KeyPoint &kp = keypoints[iR];
		const int &octave = kp.octave;
		const float &kpY = kp.pt.y;
		const float r = 2.0f*pow(1.2f,keypoints[iR].octave);
		const int maxr = floor(kpY+r);
		const int minr = ceil(kpY-r);

		for(int yi=minr;yi<=maxr;yi++)
			mvRowIndices[octave][yi].push_back(iR);
	}
}


std::vector<unsigned int> ImgBufferEntry::GetStereoFeaturesInLine(const float  &yL, const int &octave) const
{
	return mvRowIndices[octave][yL];
}

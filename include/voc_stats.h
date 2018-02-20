/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/

#pragma once


#include "utils.h"

class CodingStats
{
public:
	CodingStats()
{
		mDims = 0;
		mSearchRange = 10;
		mAngleBins = 32;
		mOctaves = 8;
		mScaleFactor = 1.2f;
		mRespBinSize = 0;
		mDistBinSize = 0;
};


	CodingStats( int dims )
	{
		mDims = dims;
		mSearchRange = 10;
		mAngleBins = 32;
		mOctaves = 8;
		mScaleFactor = 1.2f;
		mRespBinSize = 0;
		mDistBinSize = 0;
	}


	void updateSettings( int width, int height, float searchRange = 10.0f, int angleBins = 32, int octaves = 8,
			float scaleFactor = 1.2f, int searchRangeStereoX = 25, int searchRangeStereoY = 1)
	{
		mSearchRange = searchRange;
		mSearchRangeStereoX = searchRangeStereoX;
		mSearchRangeStereoY = searchRangeStereoY;
		mScaleFactor = scaleFactor;
		mAngleBins = angleBins;
		mOctaves = octaves;
	}


	void save(const std::string &path)
	{
		FILE *f = fopen(path.c_str(), "w");

		uint64_t v = fwrite(&mDims, sizeof(unsigned int), 1, f);
		v += fwrite(&mSearchRange, sizeof(int), 1, f);
		v += fwrite(&mSearchRangeStereoX, sizeof(int), 1, f);
		v += fwrite(&mSearchRangeStereoY, sizeof(int), 1, f);
		v += fwrite(&mScaleFactor, sizeof(float), 1, f);
		v += fwrite(&mAngleBins, sizeof(int), 1, f);
		v += fwrite(&mOctaves, sizeof(int), 1, f);

		v += fwrite(&p0_intra_, sizeof(float), 1, f);
		v += fwrite(&p0_intra_pred_, sizeof(float), 1, f);
		v += fwrite(&p0_inter_, sizeof(float), 1, f);
		v += fwrite(&p0_stereo_, sizeof(float), 1, f);


		Utils::writeVectorOfMatrix(pPosPerOctave_, f);
		Utils::writeVectorOfMatrix(pPosPerOctaveStereo_, f);

		Utils::writeMatrix(pAngleDelta_, f);
		Utils::writeMatrix(pOctaveDelta_, f);

		// Feature selection
		Utils::writeMatrix(pSigma_, f);
		Utils::writeMatrix(pCoding_, f);
		Utils::writeMatrix(pResp_, f);
		Utils::writeMatrix(pDist_, f);
		Utils::writeMatrix(pBow_, f);

		v += fwrite(&mRespBinSize, sizeof(float), 1, f);
		v += fwrite(&mDistBinSize, sizeof(float), 1, f);

		fclose(f);
	}

	void load(const std::string &path)
	{
		FILE *f = fopen(path.c_str(), "r");

		uint64_t v = fread(&mDims, sizeof(unsigned int), 1, f);
		v += fread(&mSearchRange, sizeof(int), 1, f);
		v += fread(&mSearchRangeStereoX, sizeof(int), 1, f);
		v += fread(&mSearchRangeStereoY, sizeof(int), 1, f);
		v += fread(&mScaleFactor, sizeof(float), 1, f);
		v += fread(&mAngleBins, sizeof(int), 1, f);
		v += fread(&mOctaves, sizeof(int), 1, f);

		v += fread(&p0_intra_, sizeof(float), 1, f);
		v += fread(&p0_intra_pred_, sizeof(float), 1, f);
		v += fread(&p0_inter_, sizeof(float), 1, f);
		v += fread(&p0_stereo_, sizeof(float), 1, f);


		Utils::readVectorOfMatrix(pPosPerOctave_, f);
		Utils::readVectorOfMatrix(pPosPerOctaveStereo_, f);

		Utils::readMatrix(pAngleDelta_, f);
		Utils::readMatrix(pOctaveDelta_, f);

		// Feature selection - filled in by matlab
		Utils::readMatrix(pSigma_, f);
		Utils::readMatrix(pCoding_, f);
		Utils::readMatrix(pResp_, f);
		Utils::readMatrix(pDist_, f);
		Utils::readMatrix(pBow_, f);


		if( pSigma_.empty() || pCoding_.empty() || pResp_.empty() || pBow_.empty())
			std::cerr << "Probabilities empty!" << std::endl;

		pSigma_.convertTo(pSigma_, CV_32F);
		pCoding_.convertTo(pCoding_, CV_32F);
		pResp_.convertTo(pResp_, CV_32F);
		pDist_.convertTo(pDist_, CV_32F);
		pBow_.convertTo(pBow_, CV_32F);

		v += fread(&mRespBinSize, sizeof(float), 1, f);
		v += fread(&mDistBinSize, sizeof(float), 1, f);

		fclose(f);
	}


public:
	unsigned int mDims;

	// Config
	int mSearchRange;
	int mSearchRangeStereoX;
	int mSearchRangeStereoY;
	float mScaleFactor;
	int mAngleBins;
	int mOctaves;



	// Intra Coding
	float p0_intra_;

	// Intra Pred Coding
	float p0_intra_pred_;

	// Inter Coding
	float p0_inter_;

	// Stereo Coding
	float p0_stereo_;


	std::vector<cv::Mat> pPosPerOctave_;
	std::vector<cv::Mat> pPosPerOctaveStereo_;

	cv::Mat pAngleDelta_;
	cv::Mat pOctaveDelta_;


	// Feature selection parameter
	cv::Mat pSigma_;
	cv::Mat pCoding_;
	cv::Mat pResp_;
	cv::Mat pDist_;
	cv::Mat pBow_;


	float mRespBinSize;
	float mDistBinSize;
};


// Visualization only
class FeatureStats
{
public:
	FeatureStats()
{
		mDims = 0;
		mNumImages = 0;
		mNumDescriptors = 0;
		mSearchRange = 10;
		mAngleBins = 32;
		mOctaves = 8;
		mScaleFactor = 1.2f;
		bFinalize = false;
};


	FeatureStats( int dims )
	{
		mDims = dims;
		mNumImages = 0;
		mNumDescriptors = 0;
		mSearchRange = 10;
		mAngleBins = 32;
		mOctaves = 8;
		mScaleFactor = 1.2f;
		bFinalize = false;
	};


	void updateSettings( int width, int height, float searchRange = 10.0f, int angleBins = 32, int octaves = 8, float scaleFactor = 1.2f)
	{
		mSearchRange = searchRange;
		mScaleFactor = scaleFactor;
		mAngleBins = angleBins;
		mOctaves = octaves;

		// Set up matrices
		mAngleHist = cv::Mat(1, mAngleBins, CV_32F, cv::Scalar::all(0));
		mOctaveHist = cv::Mat(1, mOctaves, CV_32S, cv::Scalar::all(0));
	}


	void save(const std::string &path)
	{
		finalize();

		FILE *f = fopen(path.c_str(), "w");

		fwrite(&mDims, sizeof(unsigned int), 1, f);
		fwrite(&mNumImages, sizeof(unsigned int), 1, f);
		fwrite(&mNumDescriptors, sizeof(unsigned long long), 1, f);

		// Predictive coding
		fwrite(&mSearchRange, sizeof(int), 1, f);
		fwrite(&mScaleFactor, sizeof(float), 1, f);
		fwrite(&mAngleBins, sizeof(int), 1, f);
		fwrite(&mOctaves, sizeof(int), 1, f);


		Utils::writeMatrix(mAngleHist, f);
		Utils::writeMatrix(mOctaveHist, f);

		fclose(f);
	}


	void load(const std::string &path)
	{
		FILE *f = fopen(path.c_str(), "r");

		uint64_t v = fread(&mDims, sizeof(unsigned int), 1, f);
		v += fread(&mNumImages, sizeof(unsigned int), 1, f);
		v += fread(&mNumDescriptors, sizeof(unsigned long long), 1, f);

		// Predictive coding
		v += fread(&mSearchRange, sizeof(int), 1, f);
		v += fread(&mScaleFactor, sizeof(float), 1, f);
		v += fread(&mAngleBins, sizeof(int), 1, f);
		v += fread(&mOctaves, sizeof(int), 1, f);

		if( v <= 0 )
			std::cout << "Error loading" << std::endl;

		Utils::readMatrix(mAngleHist, f);
		Utils::readMatrix(mOctaveHist, f);

		bFinalize = true;

		fclose(f);
	}

	void finalize()
	{
		if(bFinalize)
			return;

		for( int d = 0; d < mAngleHist.cols; d++ )
			mAngleHist.at<float>(d) /= mNumDescriptors;

		for( int d = 0; d < mOctaveHist.cols; d++ )
			mOctaveHist.at<float>(d) /= mNumDescriptors;


		bFinalize = true;
	}


public:
	unsigned int mDims;
	unsigned int mNumImages;
	unsigned long long mNumDescriptors;
	bool bFinalize;

	// Config
	int mSearchRange;
	float mScaleFactor;
	int mAngleBins;
	int mOctaves;

	// Angle histogramm
	cv::Mat mAngleHist;
	cv::Mat mOctaveHist;
};

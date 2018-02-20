/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/

#pragma once


#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

class Utils
{
public:
	static bool writeMatrix(const cv::Mat &mat, const std::string &path);
	static bool readMatrix(cv::Mat &mat, const std::string &path);

	static bool writeMatrix(const cv::Mat &mat, FILE *f);
	static bool readMatrix(cv::Mat &mat, FILE *f);

	static bool writeVectorOfMatrix(const std::vector<cv::Mat> &mat, FILE *f);
	static bool readVectorOfMatrix(std::vector<cv::Mat> &mat, FILE *f);

	static bool writeBitstream( const std::vector<std::vector<uchar> > &bitstream, FILE *f);
	static bool readBitstream( std::vector<std::vector<uchar> > &bitstream, FILE *f);


	static void bin2mat( cv::InputArray _src, cv::OutputArray _dst, int type = CV_8U, bool norm = false  );
	static void mat2bin( cv::InputArray _src, cv::OutputArray _dst );

	static void LoadImagesTUM(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps);

	static double getMedianFromVector(std::vector<double> scores);
	static double getMeanFromVector(std::vector<double> v);
};

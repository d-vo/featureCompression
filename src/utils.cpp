/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/


#include "utils.h"

#include <fstream>
#include <iostream>
#include <iomanip>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string.hpp"
#include <boost/lexical_cast.hpp>

#define CHECK_BIT(var,pos) ((var >> pos) & 1)


bool Utils::writeMatrix(const cv::Mat &mat, const std::string &path)
{
	FILE *f = fopen(path.c_str(), "w");
	bool ok = writeMatrix(mat, f);
	fclose(f);
	return ok;
}


bool Utils::writeMatrix(const cv::Mat &mat, FILE *f)
{
	int type = mat.type();
	if( fwrite(&type, sizeof(type), 1, f) != 1)\
			{
		printf("Cannot write matrix type\n");
		return false;
			}
	if( fwrite(&mat.rows, sizeof(mat.rows), 1, f) != 1)
	{
		printf("Cannot write matrix rows\n");
		return false;
	}
	if( fwrite(&mat.cols, sizeof(mat.cols), 1, f) != 1)
	{
		printf("Cannot write matrix cols\n");
		return false;
	}
	if( (int) fwrite(mat.data, mat.elemSize()*mat.cols, mat.rows, f) != mat.rows )
	{
		printf("Cannot write matrix\n");
		return false;
	}
	return true;
}

bool Utils::readMatrix(cv::Mat &mat, const std::string &path)
{
	FILE *f = fopen(path.c_str(), "r");
	bool ok = readMatrix(mat, f);
	fclose(f);
	return ok;
}


bool Utils::readMatrix(cv::Mat &mat, FILE *f)
{
	int type, rows, cols;
	if( fread(&type, sizeof(type), 1, f) != 1)\
			{
		printf("Cannot read matrix type\n");
		return false;
			}
	if (type == -1)
	{
		printf("No matrix written\n");
		return false;
	}

	if( fread(&rows, sizeof(rows), 1, f) != 1)
	{
		printf("Cannot read matrix rows\n");
		return false;
	}
	if( fread(&cols, sizeof(cols), 1, f) != 1)
	{
		printf("Cannot read matrix cols\n");
		return false;
	}
	mat.create(rows, cols, type);
	if( (int) fread(mat.data, mat.elemSize()*mat.cols, mat.rows, f) != mat.rows )
	{
		printf("Cannot read matrix\n");
		return false;
	}
	return true;
}


bool Utils::writeVectorOfMatrix(const std::vector<cv::Mat> &mat, FILE *f)
{
	uint64_t num = mat.size();
	if( fwrite(&num, sizeof(uint64_t), 1, f) != 1)\
			{
		printf("Cannot read number of matrix\n");
		return false;
			}


	for( uint64_t i = 0; i < num; i++)
		writeMatrix(mat[i], f);

	return true;
}

bool Utils::readVectorOfMatrix(std::vector<cv::Mat> &mat, FILE *f)
{
	uint64_t num;
	if( fread(&num, sizeof(uint64_t), 1, f) != 1)
	{
		printf("Cannot read number of matrix\n");
		return false;
	}

	mat.resize(num);
	for( uint64_t i = 0; i < num; i++)
		readMatrix(mat[i], f);

	return true;
}





bool Utils::writeBitstream( const std::vector<std::vector<uchar> > &bitstream, FILE *f)
{
	uint64_t numImages = bitstream.size();
	fwrite(&numImages, sizeof(uint64_t), 1, f);

	for( uint64_t i = 0; i < numImages; i++)
	{
		uint64_t sizeBitstream = bitstream[i].size();
		fwrite(&sizeBitstream, sizeof(uint64_t), 1, f);
		fwrite(&bitstream[i][0], sizeof(uchar), sizeBitstream, f);
	}

	return true;
}



bool Utils::readBitstream( std::vector<std::vector<uchar> > &bitstream, FILE *f )
{
	uint64_t numImages;
	uint64_t v = fread(&numImages, sizeof(uint64_t), 1, f);

	bitstream.resize(numImages);
	for( uint64_t i = 0; i < numImages; i++)
	{
		uint64_t sizeBitstream;
		v += fread(&sizeBitstream, sizeof(uint64_t), 1, f);

		bitstream[i].resize(sizeBitstream);
		v += fread(&bitstream[i][0], sizeof(uchar), sizeBitstream, f);
	}

	return v > 0;
}



void Utils::bin2mat(cv::InputArray _src, cv::OutputArray _dst, int type, bool norm )
{
	cv::Mat src = _src.getMat();
	cv::Mat &dst = _dst.getMatRef();

	cv::Mat tmp(src.rows, src.cols*8, CV_8U);
	cv::Mat test(src.rows, src.cols*8, CV_8U);


	for( int i = 0; i < src.rows; i++)
	{
		const uchar *src_ptr = src.ptr(i);
		uchar *ptr = tmp.ptr(i);

		for( int k = 0; k < src.cols; k++)
		{
			int j = k * 8;
			const uchar tmp_char = src_ptr[k * src.step[1]];

			ptr[j] 	= 	CHECK_BIT(tmp_char,7);
			ptr[j+1] = 	CHECK_BIT(tmp_char,6);
			ptr[j+2] = 	CHECK_BIT(tmp_char,5);
			ptr[j+3] = 	CHECK_BIT(tmp_char,4);
			ptr[j+4] = 	CHECK_BIT(tmp_char,3);
			ptr[j+5] = 	CHECK_BIT(tmp_char,2);
			ptr[j+6] = 	CHECK_BIT(tmp_char,1);
			ptr[j+7] = 	CHECK_BIT(tmp_char,0);
		}

	}

	dst = tmp;
	if( type != CV_8U)
		dst.convertTo(dst, type);

	if( norm && type == CV_32F )
		cv::norm(dst, dst);
}


void Utils::mat2bin(cv::InputArray _src, cv::OutputArray _dst )
{
	cv::Mat src = _src.getMat();
	cv::Mat &dst = _dst.getMatRef();

	cv::Mat tmpDst(src.rows, (int) (src.cols/8 + 0.5), CV_8U, cv::Scalar::all(0));

	for( int i = 0; i < src.rows; i++)
	{
		uchar *rowPtr = tmpDst.ptr(i);
		for( int k = 0; k < src.cols; k += 8)
		{
			const int byte = k/8;
			if( src.at<uchar>(i,k) > 0)
				rowPtr[byte] |= (1 << 7);

			if( src.at<uchar>(i,k+1) > 0)
				rowPtr[byte] |= (1 << 6);

			if( src.at<uchar>(i,k+2) > 0)
				rowPtr[byte] |= (1 << 5);

			if( src.at<uchar>(i,k+3) > 0)
				rowPtr[byte] |= (1 << 4);

			if( src.at<uchar>(i,k+4) > 0)
				rowPtr[byte] |= (1 << 3);

			if( src.at<uchar>(i,k+5) > 0)
				rowPtr[byte] |= (1 << 2);

			if( src.at<uchar>(i,k+6) > 0)
				rowPtr[byte] |= (1 << 1);

			if( src.at<uchar>(i,k+7) > 0)
				rowPtr[byte] |= (1 << 0);
		}
	}
	dst = tmpDst;
}



void Utils::LoadImagesTUM(const std::string &full_path, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
	std::string strFile = full_path + "rgb.txt";

	std::ifstream f;
	f.open(strFile.c_str());


	// skip first three lines
	std::string s0;
	getline(f,s0);
	getline(f,s0);
	getline(f,s0);

	while(!f.eof())
	{
		std::string s;
		getline(f,s);
		if(!s.empty())
		{
			std::stringstream ss;
			ss << s;
			double t;
			std::string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			std::string path = full_path + "/" + sRGB;
			vstrImageFilenames.push_back(path);
		}
	}
}



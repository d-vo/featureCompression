/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/

#pragma once

#include <opencv2/opencv.hpp>

#include <list>
#include <queue>

#include "ac_extended.h"
#include "utils.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2//DBoW2/TemplatedVocabulary.h"
#include "voc_stats.h"
#include "ImgBufferEntry.h"




typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;


enum CodingMode {
	INTRA = 0,
	INTER = 1,
	INTER_SKIP = 2,
	UNUSED = 3,
	NONE = -1
};


// Signaling
struct EncodeInfo
{
	uint32_t numFeatures;
	uint32_t numFeaturesRight;
	uint32_t fixedLengthSize;
};


struct Candidate
{
	bool skipMode = false;
	int candidateId = -1;
	int numCandidates = 0;
	int keypointId = -1;
	int imageId = -1;
	cv::Mat residual;
};


struct ModeDecision
{
	CodingMode mode = CodingMode::NONE;
	unsigned int visualWord;
	int nodeId = -1;
	Candidate candidate;
	cv::Mat residual;
	float rate;
	int keypointId = -1;
	int octave = -1;
	float r;

	bool operator < (const ModeDecision& str) const
	{
			return r > str.r;
	}
};


struct ACEncodeContext
{
	ACEncodeContext()
	{
		ac_encoder_init (&ace, bitstream);
	}

	~ACEncodeContext()
	{
		// Intra
		ac_model_done(&acm_bow);
		ac_model_done(&acm_intra_desc);
		ac_model_done(&acm_intra_angle);
		ac_model_done(&acm_intra_octave);

		for( auto &model : v_acm_intra_kpt_x )
			ac_model_done(&model);
		for( auto &model : v_acm_intra_kpt_y )
			ac_model_done(&model);

		// Inter
		ac_model_done(&acm_inter_desc);
		ac_model_done(&acm_inter_angle);
		ac_model_done(&acm_inter_octave);
		ac_model_done(&acm_inter_candidate);
		for( auto &model : v_acm_inter_kpt )
			ac_model_done(&model);



		finish();
	}

	void finish()
	{
		ac_encoder_done(&ace);
	}

	void clear()
	{
		bitstream.clear();
		ac_encoder_init (&ace, bitstream);
	}

	size_t bits()
	{
		return ace.total_bits;
	}

	// Setup encoder for descriptors
	ac_encoder ace;

	// Intra
	ac_model   acm_bow;
	ac_model   acm_bow_ten;
	ac_model   acm_intra_desc;
	ac_model   acm_intra_angle;
	ac_model   acm_intra_octave;
	std::vector<ac_model>   v_acm_intra_kpt_x;
	std::vector<ac_model>   v_acm_intra_kpt_y;

	// Inter
	ac_model   acm_inter_desc;
	ac_model   acm_inter_angle;
	ac_model   acm_inter_octave;
	ac_model   acm_inter_candidate;
	std::vector<ac_model>   v_acm_inter_kpt;


	vector<uchar> bitstream;
};

struct EncodeContext
{
	EncodeContext(){};
	~EncodeContext()
	{
		finish();
	}

	size_t bits()
	{
		return (bitstream.size() * 8 + 7-bit_idx);
	}


	void finish()
	{
		// append the remaining bits, if any
		if( bit_idx!=7 ){
			bitstream.push_back(buffer);
		}
	}

	void clear()
	{
		cur_byte = 0;
		bit_idx = 7;
		cur_bit = 0;
		buffer = 0;
		bitstream.clear();
	}

	uchar cur_byte = 0;
	int bit_idx = 7;
	int cur_bit = 0;
	uchar buffer = 0;
	vector<uchar> bitstream;
};

struct ACDecodeContext
{
	ACDecodeContext(){};
	~ACDecodeContext()
	{
		// Intra
		ac_model_done(&acm_bow);
		ac_model_done(&acm_intra_desc);
		ac_model_done(&acm_intra_angle);
		ac_model_done(&acm_intra_octave);

		for( auto &model : v_acm_intra_kpt_x )
			ac_model_done(&model);
		for( auto &model : v_acm_intra_kpt_y )
			ac_model_done(&model);

		// Inter
		ac_model_done(&acm_inter_desc);
		ac_model_done(&acm_inter_angle);
		ac_model_done(&acm_inter_octave);
		ac_model_done(&acm_inter_candidate);
		for( auto &model : v_acm_inter_kpt )
			ac_model_done(&model);

		ac_decoder_done(&acd);
	}

	void setBitstream( std::list<uchar> &_bitstream )
	{
		ac_decoder_init (&acd, _bitstream);
	}


	// If arithmetic coder
	ac_decoder acd;

	// Intra
	ac_model   acm_bow;
	ac_model   acm_intra_desc;
	ac_model   acm_intra_angle;
	ac_model   acm_intra_octave;
	std::vector<ac_model>   v_acm_intra_kpt_x;
	std::vector<ac_model>   v_acm_intra_kpt_y;

	// Inter
	ac_model   acm_inter_desc;
	ac_model   acm_inter_angle;
	ac_model   acm_inter_octave;
	ac_model   acm_inter_candidate;
	std::vector<ac_model>   v_acm_inter_kpt;
};


struct DecodeContext
{
	void clear()
	{
		cur_byte = 0;
		byte_idx = 0;
		bit_idx = -1;
		cur_bit = 0;
		bitstream.clear();
	}

	uchar cur_byte = 0;
	int byte_idx = 0;
	int bit_idx = -1;
	int cur_bit = 0;

	vector<uchar> bitstream;
};



class FeatureCoder
{
public:
	FeatureCoder( ORBVocabulary &voc, CodingStats &model, int imWidth, int imHeight, int maxOctave, int angleBins,
			int bufferSize, bool inter = true);

	// Coder control
	void setMaxT(double ms);
	void setMaxR(double bits);
	void setMaxN(int N);
	void setPriorForInit(bool init);

	void initEncoderModels( ACEncodeContext &globalACCoderContext );
	void initDecoderModels( ACDecodeContext &globalACDecoderContext );

	unsigned int encodeFeature(const ModeDecision &decision, const cv::KeyPoint &keypoints, const cv::Mat &descriptor,
			EncodeContext &globalCoderContext, ACEncodeContext &globalACCoderContext);

	void decodeFeature(cv::KeyPoint &keypoints, cv::Mat &descriptor, unsigned int &visualWord, DecodeContext &globalDecoderContext, ACDecodeContext &globalACDecoderContext);

	void encodeImage( std::vector<cv::KeyPoint> &kpts, const cv::Mat &descriptor, vector<uchar> &bitstream );
	void decodeImage( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &kpts, cv::Mat &descriptor, std::vector<unsigned int> &visualWords );

	// -- Mode decision --
	float intraCosts( const cv::KeyPoint &currentKpt, const cv::Mat &descriptor, unsigned int &visualWord, cv::Mat &intraResidualMat);
	float interCandidateSelection( const cv::KeyPoint &currentKpt, const cv::Mat &descriptor, Candidate &cand);

	ModeDecision modeDecision( const cv::KeyPoint &keypoint, const cv::Mat &descriptor);

	// -- MODE CODING --
	size_t encodeMode(const CodingMode &mode,  EncodeContext &ctxt);
	size_t decodeMode(DecodeContext &modeCtxt, CodingMode &mode);

	size_t encodeSkipMode(int nMode,  EncodeContext &ctxt);
	int decodeSkipMode(DecodeContext &ctxt);


	// -- INTRA CODING --
	size_t IntraEncodeBow(unsigned int visualWord, EncodeContext &bowCtxt);
	void IntraDecodeBow(DecodeContext &bowCtxt, unsigned int &visualWord );

	size_t IntraEncodeBowAC(unsigned int visualWord, ACEncodeContext &bowCtxt);
	void IntraDecodeBowAC(ACDecodeContext &bowCtxt, unsigned int &visualWord );

	size_t IntraEncodeKeyPoint(const cv::KeyPoint &keypoint, EncodeContext &kptCtxt);
	void IntraDecodeKeyPoint(DecodeContext &kptCtxt, cv::KeyPoint &keypoint);


	size_t IntraEncodeKeyPointAC(const cv::KeyPoint &keypoint, ACEncodeContext &accontext);
	void IntraDecodeKeyPointAC(ACDecodeContext &kptCtxt, cv::KeyPoint &keypoint);

	size_t IntraEncodeResidual(const cv::Mat &residual, ACEncodeContext &resCtxt);
	void IntraDecodeResidual(ACDecodeContext &resCtxt, cv::Mat &residual);
	cv::Mat IntraReconstructDescriptor(const unsigned int &visualWord, cv::Mat &residual);

	// -- REFERENCE CODING --
	size_t encodeReference(int reference, int numCandidates, EncodeContext &ctxt);
	int decodeReference(DecodeContext &kptCtxt, int numCandidates);


	// -- INTER CODING --
	size_t InterEncodeReferenceAC(int reference, ACEncodeContext &accontext);
	int InterDecodeReferenceAC(ACDecodeContext &accontext);


	size_t InterEncodeKeypoint(const cv::KeyPoint &refKeypoint, const cv::KeyPoint &currentKeypoint, ACEncodeContext &accontext, EncodeContext &context);
	void InterDecodeKeypoint(ACDecodeContext &accontext, DecodeContext &context, const cv::KeyPoint &refKeypoint, cv::KeyPoint &currentKeypoint);

	size_t InterEncodeResidual(const cv::Mat &residual, ACEncodeContext &accontext);
	void InterDecodeResidual(ACDecodeContext &accontext, cv::Mat&residual);

	cv::Mat InterReconstructDescriptor(const cv::Mat &referenceDescriptor, const cv::Mat &residual);

	void KeyPointDiffToIndex(int dx, int dy, int octave, int &index);
	void IndexToKeyPointDiff(int index, int octave, int &x, int &y);


	int GetNumInterCandidates();

	// Fake coding to keep encoder and decoder in sync
	cv::KeyPoint fakeCode(const cv::KeyPoint &keyPoint);

private:
	static long long imageId;

	CodingStats &mModel;
	ORBVocabulary &mVoc;

	int mnAngleOffset;
	int mnOctaveOffset;


	// Intra
	std::vector<int> mFreqIntraRes;
	std::vector<int> mFreqIntraBow;
	std::vector<cv::Mat> mFreqIntraPosX;
	std::vector<cv::Mat> mFreqIntraPosY;
	std::vector<float> mLutRIntra;
	std::vector<int> mFreqIntraOctave;
	std::vector<int> mFreqIntraAngle;



	// Inter
	std::vector<int> mFreqInterRes;
	std::vector<int> mFreqInterAngleDiff;
	std::vector<int> mFreqInterOctaveDiff;
	std::vector<int> mFreqInterCandidate;
	std::vector<cv::Mat> mFreqInterKeyPoint;
	std::vector<cv::Mat> mPInterKeyPoint;
	std::vector<float> mLutRInter;


	std::vector<float> mScaleFactors;
	std::vector<int> mvPyramidWidth;
	std::vector<int> mvPyramidHeight;

	std::vector<float> mvBitsPyramidWidth;
	std::vector<float> mvBitsPyramidHeight;



	// Feature settings
	int mImWidth;
	int mImHeight;
	int mLevels;
	int mAngleBins;
	float mAngleBinSize;
	unsigned int mBufferSize;
	unsigned int mInterReferenceImages;


	float mnBitsAngle;
	float mnBitsOctave;
	float mnBitsBow;

	bool bInterPred;

	double mMaxT;
	double mMaxR;
	unsigned int mMaxN;
	bool mbInit;

	double mtFinish;


	// Current image
	unsigned long mCurrentImageId;
	bool mbCurrentViewRight;

	EncodeContext mGlobalCoderContext;
	ACEncodeContext mGlobalACCoderContext;

	DecodeContext mGlobalDecoderContext;
	ACDecodeContext mGlobalACDecoderContext;

	// Buffer
	ImgBufferEntry mCurrentImageBuffer;
	std::list<ImgBufferEntry> mLeftImageBuffer;
};

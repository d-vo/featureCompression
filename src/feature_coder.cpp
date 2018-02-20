/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/


#include "feature_coder.h"
#include <chrono>

//#define DO_DEBUG
//#define ANDROID

#define MAX_NUM_REF_FEAT 64
#define MAX_NUM_FEATURES 2000

long long FeatureCoder::imageId = 0;

FeatureCoder::FeatureCoder(ORBVocabulary &voc, CodingStats &model, int imWidth, int imHeight, int maxOctave, int angleBins,
		int bufferSize, bool inter)
: mModel(model), mVoc(voc), mImWidth(imWidth), mImHeight(imHeight), mLevels(maxOctave), mAngleBins(angleBins),
  mBufferSize(bufferSize), bInterPred(inter)
{
	mMaxT = std::numeric_limits<double>::max();
	mMaxR = std::numeric_limits<double>::max();
	mMaxN = std::numeric_limits<int>::max();


	mbInit = false;
	mAngleBinSize = 360.0 / mAngleBins;

	// Intra coding
	mFreqIntraRes.resize(2);
	mFreqIntraRes[0] = (int)max(1, (int)round( mModel.p0_intra_ * (double)AC_PRECISION ) );
	mFreqIntraRes[1] = AC_PRECISION - mFreqIntraRes[0];

	mFreqInterAngleDiff.resize(mModel.pAngleDelta_.cols);
	for( int d = 0; d < mModel.pAngleDelta_.cols; d++ )
	{
		const float &prob = mModel.pAngleDelta_.at<float>(d);
		mFreqInterAngleDiff[d] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
	}


	mFreqInterOctaveDiff.resize(mModel.pOctaveDelta_.cols);
	for( int d = 0; d < mModel.pOctaveDelta_.cols; d++ )
	{
		float prob = mModel.pOctaveDelta_.at<float>(d);
		mFreqInterOctaveDiff[d] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
	}

	// Pyramid sizes for intra coding
	mvPyramidWidth.resize(mModel.mOctaves);
	mvPyramidHeight.resize(mModel.mOctaves);
	mvPyramidWidth[0] = mImWidth;
	mvPyramidHeight[0] = mImHeight;

	// Number of bits for intra coding
	mvBitsPyramidWidth.resize(mModel.mOctaves);
	mvBitsPyramidHeight.resize(mModel.mOctaves);
	mvBitsPyramidWidth[0] = log2(mvPyramidWidth[0]);
	mvBitsPyramidHeight[0] = log2(mvPyramidHeight[0]);

	mScaleFactors.resize(mModel.mOctaves);
	mScaleFactors[0] = 1.0;


	for( int d = 1; d < mModel.mOctaves; d++ )
	{
		mScaleFactors[d] = mScaleFactors[d-1]*mModel.mScaleFactor;
		mvPyramidWidth[d] = ceil(((float) mImWidth) / mScaleFactors[d]);
		mvPyramidHeight[d] = ceil(((float) mImHeight) / mScaleFactors[d]);

		mvBitsPyramidWidth[d] = log2(mvPyramidWidth[d]);
		mvBitsPyramidHeight[d] = log2(mvPyramidHeight[d]);
	}

	mnBitsOctave = log2(mLevels);
	mnBitsAngle = log2(mAngleBins);
	mnBitsBow = log2(mVoc.size());

	mnAngleOffset = mModel.mAngleBins-1;
	mnOctaveOffset =  mModel.mOctaves-1;


	size_t voc_size = voc.size();
	mFreqIntraBow.resize(voc_size);
	for( size_t i = 0; i < voc_size; i++ )
	{
		const double prob = 1.0 / voc_size;
		mFreqIntraBow[i] = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
	}


	mFreqIntraPosX.resize(mModel.mOctaves);
	mFreqIntraPosY.resize(mModel.mOctaves);
	for( int d = 0; d < mModel.mOctaves; d++ )
	{
		mFreqIntraPosX[d] = cv::Mat(1, mvPyramidWidth[d], CV_32S, cv::Scalar::all(1 ));
		mFreqIntraPosY[d] = cv::Mat(1, mvPyramidHeight[d], CV_32S, cv::Scalar::all(1 ));
	}

	mFreqIntraOctave.resize(mLevels);
	std::fill(mFreqIntraOctave.begin(), mFreqIntraOctave.end(), 1);

	mFreqIntraAngle.resize(mAngleBins);
	std::fill(mFreqIntraAngle.begin(), mFreqIntraAngle.end(), 1);



	// Inter coding
	mFreqInterRes.resize(2);
	mFreqInterRes[0] = (int)max(1, (int)round( mModel.p0_inter_ * (double)AC_PRECISION ) );
	mFreqInterRes[1] = AC_PRECISION - mFreqInterRes[0];



	// Inter coding
	mFreqInterKeyPoint.resize(mModel.mOctaves);
	mPInterKeyPoint.resize(mModel.mOctaves);

	for( int d = 0; d < mModel.mOctaves; d++ )
	{
		mPInterKeyPoint[d] = mModel.pPosPerOctave_[d];
		mFreqInterKeyPoint[d] = cv::Mat(mPInterKeyPoint[d].rows, mPInterKeyPoint[d].cols, CV_32S, cv::Scalar::all(0));
		for( int x = 0; x <  mModel.pPosPerOctave_[d].cols; x++)
		{
			for( int y = 0; y <  mModel.pPosPerOctave_[d].rows; y++)
			{
				float prob = mPInterKeyPoint[d].at<float>(y,x);
				mFreqInterKeyPoint[d].at<int>(y,x) = (int)max(1, (int)round( prob * (double)AC_PRECISION ) );
			}
		}
	}



	mLutRIntra.resize(257);
	mLutRInter.resize(257);
	for( int d = 0; d < 257; d++ )
	{
		mLutRIntra[d] =  -((float)(256-d)) * log2(mModel.p0_intra_) - ((float) d) * log2(1.0 - mModel.p0_intra_);
		mLutRInter[d] =  -((float)(256-d)) * log2(mModel.p0_inter_) - ((float) d) * log2(1.0 - mModel.p0_inter_);
	}

	mCurrentImageId = 0;

	mtFinish = 1.0;


	initEncoderModels(mGlobalACCoderContext);
	initDecoderModels(mGlobalACDecoderContext);
}


// Coder control
void FeatureCoder::setMaxT(double ms)
{
	mMaxT = ms;
}

void FeatureCoder::setMaxR(double bits)
{
	mMaxR = bits;
}

void FeatureCoder::setMaxN(int N)
{
	mMaxN = N;
}

void FeatureCoder::setPriorForInit(bool init)
{
	mbInit = init;
}

unsigned int FeatureCoder::encodeFeature(const ModeDecision &decision, const cv::KeyPoint &keypoint, const cv::Mat &descriptor,
		EncodeContext &globalCoderContext, ACEncodeContext &globalACCoderContext)
{
	const unsigned int bits_start = globalCoderContext.bits() + globalACCoderContext.bits();


	// Encode mode
	encodeMode(decision.mode, globalCoderContext);

	// Intra coding
	if( decision.mode == CodingMode::INTRA)
	{
		// Encode
		{
			// Encode Global
#ifdef ANDROID
			stats.intraEncStats.bitsBow += IntraEncodeBow(decision.visualWord, globalCoderContext);
			stats.intraEncStats.bitsKeypoints += IntraEncodeKeyPoint(keypoint, globalCoderContext);
#else
			IntraEncodeBowAC(decision.visualWord, globalACCoderContext);
			IntraEncodeKeyPointAC(keypoint, globalACCoderContext);
#endif
			IntraEncodeResidual(decision.residual, globalACCoderContext);


			const cv::KeyPoint &decKeypoint = fakeCode(keypoint);
			mCurrentImageBuffer.addFeature(decKeypoint, descriptor);
		}
	}
	else if( decision.mode == CodingMode::INTER )
	{
		{
			// Encode global
			const int &referenceId = decision.candidate.candidateId;


			// Keypoint
			std::list<ImgBufferEntry>::const_iterator it;
			it = mLeftImageBuffer.begin();

			std::advance(it, decision.candidate.imageId);
			const cv::KeyPoint &refKeypoint = it->keypoints[decision.candidate.keypointId];

#ifdef ANDROID
			encodeReference(referenceId, mInterReferenceImages, globalCoderContext);
#else
			InterEncodeReferenceAC(referenceId, globalACCoderContext);
#endif
			InterEncodeKeypoint(refKeypoint, keypoint, globalACCoderContext, globalCoderContext);
			InterEncodeResidual(decision.residual, globalACCoderContext);

			const cv::KeyPoint &decKeypoint = fakeCode(keypoint);
			mCurrentImageBuffer.addFeature(decKeypoint, descriptor);
		}
	}
	else if( decision.mode == CodingMode::INTER_SKIP )
	{
		{
			// Encode global
			const int &referenceId = decision.candidate.candidateId;


			// Keypoint
			std::list<ImgBufferEntry>::const_iterator it;
			it = mLeftImageBuffer.begin();


			std::advance(it, decision.candidate.imageId);
			const cv::KeyPoint &refKeypoint = it->keypoints[decision.candidate.keypointId];

			InterEncodeReferenceAC(referenceId, globalACCoderContext);

			const cv::KeyPoint &decKeypoint = refKeypoint;
			mCurrentImageBuffer.addFeature(decKeypoint, it->descriptors.row(decision.candidate.keypointId));
		}
	}

	const unsigned int bits_end = globalCoderContext.bits() + globalACCoderContext.bits();


	return bits_end - bits_start;
}


void FeatureCoder::decodeFeature(cv::KeyPoint &decKeypoint, cv::Mat &recDescriptor, unsigned int &visualWord,
		DecodeContext &globalDecoderContext, ACDecodeContext &globalACDecoderContext)
{

	CodingMode mode;
	decodeMode(globalDecoderContext, mode);
	if( mode == CodingMode::INTRA)
	{
		// Decode
		cv::Mat residual;
#ifdef ANDROID
		IntraDecodeBow(globalDecoderContext, visualWord);
		IntraDecodeKeyPoint(globalDecoderContext, decKeypoint);
#else
		IntraDecodeBowAC(globalACDecoderContext, visualWord);
		IntraDecodeKeyPointAC(globalACDecoderContext, decKeypoint);
#endif

		IntraDecodeResidual(globalACDecoderContext, residual);
		recDescriptor = IntraReconstructDescriptor(visualWord, residual);

		decKeypoint.class_id = CodingMode::INTRA;
	}
	else if( mode == CodingMode::INTER )
	{
		// Decode
		cv::Mat residual;
#ifdef ANDROID
		const int recReferenceId = decodeReference(globalDecoderContext, mInterReferenceImages);
#else
		const int recReferenceId = InterDecodeReferenceAC(globalACDecoderContext);
#endif
		// Has to  be N-Sync with  the encoder
		std::list<ImgBufferEntry>::const_iterator it, itEnd;
		it = mLeftImageBuffer.begin();
		itEnd = mLeftImageBuffer.end();


		int keypointId = -1;
		int numKeypoints = 0;
		for(; it != itEnd; it++ )
		{
			if( recReferenceId >= numKeypoints && recReferenceId < numKeypoints + (int) it->keypoints.size())
			{
				keypointId = recReferenceId - numKeypoints;
				break;
			}

			numKeypoints += it->keypoints.size();
		}


		const cv::KeyPoint &recRefKeypoint = it->keypoints[keypointId];
		const cv::Mat &recRefDescriptor =  it->descriptors.row(keypointId);


		InterDecodeKeypoint(globalACDecoderContext, globalDecoderContext, recRefKeypoint, decKeypoint);
		InterDecodeResidual(globalACDecoderContext, residual);
		recDescriptor = InterReconstructDescriptor(recRefDescriptor, residual);


		decKeypoint.class_id = CodingMode::INTER;
		mVoc.transform(recDescriptor, visualWord);
	}
	else if( mode == CodingMode::INTER_SKIP )
	{
		// Decode
		cv::Mat residual;
		const int recReferenceId = InterDecodeReferenceAC(globalACDecoderContext);


		// Has to  be N-Sync with  the encoder
		std::list<ImgBufferEntry>::const_iterator it, itEnd;
		it = mLeftImageBuffer.begin();
		itEnd = mLeftImageBuffer.end();


		int keypointId = -1;
		int numKeypoints = 0;
		for(; it != itEnd; it++ )
		{
			if( recReferenceId >= numKeypoints && recReferenceId < numKeypoints + (int) it->keypoints.size())
			{
				keypointId = recReferenceId - numKeypoints;
				break;
			}

			numKeypoints += it->keypoints.size();
		}


		const cv::KeyPoint &recRefKeypoint = it->keypoints[keypointId];
		cv::Mat recRefDescriptor =  it->descriptors.row(keypointId);

		decKeypoint = recRefKeypoint;
		recDescriptor = recRefDescriptor;

		decKeypoint.class_id = CodingMode::INTER_SKIP;
		mVoc.transform(recDescriptor, visualWord);
	}

}



void FeatureCoder::initEncoderModels( ACEncodeContext &accontext )
{
	// Init models

	// Intra
	ac_model_init (&accontext.acm_bow, mVoc.size(), &mFreqIntraBow[0], 0);
	ac_model_init (&accontext.acm_intra_desc, 2, &mFreqIntraRes[0], 0);
	ac_model_init (&accontext.acm_intra_angle, mAngleBins, &mFreqIntraAngle[0], 0);
	ac_model_init (&accontext.acm_intra_octave, mLevels, &mFreqIntraOctave[0], 0);

	accontext.v_acm_intra_kpt_x.resize(mLevels);
	accontext.v_acm_intra_kpt_y.resize(mLevels);

	for( int octave = 0; octave < mLevels; octave++ )
	{
		ac_model_init (&accontext.v_acm_intra_kpt_x[octave], mFreqIntraPosX[octave].cols, (int *) mFreqIntraPosX[octave].data, 0);
		ac_model_init (&accontext.v_acm_intra_kpt_y[octave], mFreqIntraPosY[octave].cols, (int *) mFreqIntraPosY[octave].data, 0);
	}


	// Inter
	ac_model_init (&accontext.acm_inter_desc, 2, &mFreqInterRes[0], 0);
	ac_model_init (&accontext.acm_inter_angle, mFreqInterAngleDiff.size(), (int *) &mFreqInterAngleDiff[0], 0);
	ac_model_init (&accontext.acm_inter_octave, mFreqInterOctaveDiff.size(), (int *) &mFreqInterOctaveDiff[0], 0);

	accontext.v_acm_inter_kpt.resize(mLevels);
	for( int octave = 0; octave < mLevels; octave++ )
	{
		const int inter_range = mFreqInterKeyPoint[octave].rows*mFreqInterKeyPoint[octave].cols;
		ac_model_init (&accontext.v_acm_inter_kpt[octave], inter_range, (int *) mFreqInterKeyPoint[octave].data, 0);
	}



	const int numInterCandidates = mBufferSize*MAX_NUM_FEATURES;
	mFreqInterCandidate.resize(numInterCandidates);
	std::fill(mFreqInterCandidate.begin(), mFreqInterCandidate.end(), 1);
	ac_model_init (&accontext.acm_inter_candidate, numInterCandidates, (int *) &mFreqInterCandidate[0], 0);
}



void FeatureCoder::initDecoderModels( ACDecodeContext &accontext )
{
	// Intra
	ac_model_init (&accontext.acm_bow, mVoc.size(), &mFreqIntraBow[0], 0);
	ac_model_init (&accontext.acm_intra_desc, 2, &mFreqIntraRes[0], 0);
	ac_model_init (&accontext.acm_intra_angle, mAngleBins, &mFreqIntraAngle[0], 0);
	ac_model_init (&accontext.acm_intra_octave, mLevels, &mFreqIntraOctave[0], 0);

	accontext.v_acm_intra_kpt_x.resize(mLevels);
	accontext.v_acm_intra_kpt_y.resize(mLevels);

	for( int octave = 0; octave < mLevels; octave++ )
	{
		ac_model_init (&accontext.v_acm_intra_kpt_x[octave], mFreqIntraPosX[octave].cols, (int *) mFreqIntraPosX[octave].data, 0);
		ac_model_init (&accontext.v_acm_intra_kpt_y[octave], mFreqIntraPosY[octave].cols, (int *) mFreqIntraPosY[octave].data, 0);
	}


	// Inter
	ac_model_init (&accontext.acm_inter_desc, 2, &mFreqInterRes[0], 0);
	ac_model_init (&accontext.acm_inter_angle, mFreqInterAngleDiff.size(), (int *) &mFreqInterAngleDiff[0], 0);
	ac_model_init (&accontext.acm_inter_octave, mFreqInterOctaveDiff.size(), (int *) &mFreqInterOctaveDiff[0], 0);

	accontext.v_acm_inter_kpt.resize(mLevels);
	for( int octave = 0; octave < mLevels; octave++ )
	{
		const int inter_range = mFreqInterKeyPoint[octave].rows*mFreqInterKeyPoint[octave].cols;
		ac_model_init (&accontext.v_acm_inter_kpt[octave], inter_range, (int *) mFreqInterKeyPoint[octave].data, 0);
	}


	const int numInterCandidates = mBufferSize*MAX_NUM_FEATURES;
	mFreqInterCandidate.resize(numInterCandidates);
	std::fill(mFreqInterCandidate.begin(), mFreqInterCandidate.end(), 1);
	ac_model_init (&accontext.acm_inter_candidate, numInterCandidates, (int *) &mFreqInterCandidate[0], 0);
}



void FeatureCoder::encodeImage( std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors, vector<uchar> &bitstream )
{
	const auto t1 = chrono::high_resolution_clock::now();

	bitstream.clear();
	mbCurrentViewRight = false;

	// Reset buffer
	mCurrentImageBuffer = ImgBufferEntry(mImWidth, mImHeight, mLevels);
	mCurrentImageBuffer.mnImageId = imageId++;

	// Prepare coder
	EncodeContext &globalCoderContext = mGlobalCoderContext;
	ACEncodeContext &globalACCoderContext = mGlobalACCoderContext;


	// Pre-calculate decisions
	std::vector<ModeDecision> decisions(keypoints.size());
	#pragma omp parallel for
	for( size_t i = 0; i < keypoints.size(); i++ )
	{
		decisions[i] = modeDecision(keypoints[i], descriptors.row(i));
		decisions[i].keypointId = i;
	}

	std::sort(decisions.begin(), decisions.end());


	// Encode
	const unsigned int signalingBits = 350;
	unsigned int nCoded = 0;
	unsigned int R_bits = 0.0;
	unsigned int R_bits_exp = 0.0;
	for( size_t i = 0; i < decisions.size(); i++ )
	{
		// Break if time limit reached
		double t_now = std::chrono::duration<double, milli> (chrono::high_resolution_clock::now()-t1).count();

		if( t_now >= mMaxT-mtFinish-0.8)
			break;

		// Break if rate limit reached
		if( R_bits + signalingBits >= mMaxR )
			break;

		if( i >= mMaxN )
			break;


		// Let's go
		const ModeDecision &decision = decisions[i];
		const int &kptId = decision.keypointId;
		const cv::KeyPoint &keypoint = keypoints[kptId];
		const cv::Mat &descriptor = descriptors.row(kptId);

		const unsigned int R = encodeFeature(decision, keypoint, descriptor, globalCoderContext, globalACCoderContext);

		R_bits += R;
		R_bits_exp += decision.rate + 2;

		nCoded++;
	}

	const auto t_finish_start = chrono::high_resolution_clock::now();

	globalCoderContext.finish();
	globalACCoderContext.finish();

	bitstream.reserve(sizeof(EncodeInfo) + globalCoderContext.bitstream.size() + globalACCoderContext.bitstream.size());
	bitstream.resize(sizeof(EncodeInfo));
	EncodeInfo *info = (EncodeInfo *) &bitstream[0];
	info->numFeatures = mCurrentImageBuffer.keypoints.size();
	info->fixedLengthSize = globalCoderContext.bitstream.size();

	bitstream.insert(bitstream.end(), globalCoderContext.bitstream.begin(), globalCoderContext.bitstream.end());
	bitstream.insert(bitstream.end(), globalACCoderContext.bitstream.begin(), globalACCoderContext.bitstream.end());


	globalCoderContext.clear();
	globalACCoderContext.clear();


	mCurrentImageBuffer.AssignFeatures();
	mLeftImageBuffer.push_back(mCurrentImageBuffer);
	if( mLeftImageBuffer.size() > mBufferSize)
		mLeftImageBuffer.pop_front();


	mCurrentImageId++;

	const auto t_finish_end = chrono::high_resolution_clock::now();
	mtFinish = std::chrono::duration<double, milli> (t_finish_end-t_finish_start).count();
}


void FeatureCoder::decodeImage( const vector<uchar> &bitstream, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
		std::vector<unsigned int> &visualWords )
{
	EncodeInfo *info = (EncodeInfo *) &bitstream[0];
	const int numFeatures = info->numFeatures;


	size_t offset= sizeof(EncodeInfo);
	size_t offsetKeypointPos = offset + info->fixedLengthSize;

	mbCurrentViewRight = false;

	// Split bitstream
	vector<uchar> flBitstream(bitstream.begin() + offset, bitstream.begin() + offsetKeypointPos);
	list<uchar> acBitstream(bitstream.begin() + offsetKeypointPos, bitstream.end());

	// Prepare coder
	DecodeContext &globalDecoderContext = mGlobalDecoderContext;
	ACDecodeContext &globalACDecoderContext = mGlobalACDecoderContext;

	globalACDecoderContext.setBitstream(acBitstream);
	globalDecoderContext.clear();
	globalDecoderContext.bitstream = flBitstream;

	mCurrentImageBuffer = ImgBufferEntry(mImWidth, mImHeight, mLevels );
	mCurrentImageBuffer.mnImageId = imageId++;

	for( int i = 0; i < numFeatures; i++ )
	{
		unsigned int visualWord = 0;
		cv::KeyPoint keypoint;
		cv::Mat descriptor;

		decodeFeature(keypoint, descriptor, visualWord, globalDecoderContext, globalACDecoderContext);
		mCurrentImageBuffer.addFeature(keypoint, descriptor, visualWord);
	}

	mCurrentImageBuffer.AssignFeatures();


	descriptors = mCurrentImageBuffer.descriptors;
	keypoints = mCurrentImageBuffer.keypoints;
	visualWords = mCurrentImageBuffer.visualWords;



	mLeftImageBuffer.push_back(mCurrentImageBuffer);
	if( mLeftImageBuffer.size() > mBufferSize)
		mLeftImageBuffer.pop_front();
}



float FeatureCoder::intraCosts( const cv::KeyPoint &currentKpt, const cv::Mat &descriptor, unsigned int &visualWord, cv::Mat &intraResidualMat)
{
	mVoc.transform(descriptor, visualWord);
	const cv::Mat &visualWordDesc = mVoc.getWord(visualWord);

	cv::bitwise_xor(visualWordDesc, descriptor, intraResidualMat);
	const int d = cv::norm(intraResidualMat, cv::NORM_HAMMING);
	float R_intra_res = mLutRIntra[d];

	const int &octave = currentKpt.octave;
	const float &nbits_x = mvBitsPyramidWidth[octave];
	const float &nbits_y = mvBitsPyramidHeight[octave];
	const float R_intra_kpt = mnBitsAngle + mnBitsOctave + nbits_x + nbits_y;

	return mnBitsBow + R_intra_res + R_intra_kpt;
}


float FeatureCoder::interCandidateSelection( const cv::KeyPoint &currentKpt, const cv::Mat &descriptor, Candidate &cand)
{
	// Search best reference keypoint
	std::list<ImgBufferEntry>::iterator it;
	std::list<ImgBufferEntry>::iterator itEnd;
	it = mLeftImageBuffer.begin();
	itEnd = mLeftImageBuffer.end();


	std::list<ImgBufferEntry>::iterator bestIt;
	float bestR = std::numeric_limits<float>::max();
	int bestIdx = -1;
	int bestImg = -1;
	int bestReference = -1;

	int img = 0;
	const int numCandidates = GetNumInterCandidates();
	const float R_inter_ref = ceil(log2(numCandidates));

	unsigned int candidatesCount = 0;
	for( ; it != itEnd; it++ )
	{
		const std::vector<unsigned int> &vIndices = it->GetFeaturesInArea(currentKpt.pt.x, currentKpt.pt.y, mModel.mSearchRange, currentKpt.octave-1, currentKpt.octave+1);
		for( size_t p = 0; p < vIndices.size(); p++ )
		{
			cv::Mat residual;
			cv::bitwise_xor(it->descriptors.row(vIndices[p]), descriptor, residual);
			const int dist = cv::norm(residual, cv::NORM_HAMMING);



			const cv::KeyPoint &refKeypoint = it->keypoints[vIndices[p]];
			const int &octave = currentKpt.octave;

			int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
			int curAngleBin = floor(currentKpt.angle / mAngleBinSize);


			int angleDiff = curAngleBin - refAngleBin + mnAngleOffset;
			assert( angleDiff >= 0 && angleDiff< mModel.pAngleDelta_.cols);

			float R_angleDiff = -log2(mModel.pAngleDelta_.at<float>(angleDiff));


			// Octave coding
			int octaveDiff = currentKpt.octave - refKeypoint.octave + mnOctaveOffset;
			float R_inter_octave = -log2(mModel.pOctaveDelta_.at<float>(octaveDiff));

			// When octave same use relative coding
			const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
			const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);

			const int sCurX = round(currentKpt.pt.x / mScaleFactors[octave]);
			const int sCurY = round(currentKpt.pt.y / mScaleFactors[octave]);

			const int dx = sCurX - sRefx;
			const int dy = sCurY - sRefy;

			const int tdx = dx + (mFreqInterKeyPoint[octave].cols-1)/2;
			const int tdy = dy + (mFreqInterKeyPoint[octave].rows-1)/2;

			int index;
			KeyPointDiffToIndex(tdx, tdy, octave, index);

			const float R_inter_res = mLutRInter[dist];

			const float R_inter_xy = -log2(mPInterKeyPoint[octave].at<float>(index));
			const float R_inter_kpt = R_angleDiff + R_inter_octave + R_inter_xy;

			// Skip mode
			float R_inter = R_inter_kpt + R_inter_res + R_inter_ref;
			if( (dist < 5) && (dx == 0) && (dy == 0) && (curAngleBin == refAngleBin) && (currentKpt.octave == refKeypoint.octave) )
			{
				cand.skipMode = true;
				R_inter = R_inter_ref;
			}


			if( R_inter < bestR )
			{
				bestR = R_inter;
				bestIdx = vIndices[p];
				bestImg = img;
				bestReference = candidatesCount + vIndices[p];
				cand.residual = residual;
			}
		}

		candidatesCount +=  it->keypoints.size();
		img++;
	}

	cand.imageId = bestImg;
	cand.keypointId = bestIdx;
	cand.candidateId = bestReference;
	cand.numCandidates = numCandidates;


	if( cand.keypointId == -1 )
		return std::numeric_limits<float>::max();




	return bestR;
}





ModeDecision FeatureCoder::modeDecision( const cv::KeyPoint &currentKpt, const cv::Mat &descriptor )
{
	unsigned int visualWord;
	cv::Mat intraResidualMat;

	float R_intra = intraCosts(currentKpt, descriptor, visualWord, intraResidualMat);
	float R_inter = std::numeric_limits<float>::max();

	Candidate interCandidate;
	Candidate intraPredCandidate;

	if( bInterPred )
		R_inter = interCandidateSelection(currentKpt, descriptor, interCandidate);


	ModeDecision decision;
	if( R_intra <= R_inter  )
	{
		decision.visualWord = visualWord;
		decision.mode = CodingMode::INTRA;
		decision.residual = intraResidualMat;
		decision.rate = R_intra;
	}
	if( R_inter <= R_intra  )
	{
		decision.mode = CodingMode::INTER;
		if( interCandidate.skipMode )
			decision.mode = CodingMode::INTER_SKIP;

		decision.residual = interCandidate.residual;
		decision.candidate = interCandidate;
		decision.rate = R_inter;
	}



	// Calculate rating
	if( !mModel.pSigma_.empty() )
	{
		// Coding -> Laueft
		float pMode = 1.0;
		int mode = decision.mode;
		if( mode >= 0 && mode < mModel.pCoding_. cols)
			pMode = mModel.pCoding_.at<float>(mode);

		// Response:
		float pResponse = 1.0;
		int responseBin = floor(currentKpt.response / mModel.mRespBinSize);
		if( responseBin >= mModel.pResp_.cols)
			responseBin = mModel.pResp_.cols-1;

		pResponse = mModel.pResp_.at<float>(responseBin);


		decision.r = pResponse * pMode;

		if( mbInit && currentKpt.octave == 0)
			decision.r = 1.0;
	}
	else
	{
		decision.r = 1.0;
	}


	return decision;
}



size_t FeatureCoder::encodeMode(const CodingMode &mode,  EncodeContext &ctxt)
{
	int nMode = 0;
	if( mode == CodingMode::INTER )
		nMode = 1;
	if( mode == CodingMode::INTER_SKIP )
		nMode = 2;
	if( mode == CodingMode::UNUSED )
		nMode = 3;

	const int nBitsMode = 2;
	for( int i = 0; i < nBitsMode; i++)
	{
		ctxt.cur_bit = ( nMode >> (nBitsMode - i - 1) ) & 0x0001;
		// update the 8-bits buffer
		ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
		ctxt.bit_idx--;

		// when the buffer is full, append it to the vector; then reset the buffer
		if (ctxt.bit_idx<0){
			ctxt.bit_idx = 7;
			ctxt.bitstream.push_back(ctxt.buffer);
			ctxt.buffer = 0;
		}
	}

	return nBitsMode;
}


size_t FeatureCoder::decodeMode(DecodeContext &ctxt, CodingMode &mode)
{
	int nMode = 0;
	const int nBitsMode = 2;
	for( int i = 0; i < nBitsMode; i++ )
	{
		if(ctxt.bit_idx<0){
			ctxt.bit_idx = 7;
			ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
			ctxt.byte_idx++;
		}
		ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
		ctxt.bit_idx--;

		nMode |= (ctxt.cur_bit << (nBitsMode - i - 1) );
	}

	if( nMode == 0)
		mode = CodingMode::INTRA;
	else if (nMode == 1 )
		mode = CodingMode::INTER;
	else if (nMode == 2 )
		mode = CodingMode::INTER_SKIP;
	else if( nMode == 3 )
		mode = CodingMode::UNUSED;

	return 1;
}


size_t FeatureCoder::IntraEncodeBow(unsigned int visualWord,  EncodeContext &bowCtxt)
{
	const int bitsBow = ceil(mnBitsBow);
	for( int i = 0; i < bitsBow; i++)
	{
		bowCtxt.cur_bit = ( visualWord >> (bitsBow - i - 1) ) & 0x0001;		// update the 8-bits buffer
		bowCtxt.buffer |= bowCtxt.cur_bit << bowCtxt.bit_idx;
		bowCtxt.bit_idx--;

		// when the buffer is full, append it to the vector; then reset the buffer
		if (bowCtxt.bit_idx<0){
			bowCtxt.bit_idx = 7;
			bowCtxt.bitstream.push_back(bowCtxt.buffer);
			bowCtxt.buffer = 0;
		}
	}

	return mnBitsBow;
}


void FeatureCoder::IntraDecodeBow(DecodeContext &bowCtxt, unsigned int &visualWord )
{
	const int bitsBow = ceil(mnBitsBow);
	visualWord = 0;
	for( int i = 0; i < bitsBow; i++ )
	{
		if(bowCtxt.bit_idx<0){
			bowCtxt.bit_idx = 7;
			bowCtxt.cur_byte = bowCtxt.bitstream[bowCtxt.byte_idx];
			bowCtxt.byte_idx++;
		}
		bowCtxt.cur_bit = (bowCtxt.cur_byte >> bowCtxt.bit_idx) & 0x01;
		bowCtxt.bit_idx--;

		visualWord |= (bowCtxt.cur_bit << (bitsBow - i - 1) );
	}

	assert( visualWord <= mVoc.size() );
}



size_t FeatureCoder::IntraEncodeBowAC(unsigned int visualWord,  ACEncodeContext &accontext)
{
	const size_t bits_start = accontext.bits();
	ac_encode_symbol(&accontext.ace, &accontext.acm_bow, visualWord);
	return accontext.bits() - bits_start;
}


void FeatureCoder::IntraDecodeBowAC(ACDecodeContext &accontext, unsigned int &visualWord )
{
	// Setup decoder for descriptor
	visualWord = ac_decode_symbol(&accontext.acd, &accontext.acm_bow);
}

size_t FeatureCoder::IntraEncodeKeyPoint(const cv::KeyPoint &keypoint, EncodeContext &context)
{
	const int &octave = keypoint.octave;

	int angleBin = floor(keypoint.angle / mAngleBinSize);

	const int bitsAngle = ceil(mnBitsAngle);
	for( int i = 0; i < bitsAngle; i++)
	{
		context.cur_bit = ( angleBin >> (bitsAngle - i - 1) ) & 0x0001;
		context.buffer |= context.cur_bit << context.bit_idx;
		context.bit_idx--;

		if (context.bit_idx<0){
			context.bit_idx = 7;
			context.bitstream.push_back(context.buffer);
			context.buffer = 0;
		}
	}

	const int bitsOctave = ceil(mnBitsOctave);
	for( int i = 0; i < bitsOctave; i++)
	{
		context.cur_bit = ( octave >> (bitsOctave - i - 1) ) & 0x0001;
		context.buffer |= context.cur_bit << context.bit_idx;
		context.bit_idx--;

		if (context.bit_idx<0){
			context.bit_idx = 7;
			context.bitstream.push_back(context.buffer);
			context.buffer = 0;
		}
	}

	// Resize Keypoints to integer resolution
	const int nbits_x = ceil(mvBitsPyramidWidth[octave]);
	const int nbits_y = ceil(mvBitsPyramidHeight[octave]);

	int qx = round(keypoint.pt.x / mScaleFactors[octave]);
	int qy = round(keypoint.pt.y / mScaleFactors[octave]);

	for( int i = 0; i < nbits_x; i++)
	{
		context.cur_bit = ( qx >> (nbits_x - i - 1) ) & 0x0001;
		context.buffer |= context.cur_bit << context.bit_idx;
		context.bit_idx--;

		if (context.bit_idx<0){
			context.bit_idx = 7;
			context.bitstream.push_back(context.buffer);
			context.buffer = 0;
		}
	}


	for( int i = 0; i < nbits_y; i++)
	{
		context.cur_bit = ( qy >> (nbits_y - i - 1) ) & 0x0001;
		context.buffer |= context.cur_bit << context.bit_idx;
		context.bit_idx--;

		if (context.bit_idx<0){
			context.bit_idx = 7;
			context.bitstream.push_back(context.buffer);
			context.buffer = 0;
		}
	}


	return bitsAngle + bitsOctave + nbits_x + nbits_y;
}



void FeatureCoder::IntraDecodeKeyPoint(DecodeContext &context, cv::KeyPoint &keypoint)
{
	const int bitsAngle = ceil(mnBitsAngle);
	int qangle = 0, qoctave = 0, qx = 0, qy = 0;
	for( int i = 0; i < bitsAngle; i++ )
	{
		if(context.bit_idx<0){
			context.bit_idx = 7;
			context.cur_byte = context.bitstream[context.byte_idx];
			context.byte_idx++;
		}
		context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
		context.bit_idx--;

		qangle |= (context.cur_bit << (bitsAngle - i - 1) );
	}

	const int bitsOctave = ceil(mnBitsOctave);
	for( int i = 0; i < bitsOctave; i++){
		if(context.bit_idx<0){
			context.bit_idx = 7;
			context.cur_byte = context.bitstream[context.byte_idx];
			context.byte_idx++;
		}
		context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
		context.bit_idx--;

		qoctave |= (context.cur_bit << (bitsOctave - i - 1) );
	}

	const int nbits_x = ceil(mvBitsPyramidWidth[qoctave]);
	const int nbits_y = ceil(mvBitsPyramidHeight[qoctave]);

	for( int i = 0; i <  nbits_x; i++)
	{
		if(context.bit_idx<0){
			context.bit_idx = 7;
			context.cur_byte = context.bitstream[context.byte_idx];
			context.byte_idx++;
		}
		context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
		context.bit_idx--;

		qx |= (context.cur_bit << (nbits_x - i - 1) );
	}


	for( int i = 0; i < nbits_y; i++ )
	{
		if(context.bit_idx<0){
			context.bit_idx = 7;
			context.cur_byte = context.bitstream[context.byte_idx];
			context.byte_idx++;
		}
		context.cur_bit = (context.cur_byte >> context.bit_idx) & 0x01;
		context.bit_idx--;

		qy |= (context.cur_bit << (nbits_y- i - 1) );
	}


	keypoint.pt.x = (float) qx * mScaleFactors[qoctave];
	keypoint.pt.y = (float) qy * mScaleFactors[qoctave];
	keypoint.octave = qoctave;
	keypoint.angle = (float) qangle * mAngleBinSize + mAngleBinSize / 2;
	assert(keypoint.angle >= 0 && keypoint.angle < 360.0);
};




size_t FeatureCoder::IntraEncodeKeyPointAC(const cv::KeyPoint &keypoint, ACEncodeContext &accontext)
{
	const size_t bits_start = accontext.bits();

	const int &octave = keypoint.octave;
	const int angleBin = floor(keypoint.angle / mAngleBinSize);
	assert( angleBin >= 0 && angleBin <= mAngleBins);


	ac_encode_symbol(&accontext.ace, &accontext.acm_intra_octave, octave);
	ac_encode_symbol(&accontext.ace, &accontext.acm_intra_angle, angleBin);

	// Resize Keypoints to integer resolution
	int qx = round(keypoint.pt.x / mScaleFactors[octave]);
	int qy = round(keypoint.pt.y / mScaleFactors[octave]);

	ac_encode_symbol(&accontext.ace, &accontext.v_acm_intra_kpt_x[octave], qx);
	ac_encode_symbol(&accontext.ace, &accontext.v_acm_intra_kpt_y[octave], qy);

	return accontext.bits() - bits_start;
}

void FeatureCoder::IntraDecodeKeyPointAC(ACDecodeContext &accontext, cv::KeyPoint &keypoint)
{
	const int octave = ac_decode_symbol(&accontext.acd, &accontext.acm_intra_octave);
	const int angleBin = ac_decode_symbol(&accontext.acd, &accontext.acm_intra_angle);

	const int qx = ac_decode_symbol(&accontext.acd, &accontext.v_acm_intra_kpt_x[octave]);
	const int qy = ac_decode_symbol(&accontext.acd, &accontext.v_acm_intra_kpt_y[octave]);

	keypoint.pt.x = (float) qx * mScaleFactors[octave];
	keypoint.pt.y = (float) qy * mScaleFactors[octave];
	keypoint.octave = octave;
	keypoint.angle = (float) angleBin * mAngleBinSize + mAngleBinSize / 2;
	assert(keypoint.angle >= 0 && keypoint.angle < 360.0);
}


size_t FeatureCoder::IntraEncodeResidual(const cv::Mat &residual, ACEncodeContext &accontext)
{
	const size_t bits_start = accontext.bits();

	// Code residuals
	cv::Mat exp_residuals;
	Utils::bin2mat(residual,exp_residuals);

	for( int d = 0; d < exp_residuals.cols; d++ )
	{
		const int &current_bit = exp_residuals.at<uchar>(d);
		ac_encode_symbol(&accontext.ace, &accontext.acm_intra_desc, current_bit);
	}

	return accontext.bits() - bits_start;
}


void FeatureCoder::IntraDecodeResidual(ACDecodeContext &resCtxt, cv::Mat &residual)
{
	// Setup decoder for descriptor
	cv::Mat exp_residuals(1, mModel.mDims, CV_8U);
	for( size_t d = 0; d < mModel.mDims; d++ )
		exp_residuals.at<uchar>(d) = ac_decode_symbol(&resCtxt.acd, &resCtxt.acm_intra_desc);

	cv::Mat binMat;
	Utils::mat2bin(exp_residuals, residual);
}


cv::Mat FeatureCoder::IntraReconstructDescriptor(const unsigned int &visualWord, cv::Mat &residual)
{
	// Reconstruct the descriptor
	const cv::Mat &visualCluster = mVoc.getWord(visualWord);

	cv::Mat descriptor;
	cv::bitwise_xor(residual, visualCluster, descriptor);
	return descriptor;
}


size_t FeatureCoder::InterEncodeReferenceAC(int reference, ACEncodeContext &accontext)
{
	const size_t bits_start = accontext.bits();
	ac_encode_symbol(&accontext.ace, &accontext.acm_inter_candidate, reference);
	return accontext.bits() - bits_start;
}

int FeatureCoder::InterDecodeReferenceAC(ACDecodeContext &accontext)
{
	// First we need the octave and angle.
	const int reference = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_candidate);
	return reference;
}



size_t FeatureCoder::InterEncodeKeypoint(const cv::KeyPoint &refKeypoint, const cv::KeyPoint &currentKeypoint,
		ACEncodeContext &accontext, EncodeContext &context)
{
	const size_t bits_start = accontext.bits();
	const size_t flbits = 0;


	const int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
	const int curAngleBin = floor(currentKeypoint.angle / mAngleBinSize);
	assert( refAngleBin >= 0 && refAngleBin < mAngleBins);
	assert( curAngleBin >= 0 && curAngleBin < mAngleBins);

	// Angle coding
	const int diff = curAngleBin - refAngleBin;
	assert( diff > -32 && diff < 32);

	const int angleDiff = diff + mnAngleOffset;
	assert(angleDiff >= 0 && angleDiff < mFreqInterAngleDiff.size() );

	ac_encode_symbol(&accontext.ace, &accontext.acm_inter_angle, angleDiff);


	// Octave coding
	const int octaveDiff = currentKeypoint.octave - refKeypoint.octave + mnOctaveOffset;
	ac_encode_symbol(&accontext.ace, &accontext.acm_inter_octave, octaveDiff);


	// Keypoint coding
	const int &octave = currentKeypoint.octave;
	assert( fabs(refKeypoint.pt.x - currentKeypoint.pt.x) <= mModel.mSearchRange );
	assert( fabs(refKeypoint.pt.y - currentKeypoint.pt.y) <= mModel.mSearchRange );


	const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
	const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);

	const int sCurX = round(currentKeypoint.pt.x / mScaleFactors[octave]);
	const int sCurY = round(currentKeypoint.pt.y / mScaleFactors[octave]);

	const int dx = sCurX - sRefx;
	const int dy = sCurY - sRefy;

	const int tdx = dx + (mFreqInterKeyPoint[octave].cols-1)/2;
	const int tdy = dy + (mFreqInterKeyPoint[octave].rows-1)/2;

	int index;
	KeyPointDiffToIndex(tdx, tdy, octave, index);


	// Coding xy-value
	ac_encode_symbol(&accontext.ace, &accontext.v_acm_inter_kpt[octave], index);


	const size_t acbits = accontext.bits() - bits_start;
	return acbits + flbits;
}



void FeatureCoder::InterDecodeKeypoint(ACDecodeContext &accontext, DecodeContext &context, const cv::KeyPoint &refKeypoint, cv::KeyPoint &currentKeypoint)
{
	// Angle decoding
	const int refAngleBin = floor(refKeypoint.angle / mAngleBinSize);
	const int angleDiff = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_angle);

	// Octave decoding
	const int octaveDiff =  ac_decode_symbol(&accontext.acd, &accontext.acm_inter_octave);

	const int angleBin = angleDiff - mnAngleOffset + refAngleBin;
	const int octave = octaveDiff - mnOctaveOffset + refKeypoint.octave;


	// Keypoint decoding
	int x = 0, y = 0;


	// Position decoding
	const int index = ac_decode_symbol(&accontext.acd, &accontext.v_acm_inter_kpt[octave]);

	int dx,dy;
	IndexToKeyPointDiff(index, octave, dx, dy);

	dx = dx - (mFreqInterKeyPoint[octave].cols-1)/2;
	dy = dy - (mFreqInterKeyPoint[octave].rows-1)/2;

	const int sRefx = round(refKeypoint.pt.x / mScaleFactors[octave]);
	const int sRefy = round(refKeypoint.pt.y / mScaleFactors[octave]);

	x  = (dx + sRefx);
	y  = (dy + sRefy);

	currentKeypoint.pt.x = x *  mScaleFactors[octave];
	currentKeypoint.pt.y = y *  mScaleFactors[octave];
	currentKeypoint.angle = angleBin * mAngleBinSize + mAngleBinSize / 2;
	currentKeypoint.octave = octave;
	assert(currentKeypoint.angle >= 0 && currentKeypoint.angle < 360.0);
}


size_t FeatureCoder::InterEncodeResidual(const cv::Mat &residual, ACEncodeContext &accontext)
{
	const size_t bits_start = accontext.bits();
	cv::Mat expResidual;
	Utils::bin2mat(residual, expResidual);

	for( int d = 0; d < expResidual.cols; d++ )
	{
		const uchar &current_bit = expResidual.at<uchar>(d);
		ac_encode_symbol(&accontext.ace, &accontext.acm_inter_desc, current_bit);
	}

	return accontext.bits() - bits_start;
}



void FeatureCoder::InterDecodeResidual(ACDecodeContext &accontext, cv::Mat&residual)
{
	// We can get the residual vector
	cv::Mat exp_residuals(1, mModel.mDims, CV_8U);
	for( unsigned int d = 0; d < mModel.mDims; d++ )
		exp_residuals.at<uchar>(d) = ac_decode_symbol(&accontext.acd, &accontext.acm_inter_desc);

	Utils::mat2bin(exp_residuals, residual);
}

cv::Mat FeatureCoder::InterReconstructDescriptor(const cv::Mat &referenceDescriptor, const cv::Mat &residual)
{
	cv::Mat descriptor;
	cv::bitwise_xor(referenceDescriptor, residual, descriptor);
	return descriptor;
}

void FeatureCoder::KeyPointDiffToIndex(int dx, int dy, int octave, int &index)
{
	index= dy * mFreqInterKeyPoint[octave].cols + dx;
}

void FeatureCoder::IndexToKeyPointDiff(int index, int octave, int &x, int &y)
{
	x = index % mFreqInterKeyPoint[octave].cols;
	y = index / mFreqInterKeyPoint[octave].cols;
}


int FeatureCoder::GetNumInterCandidates()
{
	std::list<ImgBufferEntry>::iterator it;
	std::list<ImgBufferEntry>::iterator itEnd;
	it = mLeftImageBuffer.begin();
	itEnd = mLeftImageBuffer.end();


	int numCandidates = 0;
	for( ; it != itEnd; it++ )
		numCandidates += it->keypoints.size();

	return numCandidates;
}


size_t FeatureCoder::encodeReference(int reference, int numCandidates, EncodeContext &ctxt)
{
	assert( reference < numCandidates );
	const int nBitsReference = ceil(log2(numCandidates));
	for( int i = 0; i < nBitsReference; i++)
	{
		ctxt.cur_bit = ( reference >> (nBitsReference - i - 1) ) & 0x0001;
		ctxt.buffer |= ctxt.cur_bit << ctxt.bit_idx;
		ctxt.bit_idx--;

		if (ctxt.bit_idx<0){
			ctxt.bit_idx= 7;
			ctxt.bitstream.push_back(ctxt.buffer);
			ctxt.buffer = 0;
		}
	}

	return nBitsReference;
}

int FeatureCoder::decodeReference(DecodeContext &ctxt, int numCandidates)
{
	// First we need the octave and angle.
	const int nBitsReference = ceil(log2(numCandidates));
	int referenceId = 0;
	for( int i = 0; i < nBitsReference; i++ )
	{
		if(ctxt.bit_idx<0){
			ctxt.bit_idx = 7;
			ctxt.cur_byte = ctxt.bitstream[ctxt.byte_idx];
			ctxt.byte_idx++;
		}
		ctxt.cur_bit = (ctxt.cur_byte >> ctxt.bit_idx) & 0x01;
		ctxt.bit_idx--;

		referenceId |= (ctxt.cur_bit << (nBitsReference - i - 1) );
	}

	return referenceId;
}



cv::KeyPoint FeatureCoder::fakeCode(const cv::KeyPoint &keyPoint)
{
	// Angle decoding
	const int angleBin = floor(keyPoint.angle / mAngleBinSize);
	assert(angleBin >= 0 && angleBin < mAngleBins);

	cv::KeyPoint decodedKeyPoint;
	decodedKeyPoint.pt = keyPoint.pt;
	decodedKeyPoint.angle = angleBin * mAngleBinSize + mAngleBinSize/2;
	decodedKeyPoint.octave = keyPoint.octave;
	assert( decodedKeyPoint.angle >= 0 && decodedKeyPoint.angle  < 360.0);
	return decodedKeyPoint;
}


/**
* This file is part of the Remote Visual SLAM Extension for ORB-SLAM2.
* 2017 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*/

#pragma once

class EvaluationStats
{
public:
	EvaluationStats()
{
		// Bow matching stats
		nBowMatches = 0;
		nBowRansacInliers = 0;
		nBowTrueInliers = 0;
		fBowReproError = 0.0;

		// Desc matching stats
		nDescMatches = 0;
		nDescRansacInliers = 0;
		nDescTrueInliers = 0;
		fDescReproError = 0.0;
}

	void save(const std::string &path) const
	{
		FILE *f = fopen(path.c_str(), "w");

		encodeStats.save(f);
		decodeStats.save(f);

		// BoW matching stats
		fwrite(&nBowMatches, sizeof(size_t), 1, f);
		fwrite(&nBowRansacInliers, sizeof(size_t), 1, f);
		fwrite(&nBowTrueInliers, sizeof(size_t), 1, f);
		fwrite(&fBowReproError, sizeof(float), 1, f);

		// Desc matching stats
		fwrite(&nDescMatches, sizeof(size_t), 1, f);
		fwrite(&nDescRansacInliers, sizeof(size_t), 1, f);
		fwrite(&nDescTrueInliers, sizeof(size_t), 1, f);
		fwrite(&fDescReproError, sizeof(float), 1, f);

		fclose(f);
	}


public:
	// Coding statistics
	EncodeStats encodeStats;
	DecodeStats decodeStats;


	// BoW matching stats
	size_t nBowMatches;
	size_t nBowRansacInliers;
	size_t nBowTrueInliers;
	float fBowReproError;

	// Desc matching stats
	size_t nDescMatches;
	size_t nDescRansacInliers;
	size_t nDescTrueInliers;
	float fDescReproError;
};

#include "DronClassifier2.h"
#include <queue>
#include "Triangulator.h"
#define MAX_ERROR  1000000
void DronClassifier2::triangulateMultipleDrones(const std::vector<std::vector<std::vector<cv::Point2d>>>& points, std::vector<std::vector<cv::Point3d>>& triangulatedPoints, int n_drones,Triangulator* triangulator)
{
	for (int i = 0; i < n_drones; i++)
	{
		triangulatedPoints.push_back(std::vector<cv::Point3d>{});
	}
	for (int frame = 0; frame < points[0].size(); frame++)
	{
		std::vector<int> sizes;
		for (int camera = 0; camera < points.size(); camera++)
		{
			sizes.push_back(points[camera][frame].size());
		}
		std::priority_queue<Combination> combinationQueue;
		Combination toTest{ 2,sizes };

		while (toTest.increment())
		{
			std::vector<Triangulator::CamPointPair> pairs;
			for (int label = 0; label < toTest.combination_.size(); label++)
			{

				if (toTest.combination_[label] == 0)
					continue;
				pairs.push_back(Triangulator::CamPointPair{ triangulator->getCamera(label),points[label][frame][toTest.combination_[label] - 1] });
			}
			auto result = triangulator->triangulatePoint(pairs);

			combinationQueue.push(Combination{ toTest.combination_,result.first,result.second });
		}

		// Pick n_drones best combinations
		std::vector<Combination> finalCombinations;
		while (!combinationQueue.empty() && finalCombinations.size() < n_drones) {
			Combination c = combinationQueue.top();
			if (c.isCombinationUnique(finalCombinations)) { finalCombinations.push_back(c); }
			combinationQueue.pop();
		}

		if (frame == 0) {
			for (int i = 0; i < finalCombinations.size(); i++) {
				triangulatedPoints[i].push_back(finalCombinations[i].point);
			}
		}
		else {
			// Classify every new point to path
			for (int i = 0; i < finalCombinations.size(); i++) {
				size_t bestPath = 0;
				double bestDist = 1e+6;
				for (int j = 0; j < n_drones; j++) {
					double dist = cv::norm(triangulatedPoints[j].back() - finalCombinations[i].point);
					if (dist < bestDist) {
						bestDist = dist;
						bestPath = j;
					}
				}
				if (bestDist < 200)
					triangulatedPoints[bestPath].push_back(finalCombinations[i].point);
			}

		}
	}
}


DronClassifier2::Combination::Combination(std::vector<size_t> combination_, cv::Point3d point, double error):combination_(combination_), point(point), error(error)
{
}

DronClassifier2::Combination::Combination(int min_drones,std::vector<int> sizes)
{
	this->sizes = sizes;
	this->min_drones = min_drones;
	this->combination_ = std::vector<size_t>{};
	this->drones = 0;
	for (int i = 0; i < sizes.size(); i++)
	{
		combination_.push_back(0);
	}

}
bool DronClassifier2::Combination::increment()
{
	if (combination_.size()==0)
		return false;
	for (int i = this->combination_.size() - 1; i >=0; i--)
	{
		combination_[i] = (combination_[i] + 1)%(sizes[i]+1);
		if (combination_[i] != 0)
		{
			if (combination_[i] == 1) drones += 1;

			if(drones>=min_drones)
				return true;
			else
				return this->increment();
		}
		else
		{
			drones -= 1;
		}
	}
	return false;

	
}


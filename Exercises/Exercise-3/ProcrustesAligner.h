#pragma once
#include <numeric>

#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		const auto sourceMean = computeMean(sourcePoints);
		const auto targetMean = computeMean(targetPoints);
		
		const auto rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		const auto translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		Vector3f mean;
		for (const auto& point : points)
		{
			mean += point;
		}
		return mean/points.size();
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).
		MatrixXf sourcePointMat(sourcePoints.size(), 3);
		MatrixXf targetPointMat(targetPoints.size(), 3);
		for (int i = 0; i < sourcePoints.size(); i++)
		{
			sourcePointMat.block(i, 0, 1, 1) = sourcePoints[i] - sourceMean;
		}
		for (int i = 0; i < targetPoints.size(); i++)
		{
			targetPointMat.block(i, 0, 1, 1) = targetPoints[i] - targetMean;
		}

		Matrix3f rotation = targetPointMat  * targetPointMat.normalized().transpose();
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation = -rotation * sourceMean + targetMean;
        return translation;
	}
};

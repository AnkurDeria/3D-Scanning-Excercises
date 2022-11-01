#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

bool DEPTH_INTRINSIC_INV_MUL = true;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// position stored as 4 floats (4th component is supposed to be 1.0)
		Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

static inline bool ValidTriangle(Vertex* vertices, unsigned idx1, unsigned idx2, unsigned idx3, float edgeThreshold) {
	if (vertices[idx1].position.x() != MINF
		&& vertices[idx2].position.x() != MINF
		&& vertices[idx3].position.x() != MINF)
	{
		if ((vertices[idx1].position - vertices[idx2].position).norm() < edgeThreshold
			&& (vertices[idx1].position - vertices[idx3].position).norm() < edgeThreshold
			&& (vertices[idx2].position - vertices[idx3].position).norm() < edgeThreshold)
		{
			return true;
		}
	}
	return false;
}



static inline void AddFaces(Vertex* vertices, unsigned int idx1, unsigned int idx2, unsigned int idx3, float edgeThreshold, std::vector<std::string>& faces, unsigned int& nFaces)
{
	if (ValidTriangle(vertices, idx1, idx2, idx3, edgeThreshold))
	{
		faces.push_back(std::to_string(3) + " " + std::to_string(idx1) + " " + std::to_string(idx2) + " " + std::to_string(idx3));
		++nFaces;
	}
}


static inline bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	unsigned maxNFaces = (width - 1) * 2 * (height - 1);

	std::vector<std::string> faces;

	for (unsigned int row = 0; row < height - 1; ++row) {
		for (unsigned int col = 0; col < width - 1; ++col)
		{
			unsigned idx1 = row * width + col;
			unsigned idx2 = (row + 1) * width + col;
			unsigned idx3 = row * width + col + 1;
			unsigned idx4 = (row + 1) * width + col + 1;
			AddFaces(vertices, idx1, idx2, idx4, edgeThreshold, faces, nFaces);
			AddFaces(vertices, idx1, idx4, idx3, edgeThreshold, faces, nFaces);
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	outFile << "# list of vertices" << std::endl;
	outFile << "# X Y Z R G B A" << std::endl;
	for (unsigned int idx = 0; idx < width * height; ++idx) {
		if (vertices[idx].position[0] != MINF && vertices[idx].position[1] != MINF && vertices[idx].position[2] != MINF) {

			outFile << vertices[idx].position.x() << " " << vertices[idx].position.y() << " " << vertices[idx].position.z() << " "
				<< ((int)(vertices[idx].color[0])) << " "
				<< ((int)(vertices[idx].color[1])) << " "
				<< ((int)(vertices[idx].color[2])) << " "
				<< ((int)(vertices[idx].color[3])) << std::endl;

		}
		else {
			outFile << 0.0 << " " << 0.0 << " "
				<< 0.0 << " " << (int)vertices[idx].color[0]
				<< " " << (int)vertices[idx].color[1]
				<< " " << (int)vertices[idx].color[2]
				<< " " << (int)vertices[idx].color[3]
				<< std::endl;
		}
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	for (unsigned idx = 0; idx < nFaces; ++idx) {
		outFile << faces[idx] << std::endl;
	}

	// close file
	outFile.close();

	return true;
}

static inline Vector4f screenToCamera(unsigned int ux, unsigned int uy, float depth, float cX, float fX, float cY, float fY)
{
	const float x = ((float)ux - cY) / fY;
	const float y = ((float)uy - cX) / fX;
	return Vector4f(depth * x, depth * y, depth, 1.0f);
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return EXIT_SUCCESS;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for (unsigned int row = 0; row < sensor.GetDepthImageHeight(); ++row) {
			for (unsigned int col = 0; col < sensor.GetDepthImageWidth(); ++col) {
				int idx = row * sensor.GetDepthImageWidth() + col;
				if (depthMap[idx] == MINF)
				{
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
				else
				{
					float depth = depthMap[idx];
					Vector4f pointCameraSpace;
					Vector3f point;

					if (DEPTH_INTRINSIC_INV_MUL)
					{
						point = depthIntrinsicsInv * Vector3f(float(col), float(row), 1.0f);
						pointCameraSpace = Vector4f(point[0] * depth, point[1] * depth, point[2] * depth, 1.0f);
					}
					else
					{
						pointCameraSpace = screenToCamera(col, row, depth, cX, fX, cY, fY);
					}

					Vector4f pointWorldSpace = trajectoryInv * depthExtrinsicsInv * pointCameraSpace;
					vertices[idx].position = pointWorldSpace;

					vertices[idx].color[0] = colorMap[4 * idx + 0];
					vertices[idx].color[1] = colorMap[4 * idx + 1];
					vertices[idx].color[2] = colorMap[4 * idx + 2];
					vertices[idx].color[3] = colorMap[4 * idx + 3];
				}
			}
			//std::cout << std::endl;
		}

		std::cout << "----------------------------------------------" << std::endl;

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}

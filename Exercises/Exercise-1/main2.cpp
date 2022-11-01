#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};


float lengthSq(Vector4f v) {
	return v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm
	float edgeThresholdSq = edgeThreshold * edgeThreshold; // 1cm

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
	std::vector<std::string> vectorOfStrings;
	for (int idx = 0; idx < nVertices - width; idx++) {
		int currentRow = idx / width;
		int one = idx;
		int two = (currentRow + 1) * width + (idx % width);
		int three = idx + 1;
		int four = (currentRow + 1) * width + (idx % width) + 1;
		if (vertices[one].position.x() != MINF &&
			vertices[two].position.x() != MINF &&
			vertices[three].position.x() != MINF) {

			//std::cout << "length: " << lengthSq(vertices[one].position - vertices[two].position) << std::endl;
			if (lengthSq((vertices[one].position - vertices[two].position)) < edgeThresholdSq &&
				lengthSq((vertices[one].position - vertices[three].position)) < edgeThresholdSq &&
				lengthSq((vertices[two].position - vertices[three].position)) < edgeThresholdSq) {
				vectorOfStrings.push_back("3 " + std::to_string(one) + " " + std::to_string(two) + " " + std::to_string(three));
				nFaces++;
			}
		}
		if (vertices[three].position.x() != MINF &&
			vertices[two].position.x() != MINF &&
			vertices[four].position.x() != MINF) {
			if (lengthSq((vertices[two].position - vertices[four].position)) < edgeThreshold &&
				lengthSq((vertices[two].position - vertices[three].position)) < edgeThreshold &&
				lengthSq((vertices[four].position - vertices[three].position)) < edgeThreshold) {
				vectorOfStrings.push_back("3 " + std::to_string(three) + " " + std::to_string(two) + " " + std::to_string(four));
				nFaces++;
			}
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

	for (int idx = 0; idx < nVertices; idx++) {
		if (vertices[idx].position.z() == MINF) {
			outFile << "0 0 0";
			outFile << " 255 255 255 255" << std::endl;
		}
		else
		{
			outFile << vertices[idx].position.x() << " " << vertices[idx].position.y() << " " << vertices[idx].position.z();
			outFile << " " 
				<< ((int)(vertices[idx].color[0])) << " " 
				<< ((int)(vertices[idx].color[1])) << " "
				<< ((int)(vertices[idx].color[2])) << " "
				<< ((int)(vertices[idx].color[3])) << std::endl;
		}
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for (auto& s : vectorOfStrings) {
		outFile << s << std::endl;
	}
	

	// close file
	outFile.close();

	return true;
}


// pos.xyz in meters; output in pixels/meters
static inline Vector4f cameraToScreen(float x, float y, float z, const float& fx, const float& fy, const float& mx, const float& my)
{
	Vector4f res = Vector4f(
		x * fx / z + mx,
		y * fy / z + my,
		z,
		1
	);

	return res;
}

// ux, uy in pixels; depth in meters
static inline Vector4f screenToCamera(int ux, int uy, float depth, float fx, float fy, float mx, float my)
{
	const float x = ((float)ux - mx) / fx;
	const float y = ((float)uy - my) / fy;
	return Vector4f(depth * x, depth * y, depth, 1);
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	//for (int i = 0; i < 10; i++) {
	//	sensor.ProcessNextFrame();
	//}

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

		int vertices_len = sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight();
		Vertex* vertices = new Vertex[vertices_len];
		
		int y = 0;
		int x = 0;
		for (int idx = 0; idx < vertices_len; idx++) {
			if (depthMap[idx] == MINF) {
				vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
				vertices[idx].color = Vector4uc(0, 0, 0, 0);
			}
			else {
				Vector4f camSpace = screenToCamera(x, y, depthMap[idx], fX, fY, cX, cY);
				vertices[idx].position = trajectoryInv*depthExtrinsicsInv *camSpace;
				vertices[idx].color = Vector4uc(colorMap[idx * 4], colorMap[idx * 4+1], colorMap[idx * 4 + 2], colorMap[idx * 4 + 3]);
			}

			if (idx % sensor.GetDepthImageWidth() == 0) {
				x = -1;
				y++;
			}
			x++;
		}

		

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

		//return 0;
	}

	return 0;
}
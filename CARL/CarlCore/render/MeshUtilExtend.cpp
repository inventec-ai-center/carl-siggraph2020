#include "MeshUtilExtend.h"


void cMeshUtilExtend::BuildBoneSolidMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_faces = 8;
	const int vert_size = num_faces * 3 * cMeshUtil::gPosDim;
	const int norm_size = num_faces * 3 * cMeshUtil::gNormDim;
	const int coord_size = num_faces * 3 * cMeshUtil::gCoordDim;
	const int idx_size = num_faces * 3;
	const float size = 1.0f / 7.0f;

	const float vert_data[vert_size] = {
		-size, 0.200000, -size, // 3
		size, 0.200000, -size,  // 4
		0, 0, 0,				// 6

		size, 0.200000, -size,  // 4
		-size, 0.200000, -size, // 3
		0, 1, 0,				// 5

		-size, 0.200000, size,  // 1
		size, 0.200000, size,   // 2
		0, 1, 0,				// 5

		size, 0.200000, size,   // 2
		size, 0.200000, -size,  // 4
		0, 1, 0,				// 5

		-size, 0.200000, -size, // 3
		-size, 0.200000, size,  // 1
		0, 1, 0,				// 5

		size, 0.200000, size,   // 2
		-size, 0.200000, size,  // 1
		0, 0, 0,				// 6

		size, 0.200000, -size,  // 4
		size, 0.200000, size,   // 2
		0, 0, 0,				// 6

		-size, 0.200000, size,  // 1
		-size, 0.200000, -size, // 3
		0, 0, 0,				// 6
	};

	const float norm_data[vert_size] = {
		0.0000, -0.5812, -0.8137,		// 1
		0.0000, -0.5812, -0.8137,		// 1
		0.0000, -0.5812, -0.8137,		// 1

		0.0000, 0.1758, -0.9844,		// 2
		0.0000, 0.1758, -0.9844,		// 2
		0.0000, 0.1758, -0.9844,		// 2

		0.0000, 0.1758, 0.9844,			// 3
		0.0000, 0.1758, 0.9844,			// 3
		0.0000, 0.1758, 0.9844,			// 3

		0.9844, 0.1758, 0.0000,			// 4
		0.9844, 0.1758, 0.0000,			// 4
		0.9844, 0.1758, 0.0000,			// 4

		-0.9844, 0.1758, 0.0000,		// 5
		-0.9844, 0.1758, 0.0000,		// 5
		-0.9844, 0.1758, 0.0000,		// 5

		0.0000, -0.5812, 0.8137,		// 6
		0.0000, -0.5812, 0.8137,		// 6
		0.0000, -0.5812, 0.8137,		// 6

		0.8137, -0.5812, 0.0000,		// 7
		0.8137, -0.5812, 0.0000,		// 7
		0.8137, -0.5812, 0.0000,		// 7

		-0.8137, -0.5812, 0.0000,		// 8
		-0.8137, -0.5812, 0.0000,		// 8
		-0.8137, -0.5812, 0.0000,		// 8
	};


	const float coord_data[coord_size] = {
		0, 0,
		1, 0,
		1, 1,

		1, 1,
		0, 1,
		0, 0,

		1, 0, 
		1, 1,
		0, 1,

		0, 1,
		0, 0,
		1, 0,

		0, 0, 
		1, 0,
		1, 1,

		1, 1,
		0, 1,
		0, 0,

		0, 0,
		1, 0,
		1, 1,

		1, 1,
		0, 1,
		0, 0,
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	cMeshUtil::BuildDrawMesh(vert_data, vert_size, norm_data, norm_size, coord_data, coord_size, idx_data, idx_size, out_mesh.get());
}

void cMeshUtilExtend::BuildBoneWireMesh(std::unique_ptr<cDrawMesh>& out_mesh)
{
	const int num_edges = 12;
	const int vert_size = num_edges * 2 * cMeshUtil::gPosDim;
	const int norm_size = num_edges * 2 * cMeshUtil::gNormDim;
	const int coord_size = num_edges * 2 * cMeshUtil::gCoordDim;
	const int idx_size = num_edges * 2;
	const float size = 1.0f / 7.0f;

	const float vert_data[vert_size] = {
		0, 0, 0,
		-size, 0.2f, -size,

		0, 0, 0,
		size, 0.2f, -size,

		0, 0, 0,
		-size, 0.2f, size,

		0, 0, 0,
		size, 0.2f, size,

		-size, 0.2f, -size,
		0, 1, 0,

		size, 0.2f, -size,
		0, 1, 0,

		-size, 0.2f, size,
		0, 1, 0,

		size, 0.2f, size,
		0, 1, 0,

		-size, 0.2f, -size,
		size, 0.2f, -size,

		size, 0.2f, -size,
		size, 0.2f, size,

		size, 0.2f, size,
		-size, 0.2f, size,

		-size, 0.2f, size,
		-size, 0.2f, -size
	};

	const float norm_data[vert_size] = {
		0, 1, 0, // top
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,

		0, -1, 0, // bottom
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,

		1, 0, 0, // front
		1, 0, 0,

		-1, 0, 0, // back
		-1, 0, 0,

		0, 0, -1, // left
		0, 0, -1,

		0, 0, 1, // right
		0, 0, 1
	};


	const float coord_data[vert_size] = {
		0, 0, // top
		1, 0,
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,

		1, 0, // bottom
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		0, 0,
		1, 0,

		0, 0, // front
		0, 1,

		0, 0, // back
		0, 1,

		0, 1, // left
		0, 0,

		0, 1, // right
		0, 0
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}

	out_mesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	cMeshUtil::BuildDrawMesh(vert_data, vert_size, norm_data, norm_size, coord_data, coord_size, idx_data, idx_size, out_mesh.get());
}
 

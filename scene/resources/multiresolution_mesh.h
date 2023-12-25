#ifndef MULTIRESOLUTION_MESH_H
#define MULTIRESOLUTION_MESH_H

#include "core/io/resource.h"
#include "core/math/face3.h"
#include "core/math/triangle_mesh.h"
#include "core/math/convex_hull.h"
#include "core/math/random_pcg.h"
#include "scene/resources/mesh.h"
#include "scene/resources/importer_mesh.h"
#include "servers/rendering_server.h"

#include "scene/resources/concave_polygon_shape_3d.h"
#include "scene/resources/convex_polygon_shape_3d.h"
#include "scene/resources/navigation_mesh.h"

class ConcavePolygonShape3D;
class ConvexPolygonShape3D;
class MeshConvexDecompositionSettings;
class Shape3D;

class MultiresolutionMesh : public Mesh {
	GDCLASS(MultiresolutionMesh, Mesh);


};



class SymetricMatrix {
public:
	// Constructor

	SymetricMatrix(double c = 0) {
		for (int i = 0; i < 10; ++i) {
			m[i] = c;
		}	
	}

	SymetricMatrix(double m11, double m12, double m13, double m14,
			double m22, double m23, double m24,
			double m33, double m34,
			double m44) {
		m[0] = m11;
		m[1] = m12;
		m[2] = m13;
		m[3] = m14;
		m[4] = m22;
		m[5] = m23;
		m[6] = m24;
		m[7] = m33;
		m[8] = m34;
		m[9] = m44;
	}

	// Make plane

	SymetricMatrix(double a, double b, double c, double d) {
		m[0] = a * a;
		m[1] = a * b;
		m[2] = a * c;
		m[3] = a * d;
		m[4] = b * b;
		m[5] = b * c;
		m[6] = b * d;
		m[7] = c * c;
		m[8] = c * d;
		m[9] = d * d;
	}

	double operator[](int c) const { return m[c]; }

	// Determinant

	double det(int a11, int a12, int a13,
			int a21, int a22, int a23,
			int a31, int a32, int a33) {
		double det = m[a11] * m[a22] * m[a33] + m[a13] * m[a21] * m[a32] + m[a12] * m[a23] * m[a31] - m[a13] * m[a22] * m[a31] - m[a11] * m[a23] * m[a32] - m[a12] * m[a21] * m[a33];
		return det;
	}

	const SymetricMatrix operator+(const SymetricMatrix &n) const {
		return SymetricMatrix(m[0] + n[0], m[1] + n[1], m[2] + n[2], m[3] + n[3],
				m[4] + n[4], m[5] + n[5], m[6] + n[6],
				m[7] + n[7], m[8] + n[8],
				m[9] + n[9]);
	}

	SymetricMatrix &operator+=(const SymetricMatrix &n) {
		m[0] += n[0];
		m[1] += n[1];
		m[2] += n[2];
		m[3] += n[3];
		m[4] += n[4];
		m[5] += n[5];
		m[6] += n[6];
		m[7] += n[7];
		m[8] += n[8];
		m[9] += n[9];
		return *this;
	}

	double m[10];
};

class MultiresolutionMeshBuilder{
	//GDCLASS(MultiresolutionMeshBuilder, ImporterMesh);

public:
	struct Triangle {
		int indices[3]; 
		double err[4];
		int deleted, dirty;
		Vector3 n;
	};
	struct Vertex {
		Vector3 p;
		int tstart; // start of triangle references
		int tcount; // the number of triangles references that contains this vertex
		SymetricMatrix q;
		int border;
	};

	struct VertRef {
		int tid;
		int tvertex; // range 0, 1 ,2
	};

	Vector<Triangle> triangles;
	Vector<Vertex> vertices;
	// refs is in the order of vertices, where each "block" is size of numer of references (t.tstart + v.tcount) with ref metadata to that vert.
	//  example:
	//   v0 ref 0, v0 ref 1, v0 ref 2, v1 ref 0, v1 ref 1, v2 ref0, v2 ref1, v2 ref2
	//   |____________v0____________|  |________v1______|  |________v2_____________|
	Vector<VertRef> refs;

	const int node_size = 300; // maybe 1<<15

public:
	void generate_multiresolution_mesh(Ref<ImporterMesh> p_mesh, float p_normal_merge_angle, float p_normal_split_angle, Array p_skin_pose_transform_array);
	//void generate_multiresolution_mesh(Vector<Surface> &surfaces, float p_normal_merge_angle, float p_normal_split_angle, Array p_skin_pose_transform_array);
	void simplify_verticies_indicies_by_quadric_edge_collapse(Vector<Vector3> &p_verticies, List<int> &p_indices);
	void group_triangles_to_nodes(PackedInt32Array indices);

	// Simplify
	void simplify_by_quadric_edge_collapse(int target_count, double agressiveness = 7);
	bool flipped(Vector3 p, int i0, int i1, const Vertex &v0, const Vertex &v1, Vector<int> &deleted);
	void update_triangles(int i0, const Vertex &v, const Vector<int> &deleted, int &deleted_triangles);
	void update_mesh(int iteration);
	void compact_mesh();
	double vertex_error(SymetricMatrix q, double x, double y, double z);
	double calculate_error(int id_v1, int id_v2, Vector3 &p_result);
};


#endif // MULTIRESOLUTION_MESH_H

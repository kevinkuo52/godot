#include "multiresolution_mesh.h"

#include "core/io/marshalls.h"
#include "core/math/convex_hull.h"
#include "core/math/random_pcg.h"
#include "core/math/static_raycaster.h"
#include "scene/resources/surface_tool.h"

void MultiresolutionMeshBuilder::generate_multiresolution_mesh(Ref<ImporterMesh> p_mesh, float p_normal_merge_angle, float p_normal_split_angle, Array p_skin_pose_transform_array) {
	/* try {

	} catch (const std::string &ex) {
		print_line("generate_multiresolution_mesh caught an exception:\n ", ex);
	}*/

	const int surf_count = p_mesh->get_surface_count();
	const int blendshape_count = p_mesh->get_blend_shape_count();

	struct LocalSurfData {
		Mesh::PrimitiveType prim = {};
		Array arr;
		Array bsarr;
		Dictionary lods;
		String name;
		Ref<Material> mat;
		uint64_t fmt_compress_flags = 0;
	};

	Vector<LocalSurfData> surf_data_by_mesh;

	Vector<String> blendshape_names;
	for (int bsidx = 0; bsidx < blendshape_count; bsidx++) {
		blendshape_names.append(p_mesh->get_blend_shape_name(bsidx));
	}

	for (int surf_idx = 0; surf_idx < surf_count; surf_idx++) {
		Mesh::PrimitiveType prim = p_mesh->get_surface_primitive_type(surf_idx);
		const uint64_t fmt_compress_flags = p_mesh->get_surface_format(surf_idx);
		Array arr = p_mesh->get_surface_arrays(surf_idx);
		String name = p_mesh->get_surface_name(surf_idx);
		Dictionary lods;
		Ref<Material> mat = p_mesh->get_surface_material(surf_idx);
		Array blendshapes;
		for (int bsidx = 0; bsidx < blendshape_count; bsidx++) {
			Array current_bsarr = p_mesh->get_surface_blend_shape_arrays(surf_idx, bsidx);
			blendshapes.push_back(current_bsarr);
		}

		LocalSurfData surf_data_dictionary = LocalSurfData();

		if (prim == Mesh::PRIMITIVE_TRIANGLES) {
			PackedInt32Array indices = arr[RS::ARRAY_INDEX];
			Vector<Vector3> _normals = arr[RS::ARRAY_NORMAL];

			for (Vector3 vert : (Vector<Vector3>)arr[RS::ARRAY_VERTEX]) {
				vertices.append(Vertex{ vert });
			}

			for (int i = 0; i < indices.size(); i += 3) {
				triangles.append(Triangle{ { indices[i], indices[i + 1], indices[i + 2] } });
			}

			simplify_by_quadric_edge_collapse(triangles.size() / 2);

			Vector<Vector3> multi_res_verticies;
			PackedInt32Array multi_res_indices;
			for (Vertex vert : vertices) {
				multi_res_verticies.append(vert.p);
			}

			for (Triangle tri : triangles) {
				for (int index : tri.indices) {
					multi_res_indices.append(index);
				}
			}

			arr[ArrayMesh::ARRAY_VERTEX] = multi_res_verticies;
			arr[ArrayMesh::ARRAY_INDEX] = multi_res_indices;
		}

		surf_data_dictionary.prim = prim;
		surf_data_dictionary.arr = arr;
		surf_data_dictionary.bsarr = blendshapes;
		surf_data_dictionary.lods = lods;
		surf_data_dictionary.fmt_compress_flags = fmt_compress_flags;
		surf_data_dictionary.name = name;
		surf_data_dictionary.mat = mat;
		
		surf_data_by_mesh.append(surf_data_dictionary);
	}

	p_mesh->clear();
	for (int surf_idx = 0; surf_idx < surf_count; surf_idx++) {
		const Mesh::PrimitiveType prim = surf_data_by_mesh[surf_idx].prim;
		const Array arr = surf_data_by_mesh[surf_idx].arr;
		const Array bsarr = surf_data_by_mesh[surf_idx].bsarr;
		const Dictionary lods = surf_data_by_mesh[surf_idx].lods;
		const uint64_t fmt_compress_flags = surf_data_by_mesh[surf_idx].fmt_compress_flags;
		const String name = surf_data_by_mesh[surf_idx].name;
		const Ref<Material> mat = surf_data_by_mesh[surf_idx].mat;

		p_mesh->add_surface(prim, arr, bsarr, lods, mat, name, fmt_compress_flags);
	}
}

PackedInt32Array MultiresolutionMeshBuilder::simplify_by_lod(Vector<Vector3> &p_verticies, List<int> &p_indices) {
		Vector<Vector3> _vertices = p_verticies;
		PackedInt32Array _indices;
		Vector<Vector3> normals;

		for (auto index : p_indices) {
			_indices.append(index);
		}

		unsigned int index_count = _indices.size();
		unsigned int vertex_count = _vertices.size();

		const Vector3 *vertices_ptr = _vertices.ptr();
		const int *indices_ptr = _indices.ptr();

			normals.resize(index_count);
			Vector3 *n_ptr = normals.ptrw();
			for (unsigned int j = 0; j < index_count; j += 3) {
				const Vector3 &v0 = vertices_ptr[indices_ptr[j + 0]];
				const Vector3 &v1 = vertices_ptr[indices_ptr[j + 1]];
				const Vector3 &v2 = vertices_ptr[indices_ptr[j + 2]];
				Vector3 n = vec3_cross(v0 - v2, v0 - v1).normalized();
				n_ptr[j + 0] = n;
				n_ptr[j + 1] = n;
				n_ptr[j + 2] = n;
			}
		float normal_merge_threshold = Math::cos(Math::deg_to_rad(normal_merge_angle));
		float normal_pre_split_threshold = Math::cos(Math::deg_to_rad(MIN(180.0f, normal_split_angle * 2.0f)));
		float normal_split_threshold = Math::cos(Math::deg_to_rad(normal_split_angle));
		const Vector3 *normals_ptr = normals.ptr();

		HashMap<Vector3, LocalVector<Pair<int, int>>> unique_vertices;

		LocalVector<int> vertex_remap;
		LocalVector<int> vertex_inverse_remap;
		LocalVector<Vector3> merged_vertices;
		LocalVector<Vector3> merged_normals;
		LocalVector<int> merged_normals_counts;

		for (unsigned int j = 0; j < vertex_count; j++) {
			const Vector3 &v = vertices_ptr[j];
			const Vector3 &n = normals_ptr[j];

			HashMap<Vector3, LocalVector<Pair<int, int>>>::Iterator E = unique_vertices.find(v);

			if (E) {
				const LocalVector<Pair<int, int>> &close_verts = E->value;

				bool found = false;
				for (const Pair<int, int> &idx : close_verts) {
					//ERR_FAIL_INDEX(idx.second, normals.size());
					bool is_normals_close = normals[idx.second].dot(n) > normal_merge_threshold;
					if (is_normals_close) {
						vertex_remap.push_back(idx.first);
						merged_normals[idx.first] += normals[idx.second];
						merged_normals_counts[idx.first]++;
						found = true;
						break;
					}
				}

				if (!found) {
					int vcount = merged_vertices.size();
					unique_vertices[v].push_back(Pair<int, int>(vcount, j));
					vertex_inverse_remap.push_back(j);
					merged_vertices.push_back(v);
					vertex_remap.push_back(vcount);
					merged_normals.push_back(normals_ptr[j]);
					merged_normals_counts.push_back(1);
				}
			} else {
				int vcount = merged_vertices.size();
				unique_vertices[v] = LocalVector<Pair<int, int>>();
				unique_vertices[v].push_back(Pair<int, int>(vcount, j));
				vertex_inverse_remap.push_back(j);
				merged_vertices.push_back(v);
				vertex_remap.push_back(vcount);
				merged_normals.push_back(normals_ptr[j]);
				merged_normals_counts.push_back(1);
			}
		}

		LocalVector<int> merged_indices;
		merged_indices.resize(index_count);
		for (unsigned int j = 0; j < index_count; j++) {
			merged_indices[j] = vertex_remap[_indices[j]];
		}

		unsigned int merged_vertex_count = merged_vertices.size();
		const Vector3 *merged_vertices_ptr = merged_vertices.ptr();
		const int32_t *merged_indices_ptr = merged_indices.ptr();

		{
			const int *counts_ptr = merged_normals_counts.ptr();
			Vector3 *merged_normals_ptrw = merged_normals.ptr();
			for (unsigned int j = 0; j < merged_vertex_count; j++) {
				merged_normals_ptrw[j] /= counts_ptr[j];
			}
		}

		LocalVector<float> normal_weights;
		normal_weights.resize(merged_vertex_count);
		for (unsigned int j = 0; j < merged_vertex_count; j++) {
			normal_weights[j] = 2.0; // Give some weight to normal preservation, may be worth exposing as an import setting
		}

		Vector<float> merged_vertices_f32 = vector3_to_float32_array(merged_vertices_ptr, merged_vertex_count);
		float scale = SurfaceTool::simplify_scale_func(merged_vertices_f32.ptr(), merged_vertex_count, sizeof(float) * 3);

		unsigned int index_target = index_count / 2; // Start with the smallest target, 4 triangles
		unsigned int last_index_count = 0;

		int split_vertex_count = vertex_count;
		LocalVector<Vector3> split_vertex_normals;
		LocalVector<int> split_vertex_indices;
		split_vertex_normals.reserve(index_count / 3);
		split_vertex_indices.reserve(index_count / 3);

		RandomPCG pcg;
		pcg.seed(123456789); // Keep seed constant across imports

		Ref<StaticRaycaster> raycaster = StaticRaycaster::create();
		if (raycaster.is_valid()) {
			raycaster->add_mesh(_vertices, _indices, 0);
			raycaster->commit();
		}

		const float max_mesh_error = FLT_MAX; // We don't want to limit by error, just by index target
		float mesh_error = 0.0f;

			PackedInt32Array new_indices;
			new_indices.resize(index_count);

			Vector<float> merged_normals_f32 = vector3_to_float32_array(merged_normals.ptr(), merged_normals.size());
			const int simplify_options = SurfaceTool::SIMPLIFY_LOCK_BORDER;

			size_t new_index_count = SurfaceTool::simplify_with_attrib_func(
					(unsigned int *)new_indices.ptrw(),
					(const uint32_t *)merged_indices_ptr, index_count,
					merged_vertices_f32.ptr(), merged_vertex_count,
					sizeof(float) * 3, // Vertex stride
					index_target,
					max_mesh_error,
					simplify_options,
					&mesh_error,
					merged_normals_f32.ptr(),
					normal_weights.ptr(), 3);

			if (new_index_count < last_index_count * 1.5f) {
				index_target = index_target * 1.5f;
				return _indices;
			}

			if (new_index_count == 0 || (new_index_count >= (index_count * 0.75f))) {
			}
			if (new_index_count > 5000000) {
				// This limit theoretically shouldn't be needed, but it's here
				// as an ad-hoc fix to prevent a crash with complex meshes.
				// The crash still happens with limit of 6000000, but 5000000 works.
				// In the future, identify what's causing that crash and fix it.
				WARN_PRINT("Mesh LOD generation failed for mesh surface 1, mesh is too complex. Some automatic LODs were not generated.");
				return _indices;
			}

			new_indices.resize(new_index_count);

			LocalVector<LocalVector<int>> vertex_corners;
			vertex_corners.resize(vertex_count);
			{
				int *ptrw = new_indices.ptrw();
				for (unsigned int j = 0; j < new_index_count; j++) {
					const int &remapped = vertex_inverse_remap[ptrw[j]];
					vertex_corners[remapped].push_back(j);
					ptrw[j] = remapped;
				}
			}

			if (raycaster.is_valid()) {
				float error_factor = 1.0f / (scale * MAX(mesh_error, 0.15));
				const float ray_bias = 0.05;
				float ray_length = ray_bias + mesh_error * scale * 3.0f;

				Vector<StaticRaycaster::Ray> rays;
				LocalVector<Vector2> ray_uvs;

				int32_t *new_indices_ptr = new_indices.ptrw();

				int current_ray_count = 0;
				for (unsigned int j = 0; j < new_index_count; j += 3) {
					const Vector3 &v0 = vertices_ptr[new_indices_ptr[j + 0]];
					const Vector3 &v1 = vertices_ptr[new_indices_ptr[j + 1]];
					const Vector3 &v2 = vertices_ptr[new_indices_ptr[j + 2]];
					Vector3 face_normal = vec3_cross(v0 - v2, v0 - v1);
					float face_area = face_normal.length(); // Actually twice the face area, since it's the same error_factor on all faces, we don't care
					if (!Math::is_finite(face_area) || face_area == 0) {
						WARN_PRINT_ONCE("Ignoring face with non-finite normal in LOD generation.");
						continue;
					}

					Vector3 dir = face_normal / face_area;
					int ray_count = CLAMP(5.0 * face_area * error_factor, 16, 64);

					rays.resize(current_ray_count + ray_count);
					StaticRaycaster::Ray *rays_ptr = rays.ptrw();

					ray_uvs.resize(current_ray_count + ray_count);
					Vector2 *ray_uvs_ptr = ray_uvs.ptr();

					for (int k = 0; k < ray_count; k++) {
						float u = pcg.randf();
						float v = pcg.randf();

						if (u + v >= 1.0f) {
							u = 1.0f - u;
							v = 1.0f - v;
						}

						u = 0.9f * u + 0.05f / 3.0f; // Give barycentric coordinates some padding, we don't want to sample right on the edge
						v = 0.9f * v + 0.05f / 3.0f; // v = (v - one_third) * 0.95f + one_third;
						float w = 1.0f - u - v;

						Vector3 org = v0 * w + v1 * u + v2 * v;
						org -= dir * ray_bias;
						rays_ptr[current_ray_count + k] = StaticRaycaster::Ray(org, dir, 0.0f, ray_length);
						rays_ptr[current_ray_count + k].id = j / 3;
						ray_uvs_ptr[current_ray_count + k] = Vector2(u, v);
					}

					current_ray_count += ray_count;
				}

				raycaster->intersect(rays);

				LocalVector<Vector3> ray_normals;
				LocalVector<real_t> ray_normal_weights;

				ray_normals.resize(new_index_count);
				ray_normal_weights.resize(new_index_count);

				for (unsigned int j = 0; j < new_index_count; j++) {
					ray_normal_weights[j] = 0.0f;
				}

				const StaticRaycaster::Ray *rp = rays.ptr();
				for (int j = 0; j < rays.size(); j++) {
					if (rp[j].geomID != 0) { // Ray missed
						continue;
					}

					if (rp[j].normal.normalized().dot(rp[j].dir) > 0.0f) { // Hit a back face.
						continue;
					}

					const float &u = rp[j].u;
					const float &v = rp[j].v;
					const float w = 1.0f - u - v;

					const unsigned int &hit_tri_id = rp[j].primID;
					const unsigned int &orig_tri_id = rp[j].id;

					const Vector3 &n0 = normals_ptr[indices_ptr[hit_tri_id * 3 + 0]];
					const Vector3 &n1 = normals_ptr[indices_ptr[hit_tri_id * 3 + 1]];
					const Vector3 &n2 = normals_ptr[indices_ptr[hit_tri_id * 3 + 2]];
					Vector3 normal = n0 * w + n1 * u + n2 * v;

					Vector2 orig_uv = ray_uvs[j];
					const real_t orig_bary[3] = { 1.0f - orig_uv.x - orig_uv.y, orig_uv.x, orig_uv.y };
					for (int k = 0; k < 3; k++) {
						int idx = orig_tri_id * 3 + k;
						real_t weight = orig_bary[k];
						ray_normals[idx] += normal * weight;
						ray_normal_weights[idx] += weight;
					}
				}

				for (unsigned int j = 0; j < new_index_count; j++) {
					if (ray_normal_weights[j] < 1.0f) { // Not enough data, the new normal would be just a bad guess
						ray_normals[j] = Vector3();
					} else {
						ray_normals[j] /= ray_normal_weights[j];
					}
				}

				LocalVector<LocalVector<int>> normal_group_indices;
				LocalVector<Vector3> normal_group_averages;
				normal_group_indices.reserve(24);
				normal_group_averages.reserve(24);

				for (unsigned int j = 0; j < vertex_count; j++) {
					const LocalVector<int> &corners = vertex_corners[j];
					const Vector3 &vertex_normal = normals_ptr[j];

					for (const int &corner_idx : corners) {
						const Vector3 &ray_normal = ray_normals[corner_idx];

						if (ray_normal.length_squared() < CMP_EPSILON2) {
							continue;
						}

						bool found = false;
						for (unsigned int l = 0; l < normal_group_indices.size(); l++) {
							LocalVector<int> &group_indices = normal_group_indices[l];
							Vector3 n = normal_group_averages[l] / group_indices.size();
							if (n.dot(ray_normal) > normal_pre_split_threshold) {
								found = true;
								group_indices.push_back(corner_idx);
								normal_group_averages[l] += ray_normal;
								break;
							}
						}

						if (!found) {
							normal_group_indices.push_back({ corner_idx });
							normal_group_averages.push_back(ray_normal);
						}
					}

					for (unsigned int k = 0; k < normal_group_indices.size(); k++) {
						LocalVector<int> &group_indices = normal_group_indices[k];
						Vector3 n = normal_group_averages[k] / group_indices.size();

						if (vertex_normal.dot(n) < normal_split_threshold) {
							split_vertex_indices.push_back(j);
							split_vertex_normals.push_back(n);
							int new_idx = split_vertex_count++;
							for (const int &index : group_indices) {
								new_indices_ptr[index] = new_idx;
							}
						}
					}

					normal_group_indices.clear();
					normal_group_averages.clear();
				}
			}

			index_target = MAX(new_index_count, index_target) * 2;
			last_index_count = new_index_count;
			return new_indices;
}


void MultiresolutionMeshBuilder::simplify_verticies_indicies_by_quadric_edge_collapse(Vector<Vector3> &p_verticies, List<int> &p_indices) {

	for (Vector3 vert : p_verticies) {
		vertices.append(Vertex{ vert });
	}

	auto curr = p_indices.front();
	while (curr != nullptr) {

		Triangle triangle;
		triangle.indices[0] = curr->get();
		curr = curr->next();
		triangle.indices[1] = curr->get();
		curr = curr->next();
		triangle.indices[2] = curr->get();
		curr = curr->next();
		/*
		int triangles_i = triangles.size(); 
		vertices.write[triangle.indices[0]].tstart = triangles_i;
		vertices.write[triangle.indices[1]].tstart = triangles_i;
		vertices.write[triangle.indices[2]].tstart = triangles_i;
		*/

		triangles.append(triangle);
	}

	simplify_by_quadric_edge_collapse(triangles.size() / 3);

	Vector<Vector3> multi_res_verticies;
	List<int> multi_res_indices;

	for (Vertex vert : vertices) {
		multi_res_verticies.append(vert.p);
	}

	for (Triangle tri : triangles) {
		for (int index : tri.indices) {
			multi_res_indices.push_back(index);
		}
	}

	p_verticies = multi_res_verticies;
	p_indices = multi_res_indices;
}



/*
void MultiresolutionMeshBuilder::generate_multiresolution_mesh(Vector<Surface> &surfaces, float p_normal_merge_angle, float p_normal_split_angle, Array p_skin_pose_transform_array) {

	try {
		
	} catch (const std::string &ex) {
		print_line("generate_multiresolution_mesh caught an exception:\n ", ex);
	}

	for (int i = 0; i < surfaces.size(); i++) {
		if (surfaces[i].primitive != Mesh::PRIMITIVE_TRIANGLES) {
			continue;
		}

		//surfaces.write[i].lods.clear();

		//Vector<Vector3> _vertices = surfaces[i].arrays[RS::ARRAY_VERTEX];
		PackedInt32Array indices = surfaces[i].arrays[RS::ARRAY_INDEX];
		Vector<Vector3> _normals = surfaces[i].arrays[RS::ARRAY_NORMAL];

		for (Vector3 vert : (Vector<Vector3>)surfaces[i].arrays[RS::ARRAY_VERTEX]) {
			vertices.append(Vertex{ vert });
		}

		for (int i = 0; i < indices.size(); i += 3) {
			triangles.append(Triangle{ { indices[i], indices[i + 1], indices[i + 2] } });
		}

		simplify_by_quadric_edge_collapse(triangles.size()/ 2);

		Vector<Vector3> multi_res_verticies;
		PackedInt32Array multi_res_indices;
		for (Vertex vert : vertices) {
			multi_res_verticies.append(vert.p);
		}

		for (Triangle tri: triangles) {
			for (int index : tri.indices) {
				multi_res_indices.append(index);
			}
		}
		auto a = surfaces[i].arrays[RS::ARRAY_INDEX];

		//RenderingServer::mesh_create_arrays_from_surface_data();

		//surfaces[i].arrays[RS::ARRAY_INDEX] = (const Variant)multi_res_indices;
		
		Vector<Vector2> uvs = surfaces[i].arrays[RS::ARRAY_TEX_UV];
		Vector<Vector2> uv2s = surfaces[i].arrays[RS::ARRAY_TEX_UV2];
		Vector<int> bones = surfaces[i].arrays[RS::ARRAY_BONES];
		Vector<float> weights = surfaces[i].arrays[RS::ARRAY_WEIGHTS];
	}
}*/

void MultiresolutionMeshBuilder::group_triangles_to_nodes(PackedInt32Array indices) {
}

void MultiresolutionMeshBuilder::simplify_by_quadric_edge_collapse(int target_count, double agressiveness) {
	// From: https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification/commit/f613d996cdc95c3f2f5390255b6f12ff8e9beb76

	// init
	//printf("%s - start\n", __FUNCTION__);
	//int timeStart = timeGetTime();

	for (int i = 0; i < triangles.size(); ++i) {
		triangles.write[i].deleted = 0;
	}

	// main iteration loop for quadric edge collapse

	int deleted_triangles = 0;
	Vector<int> deleted0, deleted1;
	int triangle_count = triangles.size();

	for (int iteration = 0; iteration < 100; ++iteration) {
		// target number of triangles reached ? Then break
		printf("iteration %d - triangles %d\n", iteration, triangle_count - deleted_triangles);
		if (triangle_count - deleted_triangles <= target_count) break;

		// update mesh once in a while
		if (iteration % 5 == 0) {
			update_mesh(iteration);
		}

		// clear dirty flag
		for (int i = 0; i < triangles.size(); ++i) {
			triangles.write[i].dirty = 0;
		}

		//
		// All triangles with edges below the threshold will be removed
		//
		// The following numbers works well for most models.
		// If it does not, try to adjust the 3 parameters
		//
		double threshold = 0.000000001 * pow(double(iteration + 3), agressiveness);

		// remove vertices & mark deleted triangles
		for (int i = 0; i < triangles.size(); ++i) {
			const Triangle &t = triangles[i];
			if (t.err[3] > threshold)
				continue;
			if (t.deleted)
				continue;
			if (t.dirty)
				continue;

			for (int j = 0; j < 3; ++j) if (t.err[j] < threshold) {
				int i0 = t.indices[j];
				const Vertex &v0 = vertices[i0];
				int i1 = t.indices[(j + 1) % 3];
				const Vertex &v1 = vertices[i1];

				// Border check
				if (v0.border != v1.border)
					continue;

				// Compute vertex to collapse to
				Vector3 p;
				calculate_error(i0, i1, p);

				deleted0.resize(v0.tcount); // normals temporarily
				for (auto d : deleted0) {
					d = 0;
				}
				deleted1.resize(v1.tcount); // normals temporarily
				for (auto d : deleted1) {
					d = 0;
				}

				// don't remove if flipped
				if (flipped(p, i0, i1, v0, v1, deleted0))
					continue;
				if (flipped(p, i1, i0, v1, v0, deleted1))
					continue;

				// not flipped, so remove edge
				vertices.write[i0].p = p;
				vertices.write[i0].q = v1.q + v0.q;
				int tstart = refs.size();

				update_triangles(i0, v0, deleted0, deleted_triangles);
				update_triangles(i0, v1, deleted1, deleted_triangles);

				int tcount = refs.size() - tstart;

				if (tcount <= v0.tcount) {
					// save ram
					if (tcount)
						memcpy(&refs.write[v0.tstart], &refs[tstart], tcount * sizeof(VertRef));
				}
				else {
					// append
					vertices.write[i0].tstart = tstart;
				}
									
				vertices.write[i0].tcount = tcount;
				break;
			}
			// done?
			if (triangle_count - deleted_triangles <= target_count) {
				break;
			}
		}
	}

	// clean up mesh
	compact_mesh();

	// ready
	//int timeEnd = timeGetTime();
	/*
	printf("%s - %d/%d %d%% removed in %d ms\n", __FUNCTION__,
			triangle_count - deleted_triangles,
			triangle_count, deleted_triangles * 100 / triangle_count,
			timeEnd - timeStart);*/
}

// Check if a triangle flips when this edge is removed

bool MultiresolutionMeshBuilder::flipped(Vector3 p, int i0, int i1, const Vertex &v0, const Vertex &v1, Vector<int> &deleted) {
	int bordercount = 0;
	for (int k = 0; k < v0.tcount; ++k) {
		const Triangle &t = triangles[refs[v0.tstart + k].tid];
		if (t.deleted)
			continue;

		int s = refs[v0.tstart + k].tvertex;
		int id1 = t.indices[(s + 1) % 3];
		int id2 = t.indices[(s + 2) % 3];
		
		if (id1 == i1 || id2 == i1) // delete ?
		{
			bordercount++;
			deleted.write[k] = 1;
			continue;
		}

		Vector3 d1 = vertices[id1].p - p;
		d1.normalize();
		Vector3 d2 = vertices[id2].p - p;
		d2.normalize();
		if (fabs(d1.dot(d2)) > 0.999)
			return true;

		Vector3 n = d1.cross(d2);
		n.normalize();
		deleted.write[k] = 0;
		if (n.dot(t.n) < 0.2)
			return true;
	}
	return false;
}

// Update triangle connections and edge error after a edge is collapsed

void MultiresolutionMeshBuilder::update_triangles(int i0, const Vertex &v, const Vector<int> &deleted, int &deleted_triangles) {
	Vector3 p;
	for (int k = 0; k < v.tcount; ++k) {
		const VertRef &r = refs[v.tstart + k];
		Triangle &t = triangles.write[r.tid];
		if (t.deleted)
			continue;
		if (deleted[k]) {
			t.deleted = 1;
			deleted_triangles++;
			continue;
		}
		t.indices[r.tvertex] = i0;
		t.dirty = 1;
		t.err[0] = calculate_error(t.indices[0], t.indices[1], p);
		t.err[1] = calculate_error(t.indices[1], t.indices[2], p);
		t.err[2] = calculate_error(t.indices[2], t.indices[0], p);
		t.err[3] =Math::min(t.err[0], Math::min(t.err[1], t.err[2]));
		refs.push_back(r);
	}
}

// compact triangles, compute edge error and build reference list

void MultiresolutionMeshBuilder::update_mesh(int iteration) {
	if (iteration > 0) // compact triangles
	{
		int dst = 0;
		for (int i = 0; i < triangles.size(); ++i) {
			if (!triangles[i].deleted) {
				triangles.write[dst++] = triangles[i];
			}
		}

		triangles.resize(dst);
	}

	// Init Reference ID list
	for (int i = 0; i < vertices.size(); ++i) {
		vertices.write[i].tstart = 0;
		vertices.write[i].tcount = 0;
	}
	for (int i = 0; i < triangles.size(); ++i) {
		/* for (int j = 0; j < 3; ++j) {
			vertices.write[triangles[i].indices[j]].tcount++; //count the number of triangle that contains this vertex
		}*/
		Triangle &t = triangles.write[i];
		for (int j = 0; j < 3; ++j) vertices.write[t.indices[j]].tcount++;
	}

	// tstart is the cumulative sum of of tcount 
	int tstart = 0;
	for (int i = 0; i < vertices.size(); ++i) {
		Vertex &v = vertices.write[i];
		v.tstart = tstart;
		tstart += v.tcount;
		v.tcount = 0;
	}

	// Write References
	// refs is in the order of vertices, where each "block" is size of numer of references (t.tstart + v.tcount) with ref metadata to that vert.
	//  example:
	//   v0 ref 0, v0 ref 1, v0 ref 2, v1 ref 0, v1 ref 1, v2 ref0, v2 ref1, v2 ref2
	//   |____________v0____________|  |________v1______|  |________v2_____________|
	refs.resize(triangles.size() * 3);
	for (int i = 0; i < triangles.size(); ++i) {
		Triangle &t = triangles.write[i];
		for (int j = 0; j < 3; ++j) {
			//Vertex v = vertices.write[triangles[i].indices[j]];
			Vertex &v = vertices.write[t.indices[j]];
			refs.write[v.tstart + v.tcount].tid = i;
			refs.write[v.tstart + v.tcount].tvertex = j;
			v.tcount++;
		}
	}

	// Init Quadrics by Plane & Edge Errors
	//
	// required at the beginning ( iteration == 0 )
	// recomputing during the simplification is not required,
	// but mostly improves the result for closed meshes
	//
	if (iteration == 0) {
		// Identify boundary : vertices[].border=0,1

		Vector<int> vcount, vids;

		for (int i = 0; i < vertices.size(); ++i)
				vertices.write[i]
						.border = 0;

		for (int i = 0; i < vertices.size(); ++i) {
			Vertex &v = vertices.write[i];
			vcount.clear();
			vids.clear();
			for (int j = 0; j < v.tcount; ++j) {
				int k = refs[v.tstart + j].tid;
				Triangle &t = triangles.write[k];
				for (int k = 0; k < 3; ++k) {
				int ofs = 0, id = t.indices[k];
				while (ofs < vcount.size()) {
					if (vids[ofs] == id)
						break;
					ofs++;
				}
				if (ofs == vcount.size()) {
					vcount.push_back(1);
					vids.push_back(id);
				} else
					vcount.write[ofs]++;
				}
			}
			for (int j = 0; j < vcount.size(); ++j) {
				if (vcount[j] == 1) {
					vertices.write[vids[j]].border = 1;
				}
			}
		}
		//initialize errors
		for (int i = 0; i < vertices.size(); ++i) {
			vertices.write[i].q = SymetricMatrix(0.0);
		}
			
		for (int i = 0; i < triangles.size(); ++i) {
			Triangle &t = triangles.write[i];
			Vector3 n, p[3];
			for (int j = 0; j < 3; ++j) p[j] = vertices[t.indices[j]].p;
			n = (p[1] - p[0]).cross(p[2] - p[0]);
			n.normalize();
			t.n = n;
			for (int j = 0; j < 3; ++j) {
				vertices.write[t.indices[j]].q =
						vertices[t.indices[j]].q + SymetricMatrix(n.x, n.y, n.z, -n.dot(p[0]));

			}
		}
		for (int i = 0; i < triangles.size(); ++i) {
			// Calc Edge Error
			Triangle &t = triangles.write[i];
			Vector3 p;
			for (int j = 0; j < 3; ++j) {
				t.err[j] = calculate_error(t.indices[j], t.indices[(j + 1) % 3], p);
			}

			t.err[3] = Math::min(t.err[0], Math::min(t.err[1], t.err[2]));
		}
	}
}

// Finally compact mesh before exiting

void MultiresolutionMeshBuilder::compact_mesh() {
	int dst = 0;
	for (int i = 0; i < vertices.size(); ++i) {
		vertices.write[i].tcount = 0;
	}
	for (int i = 0; i < triangles.size(); ++i){
		if (!triangles[i].deleted) {
			Triangle &t = triangles.write[i];
			triangles.write[dst++] = t;
			for (int j = 0; j < 3; ++j) {
				vertices.write[t.indices[j]].tcount = 1;
			}
		}
	}
	triangles.resize(dst);
	dst = 0;
	for (int i = 0; i < vertices.size(); ++i){
		if (vertices[i].tcount) {
			vertices.write[i].tstart = dst;
			vertices.write[dst].p = vertices[i].p;
			dst++;
		}
	}

	for (int i = 0; i < triangles.size(); ++i) {
		auto &t = triangles.write[i];
		for (int j = 0; j < 3; ++j) {
			t.indices[j] = vertices[t.indices[j]].tstart;
		}
	}
		
	vertices.resize(dst);
}

// Error between vertex and Quadric

double MultiresolutionMeshBuilder::vertex_error(SymetricMatrix q, double x, double y, double z) {
	return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x + q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y + q[7] * z * z + 2 * q[8] * z + q[9];
}

// Error for one edge

double MultiresolutionMeshBuilder::calculate_error(int id_v1, int id_v2, Vector3 &p_result) {
	// compute interpolated vertex

	SymetricMatrix q = vertices[id_v1].q + vertices[id_v2].q;
	bool border = vertices[id_v1].border & vertices[id_v2].border;
	double error = 0;
	double det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);

	if (det != 0 && !border) {
		// q_delta is invertible
		p_result.x = -1 / det * (q.det(1, 2, 3, 4, 5, 6, 5, 7, 8)); // vx = A41/det(q_delta)
		p_result.y = 1 / det * (q.det(0, 2, 3, 1, 5, 6, 2, 7, 8)); // vy = A42/det(q_delta)
		p_result.z = -1 / det * (q.det(0, 1, 3, 1, 4, 6, 2, 5, 8)); // vz = A43/det(q_delta)
		error = vertex_error(q, p_result.x, p_result.y, p_result.z);
	} else {
		// det = 0 -> try to find best result
		Vector3 p1 = vertices[id_v1].p;
		Vector3 p2 = vertices[id_v2].p;
		Vector3 p3 = (p1 + p2) / 2;
		double error1 = vertex_error(q, p1.x, p1.y, p1.z);
		double error2 = vertex_error(q, p2.x, p2.y, p2.z);
		double error3 = vertex_error(q, p3.x, p3.y, p3.z);
		error = Math::min(error1, Math::min(error2, error3));
		if (error1 == error)
			p_result = p1;
		if (error2 == error)
			p_result = p2;
		if (error3 == error)
			p_result = p3;
	}
	return error;
}

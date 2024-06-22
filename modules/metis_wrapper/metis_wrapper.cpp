/* tts.cpp */

#include "metis_wrapper.h"

#include <metis.h>

int MetisWrapper::Call_METIS_PartGraphKway(String p_txt){

	/* (idx_t *nvtxs, idx_t *ncon, idx_t *xadj,
		idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt,
		idx_t *nparts, real_t *tpwgts, real_t *ubvec, idx_t *options,
		idx_t *edgecut, idx_t *part){*/
	idx_t nvtxs;
	idx_t ncon;
	idx_t xadj;
	idx_t adjncy;
	idx_t vwgt;
	idx_t vsize;
	idx_t adjwgt;
	idx_t nparts;
	real_t tpwgts;
	real_t ubvec;
	idx_t options;
	idx_t edgecut;
	idx_t part;

	// Call the function with variables
	//return METIS_PartGraphKway(nvtxs, ncon, xadj, adjncy, vwgt, vsize, adjwgt, nparts, tpwgts, ubvec, options, edgecut, part);
	return METIS_PartGraphKway(&nvtxs, &ncon, &xadj, &adjncy, &vwgt, &vsize, &adjwgt, &nparts, &tpwgts, &ubvec, &options, &edgecut, &part);
}

void MetisWrapper::partition(const Vector<Vector3> &p_vertices, const PackedInt32Array &p_indices, const PackedInt32Array &o_edgecut, const PackedInt32Array &o_part){

	HashMap<int32_t, PackedInt32Array> adj_graph;

	int i = 0;
	for (int i = 0; i < p_indices.size(); i += 3) {
		add_adj_pair_old(adj_graph, p_indices[i], p_indices[i + 1]);
		add_adj_pair_old(adj_graph, p_indices[i + 1], p_indices[i + 2]);
		add_adj_pair_old(adj_graph, p_indices[i + 2], p_indices[i]);
	}
	
	PackedInt32Array xadj_vec;
	PackedInt32Array adjncy_vec;
	// setup xadj and adjncy in the compressed storage format specified in the metis lib manual
	int32_t xadj_val = 0;
	for (int32_t vertex_i = 0; vertex_i	< p_vertices.size(); vertex_i ++) {
		xadj_vec.append(xadj_val); // starting index of adjncy that stores the adjencies of vertex_i
		adjncy_vec.append_array(adj_graph[vertex_i]);
		xadj_val += adj_graph[vertex_i].size(); // update starting index of adjncy that stores the adjencies for the NEXT vertex
		xadj_vec.append(xadj_val);
	}

	idx_t nvtxs = p_vertices.size();
	idx_t ncon = 1;
	//idx_t xadj = (int32_t *)&xadj_vec[0];
	//idx_t adjncy;
	//idx_t vwgt;
	//idx_t vsize;
	//idx_t adjwgt = nullptr; 
	idx_t nparts = 4;
	//real_t tpwgts;
	//real_t ubvec;
	idx_t options = 200;
	idx_t edgecut;
	idx_t part;

	//auto a = o_edgecut.ptrw();
	// Call the function with variables
	//return METIS_PartGraphKway(nvtxs, ncon, xadj, adjncy, vwgt, vsize, adjwgt, nparts, tpwgts, ubvec, options, edgecut, part);

	int status = METIS_PartGraphKway(
		&nvtxs, 
		&ncon, 
		(int32_t *)&xadj_vec[0], // TODO check size > 0
		(int32_t *)&adjncy_vec[0],
		nullptr, 
		nullptr, 
		// null for adjwgt, believe for grouping vertices, edges have same weight
		// only count neighbor edge weight for grouping nodes partition
		nullptr, //&adjwgt, 
		&nparts, 
		nullptr, 
		nullptr, 
		&options, 
		(int32_t *)&o_edgecut[0], 
		(int32_t *)&o_part	[0]);
}

void MetisWrapper::add_adj_pair_old(HashMap<int32_t, PackedInt32Array> &p_adj_graph, int32_t index_a,  int32_t index_b){
	if (p_adj_graph.has(index_a)){
		p_adj_graph[index_a].append(index_b);
	}
	else {
		p_adj_graph[index_a] = Vector<int32_t>{ index_b };
	}

	if (p_adj_graph.has(index_b)){
		p_adj_graph[index_b].append(index_a);
	}
	else {
		p_adj_graph[index_b] = Vector<int32_t>{ index_a };
	}
}

void MetisWrapper::_bind_methods() {

	ClassDB::bind_method(D_METHOD("Call_METIS_PartGraphKway","txt"), &MetisWrapper::Call_METIS_PartGraphKway);
	ClassDB::bind_method(D_METHOD("partition","p_vertices", "p_indices"), &MetisWrapper::partition);
}
/*"nvtxs", "ncon", "xadj",
		"adjncy", "vwgt", "vsize", "adjwgt",
		"nparts", "tpwgts", "ubvec", "options",
		"edgecut", "part")*/
MetisWrapper::MetisWrapper() {
	//festival_initialize(true, 210000); //not the best way to do it as this should only ever be called once.
}

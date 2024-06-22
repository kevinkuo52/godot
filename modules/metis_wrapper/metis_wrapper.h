#ifndef GODOT_MetisWrapper_H
#define GODOT_MetisWrapper_H

#include "core/object/ref_counted.h"
#include <metis.h>

class MetisWrapper : public RefCounted {
	GDCLASS(MetisWrapper, RefCounted);

protected:
	static void _bind_methods();

public:
	int Call_METIS_PartGraphKway(String txt);
			/* (idx_t *nvtxs, idx_t *ncon, idx_t *xadj,
		idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt,
		idx_t *nparts, real_t *tpwgts, real_t *ubvec, idx_t *options,
		idx_t *edgecut, idx_t *part)*/;
	
	void partition(const Vector<Vector3> &p_vertices, const PackedInt32Array &p_indices, const PackedInt32Array &o_edgecut, const PackedInt32Array &o_part);
	MetisWrapper();
private:
	void add_adj_pair_old(HashMap<int32_t, PackedInt32Array> &p_adj_graph, int32_t index_a,  int32_t index_b);
};

#endif // GODOT_MetisWrapper_H

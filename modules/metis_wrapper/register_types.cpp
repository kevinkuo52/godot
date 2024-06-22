#include "register_types.h"

#include "core/object/class_db.h"
#include "metis_wrapper.h"
//#include "thirdparty/METIS/include/metis.h"

void initialize_metis_wrapper_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	ClassDB::register_class<MetisWrapper>();
}

void uninitialize_metis_wrapper_module(ModuleInitializationLevel p_level) {
	// Nothing to do here in this example.
}

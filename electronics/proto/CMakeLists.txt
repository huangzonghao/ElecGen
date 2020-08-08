protobuf_generate_cpp(PROTO_SRC PROTO_HEADER component_structure.proto)
add_library(component_structure ${PROTO_HEADER} ${PROTO_SRC})
set_target_properties(component_structure PROPERTIES PUBLIC_HEADER ${PROTO_HEADER})
if(MSVC)
    # to make protobuf run with dll
    target_compile_definitions(component_structure PUBLIC PROTOBUF_USE_DLLS)
endif()

set(PROTO_COMPILED_INCLUDE ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
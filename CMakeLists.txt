cmake_minimum_required(VERSION 3.7)

project(ezmath)

file(GLOB EZMATH_H_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*/*.h")
file(GLOB EZMATH_TCC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*/*.tcc")

add_library(ezmath INTERFACE)
target_compile_features(ezmath INTERFACE "cxx_std_20")
target_compile_features(ezmath INTERFACE "cxx_aggregate_default_initializers")
target_compile_features(ezmath INTERFACE "cxx_attributes")
target_compile_features(ezmath INTERFACE "cxx_auto_type")
target_compile_features(ezmath INTERFACE "cxx_constexpr")
target_compile_features(ezmath INTERFACE "cxx_decltype")
target_compile_features(ezmath INTERFACE "cxx_decltype_auto")
target_compile_features(ezmath INTERFACE "cxx_explicit_conversions")
target_compile_features(ezmath INTERFACE "cxx_final")
target_compile_features(ezmath INTERFACE "cxx_lambdas")
target_compile_features(ezmath INTERFACE "cxx_noexcept")
target_compile_features(ezmath INTERFACE "cxx_nullptr")
target_compile_features(ezmath INTERFACE "cxx_override")
target_compile_features(ezmath INTERFACE "cxx_range_for")
target_compile_features(ezmath INTERFACE "cxx_rvalue_references")
target_compile_features(ezmath INTERFACE "cxx_variable_templates")
target_compile_features(ezmath INTERFACE "cxx_variadic_macros")
target_compile_features(ezmath INTERFACE "cxx_variadic_templates")
target_compile_features(ezmath INTERFACE "cxx_template_template_parameters")

target_sources(ezmath INTERFACE ${EZMATH_H_FILES} ${EZMATH_TCC_FILES})
target_include_directories(ezmath INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include/Math")

# ======================================================================
# Dependencies =========================================================
# ======================================================================

# ezcommon
if (NOT TARGET ezcommon)
  add_subdirectory(deps/ezcommon)
  target_link_libraries(ezmath INTERFACE ezcommon)
endif()

# ======================================================================
# ======================================================================
# ======================================================================

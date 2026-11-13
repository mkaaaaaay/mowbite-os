# Registers a component library like rclcpp_components_register_node (same arguments) and lets only the registration
# leave it: the symbols of the templates and of the static cores stay inside, which the loader would otherwise map
# and relocate in every process (22 MB → 4.3 MB RSS for the schedule, measured with test_nx60i_stack).
function(mowbite_component_register target)
  set_target_properties(${target} PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
  target_link_options(${target} PRIVATE -Wl,--exclude-libs,ALL)
  target_include_directories(${target} PRIVATE
    $<TARGET_PROPERTY:mowbite_component::mowbite_component,INTERFACE_INCLUDE_DIRECTORIES>)
  rclcpp_components_register_node(${target} ${ARGN})
endfunction()

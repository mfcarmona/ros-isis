FILE(REMOVE_RECURSE
  "../src/collaborative_control/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/collaborative_control/srv/__init__.py"
  "../src/collaborative_control/srv/_collaborative_control_s.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

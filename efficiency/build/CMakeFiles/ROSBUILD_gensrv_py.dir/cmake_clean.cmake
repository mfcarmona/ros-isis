FILE(REMOVE_RECURSE
  "../src/efficiency/msg"
  "../src/efficiency/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/efficiency/srv/__init__.py"
  "../src/efficiency/srv/_Efficiency_s.py"
  "../src/efficiency/srv/_Efficiency_s2.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

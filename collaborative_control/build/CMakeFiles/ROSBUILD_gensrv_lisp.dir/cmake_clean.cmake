FILE(REMOVE_RECURSE
  "../src/collaborative_control/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/collaborative_control_s.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_collaborative_control_s.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

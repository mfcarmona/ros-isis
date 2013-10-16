FILE(REMOVE_RECURSE
  "../src/efficiency/msg"
  "../src/efficiency/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/Efficiency_s.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Efficiency_s.lisp"
  "../srv_gen/lisp/Efficiency_s2.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Efficiency_s2.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

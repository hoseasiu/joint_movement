FILE(REMOVE_RECURSE
  "../src/joint_movement/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/joint_state.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_joint_state.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

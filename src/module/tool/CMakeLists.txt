add_subdirectory (VGRModeler)
add_subdirectory (multicalib)

if (GLUT_FOUND)
  add_subdirectory (calibtest)
endif (GLUT_FOUND)

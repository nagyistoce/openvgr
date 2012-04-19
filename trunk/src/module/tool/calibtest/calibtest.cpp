#include <cstdio>

#include <cv.h>
#include "glwindow.hpp"

int
main (int argc, char** argv)
{
  glw_init (&argc, argv);

  glw_run ();

  return EXIT_SUCCESS;
}

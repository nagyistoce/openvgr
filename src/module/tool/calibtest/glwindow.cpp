#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <GL/freeglut.h>

#include <quaternion.h>

#include "calibtest.hpp"
#include "capture.hpp"
#include "glwindow.hpp"
#include "globj.hpp"
#include "cv3d.hpp"

const static int update_interval = 250;

enum mouse_state {
  MS_NORMAL, MS_ROTATE, MS_TRANS
};

static int s_root_window = 0;
static int s_window_width  = 500;
static int s_window_height = 500;

static mouse_state s_mouse_state = MS_NORMAL;
static int s_mouse_pos[2] = {0, 0};
static float s_mouse_depth = 1.0;

static globj::Scene s_scene;
static globj::CoordinateAxes s_axes;

typedef std::vector<globj::TransformedCamera> TransformedCamera_list_t;
static TransformedCamera_list_t s_cameras;

static globj::DepthMap s_depth_map;

static quaternion_t s_view_q = QUAT_INIT_ONE;
static float s_view_pos[3] = {0.0, 0.0, 0.0};

static Capture* s_cap = 0;
static int s_selected_camera = 0;

/* callback functions */
void display ();
void reshape (int width, int height);
void keyboard (unsigned char key, int x, int y);
void mouse (int button, int state, int x, int y);
void motion (int x, int y);

/* static functions */
static TransformedCamera_list_t create_cameras (Capture& cap);
static void set_projection_matrix ();
static void mult_qt (const quaternion_t q, const float t[3]);
static void update_view_pose ();
static void move_view_pose (const int x1, const int y1, const int x2, const int y2);
static void rotate_view_pose (const int x1, const int y1, const int x2, const int y2);

static cv::Mat convert_to_Mat (const capture_frame_t& frame);
static void update_image ();

template <class T>
static void print_mat44 (T* mat);

void update(int) {
  update_image();
  glutTimerFunc (update_interval, update, 0);
}

/*
 * global functions
 */
void
glw_init (int* argcp, char** argv)
{
  try
    {
      s_cap = new Capture();
    }
  catch (const Capture::Exception& e)
    {
      printf ("oops: %s\n", e.message().c_str());
      return exit (EXIT_FAILURE);
    }

  /* initialization */
  glutInit (argcp, argv);
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize (s_window_width, s_window_height);

  /* create a root window */
  s_root_window = glutCreateWindow ("calibtest");
  
  /* set callback functions for the root window */
  glutDisplayFunc (display);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);

  /* set camera parameters */
  s_cameras = create_cameras (*s_cap);
  s_depth_map = globj::DepthMap (s_cap->m_params[0], s_cap->m_params[1], cv::Size (s_cap->width (), s_cap->height ()));
}

void
glw_run ()
{
  glutTimerFunc (500, update, 0);
  glutMainLoop ();
}

void
glw_exit ()
{
  glutDestroyWindow (s_root_window);
  delete s_cap;

#ifdef GLUT_ACTION_EXIT
  glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  glutLeaveMainLoop ();
#else
  exit (EXIT_SUCCESS);
#endif
}


/*
 * callback functions
 */

void
display ()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable (GL_DEPTH_TEST);

  s_axes.draw ();

  s_depth_map.draw ();

  for (TransformedCamera_list_t::iterator cam = s_cameras.begin (); cam != s_cameras.end (); ++cam)
    {
      (*cam).draw ();
    }

  glutSwapBuffers ();
}

void
reshape (int width, int height)
{
  s_window_width = width;
  s_window_height = height;

  glViewport (0, 0, width, height);

  set_projection_matrix ();
}

void
keyboard (unsigned char key, int x, int y)
{
  switch (key)
    {
    case 'q':
    case 'Q':
    case 0x1b:
      glw_exit();
      break;

    case '1':
      s_selected_camera = 0;
      set_projection_matrix ();
      glutPostRedisplay ();
      break;

    case '2':
      s_selected_camera = 1;
      set_projection_matrix ();
      glutPostRedisplay ();
      break;
    }
}

void
mouse (int button, int state, int x, int y)
{
  if (state == GLUT_DOWN)
    {
      glReadPixels (x, s_window_height - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &s_mouse_depth);
      //printf("depth: %f\n", s_mouse_depth);
      s_mouse_pos[0] = x;
      s_mouse_pos[1] = y;

      switch (button)
        {
        case GLUT_LEFT_BUTTON:
          s_mouse_state = MS_ROTATE;
          break;

        case GLUT_RIGHT_BUTTON:
          s_mouse_state = MS_TRANS;
          break;

        case GLUT_MIDDLE_BUTTON:
          {
            GLdouble model_mat[4*4], proj_mat[4*4], pos[3];
            GLint viewport[4];
            
            glGetDoublev (GL_MODELVIEW_MATRIX, model_mat);
            glGetDoublev (GL_PROJECTION_MATRIX, proj_mat);
            glGetIntegerv (GL_VIEWPORT, viewport);

            gluUnProject (x, viewport[3] - y, s_mouse_depth, model_mat, proj_mat, viewport, &pos[0], &pos[1], &pos[2]);
            printf ("% f % f % f\n", pos[0], pos[1], pos[2]);
          }
          break;

        default:
          s_mouse_state = MS_NORMAL;
        }
    }
  else
    {
      s_mouse_state = MS_NORMAL;
    }
}

void
motion (int x, int y)
{
  switch (s_mouse_state)
    {
    case MS_ROTATE:
      rotate_view_pose (x, y, s_mouse_pos[0], s_mouse_pos[1]);
      s_mouse_pos[0] = x;
      s_mouse_pos[1] = y;

      update_view_pose ();
      break;

    case MS_TRANS:
      move_view_pose (x, y, s_mouse_pos[0], s_mouse_pos[1]);
      s_mouse_pos[0] = x;
      s_mouse_pos[1] = y;

      update_view_pose ();
      break;

    default:
      ;
    }
}

/*
 * misc functions
 */
static TransformedCamera_list_t
create_cameras (Capture& cap)
{
  TransformedCamera_list_t cam_list;
  typedef Capture::CameraParameter_set::const_iterator iterator_type;

  for (iterator_type param = cap.m_params.begin(); param != cap.m_params.end(); ++param)
    {
#if 0
      for (int i = 0; i < 4; ++i)
        {
          for (int j = 0; j < 4; ++j)
            {
              printf("% f ", param->ext.at<double>(i, j));
            }
          printf("\n");
        }
      printf("\n");
#endif

      cv::Mat R = param->ext (cv::Rect (0, 0, 3, 3)).t ();
      cv::Mat t = - (R * param->ext (cv::Rect (3, 0, 1, 3)));

      float mat[4*4];
      for (int i = 0; i < 3; ++i)
        {
          for (int j = 0; j < 3; ++j)
            {
              mat[j + 4*i] = R.at<double>(j, i);
            }
          mat[i + 4*3] = t.at<double>(i, 0);
        }
      for (int i = 0; i < 4; ++i)
        {
          mat[3 + 4*i] = (i < 3) ? 0.0 : 1.0;
        }

      cam_list.push_back (globj::TransformedCamera (mat));
    }

  return cam_list;
}

static void
set_projection_matrix ()
{
  const CameraParameter& param = s_cap->m_params[s_selected_camera];
  const double camera_width = s_cap->width (s_selected_camera);
  const double camera_height = s_cap->height (s_selected_camera);
  const double divisor = (camera_width > camera_height) ? camera_width : camera_height;
  const double aspect_ratio = (double)s_window_width / (double)s_window_height;

  const static double near = 1.0, far = 5000.0;

  glMatrixMode (GL_PROJECTION);

  glPopMatrix ();

  double proj_mat[4*4];
#if 0
  glLoadIdentity ();
  gluPerspective (37.8, aspect_ratio, 1, 2000);

# if 0
  glGetDoublev (GL_PROJECTION_MATRIX, proj_mat);
  print_mat44 (proj_mat);
# endif
#else

  for (int i = 0; i < 4*4; ++i)
    {
      proj_mat[i] = 0.0;
    }
  proj_mat[0 + 4*0] = param.intr.at<double>(0, 0) * 2.0 / divisor / aspect_ratio;

  proj_mat[0 + 4*1] = param.intr.at<double>(0, 1) * 2.0 / divisor / aspect_ratio;
  proj_mat[1 + 4*1] = param.intr.at<double>(1, 1) * 2.0 / divisor;

  proj_mat[2 + 4*2] = -(far + near) / (far - near);
  proj_mat[3 + 4*2] = -1.0;

  proj_mat[2 + 4*3] = -2.0 * far * near / (far - near);

  //print_mat44 (proj_mat);

  glLoadMatrixd (proj_mat);
#endif

  glPushMatrix (); // save the original projection matrix

#if 0
  s_view_pos[0] = 0;
  s_view_pos[1] = -10;
  s_view_pos[2] = -100;

  quat_im (s_view_q, 0) = sin (M_PI / 2.0);
  quat_im (s_view_q, 1) = 0.0;
  quat_im (s_view_q, 2) = 0.0;
  quat_re (s_view_q) = cos (M_PI / 2.0);
#else
  for (int i = 0; i < 3; ++i)
    {
      s_view_pos[i] = param.ext.at<double>(i, 3);
    }

  quaternion_t q0, q;

  double R[3*3];
  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
        {
          R[j + 3*i] = param.ext.at<double>(j, i);
        }
    }
  quat_im (q0, 0) = sin (M_PI / 2.0);
  quat_im (q0, 1) = 0.0;
  quat_im (q0, 2) = 0.0;
  quat_re (q0) = cos (M_PI / 2.0);

  quat_q_from_R (q, R, 3);
  quat_conj (q, q);

  quat_mult (s_view_q, q, q0);
#endif

  mult_qt(s_view_q, s_view_pos);

  glMatrixMode (GL_MODELVIEW);
}

static void
mult_qt (const quaternion_t q, const float t[3])
{
  double mat[4*4];

  quat_R_from_q (mat, 4, q);
  for (int i = 0; i < 3; ++i)
    {
      mat[4*3 + i] = t[i];
      mat[4*i + 3] = 0.0;
    }
  mat[15] = 1.0;

  glMultMatrixd(mat);
}

static void
update_view_pose ()
{
  glMatrixMode (GL_PROJECTION);

  glPopMatrix (); // reset to the projection matrix
  glPushMatrix (); // save the original projection matrix

  mult_qt(s_view_q, s_view_pos);

  glMatrixMode (GL_MODELVIEW);

  glutPostRedisplay ();
}

static void
move_view_pose (const int x1, const int y1, const int x2, const int y2)
{
  GLdouble model_mat[4*4], proj_mat[4*4], from[3], to[3], diff[3], trans[3];
  GLint viewport[4];
  
  if (s_mouse_depth >= 1.0)
    {
      return;
    }

  glGetDoublev (GL_MODELVIEW_MATRIX, model_mat);
  glGetDoublev (GL_PROJECTION_MATRIX, proj_mat);
  glGetIntegerv (GL_VIEWPORT, viewport);

  //printf("%d %d %d %d\n", viewport[0], viewport[1], viewport[2], viewport[3]);
  //print_mat44 (model_mat); printf("\n");
  //print_mat44 (proj_mat); printf("\n");

  gluUnProject (x1, viewport[3] - y1, s_mouse_depth, model_mat, proj_mat, viewport, &from[0], &from[1], &from[2]);
  gluUnProject (x2, viewport[3] - y2, s_mouse_depth, model_mat, proj_mat, viewport, &to[0], &to[1], &to[2]);

#if 0
  printf("% f % f % f\n", from[0], from[1], from[2]);
  printf("% f % f % f\n", to[0], to[1], to[2]);
  printf("\n");
#endif

  for (int i = 0; i < 3; ++i)
    {
      diff[i] = 0;
    }

  for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
        {
          diff[j] += model_mat[j + 4*i] * (to[i] - from[i]);
        }
    }

  quat_rot (trans, s_view_q, diff);
  for (int i = 0; i < 3; ++i)
    {
      s_view_pos[i] -= trans[i];
    }

  update_view_pose ();
}

static void
rotate_view_pose (const int x1, const int y1, const int x2, const int y2)
{
  double rvec[2] = {y2 - y1, x2 - x1}, norm;
  double theta;
  quaternion_t q;

  norm = hypot (rvec[0], rvec[1]);
  theta = norm * M_PI / 180.0;

  for (int i = 0; i < 2; ++i)
    {
      quat_im (q, i) = sin (theta / 2.0) * rvec[i] / norm;
    }
  quat_im (q, 2) = 0.0;
  quat_re (q) = cos (theta / 2.0);
  
  //quat_print (q);

  quat_mult (s_view_q, q, s_view_q);
  quat_normalize (s_view_q);
}

static cv::Mat
convert_to_Mat (const capture_frame_t& frame)
{
  cv::Mat img;

  switch (frame.format)
    {
    case CAPTURE_FRAME_FORMAT_GRAY:
      img = cv::Mat (frame.height, frame.width, CV_8UC1, frame.raw_data);
      break;

    case CAPTURE_FRAME_FORMAT_RGB:
      img = cv::Mat (frame.height, frame.width, CV_8UC3, frame.raw_data);
      break;

    default:
      ;
    }

  return img;
}

static void
update_image ()
{
  const int ncameras = s_cap->num_cameras ();
  capture_frame_t* frames;

  frames = s_cap->capture ();
  //printf("num_cameras: %d\n", ncameras);

  if (ncameras < 2)
    {
      return;
    }

  s_depth_map.compute (convert_to_Mat (frames[0]), convert_to_Mat (frames[1]));

  glutPostRedisplay ();
}

template <class T>
static void
print_mat44 (T* mat)
{
  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
        {
          std::cout << std::setw(10) << mat[4*j + i] << " ";
        }
      std::cout << std::endl;
    }
}

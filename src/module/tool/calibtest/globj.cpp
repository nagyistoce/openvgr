#include <cstdio>
#include <vector>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "globj.hpp"

using namespace globj;

void Scene::draw_obj ()
{
  for (container_type::iterator i = m_objects.begin(); i != m_objects.end(); ++i)
    {
      (*i)->draw();
    }
}

GlObject *Scene::add (GlObject *obj)
{
  m_objects.push_back (obj);
  return obj;
}

void Scene::delete_all ()
{
  for (container_type::iterator i = m_objects.begin(); i != m_objects.end(); ++i)
    {
      delete *i;
    }  
}

void
CoordinateAxes::draw_obj ()
{
  glBegin (GL_LINES);

  /* x */
  glColor3f (1.0, 0.0, 0.0);
  glVertex3f (0.0, 0.0, 0.0);
  glVertex3f (m_length, 0.0, 0.0);

  /* y */
  glColor3f (0.0, 1.0, 0.0);
  glVertex3f (0.0, 0.0, 0.0);
  glVertex3f (0.0, m_length, 0.0);

  /* z */
  glColor3f (0.0, 0.0, 1.0);
  glVertex3f (0.0, 0.0, 0.0);
  glVertex3f (0.0, 0.0, m_length);

  glEnd ();
};

void
Camera::draw_obj ()
{
  glColor3f (0.0, 1.0, 1.0);
  //glTranslated (-5.0, 0.0, 0.0);
  glutSolidSphere (1.0, 18, 18);
}

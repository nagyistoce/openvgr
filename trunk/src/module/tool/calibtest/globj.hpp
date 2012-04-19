#ifndef GLOBJ_HPP
#define GLOBJ_HPP

#include <cstdio>
#include <vector>

#include <GL/gl.h>

namespace globj
{
  //! base class for all drawable classes
  class GlObject
  {
  protected:
    bool m_visible; //!< true if the object is visible.

  public:
    GlObject () : m_visible (true) {}

    virtual ~GlObject () {}

    //! Make the object visible.
    void show () { m_visible = true; }
    //! Make the object invisible.
    void hide () { m_visible = false; }

    //! A basic drawing function; draw if visible.
    virtual void draw ()
    {
      if (m_visible)
        {
          draw_obj ();
        }
    }

    //! Method to draw the shape of the object.
    virtual void draw_obj () = 0;
  };

  //! A class for a collection of GlObjects.
  class Scene : virtual public GlObject
  {
    typedef std::vector<GlObject*> container_type;
    container_type m_objects;

  public:
    void draw_obj ();
    GlObject* add (GlObject* obj);
    void delete_all ();
  };

  //! A simple object representing coordinate axes as line segments.
  class CoordinateAxes : virtual public GlObject
  {
    float m_length; //!< length of each line segment.

  public:
    CoordinateAxes (const float length = 10.0) : m_length (length)
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
    }

    ~CoordinateAxes ()
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
    }
    void draw_obj ();
  };

  //! Object with coordinate transformation.
  template <class T>
  class TransformedObject : public T
  {
    float m_mat[4*4]; //!< column-major matrix

  public:
    TransformedObject ()
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);

      for (size_t col = 0; col < 4; ++col)
        {
          for (size_t row = 0; row < 4; ++row)
            {
              m_mat[col * 4 + row] = (col != row) ? 0.0 : 1.0;
            }
        }
    }

    template <class _T>
    TransformedObject (const _T* mat)
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
      set_matrix (mat);
    }

    TransformedObject (const TransformedObject<T>& tobj)
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
      set_matrix (tobj.m_mat);
    }

    ~TransformedObject () {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
    }

    TransformedObject<T>& operator= (const TransformedObject<T>& rhs)
    {
      set_matrix (rhs.m_mat);
      return *this;
    }

    //! \brief Overriding draw function.
    //!
    //! Coordinate transformation is applied before calling T::draw().
    //! Then previous coordinate system will be recovered by glPopMatrix().
    void draw ()
    {
      glPushMatrix ();

      glMultMatrixf (m_mat);
      T::draw ();

      glPopMatrix ();
    }

    //! set matrix elements from a column-major array with 16 elements.
    template <class _T>
    void set_matrix (const _T *mat)
    {
      for (size_t i = 0; i < 4*4; ++i)
        {
          m_mat[i] = static_cast<float> (mat[i]);
        }
    }
  };

  //! Standard camera
  class Camera : virtual public GlObject
  {
    double m_alpha, m_beta, m_gamma;
    double m_u, m_v;

  public:
    Camera ()
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
    }
    ~Camera ()
    {
      //fprintf (stderr, "CALL(%d): %s\n", __LINE__, __FUNCTION__);
    }
    void draw_obj ();
  };

  //! Camera with coordinate transform
  typedef TransformedObject<Camera> TransformedCamera;
}

#endif /* GLOBJ_HPP */

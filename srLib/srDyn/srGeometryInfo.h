#ifndef	SRLIB_GEOMETRY_INFO
#define SRLIB_GEOMETRY_INFO

#ifdef WIN32
	#define _CRT_SECURE_NO_DEPRECATE 
	#define _CRT_SECURE_NO_WARNINGS 
	#define _CRT_NONSTDC_NO_DEPRECATE 

	#pragma warning ( disable:4996 )
#endif

#include "LieGroup/LieGroup.h"

/*!
	\class srGeometryInfo
	\brief Contains Geometric Information of Object which can be drawn on Screen.

	srGeometryInfo is a class for containing geometry information of srEntity and its siblings.
	Primitive geometries such as sphere, box, capsule, cylinder can be expressed.
	User can design own geometry information class derived from this.
*/
class srGeometryInfo
{
public:

	enum SHAPETYPE { ELLIPSOID, SPHERE, BOX, CAPSULE, CYLINDER, PLANE, TDS, USER };

	/*!
		Shape type of primitive geometry.
	*/
	SHAPETYPE	m_Type;

	/*!
		Dimension of primitive geometry in Vec3 form.
		ellipsoid: (scaleX, scaleY, scaleZ ).
		sphere: (diameter, not use, not use).
		box: (width, depth, height).
		capsule: (diameter, height w/o bulb, not use).
		cylinder:(diameter, height, not use).
	*/
	Vec3	m_Dimension;

	/*!
		Color of geometry in OpenGL manner. (R, G, B, alpha).
	*/
	float	m_Color[4];

	/*!
		Radius of bounding sphere wrapping geometry. This is used for collision detection.
	*/
	float	m_BoundingRadius;


	/*!
		File name.
	 */
	char	m_Filename[2048];
	/*!
		Local frame of a 3D object from the center of srEntity
	 */
	SE3		m_LocalFrame;

public:
			srGeometryInfo();

	/*!
		Get bounding radius.
	*/
	float&	GetBoundingRadius();
	/*!
		Re-calculate bounding radius in case dimension changed.
	*/
	void	UpdateBoundingRadius();
	/*!
		Set primitive geometry type.
	*/
	void	SetShape(SHAPETYPE );
	/*!
		Get primitive geometry type
	*/
	SHAPETYPE	GetShape();

	/*!
		Set file name.
	 */
	void	SetFileName(char* );

	/*!
		Get file name.
	 */
	char*	GetFileName();
	/*!
	 */
	void	SetLocalFrame(SE3 T);
	/*!
	 */
	SE3&	GetLocalFrame();
	/*!
		Set geometry dimension with Vec3-type argument.
	*/
	void	SetDimension(Vec3 );
	/*!
		Set geometry dimension with float array argument.
		argument is ensured that its type is float[3].
	*/
	void	SetDimension(float* );
	/*!
		Set geometry dimension with 3 float values.
	*/
	void	SetDimension(double w, double h = 0.0, double d = 0.0);

	/*!
		Get dimension.
	*/
	Vec3&	GetDimension();
	/*!
		Get dimension.
	*/
	void	GetDimension(float& , float& , float& );
	/*!
		Get dimension.
	*/
	void	GetDimension(SR_real& a, SR_real& b, SR_real& c);
	/*!
		Set color with float[4] array.
	*/
	void	SetColor(float* );
	/*!
		Set color with 4 float values
	*/
	void	SetColor(float, float, float);

	void	SetColor(float , float , float , float);
	/*!
		Get color.
	*/
	float*	GetColor();
	/*!
		Get color.
	*/
	void	GetColor(float&, float&, float&, float&);

};

#endif

#ifndef _UTILS_H_
#define _UTILS_H_

#include "LieGroup\rmatrix3.h"
#include "srDyn\srSpace.h"

#define SR_SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }
#define SR_SAFE_DESTROY_WINDOW(p) if(p) { p->DestroyWindow(); delete (p); (p) = NULL; }
#define SR_SAFE_DELETE_AR(p)		if(p) { delete [] p; (p) = NULL; }
#define SR_SAFE_RELEASE(p)		if(p) { (p)->Release(); (p) = NULL; }

class srUtils
{
public:
	static RMatrix getBodyJacobian(srLink* _link, SE3 offset = SE3());
	static RMatrix inverseKinematics(srLink* _link, SE3 _goal, RMatrix& q0, SE3 offset = SE3());
};

#endif // _UTILS_H_

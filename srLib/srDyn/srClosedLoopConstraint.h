#pragma once

#include "srDyn/srConstraint.h"


//**********************************************************************//
// Closed Loop
class ClosedLoop : public Constraint
{
public:
	ClosedLoop()
	{
		nd = 6;
		type1 = true;

		lambda[0] = 0.0;
		lambda[1] = 0.0;
		lambda[2] = 0.0;
		lambda[3] = 0.0;
		lambda[4] = 0.0;
		lambda[5] = 0.0;
	};

	// static members
	static void SetErp(SR_real _erp);
	static SR_real erp_closedloop;

	static void SetAllowedPenetration(SR_real _allowedpenetration);
	static SR_real allowederror;

	static void SetMaximumErpVelocity(SR_real _maximum_erp_velocity);
	static SR_real maximum_erp_velocity;

	// member variables
	static dse3 UnitImp[6]; // constraint jacobian : this is same for all closed loop, hence i chose to use static variable.
	SR_real	closedloopError[6];
	SR_real	lambda[6];

	srSystem	* pSystem;
	srLink	* pLeftMass;
	srLink	* pRightMass;

	SE3		HomeRelativeFrame;
	SE3		RelativeFrame;

	// member functions
	void		GetError(SR_real _recip_timestep);

	// virtual functions
	void		GetInformation(ConstraintInfo * info);
	void		ApplyImpulse(int _idx);
	void		GetDelVelocity(SR_real * sjari);
	void		Excite();
	void		UnExcite();
	void		SetImpulse(SR_real * _lambda);
	srSystem*	UF_Find_Constraint();

};
//**********************************************************************//


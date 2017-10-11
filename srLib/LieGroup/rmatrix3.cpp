//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3.cpp
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (zinook@kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//		SNU Robotics Lab
//		v2.896	some modifications for comming up to the ANSI/ISO C++, Syungkwon Ra(dearLenin@gmail.com) 2004.12.24
//		v2.8961 well compiled under gcc 3.3.5 of Linux
//
//////////////////////////////////////////////////////////////////////////////////


#include "rmatrix3.h"
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <cfloat>

#define DGEFA_EPS 1.0E-6   
#define LCP_EPS 1.0E-6

static clock_t _start;
void tic() { _start = clock(); }
double toc() { return (double)( clock() - _start ) / (double) CLOCKS_PER_SEC; }

int idamax(int n, double *dx)
{
	double dmax;
	int i, idamax = 0;

	if ( n < 1 ) return 0;
	if ( n == 1 ) return 1;

	dmax = fabs(dx[0]);
	for ( i = 1; i < n; i++ )
	{
		if ( fabs(dx[i]) > dmax )
		{
			idamax = i;
			dmax = fabs(dx[i]);
		}
	}
	return idamax;
}

void _dgefa(double *x, int lda, int n, int *jpvt, int &info)
{
	double t, *xk = x, *xj;
	int i, j, k, l;
	// gaussian elimination with partial pivoting
	info = -1;

	if ( n > 1 )
	{
		for ( k = 0; k < n - 1; k++, xk += lda )
		{
			// find l = pivot index
			l = idamax(n-k, xk+k) + k;
			jpvt[k] = l;
			// zero pivot implies this column already triangularized
			if ( fabs(xk[l]) < DGEFA_EPS ) info = k;
			else
			{
				// interchange if necessary
				if ( l != k )
				{
					t = xk[l];
					xk[l] = xk[k];
					xk[k] = t;
				}
				// compute multipliers
				t = -1.0 / xk[k];
				for ( j = 1 + k; j < n; j++ ) xk[j] *= t;
				// row elimination with column indexing
				for ( j = k+1, xj = xk+lda; j < n; j++, xj += lda )
				{
					t = xj[l];
					if ( l != k )
					{
						xj[l] = xj[k];
						xj[k] = t;
					}
					for ( i = 1 + k; i < n; i++ ) xj[i] += t * xk[i];
				}
			}
		}
	} else k = 0;

	jpvt[k] = k;
	if ( fabs(xk[k]) < DGEFA_EPS ) info = k;
	return;
}

void _dgesl(double *x, int lda, int n, int *jpvt, double *b, int job)
{
	double t, *xk = x;
	int k, l;

	if ( job == 0 ) 
	{
		// job = 0 , solve  a * x = b
		// first solve  l*y = b
		if ( n >= 2 )
		{
			for( k = 0; k < n-1; k++ )
			{
				l = jpvt[k];
				t = b[l];
				if ( l != k )
				{
					b[l] = b[k];
					b[k] = t;
				}
				for ( l = k+1; l < n; l++ ) b[l] += t * xk[l];
				xk += lda;				
			}
		}
		// now solve  u*x = y
		for ( k = n-1; k >= 0; k-- )
		{
			b[k] /= xk[k];
			t = -b[k];
			for ( l = 0; l < k; l++ ) b[l] += t * xk[l];
			xk -= lda;			
		}
		return;
	}

	// job = nonzero, solve  trans(a) * x = b
	// first solve  trans(u)*y = b
	for ( k = 0; k < n; k++ )
	{
		t = 0.0;
		for ( l = 0; l < k; l++ ) t += xk[l] * b[l];
		b[k] = (b[k] - t) / xk[k];
		xk += lda;
	}
	// now solve trans(l)*x = y
	if ( n >= 2 )
	{
		xk--;
		for ( k = n-1; k >= 0; k-- )
		{
			t = 0.0;			
			for ( l = 1; l < n-k; l++ ) t += xk[l] * b[k+l];
			b[k] += t;

			l = jpvt[k];
			if ( l != k )
			{
				t = b[l];
				b[l] = b[k];
				b[k] = t;
			}
			xk -= lda + 1;
		}
	}
	return;
}

// pivot x and b
// r, c : size of x
// k : starting pivot
// ipvt, jpvt : pivot index
void _fullpivoting(double *x, int r, int c, int k, int *ipvt, int *jpvt)
{
	int i, j, imax, jmax, itmp;
	double _max, dtmp, *xk = x + k + k * r, *xj;

	_max = fabs(*xk);
	imax = jmax = k;

	for ( j = k; j < c; j++ )
	{
		for ( i = k; i < r; i++ )
		{
			dtmp = fabs(*(xk++));
			if ( dtmp > _max ) { _max = dtmp; imax = i; jmax = j; }
		}
		xk += k;
	}

	// row swapping
	if ( imax != k )
	{
		itmp = ipvt[imax];	ipvt[imax] = ipvt[k]; 	ipvt[k] = itmp;
		for ( i = k, xk = x + k * r; i < c; i++, xk += r ) 
		{ 
			dtmp = xk[imax];	
			xk[imax] = xk[k];
			xk[k] = dtmp;			
		}
	}

	// column swapping
	if ( jmax != k ) 
	{
		itmp = jpvt[jmax];	jpvt[jmax] = jpvt[k];	jpvt[k] = itmp;
		for ( i = 0, xk = x + k * r, xj = x + jmax * r; i < r; i++, xk++, xj++ ) 
		{	
			dtmp = *xj;
			*xj = *xk;
			*xk = dtmp;			
		}
	}
}

// x a = b
// elimination process on x and b
// full pivoting on x and row pivoting on b
// if u want to ignore b, set bc = 0
// return value : rank of x
//
// x [ r X c ]
// b [ r X bc ]
// ipvt [ r ]
// jpvt [ c ]
// zero_tolerance : criterion for determining zero, 1e-8 will be good for usual case

int _gauss_elimination(double *x, int r, int c, int *ipvt, int *jpvt, double zero_tolerance)
{
	int i, j, k;
	double t, *xi = x, *xk;

	for ( i = 0; i < r; i++ ) ipvt[i] = i;
	for ( j = 0; j < c; j++ ) jpvt[j] = j;

	for ( i = 0; i < min(r,c); i++, xi += r )
	{
		_fullpivoting(x, r, c, i, ipvt, jpvt);
		if ( fabs(xi[i]) < zero_tolerance ) return i;
		else
		{
			for ( j = i+1; j < r; j++ )
			{
				t =  - xi[j] / xi[i];
				xi[j] = 0.0;
				for ( k = i+1, xk = xi + r; k < c; k++, xk += r ) xk[j] += t * xk[i];
			}
		}
	}
	return min(r,c);
}

int _dpofa(double *a, int n)
{
	double s, t;
	int i, j, k, info;
	// begin block with ...exits to 40
	for ( j = 0; j < n; j++ )
	{
		info = j;
		s = 0.0;
		for ( k = 0; k < j; k++ )
		{
			for ( t = 0.0, i = 0; i < k; i++ ) t += a[i+k*n] * a[i+j*n];
			t = a[k+j*n] - t;
			t /= a[k+k*n];
			a[k+j*n] = t;
			s += t * t;
		}
		s = a[j+j*n] - s;
		// exit
		if ( s <= 0.0 ) return info;
		a[j+j*n] = sqrt(s);
	}	
	return -1;
}

void _dposl(double *a, int n, double *b)
{
	double t;
	int i, k;

	// solve trans(r)*y = b
	for ( k = 0; k < n; k++ )
	{
		for ( t = 0.0, i = 0; i < k; i++ ) t += a[i+k*n] * b[i];
		b[k] = (b[k] - t) / a[k+k*n];
	}

	// solve r*x = y
	for ( k = n-1; k >= 0; k-- )
	{
		b[k] /= a[k+k*n];
		t = -b[k];
		for ( i = 0; i < k; i++ ) b[i] += t * a[i+k*n];		
	}
	return;
}

bool SolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B)
{
	if ( A.row * A.col == 1 ) 
	{
		if ( fabs(A.element[0]) < DGEFA_EPS ) return false;
		x = B / A.element[0];
		return true;
	}
	if ( A.row != B.row ) return false;
	if ( A.row != A.col )
	{
		if ( A.row > A.col ) return SolveAxEqualB(A ^ A, x, A ^ B);
		bool flag = SolveAxEqualB(A | A, x, B);
		x = A ^ x;
		return flag;
		//return QRSolveAxEqualB(A, x, B);
	}
	register int info, i;
	static IMatrix _ipvt_SlvAxB;
	static RMatrix _A_SlvAxB;

	_A_SlvAxB = A;
	x = B;

	if ( _ipvt_SlvAxB.row < _A_SlvAxB.row ) _ipvt_SlvAxB.ReNew(_A_SlvAxB.row, 1);

	_dgefa(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, _ipvt_SlvAxB.element, info);
	if ( info != -1 ) return false;
	for ( i = 0; i < x.col; i++ ) _dgesl(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, _ipvt_SlvAxB.element, x.element+x.row*i, 0);

	return true;
}

//_rmatrix operator & (const _rmatrix &m) const
/*bool SolveAtxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B)
{
if ( A.row * A.col == 1 )
{
if ( A[0] == 0.0 ) return false;
x = B / A[0];
return true;
}
if ( A.col != B.row ) return false;
if ( A.row != A.col )
return SolveAxEqualB(~A, x, B);
// return QRSolveAtxEqualB(A, x, B);

int info, i;
static IMatrix _ipvt_SlvAxB;
static RMatrix _A_SlvAxB;

_A_SlvAxB = A;
x = B;

if ( _ipvt_SlvAxB.row < _A_SlvAxB.row ) _ipvt_SlvAxB.ReNew(_A_SlvAxB.row, 1);

_dgefa(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, _ipvt_SlvAxB.element, info);
if ( info != -1 ) return false;
for ( i = 0; i < x.col; i++ ) _dgesl(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, _ipvt_SlvAxB.element, x.element+x.row*i, 1);

return true;
}
*/
double Det(RMatrix A)
{
	int info, i;
	static IMatrix _ipvt_Det;

	if ( _ipvt_Det.row < A.row ) _ipvt_Det.ReNew(A.row, 1);
	double re = 1.0;

	_dgefa(A.element, A.row, A.col, _ipvt_Det.element, info);

	for ( i = 0; i < A.row; i++ )
	{
		if ( i != _ipvt_Det.element[i] ) re = -re;
		re *= A.element[i+i*A.row];
	}
	return re;
}

bool SolvePosDefAxEqualB(RMatrix &A, RMatrix &x, const RMatrix &B)
{
	if ( A.row != A.col ) return false;
	x = B;
	if ( _dpofa(A.element, A.row) != -1 ) return false;
	for ( int i = 0; i < B.col; i++ )
		_dposl(A.element, A.row, x.element+i*x.row);
	return true;
}

double drand(double range)
{
	//	srand((unsigned)time(NULL));
	return 2.0 * range * ( (double)rand() / (double)RAND_MAX - 0.5 );
}

double drand(double min, double max)
{
	return min + (max - min) * (double)rand() / (double)RAND_MAX;
}

int GaussElimination(RMatrix &A, IMatrix &row_pivot, IMatrix &column_pivot)
{
	return GaussElimination(A, row_pivot, column_pivot, 1e-6);
}

int GaussElimination(RMatrix &A, IMatrix &row_pivot, IMatrix &column_pivot, double eps)
{
	row_pivot.ReNew(A.row);
	column_pivot.ReNew(A.col);
	return _gauss_elimination(A.element, A.row, A.col, row_pivot.element, column_pivot.element, eps);
}

#include "_array.h"
typedef _array <int> intArray;
#include <fstream>
static std::ofstream fout("lcp.log");

bool _fdirection(int d, RMatrix &delf, const RMatrix &A, const intArray &C)
{
	delf.SetZero();
	delf[d] = 1.0;

	RMatrix A11, v1, x;
	int i, j, n = C.get_size();

	A11.ReNew(n, n);
	v1.ReNew(n, 1);

	for ( i = 0; i < n; i++ )
	{
		for ( j = 0; j < n; j++ ) A11(i,j) = A(C[i], C[j]);
		v1[i] = A(C[i], d);
	}

	bool re = SolveAxEqualB(A11, x, -v1);
	if ( !re ) re = SVDSolveAxEqualB(A11, x, -v1);

	for ( i = 0; i < n; i++ ) delf[C[i]] = x[i];

	return re;
}

void _max_step(double &s, int &j, const RMatrix &f, const RMatrix &a, const RMatrix &delf, const RMatrix &dela, int d, const intArray &C, const intArray &NC)
{
	int i, k;
	double sp;

	s = DBL_MAX;
	j = -1;

	if ( dela[d] > LCP_EPS )
	{
		j = d;
		s = -a[d] / dela[d];
	}

	for ( k = 0; k < C.get_size(); k++ )
	{
		i = C[k];
		if ( delf[i] < -LCP_EPS )
		{
			sp = -f[i] / delf[i];
			if ( sp < s )
			{
				s = sp;
				j = i;
			}
		}
	}

	for ( k = 0; k < NC.get_size(); k++ )
	{
		i = NC[k];
		if ( dela[i] < -LCP_EPS )
		{
			sp = -a[i] / dela[i];
			if ( sp < s )
			{
				s = sp;
				j = i;
			}
		}
	}
}

bool _drive_to_zero(int d, const RMatrix &A, RMatrix &a, RMatrix &f, intArray &C, intArray &NC)
{
	int j, idx, cnt = 0;
	double s;
	RMatrix delf(a.RowSize(), 1);
	RMatrix dela(a.RowSize(), 1);

L1:
	if ( cnt++ > 10 * a.RowSize() * a.RowSize() )
	{
		fout << "LCP::_drive_to_zero -> fail" << std::endl;
		return false;
	}

	bool re = _fdirection(d, delf, A, C);
	if ( !re )
	{
		fout << "LCP::_fdirection -> fail" << std::endl;
		return false;
	}

	dela = A * delf;

	_max_step(s, j, f, a, delf, dela, d, C, NC);

	if ( j == -1 || s < -LCP_EPS )
	{
		fout << "LCP::_max_step -> fail" << std::endl;
		return false;
	}

	f += s * delf;
	a += s * dela;

	if ( (idx = C.find(j)) != -1 )
	{
		C.pop(idx);
		NC.add_tail(j);
		goto L1;
	} else if ( (idx = NC.find(j)) != -1 )
	{
		NC.pop(idx);
		C.add_tail(j);
		goto L1;
	} else
	{
		C.add_tail(j);
		return true;
	}
}

// find f such that Af + b >= 0 ,f >= 0 and <f, Af + b> = 0
bool SolveLCP(const RMatrix &A, RMatrix &f, const RMatrix &b)
{
	int i, idx, cnt = 0;
	RMatrix a = b;
	f = Zeros(b.row, 1);
	intArray C, NC;
	bool flag;

	while ( true )
	{
		flag = true;
		for ( i = 0; i < a.row; i++ ) 
		{
			if ( a.element[i] < -LCP_EPS )
			{
				flag = false;
				idx = i;
				break;
			}
		}

		if ( flag ) break;

		if ( !_drive_to_zero(idx, A, a, f, C, NC) ) return false;
	}

	return true;
}

RMatrix Zeros(int r, int c)
{
	RMatrix re(r, c);
	int n = r * c;
	double *_r = re.element;
	while ( n-- ) *(_r++) = 0.0;
	return re;
}

RMatrix Rand(int r, int c)
{
	//srand( (unsigned)time( NULL ) );
	RMatrix re(r, c);
	int n = r * c;
	double *_r = re.element;
	while ( n-- ) *(_r++) = drand(1.0);
	return re;		
}

RMatrix Eye(int r, int c )
{
	RMatrix re(r,c);
	int n = r * c;
	double *_r = re.element;
	while ( n-- ) *(_r++) = 0.0;

	c = min(r++,c);
	_r = re.element;
	while ( c-- )
	{
		*_r = 1.0;
		_r += r;
	}		
	return re;
}

RMatrix Eye(int r)
{
	return Eye(r,r);
}


// Made by Terry
RMatrix	GramSchmidt(RMatrix RMat_X, int dim)
{
	RMatrix* col = new RMatrix[dim];

	for(int i=0; i<dim; i++)
	{
		col[i].ReNew(dim,1);
	}

	for(int i=0; i<dim; i++)
	{
		for(int j=0; j<dim; j++)
			col[i](j,0) = RMat_X(j,i);
	}

	int nColSquare = dim-1;
	RMatrix* col_square = new RMatrix[nColSquare];
	for(int i=0; i<nColSquare; i++)
		col_square[i].ReNew(1,1);

	RMatrix** my_scalar = new RMatrix*[nColSquare];
	for(int i=0; i<nColSquare; i++)
	{
		my_scalar[i] = new RMatrix[i+1];
	}
	for(int i=0; i<nColSquare; i++)
	{
		for(int j=0; j<i+1; j++)
		{
			my_scalar[i][j].ReNew(1,1);
		}
	}

	int nScalar = 0;
	for(int i=1; i<=nColSquare; i++)
	{
		nScalar+= i;
	}
	double* adblScalar = new double[nScalar];

	nScalar = 0;

	double dblTemp;
	int nZeroColumn = -1;
	for(int i=0; i<nColSquare; i++)
	{
		col_square[i] = ~col[i]*col[i];
		dblTemp = col_square[i](0,0);
		for(int j=0; j<=i; j++)
		{
			my_scalar[i][j] = ~col[j]*col[i+1];

			if(col_square[i](0,0) > 0.0000001)
			{
				adblScalar[nScalar] = my_scalar[i][j](0,0)/col_square[j](0,0);			
				col[i+1] = col[i+1] - adblScalar[nScalar]*col[j];
			}
			else
			{
				nZeroColumn = i+1;
				break;
			}
			nScalar++;
		}
	}

	if(nZeroColumn != -1)
	{
		for(int i=0; i<nZeroColumn; i++)
			col[i].Normalize();
		for(int i=nZeroColumn; i<dim; i++)
			col[i].SetZero();
	}
	else
		for(int i=0; i<dim; i++)
			col[i].Normalize();


	RMatrix RMat_Result(dim, dim);
	for(int i=0; i<dim; i++)
	{
		for(int j=0; j<dim; j++)
			RMat_Result(j,i) = col[i](j,0);
	}


	delete []col;
	delete []col_square;
	delete []adblScalar;
	delete []my_scalar;

	return RMat_Result;
}

// Made by Terry : Gram Schmidt process when the first column is given.

RMatrix	GramSchmidt(RMatrix RMat_X, RMatrix RMat_col, int dim)
{
	RMatrix* col = new RMatrix[dim];

	for(int i=0; i<dim; i++)
	{
		col[i].ReNew(dim,1);
	}

	for(int i=0; i<dim; i++)
	{
		col[0](i,0) = RMat_col(i,0);
	}
	for(int i=1; i<dim; i++)
	{
		for(int j=0; j<dim; j++)
			col[i](j,0) = RMat_X(j,i);
	}

	int nColSquare = dim-1;
	RMatrix* col_square = new RMatrix[nColSquare];
	for(int i=0; i<nColSquare; i++)
		col_square[i].ReNew(1,1);

	RMatrix** my_scalar = new RMatrix*[nColSquare];
	for(int i=0; i<nColSquare; i++)
	{
		my_scalar[i] = new RMatrix[i+1];
	}
	for(int i=0; i<nColSquare; i++)
	{
		for(int j=0; j<i+1; j++)
		{
			my_scalar[i][j].ReNew(1,1);
		}
	}

	int nScalar = 0;
	for(int i=1; i<=nColSquare; i++)
	{
		nScalar+= i;
	}
	double* adblScalar = new double[nScalar];

	nScalar = 0;

	double dblTemp;
	int nZeroColumn = -1;
	for(int i=0; i<nColSquare; i++)
	{
		col_square[i] = ~col[i]*col[i];
		dblTemp = col_square[i](0,0);
		for(int j=0; j<=i; j++)
		{
			my_scalar[i][j] = ~col[j]*col[i+1];

			if(col_square[i](0,0) > 0.0000001)
			{
				adblScalar[nScalar] = my_scalar[i][j](0,0)/col_square[j](0,0);			
				col[i+1] = col[i+1] - adblScalar[nScalar]*col[j];
			}
			else
			{
				nZeroColumn = i+1;
				break;
			}
			nScalar++;
		}
	}

	if(nZeroColumn != -1)
	{
		for(int i=0; i<nZeroColumn; i++)
			col[i].Normalize();
		for(int i=nZeroColumn; i<dim; i++)
			col[i].SetZero();
	}
	else
		for(int i=0; i<dim; i++)
			col[i].Normalize();


	RMatrix RMat_Result(dim, dim);
	for(int i=0; i<dim; i++)
	{
		for(int j=0; j<dim; j++)
			RMat_Result(j,i) = col[i](j,0);
	}


	delete []col;
	delete []col_square;
	delete []adblScalar;
	delete []my_scalar;

	return RMat_Result;
}

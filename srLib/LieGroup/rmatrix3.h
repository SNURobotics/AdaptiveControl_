//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3.h
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (zinook@kist.re.kr)
//		last update	:	2003.9.2
//		Note		:
//
//		SNU Robotics Lab
//		v2.896	some modifications for comming up to the ANSI/ISO C++, Syungkwon Ra(dearLenin@gmail.com) 2004.12.24
//		v2.8961 well compiled under gcc 3.3.5 of Linux
//
//////////////////////////////////////////////////////////////////////////////////

/*
RMatrix3 is designed using template class.
However it is highly recommended to use only type of 'double'.
In case of using type of 'int' or 'float', there may be some internal conflictions with type of 'double'.
Hence if you find any errors or incompatibility in using _rmatrix <int>, please do not inform me that.

template <class TYPE> class _rmatrix
method : 
// constructors
_rmatrix()
_rmatrix(int r, int c)
_rmatrix(const _rmatrix &m)
template <class Type> 
_rmatrix(int r, int c, const Type d[])
// destructor
~_rmatrix()

// ith element in column order - zero based index
TYPE &operator [] (int i)
const TYPE &operator [] (int i) const
// ith row, jth column element
TYPE &operator () (int i, int j)
const TYPE &operator () (int i, int j) const
// unary plus operator
const _rmatrix &operator + (void) const
// unary minus operator
_rmatrix operator - (void) const
// transpose operator
_rmatrix operator ~ (void) const
// substitute operator
const _rmatrix &operator = (const _rmatrix &m)
// += operator
const _rmatrix &operator += (const _rmatrix &m)
// -= operator
const _rmatrix &operator -= (const _rmatrix &m)
// *= operator
const _rmatrix &operator *= (TYPE c)
// /= operator
const _rmatrix &operator /= (TYPE c)
_rmatrix operator + (const _rmatrix &m) const
_rmatrix operator - (const _rmatrix &m) const
_rmatrix operator * (const _rmatrix &m) const	:  (*this) *  m
_rmatrix operator ^ (const _rmatrix &m) const	: ~(*this) *  m
_rmatrix operator | (const _rmatrix &m) const	:  (*this) * ~m
_rmatrix operator * (TYPE c) const
_rmatrix operator / (TYPE c) const
_rmatrix operator % (const _rmatrix &m) const	: Inv(*this) * m
_rmatrix operator & (const _rmatrix &m) const	: Inv(~*this) * m

int RowSize(void) const
int ColSize(void) const
const _rmatrix &ReSize(int r, int c)
_rmatrix Sub(int rs, int re, int cs, int ce) const
const _rmatrix &Push(int i, int j, const _rmatrix &m);
_rmatrix &SetZero(void)
_rmatrix &SetZero(int r, c)

friend _rmatrix operator + (TYPE c, _rmatrix m) 
friend _rmatrix operator - (TYPE c, _rmatrix m) 
friend _rmatrix operator * (TYPE c, _rmatrix m)
friend _rmatrix operator / (TYPE c, _rmatrix m) 

friend ostream &operator << (ostream &os, const _rmatrix &m)
friend RMatrix Zeros(int r, int c)
friend RMatrix Rand(int r, int c)
friend Type SquareSum(_rmatrix m)
friend Type FNorm(_rmatrix m)
friend int GaussElimination(Rmatrix &A, IMatrix ipvt, IMatrix jpvt)
friend bool SolveAxEqualB(const RMatrix &A, RMatrix &x, const RMmatrix &B)
friend bool SolveAtxEqualB(const RMatrix &A, RMatrix &x, const RMmatrix &B)
friend bool SolvePosDefAxEqualB(RMatrix &A, RMatrix &x, const RMatrix &B)
friend void ConvColumn(RMatrix &re, const RMatrix &u, int i, const RMatrix &v, int j);
friend RMatrix Conv(const RMatrix &u, const RMatrix &v);
friend void Conv(RMatrix &re, const RMatrix &u, const RMatrix &v);
friend double Det(RMatrix A);
friend RMatrix Eig(RMatrix m);
friend void Eig(RMatrix &re, RMatrix &m);
friend void Eig(RMatrix m, RMatrix &v, RMatrix &d);
friend RMatrix Companion(const RMatrix &m);
friend void Companion(RMatrix &re, const RMatrix &m);
friend RMatrix Roots(const RMatrix& cof);
friend void Roots(RMatrix &re, const RMatrix& cof);
*/

#ifndef _RMatrix3_
#define _RMatrix3_

#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <vector>



#define _EPS 2.2204E-16
#define _INF 1E100

#ifndef max
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a, b)  (((a) < (b)) ? (a) : (b))
#endif
#define _is_same_size(m) ( row == m.row && col == m.col )
#define _size(m) ( m.row * m.col)
#define _length(m) ( max(m.row, m.col) )

template <class TYPE> class _rmatrix;
typedef _rmatrix <double> RMatrix;
typedef _rmatrix <int> IMatrix;

template <class TYPE> class _rmatrix
{
public:
	// default constructor
	_rmatrix() : row(0), col(0), mem_size(0), element(NULL) { }

	// constructor with dimension
	_rmatrix(int r, int c=1) : row(r), col(c), mem_size(r*c), element(new TYPE[mem_size]) 
	{
		for(int i=0; i<mem_size; i++)
			element[i] = 0;
	}

	// copy constructor
	_rmatrix(const _rmatrix &m) : row(m.row), col(m.col), mem_size(row*col), element(new TYPE[mem_size])
	{
		TYPE *_t = element, *_m = m.element;
		int n = mem_size;
		while ( n-- ) *(_t++) = *(_m++);
	}

	// constructor from single pointer, element is arranged along column
	_rmatrix(int r, int c, const double d[]) : row(r), col(c), mem_size(row*col), element(new TYPE[mem_size])
	{
		TYPE *_t = element;
		int n = mem_size;
		while ( n-- ) *(_t++) = *(d++);
	}

	_rmatrix(const std::vector<double>& vec) : row(vec.size()), col(1), mem_size(row*col), element(new TYPE[mem_size])
	{
		for(int i=0; i<mem_size; i++)
			element[i] = vec[i];
	}

	// destructor
	~_rmatrix()
	{
		delete [] element;
	}

	////////////////////////////////////////////////////////////////
	//
	// operators
	//
	////////////////////////////////////////////////////////////////

	// ith element in column order : zero-base, i = 0 : row*col-1
	TYPE &operator [] (int i)
	{
		assert(i >= 0 && i < row*col && "RMatrix3::operator[int] -> index over range");
		return element[i];
	}

	const TYPE &operator [] (int i) const
	{
		assert(i >= 0 && i < row*col && "RMatrix3::operator[int] -> index over range");
		return element[i];
	}

	// (i, j)th element
	TYPE &operator () (int i, int j ) 
	{
		assert(i >= 0 && i < row && j >= 0 && j < col && "RMatrix::operator(int, int) -> index over range");
		return element[i+j*row];
	}

	const TYPE &operator () (int i, int j ) const
	{
		assert(i >= 0 && i < row && j >= 0 && j < col && "RMatrix::operator(int, int) -> index over range");
		return element[i+j*row];
	}

	// unary plus operator
	const _rmatrix &operator + (void) const { return *this; }

	// unary minus operator
	_rmatrix operator - (void) const
	{
		_rmatrix re(row, col);
		TYPE *_m = re.element, *_t = element;
		int n = row * col;
		while ( n-- ) *(_m++) = -*(_t++);
		return re; 
	}

	// transpose operator
	_rmatrix operator ~ (void) const 
	{ 
		_rmatrix re(col, row);
		int i = 0, r = row, c;
		TYPE *_m = re.element, *_mt;
		while ( r-- )
		{
			_mt = element + (i++);
			c = col;
			while ( c-- )
			{
				*(_m++) = *_mt;
				_mt += row;
			}
		}
		return re;
	}

	// substitute operator
	const _rmatrix &operator = (const _rmatrix &m)
	{
		int n = m.row * m.col;
		if ( mem_size < n ) 
		{
			delete [] element;
			element = new TYPE [(mem_size = 2 * n)];
		}
		row = m.row;	col = m.col;
		TYPE *_t = element, *_m = m.element;
		while ( n-- ) *(_t++) = *(_m++);		
		return *this;
	}

	// += operator
	const _rmatrix &operator += (const _rmatrix &m)
	{
		assert(_is_same_size(m) && "_rmatrix::operator += (const _rmatrix &) -> size is not compatible");

		int n = row * col;
		TYPE *_t = element, *_m = m.element;
		while ( n-- ) *(_t++) += *(_m++);
		return *this;
	}

	// -= operator
	const _rmatrix &operator -= (const _rmatrix &m)
	{
		assert(_is_same_size(m) && "_rmatrix::operator -= (const _rmatrix &) -> size is not compatible");

		int n = row * col;
		TYPE *_t = element, *_m = m.element;
		while ( n-- ) *(_t++) -= *(_m++);
		return *this;
	}

	// *= operator
	const _rmatrix &operator *= (TYPE c)
	{
		int n = row * col;
		TYPE *_t = element;
		while ( n-- ) *(_t++) *= c;		
		return *this;
	}

	// -= operator
	const _rmatrix &operator /= (TYPE c)
	{
		int n = row * col;
		TYPE *_t = element, ci = (TYPE)1.0 / c;
		while ( n-- ) *(_t++) *= ci;		
		return *this;
	}

	// + operator 
	_rmatrix operator + (const _rmatrix &m) const
	{
		assert(_is_same_size(m) && "_rmatrix::operator + (const _rmatrix &) -> size is not compatible");

		_rmatrix re(row, col);
		int n = row * col;
		TYPE *_t = element, *_r = re.element, *_m = m.element;
		while ( n-- ) *(_r++) = *(_t++) + *(_m++);
		return re;
	}

	// - operator 
	_rmatrix operator - (const _rmatrix &m) const
	{
		assert(_is_same_size(m) && "_rmatrix::operator - (const _rmatrix &) -> size is not compatible");

		_rmatrix re(row, col);
		int n = row * col;
		TYPE *_t = element, *_r = re.element, *_m = m.element;
		while ( n-- ) *(_r++) = *(_t++) - *(_m++);
		return re;
	}

	// multiplication operator 
	_rmatrix operator * (const _rmatrix &m) const
	{
		RMatrix re;
		AMultB(re, *this, m);
		return re;
	}

	// multiplication operator A * ~B
	_rmatrix operator | (const _rmatrix &m) const
	{
		RMatrix re;
		AMultBt(re, *this, m);
		return re;	
	}

	// multiplication operator ~A * B
	_rmatrix operator ^ (const _rmatrix &m) const
	{
		RMatrix re;
		AtMultB(re, *this, m);
		return re;
	}

	_rmatrix operator * (TYPE c) const
	{
		_rmatrix re(row, col);
		int n = row * col;
		TYPE *_t = element, *_r = re.element;
		while ( n-- ) *(_r++) = c * *(_t++);
		return re;
	}

	_rmatrix operator / (TYPE c) const
	{
		_rmatrix re(row, col);
		int n = row * col;
		TYPE *_t = element, *_r = re.element, ci = (TYPE)1.0 / c;
		while ( n-- ) *(_r++) = ci * *(_t++);
		return re;
	}

	// return Inv(*this) * m
	_rmatrix operator % (const _rmatrix &m) const
	{
		_rmatrix x;
		SolveAxEqualB(*this, x, m);
		return x;
	}

	// return Inv(~(*this)) * m
	_rmatrix operator & (const _rmatrix &m) const
	{
		_rmatrix x;
		SolveAtxEqualB(*this, x, m);
		return x;
	}

	////////////////////////////////////////////////////////////////
	//
	// member functions
	//
	////////////////////////////////////////////////////////////////

	// return number of row
	int RowSize(void) const { return row; }

	// return number of column
	int ColSize(void) const { return col; }

	// renew matrix
	// newly made elements are not initialized.
	const _rmatrix &ReNew(int r, int c = 1)
	{
		assert(r >= 0 && c >= 0 && "_rmatrix::ReSize(int, int) -> index exceeds matrix dimensions");

		if ( r * c > mem_size )
		{
			delete [] element;
			element = new TYPE [(mem_size = 2 * r * c)];
		}
		row = r;
		col = c;
		return *this;
	}

	_rmatrix &SetZero(void)
	{
		int n = row * col;
		TYPE *_t = element;
		while ( n-- ) *(_t++) = (TYPE)0.0;
		return *this;
	}

	_rmatrix &SetZero(int r, int c)
	{
		int n = r * c;
		if ( n != row * col ) { delete [] element; element = new TYPE [n]; }
		row = r;
		col = c;
		TYPE *_t = element;
		while ( n-- ) *(_t++) = (TYPE)0.0;
		return *this;
	}

	_rmatrix &SetEye(int r, int c)
	{
		int n = r * c;
		if ( n != row * col ) { delete [] element; element = new TYPE [n]; }
		row = r;
		col = c;
		TYPE *_t = element;
		while ( n-- ) *(_t++) = (TYPE)0.0;

		if ( c > r ) c = r;
		r++;

		_t = element;
		while ( c-- )
		{
			*_t = (TYPE)1.0;
			_t += r;
		}
		return *this;
	}

	TYPE Normalize(void)
	{
		TYPE norm = FNorm(*this), inorm = 1.0 / norm;
		*this *= inorm;
		return norm;
	}

	////////////////////////////////////////////////////////////////
	//
	// friend functions
	//
	////////////////////////////////////////////////////////////////

	// concatenation of RMatrix
	//	friend _rmatrix Concat(int r, int c, ...);

	// * operator : * c compoenet wise
	friend _rmatrix operator * (TYPE c, _rmatrix m)
	{ 
		int n = m.row * m.col;
		TYPE *_m = m.element;
		while ( n-- ) *(_m++) *= c;
		return m;		
	}

	// ostream standard output
	friend std::ostream &operator << (std::ostream &os, const _rmatrix &m)
	{
		os.setf(std::ios::fixed);
		os << "[" << std::endl;
		for ( int i = 0; i < m.row; i++ )
			for ( int j = 0; j < m.col; j++ )
			{
				if ( m.element[i+j*m.row] >= (TYPE)0.0 ) os << " ";
				os << m.element[i+j*m.row];
				if ( j == m.col-1 ) os << " ;" << std::endl;
				else os << "  ";
			}
			os << "];" << std::endl;
			return os;
	}

	friend _rmatrix Inv(const _rmatrix &m)
	{		
		_rmatrix x;
		bool re = SolveAxEqualB(m, x, Eye(m.row, m.row));
		if ( !re ) 
			std::cerr << "no inverse of the matrix!" << std::endl;
		return x;		
	}

	friend _rmatrix pInv(const _rmatrix &m)
	{
		int i,j,r;

		i = m.RowSize();
		j = m.ColSize();
		r = Rank(m, 0.0001);

		if (j==r) // [] slim
		{
			return Inv(~m*m)*~m;
		}
		else if(i==r)
		{
			return ~m*Inv(m*~m);
		}
		else
		{
			RMatrix UM; RMatrix SM; RMatrix VM;
			SVD(m, UM, SM, VM);
			RMatrix IDSM(SM.RowSize(), SM.RowSize());
			IDSM.SetZero();
			for(int k=0; k<SM.RowSize(); k++)
			{
				//if(fabs(SM(k,0)) >= 0.00001)
				if(SM(k,0) >= 0.00001 || SM(k,0) <= -0.00001)
				{	IDSM(k,k) = 1/SM(k,0);	}
			}
			return VM*IDSM*~UM;
		}
	}



	friend TYPE SquareSum(const _rmatrix &m)
	{
		TYPE sum = (TYPE)0.0, *_m = m.element;
		int n = _size(m);
		while ( n-- ) sum += *_m * *(_m++);
		return sum;
	}

	friend TYPE Inner(const _rmatrix &a, const _rmatrix &b)
	{
		assert(_length(a) == _length(b) && "Inner(const _rmatrix &, const _rmatrix &) -> size is not compatible");

		TYPE sum = (TYPE)0.0, *_tmpa = a.element, *_tmpb = b.element;
		int n = _length(a);
		while ( n-- ) sum += *(_tmpa++) * *(_tmpb++);
		return sum;
	}

	friend TYPE FNorm(const _rmatrix &m)
	{
		return sqrt(SquareSum(m));
	}

	friend void AMultB(_rmatrix &re, const _rmatrix &a, const _rmatrix &b)
	{
		assert(a.col == b.row && "AMultB(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

		re.ReNew(a.row, b.col);
		int i, bc = b.col, k, ar;
		TYPE sum, *tmpa, *tmpb = b.element, *rij = re.element;
		while ( bc-- )
		{
			ar = a.row;
			i = 0;
			while ( ar-- )
			{
				tmpa = a.element + (i++);
				sum = (TYPE)0.0;
				k = a.col;
				while ( k-- )
				{
					sum += *tmpa * *tmpb;
					tmpa += a.row;
					tmpb++;
				}
				tmpb -= b.row;
				*(rij++) = sum;				
			}
			tmpb += b.row;
		}
	}

	friend void AMultBt(_rmatrix &re, const _rmatrix &a, const _rmatrix &b)
	{
		assert(a.col == b.col && "AMultBt(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

		int i, j = 0, br = b.row, ar, ac;
		re.ReNew(a.row, br);
		TYPE sum, *tmpa, *tmpb, *rij = re.element;

		while ( br-- )
		{
			ar = a.row;
			i = 0;
			while ( ar-- )
			{
				tmpa = a.element + (i++);
				tmpb = b.element + j;
				sum = (TYPE)0.0;
				ac = a.col;
				while ( ac-- )
				{
					sum += *tmpa * *tmpb;
					tmpa += a.row;
					tmpb += b.row;
				}
				*(rij++) = sum;
			}
			j++;
		}
	}

	friend void AtMultB(_rmatrix &re, const _rmatrix &a, const _rmatrix &b)
	{
		assert(a.row == b.row && "AtMultB(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

		re.ReNew(a.col, b.col);
		int ac, bc = b.col, ar;
		TYPE sum, *tmpa, *tmpb = b.element, *rij = re.element;
		while ( bc-- )
		{
			tmpa = a.element;
			ac = a.col;
			while ( ac-- )
			{
				sum = (TYPE)0.0;
				ar = a.row;
				while ( ar-- ) sum += *(tmpa++) * *(tmpb++);

				*(rij++) = sum;
				tmpb -= b.row;
			}
			tmpb += b.row;
		}
	}

	friend TYPE Quadratic(const _rmatrix &x, const _rmatrix &A, const _rmatrix &y)
	{
		assert((_length(x) == A.row || _length(y) == A.col) && "Quadratic(const _rmatrix &, const _rmatrix &. const _rmatrix &) -> size is not compatible");

		int r, c = A.col;
		TYPE sum = (TYPE)0.0, xa, *tmpa = A.element, *tmpx, *tmpy = y.element;
		while ( c-- )
		{
			xa = 0.0;
			tmpx = x.element;
			r = A.row;
			while ( r-- ) xa += *(tmpx++) * *(tmpa++);			
			sum += xa * *(tmpy++);
		}
		return sum;		
	}

	friend TYPE MaxVec(const _rmatrix &m, int *idx)
	{
		TYPE mx = m.element[0];
		if ( idx != NULL ) *idx = 0;
		for ( int i = 1; i < _size(m); i++ )
		{
			if ( m.element[i] > mx )
			{
				mx = m.element[i];
				if ( idx != NULL ) *idx = i;
			}
		}
		return mx;
	}

	friend TYPE MinVec(const _rmatrix &m, int *idx)
	{
		TYPE mn = m.element[0];
		if ( idx != NULL ) *idx = 0;
		for ( int i = 1; i < _size(m); i++ )
		{
			if ( m.element[i] < mn )
			{
				mn = m.element[i];
				if ( idx != NULL ) *idx = i;
			}
		}
		return mn;
	}

	friend TYPE Trace(const _rmatrix &m)
	{
		assert(m.row == m.col && "Trace(_rmatrix &) -> not square");

		TYPE tr = (TYPE)0.0, *tmp = m.element;
		int n = m.row;
		while ( n-- )
		{
			tr += *tmp;
			tmp += (m.row + 1);
		}
		return tr;
	}

	friend _rmatrix Diag(_rmatrix &m)
	{
		int n, i;
		_rmatrix re;
		if ( m.row == 1 || m.col == 1 )
		{
			n = _length(m);
			re = _rmatrix<TYPE>::Zeros(n,n);
			for ( i = 0; i < n; i++ ) re.element[i*n+i] = m.element[i];
		} else 
		{
			n = min(m.row, m.col);
			re.ReNew(n,1);
			for ( i = 0; i < n; i++ ) re.element[i] = m.element[i*m.row+i];
		}
		return re;
	}

	friend RMatrix	Zeros(int r, int c);
	friend RMatrix	Rand(int r, int c);
	friend RMatrix	Eye(int r, int c );
	friend RMatrix	Eye(int r);
	friend double	Det(RMatrix A);
	friend bool		SolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B);
	friend bool		SolveAtxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B);
	friend bool		SolvePosDefAxEqualB(RMatrix &A, RMatrix &x, const RMatrix &B);
	friend int		GaussElimination(RMatrix &A, IMatrix &row_pivot, IMatrix &column_pivot);
	friend int		GaussElimination(RMatrix &A, IMatrix &row_pivot, IMatrix &column_pivot, double eps);
	friend RMatrix	Eig(RMatrix m);
	friend void		Eig(RMatrix &re, RMatrix &m);
	friend void		Eig(RMatrix m, RMatrix &v, RMatrix &d);
	friend bool		QRSolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B);
	friend bool		QRSolveAtxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B);
	friend RMatrix	SVD(const RMatrix &M);
	friend void		SVD(const RMatrix &M, RMatrix &U, RMatrix &S, RMatrix &V);
	friend bool		SVDSolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B);
	friend int		Rank(const RMatrix &m, double singular_criteria);
	friend int		Rank(const RMatrix &m);
	friend bool		SolveLCP(const RMatrix &A, RMatrix &f, const RMatrix &b);

private:
	int row;
	int col;
	int mem_size;			// ÃÑ elementÀÇ °³¼ö
	TYPE *element;
};

void tic();
double toc();
double drand(double range = 1.0);
double drand(double min, double max);
RMatrix	Zeros(int r, int c);
RMatrix	Rand(int r, int c);
RMatrix	Eye(int r, int c );
RMatrix	Eye(int r);



// Made by Terry
RMatrix	GramSchmidt(RMatrix RMat_X, int dim);
RMatrix	GramSchmidt(RMatrix RMat_X, RMatrix RMat_col, int dim); // Gram Schmidt process when the first column is given.
#endif


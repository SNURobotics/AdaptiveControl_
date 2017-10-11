#pragma once

#include <vector>

using namespace std;
namespace vectorOp
{
	//	plus operator
	template<typename T>
	vector<T>	operator+ (const vector<T>& v1, const vector<T>& v2);
	template<typename T>
	vector<T>	operator+ (const vector<T>& v1, double k1);
	template<typename T>
	vector<T>	operator+ (double k1, const vector<T>& v1);

	template<typename T>
	vector<T>&	operator+= (vector<T>& v1, const vector<T>& v2);
	template<typename T>
	vector<T>&	operator+= (vector<T>& v1, double k1);
	
	//	minus operator
	template<typename T>
	vector<T>	operator- (const vector<T>& v1, const vector<T>& v2);
	template<typename T>
	vector<T>	operator- (const vector<T>& v1, double k1);
	template<typename T>
	vector<T>	operator- (double k1, const vector<T>& v1);

	template<typename T>
	vector<T>&	operator-= (vector<T>& v1, const vector<T>& v2);
	template<typename T>
	vector<T>&	operator-= (vector<T>& v1, double k1);

	//	multiplication  operator	
	template<typename T>
	vector<T>	operator* (const vector<T>& v1, double k1);
	template<typename T>
	vector<T>	operator* (double k1, const vector<T>& v1);

	template<typename T>
	vector<T>&	operator*= (vector<T>& v1, double k1);
	
	//	division  operator
	template<typename T>
	vector<T>	operator/ (const vector<T>& v1, double k1);
	template<typename T>
	vector<T>	operator/ (double k1, const vector<T>& v1);

	template<typename T>
	vector<T>&	operator/= (vector<T>& v1, double k1);
}


//	plus operator

template<typename T>
vector<T>	vectorOp::operator+(const vector<T>& v1, const vector<T>& v2)
{
	vector<T> v;
	if (v1.size() != v2.size())
		return v;
	else
	{
		v.resize(v1.size());
		for (unsigned int i = 0; i < v1.size(); i++)
			v[i] = v1[i] + v2[i];
	}
	return v;
}

template<typename T>
vector<T>	vectorOp::operator+ (const vector<T>& v1, double k1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] + k1;
	return v;
}

template<typename T>
vector<T>	vectorOp::operator+ (double k1, const vector<T>& v1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] + k1;
	return v;
}

template<typename T>
vector<T>&	vectorOp::operator+= (vector<T>& v1, const vector<T>& v2)
{
	for (unsigned int i = 0; i < v1.size() && i < v2.size(); i++)
		v1[i] += v2[i];
	return v1;
}

template<typename T>
vector<T>&	vectorOp::operator+= (vector<T>& v1, double k1)
{
	for (unsigned int i = 0; i < v1.size() ; i++)
		v1[i] += k1;
	return v1;
}

//	minus operator

template<typename T>
vector<T>	vectorOp::operator-(const vector<T>& v1, const vector<T>& v2)
{
	vector<T> v;
	if (v1.size() != v2.size())
		return v;
	else
	{
		v.resize(v1.size());
		for (unsigned int i = 0; i < v1.size(); i++)
			v[i] = v1[i] - v2[i];
	}
	return v;
}

template<typename T>
vector<T>	vectorOp::operator- (const vector<T>& v1, double k1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] - k1;
	return v;
}

template<typename T>
vector<T>	vectorOp::operator- (double k1, const vector<T>& v1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] - k1;
	return v;
}

template<typename T>
vector<T>&	vectorOp::operator-= (vector<T>& v1, const vector<T>& v2)
{
	for (unsigned int i = 0; i < v1.size() && i < v2.size(); i++)
		v1[i] -= v2[i];
	return v1;
}

template<typename T>
vector<T>&	vectorOp::operator-= (vector<T>& v1, double k1)
{
	for (unsigned int i = 0; i < v1.size(); i++)
		v1[i] -= k1;
	return v1;
}


//	multiplication  operator	
template<typename T>
vector<T>	vectorOp::operator* (const vector<T>& v1, double k1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] * k1;
	return v;
}

template<typename T>
vector<T>	vectorOp::operator* (double k1, const vector<T>& v1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] * k1;
	return v;
}

template<typename T>
vector<T>&	vectorOp::operator*= (vector<T>& v1, double k1)
{
	for (unsigned int i = 0; i < v1.size(); i++)
		v1[i] *= k1;
	return v1;
}

//	division  operator
template<typename T>
vector<T>	vectorOp::operator/ (const vector<T>& v1, double k1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] / k1;
	return v;
}

template<typename T>
vector<T>	vectorOp::operator/ (double k1, const vector<T>& v1)
{
	vector<T> v(v1.size());
	for (unsigned int i = 0; i < v1.size(); i++)
		v[i] = v1[i] / k1;
	return v;
}

template<typename T>
vector<T>&	vectorOp::operator/= (vector<T>& v1, double k1)
{
	for (unsigned int i = 0; i < v1.size(); i++)
		v1[i] /= k1;
	return v1;
}
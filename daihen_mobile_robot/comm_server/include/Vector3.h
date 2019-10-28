#ifndef STEP1_BOARD

#ifndef __Vector3_H__
#define __Vector3_H__

#ifdef _WINDOWS
#pragma warning(disable:4244)
#endif

class Quat;

class Vector3
{
public:
	float	x;
	float	y;
	float	z;
public:
	Vector3()
	{
		Set(0.f, 0.f, 0.f);
	}
	Vector3(float x, float y, float z)
	{
		Set(x, y, z);
	}
	Vector3(const Vector3& src)
	{
		*this = src;
	}
	void Set(float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	Vector3& operator=(const Vector3& src)
	{
		Set(src.x, src.y, src.z);
		return(*this);
	}
	Vector3 operator+(const Vector3& src) const
	{
		Vector3 t = *this;
		t.x += src.x;
		t.y += src.y;
		t.z += src.z;
		return(t);
	}
	Vector3 operator-(const Vector3& src) const
	{
		Vector3 t = *this;
		t.x -= src.x;
		t.y -= src.y;
		t.z -= src.z;
		return(t);
	}
	Vector3 operator*(const Vector3& src) const
	{
		Vector3 t = *this;
		t.x *= src.x;
		t.y *= src.y;
		t.z *= src.z;
		return(t);
	}
	Vector3 operator/(const Vector3& src) const
	{
		Vector3 t = *this;
		t.x /= src.x;
		t.y /= src.y;
		t.z /= src.z;
		return(t);
	}
	void operator+=(const Vector3& src)
	{
		*this = *this + src;
	}
	void operator-=(const Vector3& src)
	{
		*this = *this - src;
	}
	void operator*=(const Vector3& src)
	{
		*this = *this * src;
	}
	void operator/=(const Vector3& src)
	{
		*this = *this / src;
	}

	bool operator==(const Vector3& src) const
	{
		return((src.x == x) && (src.y == y) && (src.z == z));
	}
	bool operator!=(const Vector3& src) const
	{
		return(!(src == *this));
	}

	float GetRotateX() const
	{
		return(atan2(-y, z));
	}
	float GetRotateY() const
	{
		return(atan2(-z, x));
	}
	float GetRotateZ() const
	{
		return(-atan2(-y, x));
	}
	void RotateX(float t)
	{
		t = -t;
		float nz = z * cos(t) - y * sin(t);
		float ny = z * sin(t) + y * cos(t);
		z = nz;
		y = ny;
	}
	void RotateY(float t)
	{
		t = -t;
		float nx = x * cos(t) - z * sin(t);
		float nz = x * sin(t) + z * cos(t);
		x = nx;
		z = nz;
	}
	void RotateZ(float t)
	{
		float nx = x * cos(t) - y * sin(t);
		float ny = x * sin(t) + y * cos(t);
		x = nx;
		y = ny;
	}
	void Rotate(const Quat& rot);
	void Mul(float s)
	{
		x *= s;
		y *= s;
		z *= s;
	}
	void operator*=(float s)
	{
		Mul(s);
	}
	Vector3 operator*(float s) const
	{
		Vector3 t = *this;
		t *= s;
		return(t);
	}
	void Div(float s)
	{
		if( s == 0.0f || s == -0.0f )
			return;
		x /= s;
		y /= s;
		z /= s;
	}
	void operator/=(float s)
	{
		Div(s);
	}
	Vector3 operator/(float s) const
	{
		Vector3 t = *this;
		t /= s;
		return(t);
	}
	void operator*=(const Quat& rot)
	{
		Rotate(rot);
	}
	Vector3 operator*(const Quat& rot)
	{
		Vector3 v = *this;
		v.Rotate(rot);
		return(v);
	}

	void Zero()
	{
		Set(0.0f, 0.0f, 0.0f);
	}
	bool IsZero() const
	{
		return((x == 0.0f) && (y == 0.0f) && (z == 0.0f));
	}
	float Distance(const Vector3& p1) const
	{
		Vector3 t = *this - p1;
		return (float)sqrt(t.x*t.x + t.y*t.y + t.z*t.z);
	}

	void Cross(const Vector3& a, const Vector3& b)
	{
		x = (float)((a.y * b.z) - (a.z * b.y));
		y = (float)((a.z * b.x) - (a.x * b.z));
		z = (float)((a.x * b.y) - (a.y * b.x));
	}

	Vector3 Cross(const Vector3& b) const
	{
		const Vector3& a = *this;
		Vector3 t;
		t.x = (float)((a.y * b.z) - (a.z * b.y));
		t.y = (float)((a.z * b.x) - (a.x * b.z));
		t.z = (float)((a.x * b.y) - (a.y * b.x));
		return(t);
	}

	static float DotS(const Vector3& a, const Vector3& b)
	{
		return (float)((a.x * b.x) + (a.y * b.y) + (a.z * b.z));
	}

	static float AngleS(const Vector3& a, const Vector3& b)
	{
		return acos(DotS(a, b) / (a.Length() * b.Length()));
	}

	float Length() const
	{
		return(Distance(Vector3()));
	}

	void Normalize()
	{
		*this /= Length();
	}

	void Neg()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	String ToString() const
	{
		return Edit(_T("%.6f %.6f %.6f"), x, y, z);
	}

	bool Set(const TCHAR* pString)
	{
		float f1, f2, f3;
		_stscanf(pString, _T("%f %f %f"), &f1, &f2, &f3);
		Set(f1, f2, f3);
		return true;
	}
};

#endif

#endif // STEP1_BOARD

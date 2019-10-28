#ifndef STEP1_BOARD

#ifndef __Quat_H__
#define __Quat_H__

#ifndef _WINDOWS
#pragma GCC diagnostic ignored "-Wnarrowing"
#endif

class Quat
{
public:
	enum axis
	{
		AXIS_X = 1,
		AXIS_Y = 2,
		AXIS_Z = 3
	};
public:
	float x, y, z, w;
public:
	Quat()
	{
		Identity();
	}

	Quat(float x, float y, float z, float w)
	{
		Set(x, y, z, w);
	}

	Quat(const Quat& src)
	{
		*this = src;
	}

	Quat& operator=(const Quat& src)
	{
		Set(src.x, src.y, src.z, src.w);
		return(*this);
	}

	void Identity()
	{
		Set(0, 0, 0, 1);
	}

	void clear()
	{
		Identity();
	}

	void Set(float x, float y, float z, float w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}

	void SetAxis(float ax, float ay, float az, float radian)
	{
		Identity();

		float norm = ax*ax + ay*ay + az*az;
		if( norm <= 0.0 )
			return;

		norm = 1.0 / sqrt(norm);
		ax *= norm;
		ay *= norm;
		az *= norm;

		float s = sin(0.5 * radian);
		x = ax * s;
		y = ay * s;
		z = az * s;
		w = cos(0.5 * radian);
	}

	void SetAxis(const Vector3& axis, float radian)
	{
		SetAxis(axis.x, axis.y, axis.z, radian);
	}

	void RotateX(float radian)
	{
		Quat q;
		q.SetAxis(1, 0, 0, radian);
		*this *= q;
	}

	void RotateY(float radian)
	{
		Quat q;
		q.SetAxis(0, 1, 0, radian);
		*this *= q;
	}

	void RotateZ(float radian)
	{
		Quat q;
		q.SetAxis(0, 0, 1, radian);
		*this *= q;
	}

	Quat operator*(const Quat& b) const
	{
		const Quat& a = *this;

		Quat ans;
		float d1, d2, d3, d4;

		d1 =  a.w * b.w;
		d2 = -a.x * b.x;
		d3 = -a.y * b.y;
		d4 = -a.z * b.z;
		ans.w = d1 + d2 + d3 + d4;

		d1 =  a.w * b.x;
		d2 =  b.w * a.x;
		d3 =  a.y * b.z;
		d4 = -a.z * b.y;
		ans.x = d1 + d2 + d3 + d4;

		d1 =  a.w * b.y;
		d2 =  b.w * a.y;
		d3 =  a.z * b.x;
		d4 = -a.x * b.z;
		ans.y = d1 + d2 + d3 + d4;

		d1 =  a.w * b.z;
		d2 =  b.w * a.z;
		d3 =  a.x * b.y;
		d4 = -a.y * b.x;
		ans.z = d1+ d2 + d3 + d4;

		return ans;

		/*
		const Quaternion& a = *this;
		Quaternion q;
		q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
		q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
		q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
		q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
		return(q);
		*/
	}

	Quat& operator*=(const Quat& b)
	{
		*this = *this * b;
		return(*this);
	}

	void Neg()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	void Reverse()
	{
		Neg();
		w = -w;
	}

	String ToString(bool bHumanReadable=true)
	{
		if( bHumanReadable )
		{
			Vector3 v = GetAxis();
			float a = GetAmount() * RAD2DEG;
			return Edit(_T("<%7.3f %7.3f %7.3f> ; %7.3f"), v.x, v.y, v.z, a);
		}
		return Edit(_T("%.6f %.6f %.6f ; %.6f"), x, y, z, w);
	}

	void Slerp(const Quat& a, Quat b, double t)
	{
		// correct reversed axis quaternion
		{
			if( fabs(Vector3::AngleS(a.GetAxis(), b.GetAxis()) / PI) > 0.95 )
				b.Reverse();
		}

		double qr = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
		double ss = 1.0 - (qr*qr);
		double sp;

		if( (ss <= 0.0) || ((sp = sqrt(ss)) == 0.0) )
			*this = a;
		else
		{
			double ph = acos(qr);
			double pt = ph * t;
			double t1 = sin(pt) / sp;
			double t0 = sin(ph - pt) / sp;

			x = (a.x * t0) + (b.x * t1);
			y = (a.y * t0) + (b.y * t1);
			z = (a.z * t0) + (b.z * t1);
			w = (a.w * t0) + (b.w * t1);
		}
	}

	void ToMatrix(float* m) const
	{
		const Quat& q = *this;

		float xx = q.x * q.x * 2.0;
		float yy = q.y * q.y * 2.0;
		float zz = q.z * q.z * 2.0;	
		float xy = q.x * q.y * 2.0;
		float yz = q.y * q.z * 2.0;
		float zx = q.z * q.x * 2.0;
		float xw = q.x * q.w * 2.0;
		float yw = q.y * q.w * 2.0;
		float zw = q.z * q.w * 2.0;

		float r[16] = {	1.0 - yy - zz,	xy - zw,		zx + yw,		0.0,
						xy + zw,		1.0 - zz - xx,	yz - xw,		0.0,
						zx - yw,		yz + xw,		1.0 - xx - yy,	0.0,
						0.0,			0.0,			0.0,			1.0};
		/*
		float ww1 = q.w * q.w;
		float xx1 = xx / 2.0;
		float yy1 = yy / 2.0;
		float zz1 = zz / 2.0;

		float r[16] = {	ww1 + xx1 - yy1 - zz1,	xy - zw,				zx + yw,				0.0,
						xy + zw,				ww1 - xx1 + yy1 - zz1,	yz - xw,				0.0,
						zx - yw,				yz + xw,				ww1 - xx1 - yy1 + zz1,	0.0,
						0.0,					0.0,					0.0,					1.0};
		*/

		memcpy(m, r, sizeof(r));
	}

	// 1.X		2.Y		3.Z
	Vector3 ToEuler(const int order[], int nOrder) const
	{
		Vector3 result;
		Quat q = *this;

		for( int i=(nOrder - 1) ; i>=0 ; i-- )
		{
			if( order[i] == AXIS_X )
			{
				Vector3 v(0, 0, 1);
				v.Rotate(q);
				result.x = v.GetRotateX() * -1.0;	// ☆-1しないと合わないのがおかしいような気がする
				q.RotateX(-result.x);
			}
			else if( order[i] == AXIS_Y )
			{
				Vector3 v(1, 0, 0);
				v.Rotate(q);
				result.y = v.GetRotateY() * -1.0;
				q.RotateY(-result.y);
			}
			else if( order[i] == AXIS_Z )
			{
				Vector3 v(1, 0, 0);
				v.Rotate(q);
				result.z = v.GetRotateZ() * -1.0;
				q.RotateZ(-result.z);
			}
		}
		return result;
	}

	Vector3 ToEuler(int order0, int order1, int order2) const
	{
		int order[] = {order0, order1, order2};
		return ToEuler(order, 3);
	}

	/**
	 * @param order array of order value
	 * @param nOrder length for order[]
	 * order value		1.X-axis  2.Y-axis  3.Z-axis
	 */
	void FromEuler(const Vector3& vEuler, const int order[], int nOrder)
	{
		Identity();

		for( int i=0 ; i<nOrder ; i++ )
		{
			if( order[i] == AXIS_X )		RotateX(vEuler.x);
			else if( order[i] == AXIS_Y )	RotateY(vEuler.y);
			else if( order[i] == AXIS_Z )	RotateZ(vEuler.z);
		}
	}

	void FromEuler(const Vector3& vEuler, int order0, int order1, int order2)
	{
		int order[] = {order0, order1, order2};
		FromEuler(vEuler, order, 3);
	}

	Vector3 GetAxis() const
	{
		return Vector3(asin(x) * 2.0f, asin(y) * 2.0f, asin(z) * 2.0f);
	}

	float GetAmount() const
	{
		return (float)(acos(w) * 2.0f);
	}

	void Get(Vector3& vAxis, float& fAmount) const
	{
		vAxis = GetAxis();
		fAmount = GetAmount();
	}
};

/*
float m[4][4];
q.ToMatrix(m[0]);

Vector3 vtest2(1,0,0);
Vector3 vresult;
vresult.x = (vtest2.x * m[0][0]) + (vtest2.y * m[1][0]) + (vtest2.z * m[2][0]) + m[3][0];
vresult.y = (vtest2.x * m[0][1]) + (vtest2.y * m[1][1]) + (vtest2.z * m[2][1]) + m[3][1];
vresult.z = (vtest2.x * m[0][2]) + (vtest2.y * m[1][2]) + (vtest2.z * m[2][2]) + m[3][2];
*/

#endif

#endif // STEP1_BOARD

#ifndef __Variant_H
#define __Variant_H

class Variant
{
protected:
	String		m_sValue;
protected:
	void Set(const TCHAR* pValue)
	{
		m_sValue = pValue ? pValue : _T("");
	}
public:
	Variant()
	{
		Set(NULL);
	}
	Variant(const Variant& src)
	{
		*this = src;
	}
	Variant& operator=(const Variant& src)
	{
		Set(src.m_sValue);
		return *this;
	}
public:
	Variant(const TCHAR* value)	{	Set(value);								}
	Variant(int value)			{	Set(Edit(_T("%d"), value));				}
	Variant(double value)		{	Set(Edit(_T("%lf"), value));			}
	Variant(bool value)			{	Set(value ? _T("true") : _T("false"));	}

	int		Int() const			{	return _ttoi(m_sValue);					}
	int		Int(int base) const	{	return _tcstoul(m_sValue, NULL, base);	}
	float	Flt() const			{	return (float)_ttof(m_sValue);			}
	double	Dbl() const			{	return (double)_ttof(m_sValue);			}
	String	Str() const			{	return m_sValue;						}
	bool	Bol() const			{	return (m_sValue == _T("true"));		}

	void* ptr(){ return (void*)m_sValue.c_str(); };
};

#endif

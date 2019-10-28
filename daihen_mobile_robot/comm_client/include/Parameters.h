#ifndef __Parameters_H
#define __Parameters_H

class Parameters : public Map<String, Variant>
{
public:
	void Set(const TCHAR* pName, const Variant value)
	{
		(*this)[pName] = value;
	}

	bool Has(const TCHAR* pName) const
	{
        return (find(pName) != end());
	}

	Variant Get(const TCHAR* pName, const Variant vDefault) const
	{
		map<String, Variant>::const_iterator it = find(pName);
        if( it != end() )
			return it->second;
		return vDefault;
	}

	Variant Get(const TCHAR* pName) const
	{
		return Get(pName, Variant());
	}

	// paser for command line
	// command arg1=value1 arg2=value2
	String ParseCommand(const TCHAR* pLine)
	{
		String line(pLine);
		StringArray a = line.split(_T(" \t"));
		if( a.size() < 1 )
			return String(_T(""));
		if( a[0] == _T("") )
			return String(_T(""));

		for( size_t i=1 ; i<a.size() ; i++ )
		{
			StringArray pair = a[i].split(_T("="));
			if( pair.size() == 2 )
			{
				//printf("[%s] is [%s] \n", pair[0].c_str(), pair[1].c_str());
				SetStr(pair[0], pair[1]);
			}
		}
		return a[0];
	}

	// parser for ini file
	// name = value   # comment
	bool ParseIni(const TCHAR* pLine)
	{
		String line(pLine);

		line = line.split(_T("#"))[0];

		int n = line.find(_T("="));
		if( n < 0 )
			return false;

		String name = line.left(n).trim();
		String value = line.substr(n + 1).trim();

		if( (value.GetLength() >= 2) && (value[0] == '\"') && (value[value.GetLength() - 1] == '\"') )
			value = value.substr(1, value.GetLength() - 2);

		//printf("[%s] is [%s] \n", name.c_str(), value.c_str());
		SetStr(name, value);
		return true;
	}

	void SetStr(const TCHAR* pName, const TCHAR* value)				{	Set(pName, Variant(value));	}
	void SetInt(const TCHAR* pName, int value)						{	Set(pName, Variant(value));	}
	void SetFlt(const TCHAR* pName, float value)					{	Set(pName, Variant(value));	}
	void SetDbl(const TCHAR* pName, double value)					{	Set(pName, Variant(value));	}
	void SetBol(const TCHAR* pName, bool value)						{	Set(pName, Variant(value));	}

	String	Str(const TCHAR* pName, const TCHAR* def=NULL) const	{	return Get(pName, Variant(def)).Str();	}
	int		Int(const TCHAR* pName, int def=(0)) const				{	return Get(pName, Variant(def)).Int();	}
	float	Flt(const TCHAR* pName, float def=(0.0f)) const			{	return Get(pName, Variant(def)).Flt();	}
	double	Dbl(const TCHAR* pName, double def=(0.0)) const			{	return Get(pName, Variant(def)).Dbl();	}
	bool	Bol(const TCHAR* pName, bool def=false) const			{	return Get(pName, Variant(def)).Bol();	}
};

#endif

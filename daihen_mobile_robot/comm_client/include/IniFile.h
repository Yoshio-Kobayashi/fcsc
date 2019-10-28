#ifndef __IniFile_H
#define __IniFile_H

class IniFile : public Map<String, Parameters*>
{
public:
	IniFile()
	{
		m_sIniPath = _T("");
	}

	virtual ~IniFile()
	{
		for( Map<String, Parameters*>::iterator it=begin() ; it!=end() ; it++ )
			delete it->second;
	}

	bool Load(const TCHAR* pPath)
	{
		File f;
		if( !f.Open(pPath, _T("rt")) )
			return false;
		m_sIniPath = pPath;

		String sSection = _T("");
		for( String line ; f.ReadLine(line) ; )
		{
			if( (line.length() >= 2) && (line[0] == '[') && (line[line.length() - 1] == ']') )
			{
				sSection = line.substr(1, line.length() - 2);
				continue;
			}
			if( line.length() < 1 )
				continue;
			if( line[0] == _T('#') )
				continue;

			Map<String, Parameters*>::iterator it = find(sSection);
	        if( it == end() )
			{
				insert(pair<String, Parameters*>(sSection, new Parameters()));
				it = find(sSection);
			}
			it->second->ParseIni(line);
		}
		return true;
	}

	bool Save(const TCHAR* pPath=NULL) const
	{
		if( !pPath )
			pPath = m_sIniPath;

		File f;
		if( !f.Open(pPath, _T("wt")) )
			return false;

		bool bFirst = true;
		for( map<String, Parameters*>::const_iterator i=begin() ; i!=end() ; i++ )
		{
			if( bFirst )
				bFirst = false;
			else
				f.WriteLine(_T(""));

			f.WriteLine(Edit(_T("[%s]"), i->first.c_str()));
			Parameters* p = i->second;

			Array<String> aKeys = p->Keys();
			for( int j=0 ; j<aKeys.GetSize() ; j++ )
			{
				String name = aKeys[j];
				String value = p->Str(name);
				f.WriteLine(Edit(_T("%s=%s"), name.c_str(), value.c_str()));
			}
		}
		return true;
	}

	bool Has(const TCHAR* pSection, const TCHAR* pName) const
	{
		map<String, Parameters*>::const_iterator it = find(pSection);
        if( it == end() )
			return false;
		return it->second->Has(pName);
	}

	Variant Get(const TCHAR* pSection, const TCHAR* pName, Variant vDefault) const
	{
		map<String, Parameters*>::const_iterator it = find(pSection);
        if( it == end() )
			return vDefault;
		return it->second->Get(pName, vDefault);
	}

	void Set(const TCHAR* pSection, const TCHAR* pName, const Variant vValue)
	{
		map<String, Parameters*>::iterator it = find(pSection);
        if( it == end() )
		{
			insert(pair<String, Parameters*>(pSection, new Parameters()));
			it = find(pSection);
		}
		it->second->Set(pName, vValue);
	}

	String	Str(const TCHAR* pSection, const TCHAR* pName, const TCHAR* def=_T("")) const	{	return Get(pSection, pName, Variant(def)).Str();	}
	int		Int(const TCHAR* pSection, const TCHAR* pName, int def=0) const					{	return Get(pSection, pName, Variant(def)).Int();	}
	float	Flt(const TCHAR* pSection, const TCHAR* pName, float def=0.0f) const			{	return Get(pSection, pName, Variant(def)).Flt();	}
	double	Dbl(const TCHAR* pSection, const TCHAR* pName, double def=0.0) const			{	return Get(pSection, pName, Variant(def)).Dbl();	}
	bool	Bol(const TCHAR* pSection, const TCHAR* pName, bool def=false) const			{	return Get(pSection, pName, Variant(def)).Bol();	}

	void	SetStr(const TCHAR* pSection, const TCHAR* pName, const TCHAR* value)			{	return Set(pSection, pName, Variant(value));		}
	void	SetInt(const TCHAR* pSection, const TCHAR* pName, int value) 					{	return Set(pSection, pName, Variant(value));		}
	void	SetFlt(const TCHAR* pSection, const TCHAR* pName, float value) 					{	return Set(pSection, pName, Variant(value));		}
	void	SetDbl(const TCHAR* pSection, const TCHAR* pName, double value) 				{	return Set(pSection, pName, Variant(value));		}
	void	SetBol(const TCHAR* pSection, const TCHAR* pName, bool value) 					{	return Set(pSection, pName, Variant(value));		}

#ifdef _UNICODE
	String	Str(const char* pSection, const char* pName, const TCHAR* def=_T("")) const		{	return Get(M2W(pSection), M2W(pName), Variant(def)).Str();	}
	int		Int(const char* pSection, const char* pName, int def=0) const					{	return Get(M2W(pSection), M2W(pName), Variant(def)).Int();	}
	float	Flt(const char* pSection, const char* pName, float def=0.0f) const				{	return Get(M2W(pSection), M2W(pName), Variant(def)).Flt();	}
	double	Dbl(const char* pSection, const char* pName, double def=0.0) const				{	return Get(M2W(pSection), M2W(pName), Variant(def)).Dbl();	}
	bool	Bol(const char* pSection, const char* pName, bool def=false) const				{	return Get(M2W(pSection), M2W(pName), Variant(def)).Bol();	}

	void	SetStr(const char* pSection, const char* pName, const TCHAR* value)				{	return Set(M2W(pSection), M2W(pName), Variant(value));		}
	void	SetInt(const char* pSection, const char* pName, int value) 						{	return Set(M2W(pSection), M2W(pName), Variant(value));		}
	void	SetFlt(const char* pSection, const char* pName, float value) 					{	return Set(M2W(pSection), M2W(pName), Variant(value));		}
	void	SetDbl(const char* pSection, const char* pName, double value) 					{	return Set(M2W(pSection), M2W(pName), Variant(value));		}
	void	SetBol(const char* pSection, const char* pName, bool value) 					{	return Set(M2W(pSection), M2W(pName), Variant(value));		}
#endif

#ifdef _WINDOWS
	void DeleteConfigSection(const TCHAR* pSection)
	{
		WritePrivateProfileString(pSection, NULL, NULL, m_sIniPath);
	}
#endif

private:
	String	m_sIniPath;

	//! 当クラスは内部でnewを行うため、不用意な複製はSegmentFaultを招く
	IniFile(const IniFile& arg);
	IniFile operator=(const IniFile& arg);
};

#endif

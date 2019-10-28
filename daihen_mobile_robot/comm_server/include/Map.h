#ifndef __MAP_H
#define __MAP_H

#define BASE	std::map<T1, T2>

template <typename T1, typename T2>
class Map : public BASE
{
private:
	T2	m_DefaultValue;
public:
	int GetSize()
	{
		return (int)BASE::size();
	}

	bool IsContain(const T1& key) const
	{
		return (BASE::find(key) != BASE::end());
	}

	Array<T1> Keys() const
	{
		Array<T1> a;
		for( typename BASE::const_iterator i=BASE::begin() ; i!=BASE::end() ; i++ )
			a.Add(i->first);
		return a;
	}

	T2& Get(const T1& key)
	{
		typename BASE::iterator it = BASE::find(key);
		if( it != BASE::end() )
			return it->second;
		return m_DefaultValue;
	}

	const T2& GetConst(const T1& key) const
	{
		typename BASE::const_iterator it = BASE::find(key);
		if( it != BASE::end() )
			return it->second;
		return m_DefaultValue;
	}

	T2& operator[](const T1& key)
	{
		typename BASE::iterator it = BASE::find(key);
		if( it == BASE::end() )
		{
			this->insert(std::pair<T1, T2>(key, m_DefaultValue));
			it = BASE::find(key);
		}
		return it->second;
	}

	void SetDefaultValue(const T2& value)
	{
		m_DefaultValue = value;
	}
};

#undef BASE

#endif

#ifndef __Array_H
#define __Array_H

#define BASE	std::vector<T>

template <class T>
class Array : public BASE
{
public:
	int GetSize() const
	{
		return (int)BASE::size();
	}

	int Add(const T& t)
	{
		int rc = GetSize();
		this->push_back(t);
		return rc;
	}

	void Append(const Array<T>& src)
	{
		for( int i=0 ; i<src.GetSize() ; i++ )
			Add(src[i]);
	}

	void Copy(const Array<T>& src)
	{
		BASE::clear();
		Append(src);
	}

	T RemoveAt(int nIndex)
	{
		if( (nIndex >= 0) && (nIndex < GetSize()) )
		{
			int i = 0;
			for( typename Array<T>::iterator it=BASE::begin() ; it!=BASE::end() ; it++,i++ )
			{
				if( i == nIndex )
				{
					T t = *it;
					erase(it);
					return t;
				}
			}
		}
		return T();
	}

	void Push(const T& v)
	{
		Add(v);
	}

	T Pop()
	{
		return RemoveAt(GetSize() - 1);
	}

	T Shift()
	{
		return RemoveAt(0);
	}

	void Unshift(const T& t)
	{
		insert(BASE::begin(), t);
	}

	T* GetData()
	{
		if( GetSize() < 1 )
			return NULL;
		return &((*this)[0]);
	}

	const T* GetData() const
	{
		if( GetSize() < 1 )
			return NULL;
		return &((*this)[0]);
	}

	void SetSize(int nSize)
	{
		BASE::resize(nSize);
	}

	void RemoveAll()
	{
		BASE::clear();
	}

	Array<T>()
	{
	}

	Array<T>(const Array<T>& src)
	{
		Copy(src);
	}

	bool IsEmpty() const
	{
		return (GetSize() < 1);
	}
};

#undef BASE

#endif

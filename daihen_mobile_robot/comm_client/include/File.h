#ifndef __File_H
#define __File_H

class File
{
protected:
	FILE*	fp;
public:
	File()
	{
		fp = NULL;
	}

	virtual ~File()
	{
		Close();
	}

	operator FILE*()
	{
		return fp;
	}

	bool IsOpen()
	{
		return (fp != NULL);
	}

	bool Open(const TCHAR* pPath, const TCHAR* pMode)
	{
		if( fp )
			return false;
		fp = _tfopen(pPath, pMode);
		return (fp != NULL);
	}

	void Close()
	{
		if( !fp )
			return;
		fclose(fp);
		fp = NULL;
	}

	bool ReadLine(String& line)
	{
		if( !fp )
			return false;
		char buf[1024];
		if( !fgets(buf, sizeof(buf), fp) )
			return false;
		char* p = buf;
		if( !memcmp(p, "\xEF\xBB\xBF", 3) )
			p += 3;
#ifdef _UNICODE
		line = String(U2W(p)).split(_T("\r\n"))[0];
#else
		line = String(p).split(_T("\r\n"))[0];
#endif
		return true;
	}

	bool WriteLine(const char* pLine)
	{
		if( !fp )
			return false;
		if( fputs(pLine, fp) == EOF )
			return false;
		if( fputs("\n", fp) == EOF )
			return false;
		return true;
	}

#ifdef _UNICODE
	bool WriteLine(const WCHAR* pLine)
	{

/* 漢字が含まれるとエラー
		if( !fp )
			return false;
		if( _fputts(pLine, fp) == EOF )
			return false;
		if( _fputts(_T("\n"), fp) == EOF )
			return false;
		return true;
*/
		if( ftell(fp) == 0 )
			Write("\xEF\xBB\xBF", 3);
		return WriteLine(W2U(pLine));
	}
#endif

	bool Read(void* pData, int nLength)
	{
		if( !fp )
			return false;
		return (fread(pData, nLength, 1, fp) == 1);
	}

	bool Write(const void* pData, int nLength)
	{
		if( !fp )
			return false;
		return (fwrite(pData, nLength, 1, fp) == 1);
	}

	QWORD GetSize()
	{
		if( !fp )
			return 0;
		fflush(fp);

#ifdef _WINDOWS
		ULARGE_INTEGER u;
		HANDLE hFile = (HANDLE)_get_osfhandle(_fileno(fp));
		u.LowPart = GetFileSize(hFile, &u.HighPart);
		return (QWORD)u.QuadPart;
#else
		return 0;
#endif
	}

	bool SetSize(QWORD n)
	{
		if( !fp )
			return false;
		fflush(fp);

#ifdef _WINDOWS
		ULARGE_INTEGER u;
		u.QuadPart = n;

		HANDLE hFile = (HANDLE)_get_osfhandle(_fileno(fp));
		SetFilePointer(hFile, (LONG)u.LowPart, (PLONG)&u.HighPart, FILE_BEGIN);
		SetEndOfFile(hFile);
		return true;
#else
		return false;
#endif
	}

	protected:
	INT64 GetTime_sub(int nType)
	{
		if( (nType < 0) || (nType > 2) )
			return -1;
		if( !fp )
			return -2;

#ifdef _WINDOWS
		HANDLE hFile = (HANDLE)_get_osfhandle(_fileno(fp));
		if( hFile == INVALID_HANDLE_VALUE )
			return -3;

		FILETIME ft[3];
		if( !GetFileTime(hFile, ft + 0, ft + 1, ft + 2) )
			return -4;

		return *(__int64*)(ft + nType);
#else
		return -5;
#endif
	}
	public:

	INT64 GetTime_update()
	{
		return GetTime_sub(2);
	}

	static INT64 GetTime_update(const TCHAR* pPath)
	{
		File f;
		if( !f.Open(pPath, _T("r")) )
			return -1;
		return f.GetTime_update();
	}

	static bool EnumFiles(StringArray& aResult, const TCHAR* pDirPath, const TCHAR* pExt=NULL, bool bFileNameOnly=true)
	{
		String sDir = pDirPath;

#ifdef _WINDOWS
		CFileFind ent;
		if( !ent.FindFile(sDir + _T("\\*.*")) )
			return false;

		for( bool rc=true ; rc ; )
		{
			rc = (ent.FindNextFile() == TRUE);
			if( ent.IsDots() )
				continue;

			String buf;
			if( bFileNameOnly )
				buf = ent.GetFileName();
			else
				buf = ent.GetFilePath();

			if( pExt )
			{
				if( buf.right(_tcslen(pExt)) != pExt )
					continue;
			}
			aResult.Add(buf);
		}
#else
		DIR* pDir = opendir(pDirPath);
		if( !pDir )
			return false;

		for( struct dirent* p ; (p=readdir(pDir))!=NULL ; )
		{
			if( !strcmp(p->d_name, _T(".")) || !strcmp(p->d_name, _T("..")) )
				continue;

			String buf;
			if( bFileNameOnly )
				buf = p->d_name;
			else
				buf = sDir + _T("/") + p->d_name;

			if( pExt )
			{
				if( buf.right(_tcslen(pExt)) != pExt )
					continue;
			}
			aResult.Add(buf);
		}
		closedir(pDir);
#endif
		return true;
	}
};

#endif

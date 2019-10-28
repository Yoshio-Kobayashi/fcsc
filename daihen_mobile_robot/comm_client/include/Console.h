#ifndef __Console_H
#define	__Console_H

class Console
{
public:
	static void locate(int x, int y)
	{
#ifdef _WINDOWS
		HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
		COORD coord = {x, y};
		SetConsoleCursorPosition(hConsole, coord);
#else
		fputs(ESC_CURSOR(x, y), stdout);
#endif
	}

	static void home()
	{
		locate(0, 0);
	}

	static bool clear()
	{
#ifdef _WINDOWS
		COORD coord = {0, 0};
		DWORD cCharsWritten;
		CONSOLE_SCREEN_BUFFER_INFO csbi; 
		DWORD dwConSize;

		HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

		if( !GetConsoleScreenBufferInfo( hConsole, &csbi ) )
			return false;
		dwConSize = csbi.dwSize.X * csbi.dwSize.Y;

		if( !FillConsoleOutputCharacter(hConsole, _T(' '), dwConSize, coord, &cCharsWritten) )
			return false;
		if( !GetConsoleScreenBufferInfo(hConsole, &csbi) )
			return false;
		if( !FillConsoleOutputAttribute(hConsole, csbi.wAttributes, dwConSize, coord, &cCharsWritten) )
			return false;

		home();
#else
		fputs(ESC_CLEAR(), stdout);
#endif
		return true;
	}

	static void cursor(bool bShow)
	{
#ifdef _WINDOWS
#else
		fputs(bShow ? ESC_CURSOR_SHOW() : ESC_CURSOR_HIDE(), stdout);
#endif
	}

	static int getchar()
	{
		static const int STDIN = 0;
		int rc = 0;

		struct termios attr;
		struct termios attr_sv;
		if( tcgetattr(STDIN, &attr) < 0 )
			return -10;
		attr_sv = attr;

		attr.c_lflag &= ~(ICANON | ECHO);
		attr.c_cc[VMIN] = 1;
		attr.c_cc[VTIME] = 0;

		if( tcsetattr(STDIN, TCSANOW, &attr) < 0 )
			return -11;

		if( fcntl(STDIN, F_SETFL, O_NONBLOCK) >= 0 )
		{
			char c;
			if( read(0, &c, 1) == 1 )
				rc = c;
			else if( errno != EAGAIN )
				rc = -1;	// EOF
			else
				rc = -12;
		}
		tcsetattr(STDIN, TCSANOW, &attr_sv);

		return rc;
	}
};

#endif

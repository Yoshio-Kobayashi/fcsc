#ifndef __EscapeSequence_H
#define	__EscapeSequence_H

#define	ESC_CLEAR()				_T("\033[2J")					// 画面クリア
#define	ESC_HOME()				ESC_CURSOR(0, 0)				// カーソルをホームポジションへ移動

#define	ESC_CLEAR_RIGHT()		_T("\033[0K")					// カーソル位置からその行の右端までをクリア
#define	ESC_CLEAR_LEFT()		_T("\033[1K")					// カーソル行の左端からカーソル位置までをクリア
#define	ESC_CLEAR_LINE()		_T("\033[2K")					// カーソル行をクリア

#define	ESC_CURSOR(X, Y)		Edit(_T("\033[%d;%dH"), Y, X)	// カーソルを任意の位置へ移動
#define	ESC_CURSOR_UP(N)		Edit(_T("\033[%dA"), N)			// カーソルを上へ移動
#define	ESC_CURSOR_DOWN(N)		Edit(_T("\033[%dB"), N)			// カーソルを下へ移動
#define	ESC_CURSOR_LEFT(N)		Edit(_T("\033[%dD"), N)			// カーソルを左へ移動
#define	ESC_CURSOR_RIGHT(N)		Edit(_T("\033[%dC"), N)			// カーソルを右へ移動

#define	ESC_CURSOR_SHOW()		_T("\033[>5l")					// カーソルを表示
#define	ESC_CURSOR_HIDE()		_T("\033[>5h")					// カーソルを非表示

#define	ESC_COLOR(N)			Edit(_T("\033[%dm"), 30 + ((N > 7) ? N : (((N & 1) << 2) | (N >> 1))))	// 文字の色を変更
#define	ESC_COLOR_BLACK()		ESC_COLOR(0)					// 文字色を黒に変更
#define	ESC_COLOR_BLUE()		ESC_COLOR(1)					// 文字色を青に変更
#define	ESC_COLOR_RED()			ESC_COLOR(2)					// 文字色を赤に変更
#define	ESC_COLOR_MAGENTA()		ESC_COLOR(3)					// 文字色を紫に変更
#define	ESC_COLOR_GREEN()		ESC_COLOR(4)					// 文字色を緑に変更
#define	ESC_COLOR_CYAN()		ESC_COLOR(5)					// 文字色を水色に変更
#define	ESC_COLOR_YELLOW()		ESC_COLOR(6)					// 文字色を黄色に変更
#define	ESC_COLOR_WHITE()		ESC_COLOR(7)					// 文字色を白に変更
#define	ESC_COLOR_RESTORE()		ESC_COLOR(9)					// 文字色を戻す

#define	ESC_BGCOLOR(N)			Edit(_T("\033[%dm"), 40 + ((N > 7) ? N : (((N & 1) << 2) | (N >> 1))))	// 文字の色を変更
#define	ESC_BGCOLOR_BLACK()		ESC_BGCOLOR(0)					// 背景色を黒に変更
#define	ESC_BGCOLOR_BLUE()		ESC_BGCOLOR(1)					// 背景色を青に変更
#define	ESC_BGCOLOR_RED()		ESC_BGCOLOR(2)					// 背景色を赤に変更
#define	ESC_BGCOLOR_MAGENTA()	ESC_BGCOLOR(3)					// 背景色を紫に変更
#define	ESC_BGCOLOR_GREEN()		ESC_BGCOLOR(4)					// 背景色を緑に変更
#define	ESC_BGCOLOR_CYAN()		ESC_BGCOLOR(5)					// 背景色を水色に変更
#define	ESC_BGCOLOR_YELLOW()	ESC_BGCOLOR(6)					// 背景色を黄色に変更
#define	ESC_BGCOLOR_WHITE()		ESC_BGCOLOR(7)					// 背景色を白に変更
#define	ESC_BGCOLOR_RESTORE()	ESC_BGCOLOR(9)					// 背景色を戻す

#endif

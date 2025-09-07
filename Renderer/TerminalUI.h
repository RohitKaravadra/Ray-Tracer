#pragma once

#include <windows.h>
#include <iostream>
#include <string>
#include <sstream>  // for ostringstream

namespace tui {

	struct CursorPos {
		int row;
		int col;
	};

	CursorPos getCursorPos() {
		CONSOLE_SCREEN_BUFFER_INFO csbi;
		CursorPos pos{ 1,1 };

		HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
		if (hOut == INVALID_HANDLE_VALUE) return pos;

		if (GetConsoleScreenBufferInfo(hOut, &csbi)) {
			pos.row = csbi.dwCursorPosition.Y + 1; // convert 0-based to 1-based
			pos.col = csbi.dwCursorPosition.X + 1;
		}
		return pos;
	}

	inline CursorPos savedCursor;

	// =======================================================
	// Cursor / screen control
	// =======================================================
	inline void clear() {
		std::cout << "\033[2J\033[H"; // clear whole screen
	}

	inline void clearRest() {
		std::cout << "\033[J"; // clear from cursor to end of screen
	}

	inline void move(int r, int c) {
		std::cout << "\033[" << r << ";" << c << "H";
	}

	inline void saveCursor() {
		savedCursor = getCursorPos();
	}

	inline void restoreCursor() {
		std::cout << "\033[" << savedCursor.row << ";" << savedCursor.col << "H";
	}

	inline void hideCursor() { std::cout << "\033[?25l"; }
	inline void showCursor() { std::cout << "\033[?25h"; }

	inline void flush() { std::cout << std::flush; }

	// =======================================================
	// Printing helpers
	// =======================================================

	// Print text at row/col and erase the rest of that line
	inline void printAt(int r, int c, const std::string& text) {
		move(r, c);
		std::cout << text << "\033[K"; // clear remainder of line
	}

	// Continuous print like std::cout
	inline void print(const std::string& text) {
		std::cout << text << "\033[K\n"; // clear remainder of line and new line
	}

	// =======================================================
	// String concatenation (ostream-based formatting)
	// =======================================================

	// Base case: no arguments
	inline void format_impl(std::ostringstream& oss) {}

	// Recursive case: stream arguments into oss
	template <typename T, typename... Args>
	void format_impl(std::ostringstream& oss, T&& value, Args&&... args) {
		oss << std::forward<T>(value);
		format_impl(oss, std::forward<Args>(args)...);
	}

	// Public API: variadic template to build a string
	template <typename... Args>
	inline std::string format(Args&&... args) {
		std::ostringstream oss;
		format_impl(oss, std::forward<Args>(args)...);
		return oss.str();
	}

	// =======================================================
	// Color utilities
	// =======================================================
	namespace color {
		inline std::string wrap(const std::string& text, const std::string& code) {
			return code + text + "\033[0m";
		}

		// Foreground colors
		inline std::string red(const std::string& text) { return wrap(text, "\033[31m"); }
		inline std::string green(const std::string& text) { return wrap(text, "\033[32m"); }
		inline std::string yellow(const std::string& text) { return wrap(text, "\033[33m"); }
		inline std::string blue(const std::string& text) { return wrap(text, "\033[34m"); }
		inline std::string magenta(const std::string& text) { return wrap(text, "\033[35m"); }
		inline std::string cyan(const std::string& text) { return wrap(text, "\033[36m"); }
		inline std::string white(const std::string& text) { return wrap(text, "\033[37m"); }

		// Text styles
		inline std::string bold(const std::string& text) { return wrap(text, "\033[1m"); }
		inline std::string underline(const std::string& t) { return wrap(t, "\033[4m"); }
	} // namespace color

} // namespace tui

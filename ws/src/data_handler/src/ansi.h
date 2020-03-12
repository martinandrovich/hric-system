#pragma once

namespace ANSI
{
	
// https://misc.flogisoft.com/bash/tip_colors_and_formatting

constexpr auto DEFAULT = "\e[49m";
constexpr auto BLOCK_BLUE = "\e[104m";
constexpr auto BLOCK_CYAN = "\e[46m";
constexpr auto CLS = "\e[2J";
constexpr auto CURSOR_HIDE = "\e[?25l";
constexpr auto CURSOR_MV_ORIGIN = "\e[00H";

constexpr auto FORMAT = [](const auto& str, const auto& format) {
	return std::string(format) + std::string(str) + std::string(ANSI::DEFAULT);
};

} // ANSI
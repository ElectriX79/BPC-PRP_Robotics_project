#pragma once
#include <cstdint>

enum class DiscretePositions {
	LineLeft,
	LineRight,
	OnLine,
	NoLine
};
class LineEstimation() {
	public:
	static DiscretePositions estimate_discrete_line(uint16_t left_v, uint16_t right_v);
	static float estimate_continuous_line(uint16_t left_v, uint16_t right_v);
};


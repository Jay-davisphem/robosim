#include <iostream>
#include <fstream>
#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "DhParam.hpp"

using namespace std;

namespace {
	// Parses an alpha expression such as "pi/2", "-pi/2", "pi", "0.5" or "-1*pi".
	float parseAngle(const string& text)
	{
		try {
			float sign = 1.0f;
			size_t start = 0;

			if (start < text.size() && (text[start] == '+' || text[start] == '-')) {
				if (text[start] == '-') sign = -1.0f;
				++start;
			}

			auto toValue = [](const string& token) -> float {
				return token.find("pi") != string::npos
					? static_cast<float>(M_PI)
					: std::stof(token);
			};

			const size_t operation_loc = text.find_first_of("*/", start);
			const string lhs = text.substr(start, operation_loc == string::npos
				? string::npos
				: operation_loc - start);

			if (lhs.empty()) return 0.0f;
			if (operation_loc == string::npos) return sign * toValue(lhs);

			const string rhs = text.substr(operation_loc + 1);
			if (rhs.empty()) return sign * toValue(lhs);

			const float operand1 = toValue(lhs);
			const float operand2 = toValue(rhs);
			return sign * (text[operation_loc] == '*' ? operand1 * operand2 : operand1 / operand2);
		}
		catch (const std::exception&) {
			return 0.0f;
		}
	}
}

std::vector<DhParam> loadDhParamsFromFile(const std::string filename)
{
	ifstream file_input(filename, ios::in);
	string line;

	std::vector<DhParam> params;

	if (!file_input) {
		cout << "Failed to open parameter file: " << filename << endl;
		return params;
	}

	while (getline(file_input, line)) {
		// Handle CRLF files and skip blank or comment lines.
		if (!line.empty() && line.back() == '\r') line.pop_back();
		const size_t first_char = line.find_first_not_of(" \t");
		if (first_char == string::npos) continue;
		if (line[first_char] == '#') continue;

		stringstream ss(line);
		DhParam p;
		string sink;
		string alpha_str;

		if (!(ss >> p.sigma)) continue;

		if (p.sigma == "R") {
			ss >> p.a >> p.d >> alpha_str >> sink;

			p.alpha = parseAngle(alpha_str);
			p.theta = -1;
		}
		else if (p.sigma == "P") {
			ss >> p.a >> sink >> p.alpha >> p.theta;
			p.d = -1;
		}
		else {
			cout << "Invalid joint type: " << p.sigma << endl;
			continue;
		}

		params.push_back(p);
	}
	return params;
}
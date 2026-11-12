#include <iostream>
#include <string>

#include "core/Application.hpp"

// entry point
int main(int argc, char** argv) {
	if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
		std::cout << "Usage: robosim [dh-param-file]\n"
				  << "  dh-param-file  path to a space-separated DH parameter file\n"
				  << "                 (default: src/resources/params.txt)\n";
		return 0;
	}

	const std::string dh_param_file = (argc > 1) ? argv[1] : "src/resources/params.txt";

	Application app(dh_param_file);
	app.createWindow(800, 600);
	app.run();
}
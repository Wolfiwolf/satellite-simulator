#include "simulator_config.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace satellite_simulator_engine
{
	double SimulatorConfig::_timestep;
	std::string SimulatorConfig::_output_folder;
	std::string SimulatorConfig::_output_ip_address;
	int SimulatorConfig::_output_port;

	void SimulatorConfig::load_data_from_file()
	{
		std::ifstream config_file("vss.config");


		std::string line;
		while (std::getline(config_file, line))
		{
			std::string key = line.substr(0, line.find_first_of(':'));
			if (key == "timestep") {
				std::stringstream val(line.substr(line.find_first_of(':') + 2, line.size() - line.find_first_of(':')));
				double timestep;
				val >> timestep;

				_timestep = timestep;
			}
			else if (key == "output_folder") {
				std::string val = line.substr(line.find_first_of(':') + 2, line.size() - line.find_first_of(':'));
				_output_folder = val;
			}
			else if (key == "output_ip_address") {
				std::string val = line.substr(line.find_first_of(':') + 2, line.size() - line.find_first_of(':'));
				_output_ip_address = val;
			}
			else if (key == "output_port") {
				std::stringstream val(line.substr(line.find_first_of(':') + 2, line.size() - line.find_first_of(':')));
				int port;
				val >> port;

				_output_port = port;
			}
		}


		config_file.close();
	}

	double SimulatorConfig::get_timestep()
	{
		return _timestep;
	}

	const std::string& SimulatorConfig::get_output_folder()
	{
		return _output_folder;
	}

	const std::string& SimulatorConfig::get_output_ip_address()
	{
		return _output_ip_address;
	}

	int SimulatorConfig::get_output_port()
	{
		return _output_port;
	}

}


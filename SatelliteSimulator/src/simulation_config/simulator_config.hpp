#pragma once

#include <string>


namespace satellite_simulator_engine
{

	static class SimulatorConfig
	{
	public:
		static void load_data_from_file();
		static double get_timestep();
		static const std::string &get_output_folder();
		static const std::string& get_output_ip_address();
		static int get_output_port();

	private:
		static double _timestep;
		static std::string _output_folder;
		static std::string _output_ip_address;
		static int _output_port;

	};
}
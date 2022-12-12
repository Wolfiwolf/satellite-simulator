#pragma once


namespace satellite_graphics_engine
{

	class SatelliteGraphicsEngine
	{
	public:
		SatelliteGraphicsEngine();
		~SatelliteGraphicsEngine();

		void init();
		void start();
		void update();
		void set_satellite_state(
			double pos_x_ECI, 
			double pos_y_ECI, 
			double pos_z_ECI, 
			double attitude_w_ECI, 
			double attitude_x_ECI, 
			double attitude_y_ECI, 
			double attitude_z_ECI
		);
		bool should_program_end();

	private:
		void* _window;
		
	};

};
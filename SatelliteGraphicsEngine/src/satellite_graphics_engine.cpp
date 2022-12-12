#include "./satellite_graphics_engine.hpp"

#include "graphics/window/window.hpp"

namespace satellite_graphics_engine
{

	SatelliteGraphicsEngine::SatelliteGraphicsEngine() 
	{

	}

	SatelliteGraphicsEngine::~SatelliteGraphicsEngine()
	{

	}

	void SatelliteGraphicsEngine::init()
	{
		_window = new Window();
	}

	void SatelliteGraphicsEngine::start()
	{
		((Window*)_window)->start("Satellite simulator", 1200, 720);
	}

	void SatelliteGraphicsEngine::update()
	{
		((Window*)_window)->update();
	}

	void SatelliteGraphicsEngine::set_satellite_state(
		double pos_x_ECI,
		double pos_y_ECI,
		double pos_z_ECI,
		double attitude_w_ECI,
		double attitude_x_ECI,
		double attitude_y_ECI,
		double attitude_z_ECI
	)
	{

	}

	bool SatelliteGraphicsEngine::should_program_end()
	{
		return ((Window*)_window)->should_program_end();
	}

}



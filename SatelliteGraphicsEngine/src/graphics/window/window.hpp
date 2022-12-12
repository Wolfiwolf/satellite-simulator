#pragma once

#include <GLFW/glfw3.h>

namespace satellite_graphics_engine
{

	class Window
	{
	public:
		Window();
		~Window();

		void start(const char* title, int width, int height);
		void update();
		bool should_program_end();

	private:
		GLFWwindow* _window;

	};

}
#include "window.hpp"

#include <iostream>

namespace satellite_graphics_engine
{

	Window::Window()
	{

	}

	Window::~Window()
	{
		glfwTerminate();
	}

	void Window::start(const char* title, int width, int height)
	{
		/* Initialize the library */
		if (!glfwInit()) return;


		/* Create a windowed mode window and its OpenGL context */
		_window = glfwCreateWindow(width, height, title, NULL, NULL);
		if (!_window)
		{
			glfwTerminate();
			return;
		}

		/* Make the window's context current */
		glfwMakeContextCurrent(_window);

		if (glewInit() != GLEW_OK) {
			std::cout << "GLEW NOT OK!\n";
		}
	}

	void Window::update()
	{
		/* Render here */
		glClear(GL_COLOR_BUFFER_BIT);

		/* Swap front and back buffers */
		glfwSwapBuffers(_window);

		/* Poll for and process events */
		glfwPollEvents();
	}

	bool Window::should_program_end()
	{
		return glfwWindowShouldClose(_window);
	}

}
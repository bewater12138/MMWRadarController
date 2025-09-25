#pragma once
#include <glfw/glfw3.h>
#include <functional>

GLFWwindow* GetGLFWWindow();
bool ImGuiInit();
void ImGuiClose();

using ImGuiDrawFunction = std::function<void()>;
int RunGuiThread(ImGuiDrawFunction draw_func);
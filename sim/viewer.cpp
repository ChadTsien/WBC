#include "sim/viewer.hpp"
#include "sim/world.hpp"

#include <mutex>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>


namespace T1::sim
{

    SimViewer::SimViewer(const MujocoWorld& world) : world_(world) {}

    SimViewer::~SimViewer()
    {
        stop();
    }

    void SimViewer::start()
    {
        if (running_.exchange(true))
        {
            return;
        }
        thread_ = std::thread([this] { run(); });
    }

    void SimViewer::stop()
    {
        running_ = false;
        if (thread_.joinable()) 
        {
            thread_.join();
        }
    }

    void SimViewer::run()
    {
        if (!glfwInit())
        {
            running_ = false;
            return;
        }
        GLFWwindow* window = glfwCreateWindow(1280, 800, "T1 MujocoWorld", nullptr, nullptr);
        if (window == nullptr)
        {
            glfwTerminate();
            running_ = false;
            return;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        auto* model = static_cast<mjModel*>(world_.raw_model());
        auto* data = static_cast<mjData*>(world_.raw_data());

        mjvCamera cam;
        mjvOption opt;
        mjvScene scene;
        mjrContext con;
        mjv_defaultCamera(&cam);
        mjv_defaultFreeCamera(model, &cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&con);
        mjv_makeScene(model, &scene, 8000);
        mjr_makeContext(model, &con, mjFONTSCALE_150);

        struct Mouse
        {
            mjvCamera* cam = nullptr;
            mjvScene* scene = nullptr;
            mjModel* model = nullptr;
            bool left = false;
            bool right = false;
            double lastx = 0.0;
            double lasty = 0.0;
        } mouse;
        mouse.cam = &cam;
        mouse.scene = &scene;
        mouse.model = model;
        glfwSetWindowUserPointer(window, &mouse);

        glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int button, int act, int) {
            auto* m = static_cast<Mouse*>(glfwGetWindowUserPointer(w));
            m->left = (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS);
            m->right = (button == GLFW_MOUSE_BUTTON_RIGHT && act == GLFW_PRESS);
            glfwGetCursorPos(w, &m->lastx, &m->lasty);
        });

        glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
            auto* m = static_cast<Mouse*>(glfwGetWindowUserPointer(w));
            if (!m->left && !m->right)
            {
                return;
            }
            const double dx = x - m->lastx;
            const double dy = y - m->lasty;
            m->lastx = x;
            m->lasty = y;
            int width = 0, height = 1;
            glfwGetWindowSize(w, &width, &height);
            const mjtMouse action = m->right ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_V;
            mjv_moveCamera(m->model, action, dx / height, dy / height, m->scene, m->cam);
        });

        glfwSetScrollCallback(window, [](GLFWwindow* w, double, double yoff) {
            auto* m = static_cast<Mouse*>(glfwGetWindowUserPointer(w));
            mjv_moveCamera(m->model, mjMOUSE_ZOOM, 0.0, 0.05 * yoff, m->scene, m->cam);
        });
        
        while (running_ && !glfwWindowShouldClose(window))
        {
            {
                std::lock_guard<std::mutex> lock(world_.mutex());
                mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);
            }
            mjrRect viewport{0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjr_render(viewport, &scene, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
        mjr_freeContext(&con);
        mjv_freeScene(&scene);
        glfwDestroyWindow(window);
        glfwTerminate();
        running_ = false;
    }

}  // namespace T1::sim

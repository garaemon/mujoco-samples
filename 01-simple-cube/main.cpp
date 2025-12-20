#include <chrono>
#include <cstdio> // For fprintf
#include <thread>

// #include <mujoco/mjerrors.h>

#include <GLFW/glfw3.h>

#include <mujoco/mujoco.h>

mjModel *model;
mjData *d;
mjvCamera camera;
mjvOption option;
mjvScene scene;
mjrContext context;
int button_left;
int button_middle;
int button_right;
double lastx;
double lasty;

void initializeVisualization(GLFWwindow *window) {

  // Initialize visualization structures
  mjv_defaultCamera(&camera);
  mjv_defaultOption(&option);
  mjv_defaultScene(&scene);

  // Create scene and context
  mjv_makeScene(model, &scene, 2000);
  mjr_makeContext(model, &context, mjFONTSCALE_150);

  // Set camera
  camera.lookat[0] = 0.0;
  camera.lookat[1] = 0.0;
  camera.lookat[2] = 0.0;
  camera.distance = 2.0;
  camera.azimuth = 135.0;
  camera.elevation = -20.0;
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {

  // Update button state

  button_left =

      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);

  button_middle =

      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);

  button_right =

      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // Update mouse position

  glfwGetCursorPos(window, &lastx, &lasty);
}

void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos) {

  if (!button_left && !button_middle && !button_right) {

    return;
  }

  // Compute mouse displacement

  const double dx = xpos - lastx;

  const double dy = ypos - lasty;

  lastx = xpos;
  lasty = ypos;

  // Get window size

  int width, height = 0;
  glfwGetWindowSize(window, &width, &height);
  // Determine action based on mouse button

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;

  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }
  // Move camera
  mjv_moveCamera(model, action, dx / height, dy / height, &scene, &camera);
}

void render(GLFWwindow *window) {

  // Get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};

  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  // Clear buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Update scene and render
  mjv_updateScene(model, d, &option, nullptr, &camera, mjCAT_ALL, &scene);

  mjr_render(viewport, &scene, &context);
}

int main(void) {
  if (!glfwInit()) {
    printf("Could not initialize GLFW\n");
    return 1;
  }

  GLFWwindow *window =
    glfwCreateWindow(1280, 720, "Single Joint Free", nullptr, nullptr);

  if (!window) {
    printf("Could not create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);

  glfwSwapInterval(1);

  // load model from file and check for errors
  char error[1000] = {0};
  model = mj_loadXML("model.xml", NULL, error, 1000);

  if (!model) {
    printf("%s\n", error);
    return 1;
  }

  // make data corresponding to model

  d = mj_makeData(model);

  // Initialize visualization
  initializeVisualization(window);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, mouseMoveCallback);

  while (!glfwWindowShouldClose(window)) {
    mj_step(model, d);
    render(window);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // free model and data

  mj_deleteData(d);

  mj_deleteModel(model);

  return 0;
}

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include <cstdio>
#include <iostream>
#include <fstream>

#include "common.h"
#include "shader.h"
#include "mesh.h"

GLFWwindow* window;
GLfloat screenHeight = 1600, screenWidth = 1200;
std::string data_path, shader_path;

bool keys[1024];
bool pause = false;
GLboolean spin_locked = false, move_locked = false;
double frame_rate = 1;
GLfloat zoom = 90.0f;
int cursorX = 0, cursorY = 0;
glm::vec3 X(1.0f, 0.0f, 0.0f), Y(0.0f, 1.0f, 0.0f), Z(0.0f, 0.0f, 1.0f);
glm::vec3 spinX = X, spinY = Y;
glm::mat4 rotation(1);
glm::vec3 center, init_position, position, front(.0f, .0f, -1.0f), up(.0f, 1.0f, .0f), right = glm::cross(up, front);
glm::vec3 light_pos;

float deltaTime = 0.0f;
float lastFrame = 0;
double frame = 0;

MeshR mesh;
void InitViewer() {
    Vec3 sum = Vec3::Zero();
    double xmax = -INT_MAX, xmin = INT_MAX;
    for (auto v : mesh.vertices) {
        xmax = fmax(xmax, v.x(0));
        xmin = fmin(xmin, v.x(0));
        sum += v.x;
    }
    double xspan = xmax - xmin;
    sum /= mesh.vertices.size();
    center = glm::vec3(sum(0), sum(1), sum(2));
    init_position = Z * float(xspan / 2.);
    position = init_position;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    if (key >= 0 && key < 1024) {
        if  (action == GLFW_PRESS)
            keys[key] = true;
        else if (action == GLFW_RELEASE && key != GLFW_KEY_SPACE)
            keys[key] = false;
    }
}

void Move() {
    fflush(stdout);
    if (keys[GLFW_KEY_R]) {
        position = init_position;
        zoom = 90.0f;
        rotation = glm::mat4(1);
        spinX = X;
        spinY = Y;
        //frame_rate = 1;
        frame = 0;
    }
    if (keys[GLFW_KEY_UP])
        frame_rate += deltaTime;
    if (keys[GLFW_KEY_DOWN])
        frame_rate -= deltaTime;
    if (frame_rate < 1./120.)
        frame_rate = 1./120.;
    if (keys[GLFW_KEY_SPACE]) {
        pause = !pause;
        keys[GLFW_KEY_SPACE] = false;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    screenWidth = width;
    screenHeight = height;
}

void scroll_callback(GLFWwindow* window, double x, double y) {
    position.z -= y;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            spin_locked = GL_TRUE;
        } else {
            spin_locked = GL_FALSE;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            move_locked = GL_TRUE;
        } else {
            move_locked = GL_FALSE;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
    }
}

void cursor_position_callback(GLFWwindow* window, double x, double y) {
	if (spin_locked) {
        glm::quat spin = glm::angleAxis(float((y - cursorY) / 200.0f), spinX) * glm::angleAxis(float((x - cursorX) / 200.0f), spinY);
        rotation *= glm::toMat4(spin);
        spinX = spinX * spin;
        spinY = spinY * spin;
	}
    if (move_locked) {
        position += (up * float((y - cursorY) / 100) + right * float((x - cursorX) / 100));
    }

	cursorX = (int) x;
	cursorY = (int) y;
}

int main(int argc, char *argv[]) {
    for (int i = 1; i < argc; ) {
        switch (argv[i][1]) {
        case 'o':
            data_path = std::string(argv[i + 1]) + "/";
            break;
        case 's':
            shader_path = std::string(argv[i + 1]) + "/";
            break;
        default:
            break;
        }
        i += 2;
    }
  	if (!glfwInit()) {
        std::puts("Failed");
		exit(EXIT_FAILURE);
    }
	window = glfwCreateWindow(screenWidth, screenHeight, "Viewer", NULL, NULL);
	glViewport(0, 0, screenWidth, screenHeight);
	if (!window) {
		glfwTerminate();
        std::puts("Failed");
        exit(EXIT_FAILURE);
	}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);

    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Fail to Initialize GLEW\n");
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glEnable(GL_DEPTH_TEST);

    Shader shader(shader_path + "shader.vs", shader_path + "shader.fs");
    mesh.path = data_path;
    mesh.Init();
    InitViewer();
    mesh.SetUpOpenGl();

    std::puts("Lauched!");
	while (!glfwWindowShouldClose(window)) {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        glfwPollEvents();
        Move();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);		

        int fr = int(frame);
        std::ifstream frame_file(data_path + "/frames/" + std::to_string(fr), std::ios::in);
        mesh.Read(frame_file);
        shader.use();
        shader.setVec3("objectColor", 1.0f, 0.5f, 0.3f);
        shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);

        glm::mat4 projection = glm::perspective(glm::radians(zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(position, position + front, up);
        glm::mat4 model = rotation * glm::translate(glm::mat4(1), -center);
        shader.setVec3("lightPos", glm::vec3(rotation * glm::vec4(init_position, 1.0)));
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);
        shader.setMat4("model", model);

        mesh.Draw();
        
        glfwSwapBuffers(window);

        if (!pause) {
            frame += frame_rate;
            if (frame >= mesh.num_frames)
                frame -= mesh.num_frames;
        }
    }

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}
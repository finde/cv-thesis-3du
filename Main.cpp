#include "Main.h"
//#include "Model.h"

using namespace std;

// Properties
GLuint screenWidth = 800, screenHeight = 600;

// Camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
bool keys[1024];
GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

// Animation
float progress = 0.0f;
float direction = 0.0f;
float speed = 0.5f;

// cube location
int position = 0;
float tolerance = 5.0f;
float cube_speed = 2.5f;

// The MAIN function, from here we start our application and run our Game loop
int main() {

    // Init GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWwindow *window = glfwCreateWindow(screenWidth, screenHeight, "3DU", nullptr, nullptr); // Windowed

    glfwMakeContextCurrent(window);

    // Set the required callback functions
    glfwSetKeyCallback(window, key_callback);
//    glfwSetCursorPosCallback(window, mouse_callback);
//    glfwSetScrollCallback(window, scroll_callback);

    // Options
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR);

    // Initialize GLEW to setup the OpenGL Function pointers
    glewExperimental = GL_TRUE;
    glewInit();

    // Define the viewport dimensions
    glViewport(0, 0, screenWidth * 2, screenHeight * 2); // retina

    // Setup some OpenGL options
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Setup and compile our shaders
    Shader shader("shaders/advanced.vs", "shaders/advanced.frag");
    Shader skyboxShader("shaders/skybox.vs", "shaders/skybox.frag");

#pragma region "object_initialization"
    GLfloat skyboxVertices[] = {
            // Positions
            -1.0f, 1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,

            -1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,

            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,

            -1.0f, -1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,

            -1.0f, 1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, -1.0f,

            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, 1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, 1.0f,
            1.0f, -1.0f, 1.0f
    };

    // Setup skybox VAO
    GLuint skyboxVAO, skyboxVBO;
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *) 0);
    glBindVertexArray(0);

#pragma endregion

    // Load model
//    Model model;
//    model.load("data/skybox/C_pano.jpg");

    // Cubemap (Skybox)
    vector<const GLchar *> faces;
    faces.push_back("data/skybox/B_cube_0001.jpg");
    faces.push_back("data/skybox/B_cube_0003.jpg");
    faces.push_back("data/skybox/blank.jpg");
    faces.push_back("data/skybox/blank.jpg");
    faces.push_back("data/skybox/B_cube_0000.jpg");
    faces.push_back("data/skybox/B_cube_0002.jpg");
    GLuint cubemapTexture = loadCubemap(faces);

    vector<const GLchar *> faces2;
    faces2.push_back("data/skybox/C_cube_0001.jpg");
    faces2.push_back("data/skybox/C_cube_0003.jpg");
    faces2.push_back("data/skybox/blank.jpg");
    faces2.push_back("data/skybox/blank.jpg");
    faces2.push_back("data/skybox/C_cube_0000.jpg");
    faces2.push_back("data/skybox/C_cube_0002.jpg");
    GLuint cubemapTexture2 = loadCubemap(faces2);

    // Draw as wireframe
//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // loop
    while (!glfwWindowShouldClose(window)) {
        // Set frame time
        GLfloat currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check and call events
        glfwPollEvents();
        Do_Movement();

        // Clear buffers
        glClearColor(1.0f, 1.f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw skybox first
        glDepthMask(GL_FALSE);// Remember to turn depth writing off
        skyboxShader.Use();
        glm::mat4 view = glm::mat4( glm::mat3(camera.GetViewMatrix()));    // Remove any translation component of view
        glm::mat4 projection = glm::perspective(camera.Zoom, (float) screenWidth / (float) screenHeight, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(skyboxShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(skyboxShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        // animation
        glUniform1f(glGetUniformLocation(skyboxShader.Program, "progress"), Update_Progress(deltaTime));

        // skybox cube
        glBindVertexArray(skyboxVAO);
        glActiveTexture(GL_TEXTURE0);
        glUniform1i(glGetUniformLocation(skyboxShader.Program, "skybox1"), 0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);

        glActiveTexture(GL_TEXTURE1);
        glUniform1i(glGetUniformLocation(skyboxShader.Program, "skybox2"), 1);
        glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture2);

        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glDepthMask(GL_TRUE);

        // Then draw scene as normal
//        shader.Use();
//        view = camera.GetViewMatrix();
//
//        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
//        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));


        // Swap the buffers
        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}

#pragma region "User input"

float Update_Progress(float deltaTime) {
    progress += direction * deltaTime * speed;

    if (progress > 1 || progress < 0) {
        direction = 0;

//        if (progress <= 0) {
//            progress = 1;
//        } else {
//            progress = 0;
//        }
    }

    return progress;
}

// Moves/alters the camera positions based on user input
void Do_Movement() {
    // Camera controls
    if (keys[GLFW_KEY_UP]) {
        glm::vec2 cam = camera.GetPosition();
        if (position == 0 && abs(cam.x - (-54.0f)) <= tolerance) {
            direction = 1.0f;
            position = 1;
        }

        if (position == 1 && abs(cam.x - (126.0f)) <= tolerance) {
            direction = -1.0f;
            position = 0;
        }
    }

    if (keys[GLFW_KEY_LEFT]) {
//        camera.ProcessKeyboard(LEFT, deltaTime);
        camera.ProcessMouseMovement(-cube_speed, 0.0f);
    }

    if (keys[GLFW_KEY_RIGHT]) {
//        camera.ProcessKeyboard(RIGHT, deltaTime);
        camera.ProcessMouseMovement(cube_speed, 0.0f);
    }
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mode) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (action == GLFW_PRESS)
        keys[key] = true;
    else if (action == GLFW_RELEASE)
        keys[key] = false;
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    cout << xoffset << endl;
    cout << yoffset << endl;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
    camera.ProcessMouseScroll(yoffset);
}

#pragma endregion
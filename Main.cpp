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
float speed = 1.5f;

// cube location
int position = 0;
float tolerance = 5.0f;
float cube_speed = 2.5f;
glm::vec3 vWorldPosition1 = glm::vec3(0.0f, 0.0f, 0.0f);

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
    Shader shader("shaders/advanced.vert", "shaders/advanced.frag");
    Shader skyboxShader("shaders/skybox.vert", "shaders/skybox.frag");

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


    float f = 0.0f;
    // loop
    while (!glfwWindowShouldClose(window)) {
        // Set frame time
        GLfloat currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check and call events
        glfwPollEvents();
        Do_Movement(deltaTime, skyboxShader);
        Update_Progress(deltaTime, skyboxShader);

        // Clear buffers
        glClearColor(1.0f, 1.f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw skybox first
        glDepthMask(GL_FALSE);// Remember to turn depth writing off
        skyboxShader.Use();
        glm::mat4 view = glm::mat4(glm::mat3(camera.GetViewMatrix()));    // Remove any translation component of view
        glm::mat4 projection = glm::perspective(camera.Zoom, (float) screenWidth / (float) screenHeight, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(skyboxShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(skyboxShader.Program, "projection"), 1, GL_FALSE,
                           glm::value_ptr(projection));

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

//        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
//        glUniformMatrix4fv(glGetUniformLocation(shader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));


        // Swap the buffers
        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}

#pragma region "User input"

float Update_Progress(float deltaTime, Shader skyboxShader) {
    progress += direction * deltaTime * speed;

    if (progress > 1 || progress < 0) {
        direction = 0;
    } else {
        glUniform1f(glGetUniformLocation(skyboxShader.Program, "progress"), progress);

        float distance = 0.7f;
        glm::vec3 target = glm::vec3(0.340107, -0.017929, -0.550713);
        glm::vec3 target2 = glm::vec3(-distance, -distance, -distance) * target;
        glm::vec3 nvWorldPosition1 = glm::mix(vWorldPosition1, target, progress);
        glm::vec3 nvWorldPosition2 = glm::mix(target2, vWorldPosition1, progress);

        glUniform3f(glGetUniformLocation(skyboxShader.Program, "vWorldPosition1"),
                    nvWorldPosition1.x,
                    nvWorldPosition1.y,
                    nvWorldPosition1.z);

        glUniform3f(glGetUniformLocation(skyboxShader.Program, "vWorldPosition2"),
                    nvWorldPosition2.x,
                    nvWorldPosition2.y,
                    nvWorldPosition2.z);

        if (progress > 1) progress = 1.0f;
        if (progress < 0) progress = 0.0f;
    }

    return progress;
}

// Moves/alters the camera positions based on user input
void Do_Movement(float deltaTime, Shader skyboxShader) {
    if (progress > 1 || progress < 0) {
        direction = 0.0f;
    }

    // debugger -- progress
    if (keys[GLFW_KEY_T] || keys[GLFW_KEY_R]) {
        if (keys[GLFW_KEY_T]) {
            direction = 1.0f;
        }
        if (keys[GLFW_KEY_R]) {
            direction = -1.0f;
        }
    }

    // -- zoom
    if (keys[GLFW_KEY_Q]) {
        glm::mat4 view = glm::mat4(glm::mat3(camera.GetViewMatrix()));    // Remove any translation component of view
        camera.ProcessMouseScroll(-cube_speed * deltaTime);
        cout << glm::to_string(view) << endl;
    }

    if (keys[GLFW_KEY_W]) {
        glm::mat4 view = glm::mat4(glm::mat3(camera.GetViewMatrix()));    // Remove any translation component of view
        camera.ProcessMouseScroll(cube_speed * deltaTime);
        cout << glm::to_string(view) << endl;
    }

    // -- pan
    if (keys[GLFW_KEY_U] || keys[GLFW_KEY_I] || keys[GLFW_KEY_O] ||
        keys[GLFW_KEY_J] || keys[GLFW_KEY_K] || keys[GLFW_KEY_L]) {

        if (keys[GLFW_KEY_U]) {
            vWorldPosition1.x += speed * deltaTime;
        }

        if (keys[GLFW_KEY_J]) {
            vWorldPosition1.x -= speed * deltaTime;
        }

        if (keys[GLFW_KEY_I]) {
            vWorldPosition1.y += speed * deltaTime;
        }

        if (keys[GLFW_KEY_K]) {
            vWorldPosition1.y -= speed * deltaTime;
        }

        if (keys[GLFW_KEY_O]) {
            vWorldPosition1.z += speed * deltaTime;
        }

        if (keys[GLFW_KEY_L]) {
            vWorldPosition1.z -= speed * deltaTime;
        }

        glUniform3f(glGetUniformLocation(skyboxShader.Program, "vWorldPosition1"),
                    vWorldPosition1.x,
                    vWorldPosition1.y,
                    vWorldPosition1.z);

        cout << "Image vWorldPosition1: " << glm::to_string(vWorldPosition1) << endl;
    }


    // Camera controls
    if (keys[GLFW_KEY_SPACE]) {
        progress = 0.0f;
        glm::vec2 cam = camera.GetPosition();
        if (position == 0 && abs(cam.x - (-54.0f)) <= tolerance) {
            direction = 1.0f;
        }

        if (position == 1 && abs(cam.x - (126.0f)) <= tolerance) {
            direction = -1.0f;
        }
    }

    if (keys[GLFW_KEY_UP]) {
        camera.ProcessMouseMovement(0.0f, -cube_speed);
    }

    if (keys[GLFW_KEY_DOWN]) {
        camera.ProcessMouseMovement(0.0f, cube_speed);
    }

    if (keys[GLFW_KEY_LEFT]) {
        camera.ProcessMouseMovement(-cube_speed, 0.0f);
    }

    if (keys[GLFW_KEY_RIGHT]) {
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
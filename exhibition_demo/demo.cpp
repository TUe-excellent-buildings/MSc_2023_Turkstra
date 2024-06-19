#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <map>
#include <algorithm>

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/utilities/geometry.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/building_physics/bp_model.hpp>
#include <bso/grammar/grammar.hpp>
#include <bso/visualization/visualization.hpp>
#include <bso/grammar/sd_grammars/design_tangram.cpp>
#include <boost/algorithm/string.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

bso::spatial_design::ms_building MS("settings/Tangram");
bso::spatial_design::cf_building CF(MS);
bso::grammar::grammar grm(CF);

bool visualizationActive = true;
bso::visualization::viewportmanager vpmanager_local;
bso::visualization::orbitalcamera   cam_local;
int prevx, prevy;

typedef void (*ButtonCallback)(int);

// Counter for iterations
int iteration_counter = 1;
const int max_iterations = 2; // actual iterations is 1 lower

// Inactive
std::chrono::steady_clock::time_point lastInteractionTime;
const int inactivityLimit = 300000;  // 5 minutes in milliseconds

int currentModelIndex = -1; // Initialize to -1

std::vector<bso::spatial_design::sc_building> scDesigns;
std::vector<bso::spatial_design::ms_building> msDesigns;
std::vector<bso::spatial_design::ms_building> msDesignsTemp;
std::vector<bso::structural_design::sd_model> intermediateModels;

bso::structural_design::component::structure trussStructure("truss",{{"A",2250},{"E",3e4}});
bso::structural_design::component::structure beamStructure("beam",{{"width",150},{"height",150},{"poisson",0.3},{"E",3e4}});
bso::structural_design::component::structure flatShellStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e4}});
bso::structural_design::component::structure substituteStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e-2}});

struct Button {
    float x, y, width, height;
    ButtonCallback callback;
    const char* text;
    int variable;
    bool isActive;

    Button(float x, float y, float width, float height, ButtonCallback callback, const char* text, int variable, bool isActive)
        : x(x), y(y), width(width), height(height), callback(callback), text(text), variable(variable), isActive(isActive) {}
};

struct CallbackData {
    ButtonCallback callback;
    int variable;
};

std::vector<CallbackData> callbackQueue; // This holds all pending callbacks


struct TextField {
    float x, y, width, height;
    std::string text;
    int cursorPosition;
    bool isActive;

    // Constructor
    TextField() : cursorPosition(0), isActive(false), x(0), y(0), width(0), height(0) {}

    // Add a character where the cursor is
    void addChar(char c) {
        if (cursorPosition < text.length()) {
            text.insert(text.begin() + cursorPosition, c);
        } else {
            text.push_back(c);
        }
        cursorPosition++;
    }

    // Remove a character at the cursor
    void removeChar() {
        if (!text.empty() && cursorPosition > 0) {
            text.erase(text.begin() + cursorPosition - 1);
            cursorPosition--;
        }
    }

    // Handle the Enter key
    void submit() {
        std::cout << text << std::endl;
        // Clear text if needed
        text.clear();
        cursorPosition = 0;
    }
};

std::vector<Button> buttons;

// Global variables for current screen and screen dimensions
int currentScreen = 0;
int marginText = 10;
int startText;
int textWidth;
int screenHeight;
int screenWidth;

// Text margin as a percentage of the window width
const float MARGIN_PERCENT = 1.0f; // Margin as a percentage of the window width

// Function prototypes
void display();
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void updateTextureCoordinates(int width, int height);
void introScreen();
void buildingSpatialScreen();
void structuralModelScreen();
void structuralModelFloor1Screen();
void structuralModelFloor23Screen();
void removeSpaceScreen();
void iterationCompleteScreen();
void drawText(const char* text, float startX, float centerY, float textWidth, float r, float g, float b, bool bold = false);
void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable);
void drawTextField(int x, int y, int width, int height, TextField& textfield);
void onMouseClick(int button, int state, int x, int y);
void drawBuilding();
void setup2D();
void setup3D();
void outroScreen();
void introScreen();
void splitSpaces(int nSpacesToDelete);

std::string clean_str(const std::string& input);

void visualise(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "ms_building", const double& lineWidth = 1.0)
{
    vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void visualise(const bso::spatial_design::cf_building& cf, std::string type, std::string title = "sc_building")
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::Conformal_Model(cf, type, title)));
}

void visualise(const bso::structural_design::sd_model& model, const std::string& type ="component",
               const std::string& title ="sd_model", const bool& ghostly = false,
               const std::vector<std::pair<bso::utilities::geometry::vertex,
               bso::utilities::geometry::vector>>& cuttingPlanes = {})
{
    vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::SD_Model(model, type, title, ghostly, cuttingPlanes)));
}


void checkGLError(const char* action) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cout << "OpenGL error after " << action << ": " << gluErrorString(err) << std::endl;
    }
}

void buttonClicked(int variable) {
    std::cout << "Button clicked: " << variable << std::endl;
}

std::string clean_str(const std::string& input) {
    std::string result;
    for (char ch : input) {
        if (isdigit(ch)) {
            result += ch;
        }
    }
    return result;
}

unsigned int initialSpaceCount = MS.getSpacePtrs().size();
double floorArea = MS.getFloorArea();

int pendingOperationSpaceID = -1;
bool spaceRemovalStart = false;
bool spaceRemovalDone = false;
bool spaceSplittingDone = false;
bool scalingDone = false;
bool processGrammarStart = false;
bool processGrammarDone = false;
bool resetStart = false;
bool resetIteration = false;

std::chrono::steady_clock::time_point lastEnterPressTime;
const int enterDebounceInterval = 2000;

bso::structural_design::sd_model SD;

struct GrammarConfig {
    double etaBend;
    double etaAxial;
    double etaShear;
    std::string checkingOrder;
};

// Globals that are directly modified
double globalEtaBend = 0.4;
double globalEtaAxial = 0.4;
double globalEtaShear = 0.4;
std::string globalCheckingOrder = "123";
// Global variable to hold the number of spaces to delete
int globalNSpacesToDelete = 0;


// Create specific configurations
GrammarConfig config1 = {0.6, 1.0, 0, "123"};
GrammarConfig config2 = {0, 0.6, 0.8, "231"};
GrammarConfig config3 = {0.2, 0.4, 0, "312"};

void updateModel(int value) {
    if (currentScreen != 2 || currentModelIndex < 0 || currentModelIndex >= intermediateModels.size()) {
        return; // Do not update if not on screen 2 or if index is out of range
    }

    // Increment model index
    currentModelIndex++;

    // Check if we've reached the end of the list of models
    if (currentModelIndex >= intermediateModels.size()) {
        currentModelIndex = 0;  // Optionally reset to start or stop updating
        intermediateModels.clear();
        processGrammarStart = false;
        processGrammarDone = true;
        return;  // Stop calling updateModel if you do not want to loop
    }

    visualise(intermediateModels[currentModelIndex]);

    // Continue updating if still on screen 2
    if (currentScreen == 2) {
        glutTimerFunc(1000, updateModel, 0);  // Call updateModel again after a delay
    }
}

unsigned int nSpaces = MS.getSpacePtrs().size();

int endIndex = 0;

void resetModel() {
    std::cout << "Reset try";
    resetStart = true;
    // Call destructors and then use placement new
    MS.~ms_building();
    new (&MS) bso::spatial_design::ms_building("settings/Tangram");

    CF = bso::spatial_design::conformal::cf_building_model(MS);

    grm.~grammar();
    new (&grm) bso::grammar::grammar(CF);


    spaceRemovalStart = false;
    spaceRemovalDone = false;
    spaceSplittingDone = false;
    scalingDone = false;
    processGrammarStart = false;
    processGrammarDone = false;
    resetIteration = false;
    endIndex = 0;
    iteration_counter = 1;
    std::cout << "Reset done";
}

void processGrammar(const GrammarConfig& config) {
    int startIndex = intermediateModels.size() + endIndex;
    std::cout << "startIndex" << startIndex << std::endl;

    globalEtaBend = config.etaBend;
    globalEtaAxial = config.etaAxial;
    globalEtaShear = config.etaShear;
    globalCheckingOrder = config.checkingOrder;

    bso::structural_design::sd_model SD = grm.sd_grammar<bso::grammar::DESIGN_TANGRAM>(std::string("settings/sd_settings.txt"), config.etaBend, config.etaAxial, config.etaShear, 0.025, 6, config.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);

    intermediateModels = grm.getIntermediateSDModels();
    endIndex = intermediateModels.size();
    std::cout << "endIndex" << endIndex << std::endl;


    // Use startIndex to visualize new models starting from the first newly added model
    if (!grm.getIntermediateSDModels().empty()) {
        currentModelIndex = startIndex;
        visualise(intermediateModels[currentModelIndex]);  // Visualize the first new model
        glutTimerFunc(1000, updateModel, 0);  // Restart the timer
    }
}

void processGrammarConfig1(int value) {
    std::cout << "Processing new grammar configuration" << std::endl;
    processGrammar(config1);
}

void processGrammarConfig2(int value) {
    std::cout << "Processing new grammar configuration" << std::endl;
    processGrammar(config2);
}

void processGrammarConfig3(int value) {
    std::cout << "Processing new grammar configuration" << std::endl;
    processGrammar(config3);
}

void scaleModel() {
    // SCALE TO RECOVER INITIAL FLOOR AREA
    double scaleFactor = sqrt(floorArea / MS.getFloorArea());
    MS.scale({{0,scaleFactor},{1,scaleFactor}});
    MS.snapOn({{0,1},{1,1}});
    // Update the visualization to reflect the changes
    visualise(MS);
    spaceRemovalStart = false;
    scalingDone = true;
    MS.snapOn({{0,100},{1,100},{2,100}});
}

void iterationDone() {
    // Update the grammar with new MS
    CF = bso::spatial_design::conformal::cf_building_model(MS);
    grm.~grammar(); // Call the destructor manually
    new (&grm) bso::grammar::grammar(CF); // Reconstruct 'grm' in the existing memory

    // Increment iteration counter
    spaceRemovalStart = false;
    spaceRemovalDone = false;
    spaceSplittingDone = false;
    scalingDone = false;
    processGrammarStart = false;
    processGrammarDone = false;
    endIndex = 0;
    iteration_counter++;
}

void init() {
    // Start the timer only, do not visualize immediately
    if (!intermediateModels.empty()) {
        glutTimerFunc(1000, updateModel, 0); // Start the timer
    }
}

void triggerScaling(int value) {
    scaleModel();
}

void triggerSplitSpaces(int value) {
    splitSpaces(globalNSpacesToDelete);
}

void removeSpaces(int nSpacesToDelete) {
    resetStart = false;
    globalNSpacesToDelete = nSpacesToDelete;
    spaceRemovalStart = true;
    // Analyze the latest structural model
    bso::structural_design::sd_model SD = grm.sd_grammar<bso::grammar::DESIGN_TANGRAM>(std::string("settings/sd_settings.txt"), globalEtaBend, globalEtaAxial, globalEtaAxial, 0.025, 6, globalCheckingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    std::cout << "Removing " << nSpacesToDelete << " spaces." << std::endl;
    SD.analyze();
    std::cout << "analyzed model" << std::endl;
    double sdResult = SD.getTotalResults().mTotalStrainEnergy;
    std::cout << SD.getTotalResults().mTotalStrainEnergy << ","
							<< SD.getTotalResults().mTotalStructuralVolume;
    std::cout << sdResult << std::endl;

    // Get performances per space
    std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformances;
    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        spacePerformance /= space->getFloorArea();
        spacePerformances.push_back({spacePerformance, space});
    }

    // Sort spaces from worst to best (low strain energy to high strain energy)
    std::sort(spacePerformances.rbegin(), spacePerformances.rend());

    // Output ranked performances per space
    std::cout << "Ranked Performances:\n";
    int rank = 1;
    for (const auto& performance : spacePerformances) {
        auto id = performance.second->getID();
        std::cout << rank << ", Space ID: " << id << ", Performance: " << performance.first << std::endl;
        rank++;
    }

    // Delete n worst performing spaces
    for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesToDelete && j < spacePerformances.size(); --j) {
        MS.deleteSpace(*(spacePerformances[j].second));
    }
    MS.setZZero();

    std::vector<bso::spatial_design::ms_space*> floatingSpaces;
	if (MS.hasFloatingSpaces(floatingSpaces))
	{
		for (auto& i : floatingSpaces) MS.deleteSpace(i);
	}

    spaceRemovalDone = true;

    visualise(MS);
    glutPostRedisplay(); // Update the display to show the text

    // Set a timer to call triggerSplitSpaces after a delay
    int delayInMillis = 2000; // 3 seconds delay
    glutTimerFunc(delayInMillis, triggerSplitSpaces, 0);
}

void splitSpaces(int nSpacesToDelete) {
    bso::structural_design::sd_model SD = grm.sd_grammar<bso::grammar::DESIGN_TANGRAM>(std::string("settings/sd_settings.txt"), globalEtaBend, globalEtaAxial, globalEtaAxial, 0.025, 6, globalCheckingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    std::cout << "Removing " << nSpacesToDelete << " spaces." << std::endl;
    SD.analyze();
    std::cout << "analyzed model" << std::endl;
    double sdResult = SD.getTotalResults().mTotalStrainEnergy;
    std::cout << SD.getTotalResults().mTotalStrainEnergy << ","
							<< SD.getTotalResults().mTotalStructuralVolume;
    std::cout << sdResult << std::endl;

    // Get performances per space
    std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformances;
    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        spacePerformance /= space->getFloorArea();
        spacePerformances.push_back({spacePerformance, space});
    }

    // Sort spaces from worst to best (low strain energy to high strain energy)

    std::sort(spacePerformances.rbegin(), spacePerformances.rend());
    // EXCLUDE ALREADY SPLIT SPACES
    auto removeSpacesIterator = std::remove_if(spacePerformances.begin(), spacePerformances.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformances.erase(removeSpacesIterator, spacePerformances.end());
    // SPLIT n BEST PERFORMING SPACES
    //for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesDelete && j < spacePerformances.size(); --j)
    for (unsigned int j = 0; j < nSpacesToDelete && j < spacePerformances.size(); ++j)
    {
        auto spaceWithLowScore = spacePerformances[j].second;
        // FIND THE LARGEST DIMENSION
        double largestDimension = -1.0; // Initialize with a small value
        unsigned int largestDimensionIndex = 0;
        for (unsigned int k = 0; k < 3; ++k) {
            double dimension = spaceWithLowScore->getDimensions()(k);
            if (dimension > largestDimension) {
                largestDimension = dimension;
                largestDimensionIndex = k;
            }
        }
        // SPLIT THE SPACE ALONG ITS LARGEST DIMENSION
        MS.splitSpace(spaceWithLowScore, {{largestDimensionIndex, 2}});
    }
    spaceSplittingDone = true;
    visualise(MS);

    // Set a timer to call triggerScale after a delay
    int delayInMillis = 3000; // 3 seconds delay
    glutTimerFunc(delayInMillis, triggerScaling, 0);
}

GLuint loadImageAsTexture(const char* filename) {
    int width, height, nrChannels;
    unsigned char* data = stbi_load(filename, &width, &height, &nrChannels, 0);
    if (!data) {
        std::cerr << "Failed to load texture: " << filename << std::endl;
        return 0;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum format = GL_RGB; // Default to GL_RGB
    if (nrChannels == 1)
        format = GL_RED;
    else if (nrChannels == 3)
        format = GL_RGB; // Explicitly set, even though it's the default
    else if (nrChannels == 4)
        format = GL_RGBA;

    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);

    return textureID;
}

void displayTexture(GLuint texture, float x, float y, float width, float height) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(x, y);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(x + width, y);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(x + width, y + height);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(x, y + height);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

GLuint imgINTRO;
GLuint imgOUTRO;

void initializeTextures() {
    imgINTRO = loadImageAsTexture("settings/introScreen.jpg");
    imgOUTRO = loadImageAsTexture("settings/outroScreen.jpg");
    // Load more textures as needed
}

void changeScreen(int screen) {
    currentScreen = screen;
    std::cout << "Screen changed to: Screen " << screen << std::endl;
    vpmanager_local.clearviewports();  // Clear viewports to start fresh

    if (screen == 1) {
        visualise(MS);
    } else if (screen == 2) {
        visualise(MS);
    } else if (screen == 3) {
        visualise(MS);
    } else {
        visualise(MS);
    }

    visualizationActive = true;
    glutPostRedisplay();
    buttons.clear();
}

void reshape(int width, int height) {
    screenHeight = height;
    screenWidth = width;

    marginText = 20;
    startText = screenWidth / 1.28f + marginText;
    textWidth = screenWidth - startText - marginText;

    glViewport(0, 0, width, height);
    glutPostRedisplay();
    // For 2D rendering
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, screenWidth, 0.0, screenHeight);

    //For 3D rendering (commented out; use if needed)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    gluPerspective(45.0, aspectRatio, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void motion(int x, int y)
{
    lastInteractionTime = std::chrono::steady_clock::now();
    // Calculate the boundary of the 3D view area dynamically
    float viewWidth = screenWidth / 1.28f;

    // Only perform rotation if the mouse is within the 3D view area
    if (x <= viewWidth) {
        double dx = prevx - x;
        double dy = prevy - y;

        cam_local.setrotation(cam_local.getrotation() + (dx * 0.5));
        cam_local.setelevation(cam_local.getelevation() + (dy * 0.5));

        prevx = x;
        prevy = y;
    }

    // Always handle other motion events
    vpmanager_local.mousemove_event(x, y);

    glutPostRedisplay();
}


void passive_motion(int x, int y)
{
    vpmanager_local.mousemove_event(x, y);
}


void display() {
    // Clear the window with white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set up 2D projection matrix
    setup2D();
    // visualise(MS);

    // Set up modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (visualizationActive) {
        // Set viewport for the left half of the screen
        setup3D();

        // Render the visualization
        vpmanager_local.render(cam_local);
        checkGLError("render");

        // Reset the viewport to full window size for the rest of your GUI, if necessary
        setup2D();
        checkGLError("setup2D");
    }

    // Render the current screen
    switch (currentScreen) {
        case 0: introScreen(); break;
        case 1: buildingSpatialScreen(); break;
        case 2: structuralModelScreen(); break;
        case 3: removeSpaceScreen(); break;
        case 4: iterationCompleteScreen(); break;
        case 5: outroScreen(); break;
        default: break;
    }

    // Swap buffers
    glutSwapBuffers();

    // Check for any OpenGL errors
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << gluErrorString(err) << std::endl;
    }

    glutPostRedisplay();
}

void setup2D() {
    glViewport(0, 0, screenWidth, screenHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, screenWidth, 0.0, screenHeight);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);

    // Disable lighting for 2D
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
}

void setup3D() {
    GLint viewportWidth = screenWidth / 1.28;
    GLint viewportHeight = 1.1 * screenHeight;

    vpmanager_local.resize(viewportWidth, viewportHeight);

    // Set the viewport to cover the left part of the screen
    glViewport(0, 0, viewportWidth, viewportHeight);

    // Setup the projection matrix for 3D rendering
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Adjust the perspective projection to match the new aspect ratio
    GLfloat aspectRatio = (GLfloat)viewportWidth / (GLfloat)viewportHeight;
    gluPerspective(45.0, aspectRatio, 0.1f, 1000.0f);

    // Switch back to modelview matrix mode
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Enable depth testing, required for 3D rendering
    glEnable(GL_DEPTH_TEST);

    // Enable lighting if your visualization uses it
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

int getNextScreen(int currentScreen) {
    // This is a placeholder function, adjust logic as per your application's flow
    return (currentScreen + 1) % 6; // Wrap around after the last screen
}

void keyboard(unsigned char key, int x, int y) {
    auto now = std::chrono::steady_clock::now();
    lastInteractionTime = now;

    if (key == 13) {  // ASCII value for Enter
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastEnterPressTime).count();

        if (elapsed > enterDebounceInterval) {
            lastEnterPressTime = now; // Update the time of the last Enter press
            if (currentScreen == 3) {
                if (iteration_counter == 1) {
                    changeScreen(4);
                } else {
                    changeScreen(5);
                }
            }
            else if (currentScreen == 4) {
                resetIteration = true;
                iterationDone();
                changeScreen(1);  // Assuming screen 1 is the next logical screen in the workflow
            }
            else if (currentScreen == 5) {
                resetStart = true;
                resetModel();
                changeScreen(0);
            }
            else {
                // For all other screens, move to the next screen
                int nextScreen = getNextScreen(currentScreen);
                changeScreen(nextScreen);
            }
        }
    }

    glutPostRedisplay();  // Always refresh the display after handling the key
}


void checkInactivity(int value) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastInteractionTime).count();

    if (elapsed > inactivityLimit) {
        resetModel();
        changeScreen(0);  // Function to change the screen
    }

    // Re-register the timer callback
    glutTimerFunc(1000, checkInactivity, 0); // Check every second
}



void drawText(const char* text, float startX, float centerY, float textWidth, float r, float g, float b, bool bold) {
    float lineHeight = 18; // Approximate line height
    float effectiveTextWidth = textWidth; // Assuming margins are already accounted for

    // Simplified adjustments for bold text rendering
    const float offsets[][2] = {{0.0f, 0.0f}, {0.5f, 0.0f}, {-0.5f, 0.0f}};
    int numOffsets = bold ? 3 : 1;

    glColor3f(r, g, b);

    for (int i = 0; i < numOffsets; ++i) {
        float offsetX = offsets[i][0];
        float currentX = startX;
        float currentY = centerY;

        const char* word = text;
        while (*word != '\0') { // Iterate through each character in text
            const char* nextSpace = strchr(word, ' ');
            int wordLength;
            if (nextSpace != NULL) {
                wordLength = nextSpace - word;
            } else {
                wordLength = strlen(word); // Last word
            }

            // Calculate width of the next word
            float wordWidth = 0;
            for (int j = 0; j < wordLength; j++) {
                wordWidth += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, word[j]);
            }

            // Check if the word fits in the remaining line, wrap if it doesn't
            if (currentX - startX + wordWidth > effectiveTextWidth) {
                currentY -= lineHeight;
                currentX = startX;
            }

            // Draw the word
            for (int j = 0; j < wordLength; j++) {
                glRasterPos2f(currentX + offsetX, currentY);
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, word[j]);
                currentX += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, word[j]);
            }

            // Skip the space
            if (nextSpace != NULL) {
                currentX += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, ' '); // Add space width if not the last word
                word = nextSpace + 1; // Move to the start of the next word
            } else {
                break; // End of text
            }
        }
    }
}

int calculateTextWidth(const char* text, bool isBold) {
    int textWidth = 0;
    while (*text) {
        int charWidth = glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *text);
        if (isBold) {
            charWidth += 1;  // Assuming bold text might be 1 pixel wider per character
        }
        textWidth += charWidth;
        ++text;
    }
    return textWidth;
    glutPostRedisplay();
}


void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable) {
    float borderWidth = 2.0;

    // Draw button border
    glColor3f(0.0, 0.0, 0.0); // Black
    glBegin(GL_QUADS);
    glVertex2f(x - borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y + height + borderWidth);
    glVertex2f(x - borderWidth, y + height + borderWidth);
    glEnd();

    // Draw button background
    glColor3f(1.0, 1.0, 1.0); // White
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();

    // Check if this button is active to decide if text should be bold
    bool isButtonActive = false;
    for (Button& btn : buttons) {
        if (btn.x == x && btn.y == y && btn.width == width && btn.height == height) {
            isButtonActive = btn.isActive;
            break;
        }
    }

    // Accurately calculate text width and consider bold text if active
    int textWidth = calculateTextWidth(text, isButtonActive);
    // Center the text horizontally and vertically
    float textX = x + (width - textWidth) / 2.0f;
    float textY = y + (height - 18) / 2.0f; // Using 18 as an approximate height for GLUT_BITMAP_HELVETICA_18

    // Set text color and draw text, draw bold if active
    glColor3f(0.0f, 0.0f, 0.0f); // Black
    glRasterPos2f(textX, textY);
    for (const char* p = text; *p; p++) {
        if (isButtonActive) {
            // Draw each character multiple times slightly offset to mimic bold
            glRasterPos2f(textX, textY);
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
            glRasterPos2f(textX + 1, textY);
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
        } else {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
        }
        textX += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *p);
    }

    Button button = {x, y, width, height, callback, text, variable, isButtonActive};
    buttons.push_back(button);
}


void checkTextFieldClick(TextField& textField, float mouseX, float mouseY) {
    if (mouseX >= textField.x && mouseX <= textField.x + textField.width &&
        mouseY >= textField.y && mouseY <= textField.y + textField.height) {
        textField.isActive = true;
    }
    else {
        textField.isActive = false;
    }
}

void delayedCallback(int value) {
    if (value >= 0 && value < callbackQueue.size()) {
        CallbackData& data = callbackQueue[value];
        if (data.callback) {
            data.callback(data.variable);
        }
        // Optionally clear the callback from the queue if no longer needed
        callbackQueue.erase(callbackQueue.begin() + value);
    }
}


void onMouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        float mouseY = screenHeight - static_cast<float>(y);
        float mouseX = static_cast<float>(x);

        // Deactivate all buttons first
        for (auto& btn : buttons) {
            btn.isActive = false;
        }

        // Check each button if it is clicked and activate it
        for (auto& btn : buttons) {
            if (mouseX >= btn.x && mouseX <= btn.x + btn.width &&
                mouseY >= btn.y && mouseY <= btn.y + btn.height) {
                btn.isActive = true;
                processGrammarDone = false;
                processGrammarStart = true; // Activate the clicked button
                glutPostRedisplay();  // Ask GLUT to redraw the screen

                // Store the callback data
                CallbackData data { btn.callback, btn.variable };
                callbackQueue.push_back(data); // Store the callback data for later execution

                // Schedule the heavy callback to run after a short delay
                glutTimerFunc(100, delayedCallback, callbackQueue.size() - 1);  // Pass index in the callbackQueue

                break;
            }
        }
    }
}



void drawTextField(int x, int y, int width, int height, TextField& textfield) {
    textfield.x = x;
    textfield.y = y;
    textfield.width = width;
    textfield.height = height;
    float borderWidth = 2.0;

    // Calculate the adjusted width and height considering padding
    int adjustedWidth = width - 2 * borderWidth;
    int adjustedHeight = height - 2 * borderWidth;

    glColor3f(0.0, 0.0, 0.0); // Black color for border
    glBegin(GL_QUADS);
    glVertex2f(x - borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y + height + borderWidth);
    glVertex2f(x - borderWidth, y + height + borderWidth);
    glEnd();

    // Draw text field background
    glColor3f(1.0, 1.0, 1.0); // white background for text field
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();

    // Set the color for the text
    glColor3f(0.0, 0.0, 0.0); // black text

    // Implement text wrapping within the width of the text field
    // This is a simplistic approach and might need adjustment for different font widths
    int maxWidth = adjustedWidth; // maximum width for text before wrapping
    int currentWidth = 0;
    std::string line;
    std::vector<std::string> lines;

    for (char c : textfield.text) {
        if (c == '\n' || currentWidth > maxWidth) {
            lines.push_back(line);
            line.clear();
            currentWidth = 0;
        }
        if (c != '\n') {
            line += c;
            // Estimate the width of the character, adjust for the new font size (18)
            currentWidth += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, c);
        }
    }
    // Add the last line if it's not empty
    if (!line.empty()) {
        lines.push_back(line);
    }

    // Now draw the text line by line from the top
    int lineHeight = 20; // Adjusted for the new font size (18) and additional padding
    int textHeight = lines.size() * lineHeight; // Total height of the text
    int startY = y + height - lineHeight; // Calculate the starting Y coordinate from the top with padding

    for (size_t i = 0; i < lines.size(); ++i) {
        int currentY = startY - i * lineHeight; // Calculate the Y coordinate for this line
        glRasterPos2f(x + borderWidth, currentY); // Adjust for the left padding
        for (char c : lines[i]) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c); // Use GLUT_BITMAP_HELVETICA_18 for the new font size
        }
    }

    // Draw the cursor if the text field is active
    if (textfield.isActive) {
        int cursorX = x + borderWidth + glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)textfield.text.c_str()); // Adjust for left padding
        int cursorY = startY; // Use the same starting Y coordinate as the text
        glColor3f(0.0, 0.0, 0.0); // black cursor
        glBegin(GL_LINES);
        glVertex2f(cursorX + 2, cursorY + 18);
        glVertex2f(cursorX + 2, cursorY - 3);  // Adjust the Y coordinate to draw the cursor above the text
        glEnd();
    }
}

void introScreen() {
    glEnable(GL_LIGHTING); // Enable to show image becomes black
    glEnable(GL_LIGHT0); // Enable to prevent image becomes black

    float picWidth = screenWidth / 1.28f; // Width of the picture as specified.
    float picHeight = screenHeight;
    displayTexture(imgINTRO, 0, 0, picWidth, picHeight);

    glDisable(GL_LIGHTING); //Disbale for other GUI elements
    glDisable(GL_LIGHT0); //Disbale for other GUI elements
    // Draw the title
    drawText("Simulatie van Co-evolutionaire Ontwerpprocessen", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("In deze interactieve demo visualiseren we het principe van simulatie van co-evolutionaire ontwerpprocessen. Dit is een methode om ruimtelijk ontwerp iteratief te optimaliseren op basis van de constructieve prestaties van het ontwerp. Het aanpassen gebeurt door feedback van het constructief ontwerp, dat gegenereerd wordt uit het ruimtelijke ontwerp.", startText, 900, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("In deze demo gebruiken we het ruimtelijk ontwerp van Crystal Court, ontworpen door TANGRAM Architekten. Een visualisatie van dit gebouw ziet u hiernaast.", startText, 400, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Mocht u verder willen gaan naar de volgende stap, dan kunt u dat doen door op 'Enter' te drukken.", startText, 100, textWidth, 0.0f, 0.0f, 0.0f);
}

void buildingSpatialScreen() {
    drawText("Ruimtelijk Ontwerp", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Het gebouw dat zojuist zichtbaar was, is omgezet in een ruimtelijk ontwerp dat Crystal Court vertegenwoordigt. Dit ontwerp omvat 35 ruimtes verdeeld over 6 verdiepingen.", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("U kunt het model roteren door in het modelvenster met ingedrukte linkermuisknop te draaien.", startText, 700, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Mocht u verder willen gaan naar de volgende stap, dan kunt u dat doen door op 'Enter' te drukken.", startText, 100, textWidth, 0.0f, 0.0f, 0.0f);
    std::string iterationText = "Iteratie " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Ruimtelijk ontwerp", 80, 970, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Constructief ontwerp", 80, 950, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes verwijderen", 80, 930, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes splitsen", 80, 910, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ontwerp verschalen", 80, 890, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Iteratie voltooid", 80, 870, 250, 0.0f, 0.0f, 0.0f, false);
}

void structuralModelScreen() {
    drawText("Constructief Ontwerp", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("De tweede stap in het proces is het genereren van een constructief ontwerp. Dit gebeurt door per vlak te bepalen welk soort kracht er actief is en op basis daarvan een specifiek constructief element toe te wijzen. Dit proces kan worden beïnvloed door verschillende parameters. Hieronder vindt u drie scenario's voor het genereren van een constructief ontwerp. ", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Klik op een van de knoppen hieronder om een scenario te bekijken.", startText, 500, textWidth, 0.0f, 0.0f, 0.0f);

    if (processGrammarStart) {
        drawText("Een moment geduld a.u.b.", startText, 450, textWidth, 0.0f, 0.0f, 1.0f);
    }

    if (processGrammarDone) {
        drawText("Constructief ontwerp voltooid. Kies een ander scenario of ga verder naar de volgende stap.", startText, 450, textWidth, 0.0f, 0.0f, 1.0f);
    }

    drawButton("Duurzame constructie", startText, 350, textWidth, 50, processGrammarConfig1, 0);
    drawButton("Robuuste constructie", startText, 275, textWidth, 50, processGrammarConfig2, 0);
    drawButton("Gebalanceerde constructie", startText, 200, textWidth, 50, processGrammarConfig3, 0);

    drawText("Mocht u verder willen gaan naar de volgende stap, dan kunt u dat doen door op 'Enter' te drukken.", startText, 100, textWidth, 0.0f, 0.0f, 0.0f);

    std::string iterationText = "Iteratie " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Ruimtelijk ontwerp", 80, 970, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Constructief ontwerp", 80, 950, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Ruimtelijke aanpassing: hier ruimtes verwijderen", 80, 930, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes splitsen", 80, 910, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ontwerp verschalen", 80, 890, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Iteratie voltooid", 80, 870, 250, 0.0f, 0.0f, 0.0f, false);
}

void removeSpaceScreen() {
    drawText("Ruimtelijke Aanpassing", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("De derde stap in het proces is de ruimtelijke aanpassing, waarbij ruimtes die constructief ondermaats presteren verwijderd worden; dit zijn ruimtes die weinig bijdragen aan de stijfheid van het gebouw. Nadat deze ruimtes verwijderd zijn, worden er nieuwe ruimtes gecreëerd door bestaande ruimtes te splitsen, zodat het oorspronkelijke aantal ruimtes behouden blijft. Vervolgens wordt het model geschaald om het oorspronkelijke grondoppervlak te herstellen.", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("U kunt zelf bepalen hoeveel ruimtes verwijderd of gesplitst worden door op een van de knoppen hieronder te klikken.", startText, 700, textWidth, 0.0f, 0.0f, 0.0f);

    //if (!spaceRemovalDone) {
    //    drawButton("4 ruimtes", startText, 550, textWidth, 50, removeSpaces, 4);
    //    drawButton("6 ruimtes", startText, 475, textWidth, 50, removeSpaces, 6);
    //    drawButton("8 ruimtes", startText, 400, textWidth, 50, removeSpaces, 8);
    //}

    //drawButton("4 ruimtes", startText, 550, textWidth, 50, removeSpaces, 4);
    //drawButton("6 ruimtes", startText, 475, textWidth, 50, removeSpaces, 6);
    //drawButton("8 ruimtes", startText, 400, textWidth, 50, removeSpaces, 8);

    if (!spaceRemovalDone) {
        drawButton("1 ruimte", startText, 550, textWidth, 50, removeSpaces, 1);
        drawButton("2 ruimtes", startText, 475, textWidth, 50, removeSpaces, 2);
        drawButton("4 ruimtes", startText, 400, textWidth, 50, removeSpaces, 4);
        drawText("- Ruimtelijke aanpassing: hier ruimtes verwijderen", 80, 930, 800, 0.0f, 0.0f, 0.0f, true);
    }

    if (spaceRemovalStart && !scalingDone) {
        drawText("Een moment geduld a.u.b.", startText, 275, textWidth, 0.0f, 0.0f, 1.0f, true);
    }

    if (spaceRemovalDone) {
        drawText("Ruimtes succesvol verwijderd", startText, 250, textWidth, 0.0f, 0.0f, 1.0f);
        //drawText("- Ruimtes verwijderen", 80, 930, 250, 0.0f, 0.0f, 0.0f, false);
    }

    if (spaceRemovalDone && !spaceSplittingDone) {
        drawText("- Ruimtelijke aanpassing: hier ruimtes splitsen", 80, 910, 800, 0.0f, 0.0f, 0.0f, true);
    }

    if (spaceSplittingDone) {
        drawText("Ruimtes succesvol gesplitst", startText, 225, textWidth, 0.0f, 0.0f, 1.0f);
        drawText("- Ruimtelijke aanpassing: hier ontwerp verschalen", 80, 890, 800, 0.0f, 0.0f, 0.0f, true);
    }

    if (scalingDone) {
        drawText("Model succesvol geschaald", startText, 200, textWidth, 0.0f, 0.0f, 1.0f);
        drawText("Ruimtelijke aanpassing voltooid", startText, 175, textWidth, 0.0f, 0.0f, 1.0f,true);
    }

    drawText("Mocht u verder willen gaan naar de volgende stap, dan kunt u dat doen door op 'Enter' te drukken.", startText, 100, textWidth, 0.0f, 0.0f, 0.0f);

    std::string iterationText = "Iteratie " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Ruimtelijk ontwerp", 80, 970, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Constructief ontwerp", 80, 950, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes verwijderen", 80, 930, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes splitsen", 80, 910, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ontwerp verschalen", 80, 890, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Iteratie voltooid", 80, 870, 250, 0.0f, 0.0f, 0.0f, false);
}

void iterationCompleteScreen() {
    // Not the final iteration yet
    spaceRemovalDone = false;
    spaceSplittingDone = false;
    scalingDone = false;
    drawText("Iteratie Voltooid", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("U hebt zojuist de eerste iteratie van het SCDP-proces voltooid. We zijn begonnen met een ruimtelijk ontwerp, waarna een constructief ontwerp is gemaakt. Op basis van de constructieve prestaties zijn ruimtes verwijderd en gesplitst. Het nieuwe ruimtelijke ontwerp van deze iteratie vormt het uitgangspunt voor de volgende iteratie, waarbij we streven naar een steeds beter ruimtelijk ontwerp.", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);

    if (resetIteration) {
        drawText("Bezig met starten tweede iteratie...", startText, 500, textWidth, 0.0f, 0.0f, 1.0f,true);
    }

    drawText("Druk op 'Enter' om de tweede iteratie te starten.", startText, 100, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Iteratie 1", 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("- Ruimtelijk ontwerp", 80, 970, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Constructief ontwerp", 80, 950, 250, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes verwijderen", 80, 930, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ruimtes splitsen", 80, 910, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Ruimtelijke aanpassing: hier ontwerp verschalen", 80, 890, 800, 0.0f, 0.0f, 0.0f, false);
    drawText("- Iteratie voltooid", 80, 870, 250, 0.0f, 0.0f, 0.0f, true);
}

void outroScreen() {
    glEnable(GL_LIGHTING); // Enable to show image becomes black
    glEnable(GL_LIGHT0); // Enable to prevent image becomes black

    float picWidth = screenWidth / 1.28f; // Width of the picture as specified.
    float picHeight = screenHeight;
    displayTexture(imgOUTRO, 0, 0, picWidth, picHeight);

    glDisable(GL_LIGHTING); //Disbale for other GUI elements
    glDisable(GL_LIGHT0); //Disbale for other GUI elements
    // Draw the title
    drawText("Demo Voltooid", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("U hebt zojuist de tweede iteratie van het principe van simulatie van co-evolutionaire ontwerpprocessen voltooid en daarmee ook de demo. Hiernaast ziet u een render van het aangepaste ruimtelijk ontwerp. Deze is tot stand gekomen door de volgende aanpassingen:", startText, 900, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("- Constructief ontwerp met een duurzame constructie", startText, 700, 800, 0.0f, 0.0f, 0.0f);
    drawText("- Ruimtelijke aanpassing met 4 ruimtes", startText, 680, 800, 0.0f, 0.0f, 0.0f);
    drawText("- Constructief ontwerp met een robuuste constructie", startText, 660, 800, 0.0f, 0.0f, 0.0f);
    drawText("- Ruimtelijke aanpassing met 4 ruimtes", startText, 640, 800, 0.0f, 0.0f, 0.0f);

    drawText("Druk op 'Enter' om de demo opnieuw te starten.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);
    if (resetStart) {
        drawText("Bezig met herstarten van de demo...", startText, 100, textWidth, 0.0f, 0.0f, 1.0f,true);
    }
}


int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(screenWidth, screenHeight);
    glutCreateWindow("Demo Simulatie van Co-evolutionaire Ontwerpprocessen");
    glutFullScreen(); // Make the window fullscreen right from the start

    initializeTextures();

    lastEnterPressTime = std::chrono::steady_clock::now() - std::chrono::milliseconds(enterDebounceInterval);  // Allow immediate Enter press on start
    lastInteractionTime = std::chrono::steady_clock::now();  // Initialize the interaction timer
    glutTimerFunc(1000, checkInactivity, 0);

    // Set callback functions
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutMouseFunc(onMouseClick);
    glutMotionFunc(motion);
    glutPassiveMotionFunc(passive_motion);

    glShadeModel(GL_SMOOTH);

    // Main loop
    glutMainLoop();
    return 0;
}

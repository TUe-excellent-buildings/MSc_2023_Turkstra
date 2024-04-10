#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <Windows.h>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/building_physics/bp_model.hpp>
#include <bso/grammar/grammar.hpp>
#include <bso/visualization/visualization.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

bso::spatial_design::ms_building MS("Villa");
bso::spatial_design::cf_building CF(MS);

bool visualizationActive = true; // Flag to control when to activate visualization
bso::visualization::viewportmanager vpmanager_local;
bso::visualization::orbitalcamera   cam_local;
int prevx, prevy;

typedef void (*ButtonCallback)(int);

struct Button {
    float x, y, width, height;
    ButtonCallback callback;
    const char* text;
    int variable;
};

struct LogEntry {
    std::string time;
    int iteration;
    std::string description;
    std::string value;

    // Constructor
    LogEntry(int iter, const std::string& desc, const std::string& val)
        : iteration(iter), description(desc), value(val) {
            // Format the current time upon entry creation
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::ostringstream oss;
            oss << std::put_time(std::localtime(&now_c), "%H:%M:%S");
            time = oss.str();
        }
};

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
std::vector<LogEntry> logEntries;

TextField removeSpace;
TextField splitSpace;
TextField explanation;

// Global variables for current screen and screen dimensions
int currentScreen = 0;
int marginText = 10;
int startText;
int textWidth;
int screenHeight;
int screenWidth;

// Text margin as a percentage of the window width
const float MARGIN_PERCENT = 1.0f; // Margin as a percentage of the window width

// Counter for iterations
int iteration_counter = 1;
const int max_iterations = 3; // actual iterations is 1 lower

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
void splitSpaceScreen();
void iterationCompleteScreen();
void surveyScreen();
void displayConfirmationRequest(int spaceID, const std::string& message, void (*action)(int));
void drawText(const char* text, float startX, float centerY, float textWidth, float r, float g, float b, bool bold = false);
void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable);
void drawArrow(float x, float y, bool leftArrow);
void drawUndoRedoButtons();
void drawTextField(int x, int y, int width, int height, TextField& textfield);
void onMouseClick(int button, int state, int x, int y);
void drawBuilding();
void drawTwoColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::string>& column1, const std::vector<std::string>& column2);
void setWindowIcon(const char* iconPath);
void setup2D();
void setup3D();
std::string clean_str(const std::string& input);

void visualise(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "ms_building", const double& lineWidth = 1.0)
{
    vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void visualise(const bso::spatial_design::cf_building& cf, std::string type, std::string title = "sc_building")
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::Conformal_Model(cf, type, title)));
}

void visualise_2(const bso::spatial_design::cf_building& cf, std::string type = "cuboid", std::string title = "sc_building")
{
	vpmanager_local.addviewport(new bso::visualization::viewport(new bso::visualization::Conformal_Model(cf, type, title)));
}

void visualize(const bso::structural_design::sd_model& sd, const std::string& type ="component",
							 const std::string& title ="sd_model", const bool& ghostly = false,
							 const std::vector<std::pair<bso::utilities::geometry::vertex,
							 bso::utilities::geometry::vector>>& cuttingPlanes = {})
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::SD_Model(sd, type, title, ghostly, cuttingPlanes)));
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

std::set<int> removedSpaceIDs;
std::set<int> splitSpaceIDs;

unsigned int initialSpaceCount = MS.getSpacePtrs().size();
double floorArea = MS.getFloorArea();

bool scalingCompleted = false;
bool splittingConfirmed = false;
bool deletionConfirmed = false;
bool inputFieldDisabled = false;
bool spaceInputError = false;
std::string spaceInputErrorMessage;

bool awaitingConfirmation = false; 
int pendingOperationSpaceID = -1; 
std::string confirmationMessage;
void (*confirmationAction)(int); 

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

GLuint imgVilla;
GLuint imgElements;

void initializeTextures() {
    imgVilla = loadImageAsTexture("Villa.png");
    imgElements = loadImageAsTexture("Elements.png");
    // Load more textures as needed
}

std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

void logAction(int iteration, const std::string& description, const std::string& value) {
    logEntries.emplace_back(iteration, description, value);
}

void writeToProcessFile(const std::string& fileName) {
    std::ofstream processFile(fileName, std::ios::app);
    static bool headerPrinted = false;
    if (!headerPrinted) {
        processFile << "Time;Iteration;Description;Value\n";
        headerPrinted = true;
    }

    for (const auto& entry : logEntries) {
        processFile << entry.time << ";" << entry.iteration << ";"
                    << entry.description << ";" << entry.value << "\n";
    }
    processFile.close();
    logEntries.clear();
}


void changeScreen(int screen) {
    inputFieldDisabled = false;
    currentScreen = screen;
    std::cout << "Screen changed to: Screen " << screen << std::endl;
    if(screen  != 3) {
        vpmanager_local.clearviewports();
        // visualise(MS);
        visualise(MS);
        //visualise(CF, "rectangles");
        // visualise(SD_Building, 1);
        visualizationActive = true;
    } else {
        vpmanager_local.clearviewports();
        visualise(MS);
        visualise_2(CF);
        visualizationActive = true;
    }
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


void displayConfirmationRequest(int spaceID, const std::string& message, void (*action)(int)) {
    pendingOperationSpaceID = spaceID;
    confirmationMessage = message;
    confirmationAction = action;
    awaitingConfirmation = true;

    spaceInputError = true;
    spaceInputErrorMessage = confirmationMessage;
    glutPostRedisplay();
}

bool validateSpaceRemoveID(const std::string& input, int& outNumber, std::string& errorMessage) {
    try {
        outNumber = std::stoi(input);
    } catch (...) {
        errorMessage = "Input is not a valid number.";
        return false;
    }

    if (outNumber <= 0 || outNumber > MS.getLastSpaceID()) {
        errorMessage = "ID is out of range.";
        return false;
    }

    if (removedSpaceIDs.find(outNumber) != removedSpaceIDs.end()) {
        errorMessage = "Space has been removed and is not eligible.";
        return false;
    }

    if (splitSpaceIDs.find(outNumber) != splitSpaceIDs.end()) {
        errorMessage = "Space has been split and is not eligible.";
        return false;
    }

    return true;
}


void removeSpaceConfirmed(int spaceID) {
    MS.deleteSpace(MS.getSpacePtr(MS.getSpacePtrs()[MS.getSpaceLocation(spaceID)]));
    std::cout << "Space removed: " << spaceID << std::endl;
    removedSpaceIDs.insert(spaceID); // Track the removed space ID
    std::cout << "Adding to removed: " << spaceID << std::endl;
    glutPostRedisplay();

    logAction(iteration_counter, "Space Removed", std::to_string(spaceID));

    // Reset state as necessary
    awaitingConfirmation = false;
    spaceInputError = false;
    spaceInputErrorMessage = "";
    deletionConfirmed = true;
    inputFieldDisabled = true; 

    // Refresh the visualization with the updated MS
    visualise(MS);
    visualizationActive = true;
}

bool validateSpaceSplitID(const std::string& input, int& outNumber, int initialSpaceCount, std::string& errorMessage) {
    try {
        outNumber = std::stoi(input);
    } catch (...) {
        errorMessage = "Input is not a valid number.";
        return false;
    }

    if (outNumber <= 0) {
        errorMessage = "ID is out of range.";
        return false;
    }

    if (removedSpaceIDs.find(outNumber) != removedSpaceIDs.end()) {
        errorMessage = "Space has been removed and is not eligible.";
        return false;
    }

    if (outNumber > MS.getLastSpaceID()) {
        errorMessage = "ID is out of range.";
        return false;
    }
    
    if (outNumber > initialSpaceCount) {
        errorMessage = "Space is created by split and is not eligible.";
        return false;
    }

    if (splitSpaceIDs.find(outNumber) != splitSpaceIDs.end()) {
        errorMessage = "Space has been split and is not eligible.";
        return false;
    }

    return true;
}



void splitSpaceConfirmed(int spaceID) {
    auto* spaceToSplit = MS.getSpacePtr(MS.getSpacePtrs()[MS.getSpaceLocation(spaceID)]);

    if (spaceToSplit) {
        // Find the largest dimension of the space
        double largestDimension = -1.0; // Initialize with a small value
        unsigned int largestDimensionIndex = 0;
        for (unsigned int k = 0; k < 3; ++k) {
            double dimension = spaceToSplit->getDimensions()(k);
            if (dimension > largestDimension) {
                largestDimension = dimension;
                largestDimensionIndex = k;
            }
        }

        // Split the space along its largest dimension
        MS.splitSpace(spaceToSplit, {{largestDimensionIndex, 2}});

        std::cout << "Space " << spaceID << " split along its largest dimension (" << largestDimensionIndex << ")." << std::endl;
        splitSpaceIDs.insert(spaceID); // Track the split space ID
        // Update the visualization to reflect the changes
        visualise(MS);
        splittingConfirmed = true;

    } else {
        std::cout << "Failed to find space for ID: " << spaceID << std::endl;
        spaceInputError = true;
        spaceInputErrorMessage = "Failed to find space for splitting.";
    }

    logAction(iteration_counter, "Space Split", std::to_string(spaceID));

    // Reset the UI and state flags regardless of outcome
    awaitingConfirmation = false;
    spaceInputError = false; 
    spaceInputErrorMessage = "";
    inputFieldDisabled = true; 

    glutPostRedisplay();
}


void scaleModel() {
    // SCALE TO RECOVER INITIAL FLOOR AREA
    double scaleFactor = sqrt(floorArea / MS.getFloorArea());
    MS.scale({{0,scaleFactor},{1,scaleFactor}});
    MS.snapOn({{0,1},{1,1}});
    // Update the visualization to reflect the changes
    visualise(MS);

    logAction(iteration_counter, "Scale Factor", std::to_string(scaleFactor));

    // Increment iteration counter
    iteration_counter++;
}

void motion(int x, int y)
{
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
        case 3: structuralModelFloor1Screen(); break;
        case 4: structuralModelFloor23Screen(); break;
        case 5: removeSpaceScreen(); break;
        case 6: splitSpaceScreen(); break;
        case 7: iterationCompleteScreen(); break;
        case 8: surveyScreen(); break;
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


void keyboard(unsigned char key, int x, int y) {
    // Change screens based on key press
    if (key == '!') currentScreen = 0;
    if (key == '@') currentScreen = 1;
    if (key == '#') currentScreen = 2;
    if (key == '$') currentScreen = 3;

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << gluErrorString(err) << std::endl;
    }

    if (currentScreen == 5) {
        if (awaitingConfirmation) {
            if (key == 'y' || key == 'Y') {
                confirmationAction(pendingOperationSpaceID); 
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
            } else if (key == 'n' || key == 'N') {
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                removeSpace.text = ""; 
                glutPostRedisplay();
            }
            return; 
        }
        else if (key == 13) { // Enter key pressed
            if (inputFieldDisabled) {
            spaceInputError = false;
            spaceInputErrorMessage = "";
            glutPostRedisplay(); 
            return;
            }
            int spaceID;
            std::string errorMessage; // Variable to hold the validation error message
            if (validateSpaceRemoveID(removeSpace.text, spaceID, errorMessage)) { // Validate input
                // If validation succeeds, set up for confirmation
                awaitingConfirmation = true;
                // Call displayConfirmationRequest
                displayConfirmationRequest(spaceID, "Press Y to confirm, N to cancel.", removeSpaceConfirmed);
            } else {
                // Handle invalid input based on the specific error message returned by validation
                removeSpace.text = "";
                spaceInputError = true;
                spaceInputErrorMessage = errorMessage;
                glutPostRedisplay();
            }
            return;
        }
        else if (key >= 32 && key <= 126) { // Check if it's a printable ASCII character
            removeSpace.text += key; // Append the character to the input string
        } else if (key == 8 && removeSpace.text != "") { // Backspace key
            removeSpace.text.pop_back(); // Remove the last character from input string
        }
    }

    if (currentScreen == 6) {
        if (awaitingConfirmation) {
            if (key == 'y' || key == 'Y') {
                confirmationAction(pendingOperationSpaceID);
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
            } else if (key == 'n' || key == 'N') {
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                splitSpace.text = "";
                glutPostRedisplay();
            }
            return;
        }
        else if (key == 13) { // Enter key pressed
            if (inputFieldDisabled) {
            spaceInputError = false;
            spaceInputErrorMessage = "";
            glutPostRedisplay(); 
            return;
            }
            int spaceID;
            std::string errorMessage; // Variable to hold the validation error message
            if (validateSpaceSplitID(splitSpace.text, spaceID, initialSpaceCount, errorMessage)) {
                // If validation succeeds, set up for confirmation
                awaitingConfirmation = true;
                // Call displayConfirmationRequest
                displayConfirmationRequest(spaceID, "Press Y to confirm, N to cancel.", splitSpaceConfirmed);
            } else {
                // Handle invalid input based on the specific error message returned by validation
                splitSpace.text = "";
                spaceInputError = true;
                spaceInputErrorMessage = errorMessage;
                glutPostRedisplay();
            }
            return;
        }
        else if (key >= 32 && key <= 126) { // Check if it's a printable ASCII character
            splitSpace.text += key; // Append the character to the input string
        } else if (key == 8 && removeSpace.text != "") { // Backspace key
            splitSpace.text.pop_back(); // Remove the last character from input string
        }
    }
    if (key == 'f') {
        static bool isFullScreen = false;
        isFullScreen = !isFullScreen;
        if (isFullScreen) {
            glutFullScreen(); // Go full screen
        } else {
            glutReshapeWindow(screenWidth, screenHeight); // Return to window mode with initial dimensions
            glutPositionWindow(100, 100); // Optionally reposition window
        }
    }
    glutPostRedisplay();
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

int calculateTextWidth(const char* text) {
    int textWidth = 0;
    while (*text) {
        textWidth += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *text);
        ++text;
    }
    return textWidth;
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

    // Accurately calculate text width
    int textWidth = calculateTextWidth(text);
    // Center the text horizontally and vertically
    float textX = x + (width - textWidth) / 2.0f;
    // Approximate vertical centering (adjust as needed for different fonts)
    float textY = y + (height - 18) / 2.0f; // Using 18 as an approximate height for GLUT_BITMAP_HELVETICA_18

    // Set text color and draw text
    glColor3f(0.0f, 0.0f, 0.0f); // Black
    glRasterPos2f(textX, textY);
    for (const char* p = text; *p; p++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);
    }

    Button button = {x, y, width, height, callback, text, variable};
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

void onMouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        float mouseY = screenHeight - static_cast<float>(y);
        float mouseX = static_cast<float>(x);

        // Check each text field individually
        checkTextFieldClick(removeSpace, mouseX, mouseY);
        checkTextFieldClick(splitSpace, mouseX, mouseY);
        checkTextFieldClick(explanation, mouseX, mouseY);

        // Check for button clicks
        for (const auto& btn : buttons) {
            if (mouseX >= btn.x && mouseX <= btn.x + btn.width &&
                mouseY >= btn.y && mouseY <= btn.y + btn.height) {
                // Button was clicked
                if (btn.callback) {
                    btn.callback(btn.variable);
                }
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
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
        }
    }

    // Draw the cursor if the text field is active
    if (textfield.isActive) {
        int cursorX = x + borderWidth + glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)textfield.text.c_str()); // Adjust for left padding
        int cursorY = startY; // Use the same starting Y coordinate as the text
        glColor3f(0.0, 0.0, 0.0); // black cursor
        glBegin(GL_LINES);
        glVertex2f(cursorX + 2, cursorY + 18);
        glVertex2f(cursorX + 2, cursorY - 3); 
        glEnd();
    }
}

void drawTwoColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::string>& column1, const std::vector<std::string>& column2) {
    // Calculate the number of rows based on the data in the columns
    size_t numRows = std::max(column1.size(), column2.size());

    // Calculate the height of the table based on the number of rows and cell height
    int tableHeight = numRows * cellHeight;

    // Set the color for cell borders and text
    glColor3f(0.0, 0.0, 0.0); // Black color for borders and text

    // Draw cell borders and content for column 1
    int currentY = y + tableHeight;
    int column1Width = width / 2;
    int column2X = x + column1Width;

    for (size_t i = 0; i < numRows; ++i) {
        // Draw horizontal cell border
        glBegin(GL_LINES);
        glVertex2f(x, currentY);
        glVertex2f(x + width, currentY);
        glEnd();

        // Draw content for column 1
        if (i < column1.size()) {
            glRasterPos2f(x + 5, currentY - cellHeight / 2); // Add padding for text
            for (char c : column1[i]) {
                glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
            }
        }

        currentY -= cellHeight;
    }

    // Draw cell borders and content for column 2
    currentY = y + tableHeight;

    for (size_t i = 0; i < numRows; ++i) {
        // Draw vertical cell border
        glBegin(GL_LINES);
        glVertex2f(column2X, currentY);
        glVertex2f(column2X, y);
        glEnd();

        // Draw content for column 2
        if (i < column2.size()) {
            glRasterPos2f(column2X + 5, currentY - cellHeight / 2); // Add padding for text
            for (char c : column2[i]) {
                glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
            }
        }

        currentY -= cellHeight;
    }
}

void setWindowIcon(const char* iconPath) {
    // Load the icon from the given file path
    HICON hIcon = (HICON)LoadImage(NULL, iconPath, IMAGE_ICON, 0, 0, LR_LOADFROMFILE | LR_DEFAULTSIZE);

    // Set the window icon
    SendMessage(GetActiveWindow(), WM_SETICON, ICON_BIG, (LPARAM)hIcon);
}

void introScreen() {
    glEnable(GL_LIGHTING); // Enable to show image becomes black
    glEnable(GL_LIGHT0); // Enable to prevent image becomes black

    float picWidth = screenWidth / 1.28f; // Width of the picture as specified.
    float picHeight = screenHeight;
    displayTexture(imgVilla, 0, 0, picWidth, picHeight);

    glDisable(GL_LIGHTING); //Disbale for other GUI elements
    glDisable(GL_LIGHT0); //Disbale for other GUI elements

    // Draw the title
    drawText("Simulation of Co-evolutionary Design Processes", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are about to start an assignment involving a simulation of co-evolutionary design processes. You will start with a building spatial design from the villa to the left. You will optimize this spatial design by adpating the spatial design, based on the structural model.", startText, 900, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please read the instructions carefully at the top of each screen. If you have any questions, please raise your hand.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("Start assignment", startText, 80, textWidth, 50, changeScreen, 1);
}



void buildingSpatialScreen() {
    drawText("Building spatial design", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("This is your building spatial design. You can rotate the model to get familiar with the design. Each space is identified by an unique number. If you are ready, you can move on by clicking the continue button.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 2);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModelScreen() {
    drawText("Structural design", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to create a structural design for the building spatial design. Each space is identified by a unique number. Subsequently, each rectangle beloninging to a space is identified by a letter. Your task is to assign a structural type to each rectangle. You can choose between:", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    GLfloat emissionColor[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // Emit the texture's color
    glMaterialfv(GL_FRONT, GL_EMISSION, emissionColor); // Apply to front face

    float picWidth = textWidth;
    float picHeight = 136;
    displayTexture(imgElements, startText, 700, picWidth, picHeight);

    GLfloat defaultEmission[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, defaultEmission);

    drawText("Once you are finished, please continue below.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("View structural design", startText, 80, textWidth, 50, changeScreen, 3);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModelFloor1Screen() {
    drawText("Structural design floor 1", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to create a structural design for the building spatial design. Each space is identified by a unique number. Please assign a structural type to each space in the table below. You can choose between 2 types:\n\n- Box structure 6 shells creating a box\n\n- Table structure4 columns with a shell below and on top", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Box structure", 1480, 600, 100, 0.0f, 0.0f, 0.0f);
    drawText("Table structure", 1670, 600, 100, 0.0f, 0.0f, 0.0f);

    drawTwoColumnTable(1450, 300, 300, 30, {"Space ID", "1", "2", "3", "4", "5"}, {"Structural element", "Box", "Table", "Box", "Table", "Table"});

    drawText("Once you are finished, please continue below.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("View structural design", startText, 80, textWidth, 50, changeScreen, 4);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModelFloor23Screen() {
    drawText("Structural design floor 2 & 3", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to create a structural design for the building spatial design. Each space is identified by a unique number. Please assign a structural type to each space in the table below. You can choose between 2 types:\n\n- Box structure 6 shells creating a box\n\n- Table structure4 columns with a shell below and on top", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Box structure", 1480, 600, 100, 0.0f, 0.0f, 0.0f);
    drawText("Table structure", 1670, 600, 100, 0.0f, 0.0f, 0.0f);

    drawTwoColumnTable(1450, 300, 300, 30, {"Space ID", "1", "2", "3", "4", "5"}, {"Structural element", "Box", "Table", "Box", "Table", "Table"});

    drawText("Once you are finished, please continue below.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("View structural design", startText, 80, textWidth, 50, changeScreen, 5);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void removeSpaceScreen() {
    drawText("Space removal", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to remove a maximum of 1 space. Please think aloud why you choose to remove this space.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    if (deletionConfirmed) {
        // Draw the message indicating the space has been deleted
        drawText("Space deleted successfully. Click 'Continue' to proceed.", startText, 400, textWidth, 0.0f, 0.0f, 0.0f);

        // Draw the "Continue" button
        drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 6);
    } else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Please enter the space ID below to remove", startText, 600, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 500, textWidth, 25, removeSpace);
    }

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to confirm, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 400, textWidth, r, g, b);
    }

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}


void splitSpaceScreen() {
    drawText("Space splitting", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to split a maximum of 1 space. Please think aloud why you choose to split this space.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    if (splittingConfirmed) {
        // Draw the message indicating the space has been deleted
        drawText("Space split successfully. Click 'Continue' to proceed.", startText, 400, textWidth, 0.0f, 0.0f, 0.0f);
        drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 7);
        }
    else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Please enter the space ID below to split", startText, 600, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 500, textWidth, 25, splitSpace);
    }

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to confirm, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 400, textWidth, r, g, b);
    }
    scalingCompleted = false;

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}


void iterationCompleteScreen() {
    // Check if scaling has been completed
    if (!scalingCompleted) {
        drawText("Iteration complete", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
        std::string iterationText = "Iteration " + std::to_string(iteration_counter);
        drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
        drawText("You just completed an iteration. The building spatial model needs to be scaled to recover initial floor area. Press the button below to scale the model.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

        drawButton("Scale model", startText, 500, textWidth, 50, [](int){
            scaleModel(); // Perform scaling
            scalingCompleted = true; // Update state
            glutPostRedisplay(); // Trigger a redraw of the screen
        }, 0);
    } else {
        // Adjust the message and button based on whether it's the final iteration
        if (iteration_counter < max_iterations) {
            // Not the final iteration yet
            std::string nextIterationText = "Start iteration " + std::to_string(iteration_counter);
            drawText("Model scaled successfully. Ready to start the next iteration.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);
            drawButton(nextIterationText.c_str(), startText, 80, textWidth, 50, [](int){ changeScreen(1); }, 0);
        } else {
            // This is the final iteration
            drawText("You have completed the final iteration. Congratulations!", 1550, 850, 250, 0.0f, 0.0f, 0.0f);
            drawButton("Conclude", startText, 80, textWidth, 50, [](int){ changeScreen(8); }, 0);
        }
    }
    deletionConfirmed = false;
    splittingConfirmed = false;
    removeSpace.text = "";
    splitSpace.text = "";

    writeToProcessFile("processLog.csv");
}


void surveyScreen() {
    drawText("1. Did you like the assignment?", 400, 800, 400, 0.0f, 0.0f, 0.0f);
    drawButton("1", 300, 750, 50, 30, buttonClicked, 1);
    drawButton("2", 350, 750, 50, 30, buttonClicked, 1);
    drawButton("3", 400, 750, 50, 30, buttonClicked, 1);
    drawButton("4", 450, 750, 50, 30, buttonClicked, 1);
    drawButton("5", 500, 750, 50, 30, buttonClicked, 1);
    drawButton("6", 550, 750, 50, 30, buttonClicked, 1);
    drawButton("7", 600, 750, 50, 30, buttonClicked, 1);
    drawButton("8", 650, 750, 50, 30, buttonClicked, 1);
    drawButton("9", 700, 750, 50, 30, buttonClicked, 1);
    drawButton("10", 750, 750, 50, 30, buttonClicked, 1);

    drawText("Please explain:", 300, 630, 200, 0.0f, 0.0f, 0.0f);
    drawTextField(300, 400, 300, 200, explanation);

    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(1400.0f, 0.0f);    // Start point of the line at the top
    glVertex2f(1400.0f, screenHeight); // End point of the line at the bottom
    glEnd();

    drawButton("Next", 1590, 50, 200, 50, buttonClicked, 1);
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(screenWidth, screenHeight);
    glutCreateWindow("Design studio: Simulation of Co-evolutionary Design Processes");
    glutFullScreen(); // Make the window fullscreen right from the start

    initializeTextures();

    // Set window icon
    setWindowIcon("TUE.ico");

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
#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>


typedef void (*ButtonCallback)(int);

struct Button {
    float x, y, width, height;
    ButtonCallback callback;
    const char* text;
    int variable;
};

struct TextField {
    std::string text;
    int cursorPosition;
    bool isActive;

    // Constructor
    TextField() : cursorPosition(0), isActive(false) {}

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
TextField spaceTF;

// Global variables for current screen and screen dimensions
int currentScreen = 0;
const int screenWidth = 1800;
const int screenHeight = 1000;

// Text margin as a percentage of the window width
const float MARGIN_PERCENT = 5.0f; // Margin as a percentage of the window width

// Function prototypes
void display();
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void mainScreen();
void assignmentDescriptionScreen();
void screen3();
void screen4();
void drawText(const char *text, float x, float y);
void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable);
void drawArrow(float x, float y, bool leftArrow);
void drawUndoRedoButtons();
void drawTextField(int x, int y, int width, int height, TextField& textfield);
void onMouseClick(int button, int state, int x, int y);
void drawBuilding();
void drawTwoColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::string>& column1, const std::vector<std::string>& column2);

void buttonClicked(int variable) {
    std::cout << "Button clicked: " << variable << std::endl;
}

void changeScreen(int screen) {
    currentScreen = screen;
    glutPostRedisplay();
    buttons.clear();
}


void display() {
    // Clear the window with white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    // Set up 2D projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, screenWidth, 0.0, screenHeight);

    // Set up modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render the current screen
    switch (currentScreen) {
        case 0: mainScreen(); break;
        case 1: assignmentDescriptionScreen(); break;
        case 2: screen3(); break;
        case 3: screen4(); break;
        // Ensure you have a default case, even if it does nothing,
        // to handle any unexpected values of currentScreen
        default: break;
    }

    // Swap buffers
    glutSwapBuffers();

    // Check for any OpenGL errors
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << gluErrorString(err) << std::endl;
    }
}

void reshape(int width, int height) {
    // Prevent a divide by zero error by making height equal to one
    if (height == 0) {
        height = 1;
    }

    float aspectRatio = static_cast<float>(width) / static_cast<float>(height);

    // Set the viewport to cover the new window size
    glViewport(0, 0, width, height);

    // Set up the projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Set up a perspective projection matrix or an orthographic one depending on your needs
    gluPerspective(45.0, aspectRatio, 0.1, 100.0);
    // For 2D GUI you may want to use an orthographic projection instead
    // gluOrtho2D(0.0, width, 0.0, height);

    // Return to the modelview matrix mode
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
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

    if(currentScreen == 2) {
        if (key >= 32 && key <= 126) { // Check if it's a printable ASCII character
            spaceTF.text += key; // Append the character to the input string
        } else if (key == 8 && spaceTF.text != "") { // Backspace key
            spaceTF.text.pop_back(); // Remove the last character from input string
        } else if (key == 13) { // Enter key
            // Print the entered text to the terminal
            std::cout << "Entered text: " << spaceTF.text << std::endl;
            spaceTF.text = ""; // Clear the input string after processing
        }
    }


    // Redraw screen
    glutPostRedisplay();
}

void drawText(const char *text, float centerX, float centerY, float textWidth) {
    float lineHeight = 18; // Approximate line height, adjust as needed
    float effectiveTextWidth = textWidth - 2 * MARGIN_PERCENT; // Effective width after considering margins

    // Calculate the starting position (left align within the margin)
    float startX = centerX - effectiveTextWidth / 2.0f;
    float currentX = startX;
    float currentY = centerY;

    for (const char *c = text; *c != '\0'; c++) {
        // Check if we need to wrap the line
        if ((currentX - startX > effectiveTextWidth) && (*c == ' ' || *c == '\n')) {
            currentY -= lineHeight;
            currentX = startX;
        }

        glRasterPos2f(currentX, currentY);

        // Set text color to black
        glColor3f(0.0, 0.0, 0.0); // black color for text

        // Draw the character
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

        // Move to the next character position
        currentX += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *c);
    }
}


void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable) {
    float borderWidth = 2.0;

    glColor3f(0.0, 0.0, 0.0); // Black color for border
    glBegin(GL_QUADS);
    glVertex2f(x - borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y - borderWidth);
    glVertex2f(x + width + borderWidth, y + height + borderWidth);
    glVertex2f(x - borderWidth, y + height + borderWidth);
    glEnd();

    // Draw button rectangle with white background
    glColor3f(1.0, 1.0, 1.0); // white color for button background
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();

    // Centered text within the button with margin
    float centerX = x + width / 2;
    float centerY = y + height / 2;
    float textWidth = width - 2 * MARGIN_PERCENT; // Text width considering margin

    // Set text color to black
    glColor3f(0.0, 0.0, 0.0);
    drawText(text, centerX, centerY, textWidth);

    Button button = {x, y, width, height, callback, text, variable};
    buttons.push_back(button);
}

void onMouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        float mouseY = screenHeight - static_cast<float>(y);
        float mouseX = static_cast<float>(x);

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
        glVertex2f(cursorX, cursorY - 18); // Adjust the Y coordinate to draw the cursor above the text
        glVertex2f(cursorX, cursorY - 3);  // Adjust the Y coordinate to draw the cursor above the text
        glEnd();
    }
}





// Function to draw an arrow inside a button
void drawArrow(float x, float y, bool leftArrow) {
    glBegin(GL_TRIANGLES);
    if (leftArrow) {
        glVertex2f(x + 10, y + 10);
        glVertex2f(x + 30, y + 25);
        glVertex2f(x + 10, y + 40);
    } else {
        glVertex2f(x + 30, y + 10);
        glVertex2f(x + 10, y + 25);
        glVertex2f(x + 30, y + 40);
    }
    glEnd();
}

// Function to draw undo and redo buttons
void drawUndoRedoButtons() {
    // Undo Button
    glColor3f(0.7, 0.7, 0.7); // Button color
    drawButton("", 10, screenHeight - 60, 50, 50, buttonClicked, 1);
    glColor3f(0, 0, 0); // Arrow color
    drawArrow(10, screenHeight - 60, false); // Left arrow for undo

    // Redo Button
    glColor3f(0.7, 0.7, 0.7); // Button color
    drawButton("", 70, screenHeight - 60, 50, 50, buttonClicked, 1);
    glColor3f(0, 0, 0); // Arrow color
    drawArrow(70, screenHeight - 60, true); // Right arrow for redo

    // Reset Button
    glColor3f(0.7, 0.7, 0.7); // Button color
    drawButton("Reset", 10, screenHeight - 120, 110, 50, buttonClicked, 1);
}

void drawBuilding() {
    glColor3f(0.0, 0.0, 0.0); // Blue color for the building structure
    glBegin(GL_LINES);

    // Base of the building
    glVertex2f(100.0f, 300.0f);
    glVertex2f(600.0f, 300.0f);

    glVertex2f(600.0f, 300.0f);
    glVertex2f(600.0f, 800.0f);

    glVertex2f(600.0f, 800.0f);
    glVertex2f(100.0f, 800.0f);

    glVertex2f(100.0f, 800.0f);
    glVertex2f(100.0f, 300.0f);

    // Top of the building
    glVertex2f(100.0f, 200.0f);
    glVertex2f(600.0f, 200.0f);

    glVertex2f(600.0f, 200.0f);
    glVertex2f(600.0f, 700.0f);

    glVertex2f(600.0f, 700.0f);
    glVertex2f(100.0f, 700.0f);

    glVertex2f(100.0f, 700.0f);
    glVertex2f(100.0f, 200.0f);

    // Vertical lines
    glVertex2f(100.0f, 200.0f);
    glVertex2f(100.0f, 300.0f);

    glVertex2f(600.0f, 200.0f);
    glVertex2f(600.0f, 300.0f);

    glVertex2f(600.0f, 700.0f);
    glVertex2f(600.0f, 800.0f);

    glVertex2f(100.0f, 700.0f);
    glVertex2f(100.0f, 800.0f);

    // Interior lines - for simplicity, just a couple are drawn here
    glVertex2f(350.0f, 200.0f);
    glVertex2f(350.0f, 700.0f);

    glVertex2f(350.0f, 250.0f);
    glVertex2f(600.0f, 250.0f);

    glEnd();
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



void mainScreen() {
    drawBuilding();

    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(1400.0f, 0.0f);
    glVertex2f(1400.0f, screenHeight);
    glEnd();

    // Draw the message at the bottom of the structure illustration
    drawText("Initial building spatial design", 1550, 950, 250);
    drawText("This is your initial building spatial design. This is the starting point of the exercise. You can rotate and change the view of the model to get familiar with the design.", 1550, 850, 250);

    // Draw the "Next step" button in the bottom right corner
    drawButton("-> | Next step", 1590, 50, 200, 50, changeScreen, 1);
}

void assignmentDescriptionScreen() {
    drawBuilding();

    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(1400.0f, 0.0f);
    glVertex2f(1400.0f, screenHeight);
    glEnd();

    // Draw the message at the bottom of the structure illustration
    drawText("Structural design", 1550, 950, 250);
    drawText("You are asked to create a structural design for the building spatial design. Each space is identified by a unique number. Please assign a structural type to each space in the table below. You can choose between 2 types:\n\n- Box structure 6 shells creating a box\n\n- Table structure4 columns with a shell below and on top", 1550, 850, 250);

    drawText("Box structure", 1480, 600, 100);
    drawText("Table structure", 1670, 600, 100);

    drawTwoColumnTable(1450, 300, 300, 30, {"Space ID", "1", "2", "3", "4", "5"}, {"Structural element", "Box", "Table", "Box", "Table", "Table"});

    drawText("Once you are finished, please continue below.", 1580, 200, 300);


    // Draw the "Next step" button in the bottom right corner
    drawButton("View structural design", 1520, 50, 200, 50, changeScreen, 2);
}

void screen3() {
    drawBuilding();

    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(1400.0f, 0.0f);
    glVertex2f(1400.0f, screenHeight);
    glEnd();

    // Draw the message at the bottom of the structure illustration
    drawText("Space removal", 1550, 950, 250);
    drawText("You are asked to remove a maximum om 2 spaces. Please remove the spaces which performs worst in terms of structural performance; i.e. strain energy.\n\nAt the top left corner, you can switch the view.", 1550, 850, 250);

    drawText("Please enter the space ID to remove", 1550, 600, 250);
    drawTextField(1480, 500, 100, 50, spaceTF);



    // Draw the "Next step" button in the bottom right corner
    drawButton("View new spatial design", 1450, 50, 300, 50, changeScreen, 3);
}

void screen4() {
    drawText("1. Did you like the assignment?", 400, 800, 400);
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

    drawText("Please explain:", 300, 630, 200);
    drawTextField(300, 400, 300, 200, spaceTF);

    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(1400.0f, 0.0f);    // Start point of the line at the top
    glVertex2f(1400.0f, screenHeight); // End point of the line at the bottom
    glEnd();

    drawButton("-> | Next", 1590, 50, 200, 50, buttonClicked, 1);
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(screenWidth, screenHeight);
    glutCreateWindow("Menu Interface");

    // Set callback functions
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutMouseFunc(onMouseClick);

    // Main loop
    glutMainLoop();
    return 0;
}

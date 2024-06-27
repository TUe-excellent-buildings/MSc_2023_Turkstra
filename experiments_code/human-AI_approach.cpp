#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>
// #include <Windows.h>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <variant>
#include <tuple>
#include <mutex>
#include <ctime>
#include <unordered_map>

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/building_physics/bp_model.hpp>
#include <bso/grammar/grammar.hpp>
#include <bso/visualization/visualization.hpp>
#include <bso/grammar/sd_grammars/design_input.cpp>
#include <bso/grammar/sd_grammars/design_human.cpp>
#include <bso/grammar/sd_grammars/design_horizontal.cpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using namespace std;

bool visualizationActive = true;
bso::visualization::viewportmanager vpmanager_local;
bso::visualization::orbitalcamera   cam_local;
int prevx, prevy;

std::vector<int> tableClicked;
bool tableInitialized = false;

typedef void (*ButtonCallback)(int);

bso::spatial_design::ms_building MS("files_SCDP/Villa");
bso::spatial_design::cf_building CF(MS, 1e-6);
bso::grammar::grammar grm(CF);
std::vector<bso::spatial_design::ms_building> msModels;
std::vector<bso::structural_design::sd_model> sdModels;
std::vector<bso::spatial_design::ms_building> msOptions;

bso::structural_design::component::structure trussStructure("truss",{{"A",2250},{"E",3e4}});
bso::structural_design::component::structure beamStructure("beam",{{"width",150},{"height",150},{"poisson",0.3},{"E",3e4}});
bso::structural_design::component::structure flatShellStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e4}});
bso::structural_design::component::structure substituteStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e-2}});

double etaBend = 0.4;
double etaAx = 0.4;
double etaShear = 0.4;
double etaNoise = 0.025;
int etaConverge = 1;
std::string checkingOrder = "321";

int nRectangles = 50;

//bso::structural_design::sd_model SD_model;
//bso::structural_design::sd_model SD_model_2;

auto cfVertices = CF.cfVertices();
auto cfLines = CF.cfLines();
auto cfRectangles = CF.cfRectangles();

/*
for rec in cfRectangles{
    rec->getVertices().getcords();
}
*/

// CREATE SD MODEL FOR INTIIAL VISUALIZATION
std::vector<bso::structural_design::component::structure> structureAssignments(nRectangles);
std::vector<bso::structural_design::component::structure> structureAssignments_it2(nRectangles);

//bso::structural_design::sd_model SD_model_initial = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);

//bso::structural_design::sd_model SD_model_sub = grm.sd_grammar<bso::grammar::DESIGN_HUMAN>(std::string("settings/sd_settings.txt"), flatShellStructure, substituteStructure);
//auto subRectangle = SD_model_sub.getSubRectangles();

//std::vector<bso::structural_design::component::structure> structureAssignments(subRectangle.size());

//size_t splitIndex = subRectangle.size() / 2;  // This finds the midpoint for splitting

struct Button {
    float x, y, width, height;
    ButtonCallback callback;
    const char* text;
    int variable;
};

struct GrammarConfig {
    double etaBend;
    double etaAxial;
    double etaShear;
    std::string checkingOrder;
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

// IQD part
// Function to calculate volume of a space
double euclideanDistance(const bso::utilities::geometry::vertex& a, const bso::utilities::geometry::vertex& b) {
    return std::sqrt((a.x() - b.x()) * (a.x() - b.x()) +
                     (a.y() - b.y()) * (a.y() - b.y()) +
                     (a.z() - b.z()) * (a.z() - b.z()));
}

double calculateSpatialDistanceDissimilarity(const bso::spatial_design::ms_building& baseBuilding, const bso::spatial_design::ms_building& variantBuilding) {
    auto baseSpaces = baseBuilding.getSpacePtrs();
    auto variantSpaces = variantBuilding.getSpacePtrs();

    double totalDistance = 0.0;
    size_t count = std::min(baseSpaces.size(), variantSpaces.size()); // Ensure equal number of spaces for comparison

    for (size_t i = 0; i < count; i++) {
        auto baseCoords = baseSpaces[i]->getCoordinates();
        auto baseDims = baseSpaces[i]->getDimensions();
        bso::utilities::geometry::vertex baseCentroid = {
            baseCoords.x() + baseDims.x() / 2,
            baseCoords.y() + baseDims.y() / 2,
            baseCoords.z() + baseDims.z() / 2
        };

        auto variantCoords = variantSpaces[i]->getCoordinates();
        auto variantDims = variantSpaces[i]->getDimensions();
        bso::utilities::geometry::vertex variantCentroid = {
            variantCoords.x() + variantDims.x() / 2,
            variantCoords.y() + variantDims.y() / 2,
            variantCoords.z() + variantDims.z() / 2
        };

        totalDistance += euclideanDistance(baseCentroid, variantCentroid);
    }

    // Normalize by the number of compared spaces
    double dissimilarity = totalDistance / count;

    return dissimilarity;
}


// Function to calculate total volume of all spaces in a building
double totalVolume(const bso::spatial_design::ms_building& building) {
    double total = 0.0;
    auto spaces = building.getSpacePtrs();
    for (const auto& space : spaces) {
        total += space->getVolume();
    }
    return total;
}

// Calculate dissimilarity based on volume differences between two buildings
double calculateVolumeDissimilarity(const bso::spatial_design::ms_building& baseBuilding, const bso::spatial_design::ms_building& variantBuilding) {
    double baseVolume = totalVolume(baseBuilding);
    double variantVolume = totalVolume(variantBuilding);

    // Calculate dissimilarity as the absolute difference normalized by the base volume
    return std::fabs(baseVolume - variantVolume) / baseVolume;
}

//bso::spatial_design::cf_building CF_base(MS_base, 1e-6);
//bso::spatial_design::cf_building CF_1(MS_1, 1e-6);
//bso::spatial_design::cf_building CF_2(MS_2, 1e-6);
//bso::spatial_design::cf_building CF_3(MS_3, 1e-6);

void printPrisms2(std::vector<bso::spatial_design::conformal::cf_vertex*> points,
                 std::ofstream& outfile, int design, int zone) {
    outfile << zone << ",";
    for(auto point : points) {
        outfile << point->x() << ",";
        outfile << point->y() << ",";
    }

    double z_base = points[0]->z();
    double z_difference = points[1]->z();

    for(auto point : points) {
        if (point->z() != z_base) {
            z_difference = point->z();
            std::cout << "Different z";
            break;
        }
    }

    outfile << z_base << "," << z_difference << std::endl;
}


void printCF(bso::spatial_design::cf_building CF, int design) {
    std::ofstream outFile("comparison" + std::to_string(design) + ".txt");


    std::cout << "CF Model" << std::endl;
    std::cout << "Spaces: " << CF.cfSpaces().size() << std::endl;
    int k = 0;
    for (auto space : CF.cfSpaces()) {
        std::cout << "Space " << k << std::endl;
        std::cout << "Cuboids: " << space->cfCuboids().size() << std::endl;
        for(auto cuboid : space->cfCuboids()) {
            std::vector<bso::spatial_design::conformal::cf_vertex*> vertices;
            // vertices.push_back(vertex);
            std::cout << "Vertices: " << cuboid->cfVertices().size() << std::endl;
            for(auto vertex : cuboid->cfVertices()) {
                vertices.push_back(vertex);
            }
            printPrisms2(vertices, outFile, design, k);
        }
        k++;
    }
}

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// Define the variant to handle different types of values
using LogValue = std::variant<std::string, double>;

struct LogEntry {
    std::string time;
    int iteration;
    std::string description;
    LogValue value;  // Now using LogValue

    // Constructor for different types of LogValue
    LogEntry(int iter, const std::string& desc, const LogValue& val)
        : iteration(iter), description(desc), value(val) {
        // Format the current time upon entry creation
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&now_c), "%H:%M:%S");
        time = oss.str();
    }
};

std::vector<LogEntry> logEntries;
std::mutex logMutex;  // For thread-safe logging

void initializeModels() {
    bso::spatial_design::ms_building MS("files_SCDP/Villa");  // Create an MS model
    msModels.push_back(MS);  // Add the MS model to the vector
}

void logAction(int iteration, const std::string& description, const LogValue& value) {
    std::lock_guard<std::mutex> lock(logMutex); // Ensure thread safety
    logEntries.emplace_back(iteration, description, value);
}

// Helper to process variant values
struct VariantVisitor {
    std::ostream& os;

    void operator()(const std::string& val) const {
        os << "\"" << val << "\"";  // Output strings with quotes
    }

    void operator()(double val) const {
        os << val;  // Output doubles directly
    }
};

void writeToProcessFile(const std::string& fileName) {
    std::lock_guard<std::mutex> lock(logMutex); // Ensure thread safety
    std::ofstream processFile(fileName, std::ios::app);
    if (!processFile.is_open()) {
        std::cerr << "Failed to open log file" << std::endl;
        return; // Optionally handle the error more robustly here
    }

    static bool headerPrinted = false;
    if (!headerPrinted) {
        processFile << "Time;Iteration;Description;Value\n";
        headerPrinted = true;
    }

    for (const auto& entry : logEntries) {
        processFile << entry.time << ";" << entry.iteration << ";"
                    << entry.description << ";";
        std::visit(VariantVisitor{processFile}, entry.value);
        processFile << "\n";
    }
    processFile.close();
    logEntries.clear();
}


std::vector<Button> buttons;

TextField removeSpace;
TextField splitSpace;
TextField chooseSD;
TextField chooseMS;
TextField explanation;
TextField survey1;
TextField survey2;
TextField survey3;
TextField survey4;
TextField survey5;
TextField survey6;
TextField email;

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
const int max_iterations = 2;

// Vector of rectangles
std::vector<bso::utilities::geometry::polygon*> rectanglesGeometry;
std::vector<int> rectanglesSpaces;
std::vector<int> rectanglesInternalIDs;

// Function prototypes
void display();
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void updateTextureCoordinates(int width, int height);
void introScreen();
void buildingSpatialScreen();
void structuralModelScreen();
void structuralModel1Screen();
void structuralModel2Screen();
void chooseSDScreen();
void chooseMSScreen();
void removeSpaceScreen();
void splitSpaceScreen();
void iteration1CompleteScreen();
void iteration2CompleteScreen();
void surveyScreen1();
void surveyScreen2();
void surveyScreen3();
void surveyScreen4();
void surveyScreen5();
void surveyScreen6();
void outroScreen();
void structureAssignment();
void createSDmodel();
void displayConfirmationRequest(int spaceID, const std::string& message, void (*action)(int));
void drawText(const char* text, float startX, float centerY, float textWidth, float r, float g, float b, bool bold = false);
void drawButton(const char *text, float x, float y, float width, float height, ButtonCallback callback, int variable);
void drawArrow(float x, float y, bool leftArrow);
void drawUndoRedoButtons();
void drawTextField(int x, int y, int width, int height, TextField& textfield);
void onMouseClick(int button, int state, int x, int y);
void drawBuilding();
void drawFourColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::string>& column1);
// void setWindowIcon(const char* iconPath);
void setup2D();
void setup3D();
void introScreen();
bool checkIfRemovePossible();
bool checkIfSplitPossible();
void splitSpaceScreen();
std::string clean_str(const std::string& input);

void visualise(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "ms_building", const double& lineWidth = 1.0)
{
    vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void visualise(const bso::spatial_design::cf_building& cf, std::string type, std::string title = "sc_building")
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::Conformal_Model(cf, type, title)));
}

void visualise_sd(const bso::structural_design::sd_model& sd, const std::string& type ="component",
							 const std::string& title ="sd_model", const bool& ghostly = true,
							 const std::vector<std::pair<bso::utilities::geometry::vertex,
							 bso::utilities::geometry::vector>>& cuttingPlanes = {})
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::SD_Model(sd, type, title, ghostly, cuttingPlanes)));
}

void visualise_sd_2(const bso::structural_design::sd_model& sd, const std::string& type ="component",
							 const std::string& title ="", const bool& ghostly = true,
							 const std::vector<std::pair<bso::utilities::geometry::vertex,
							 bso::utilities::geometry::vector>>& cuttingPlanes = {})
{
	vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::SD_Model(sd, type, title, ghostly, cuttingPlanes)));
}

void visualise_sd_add(const bso::structural_design::sd_model& sd, const std::string& type ="component",
							 const std::string& title ="", const bool& ghostly = true,
							 const std::vector<std::pair<bso::utilities::geometry::vertex,
							 bso::utilities::geometry::vector>>& cuttingPlanes = {})
{
	vpmanager_local.addviewport(new bso::visualization::viewport(new bso::visualization::SD_Model(sd, type, title, ghostly, cuttingPlanes)));
}

void visualise_ms_0(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "", const double& lineWidth = 1.0)
{
    vpmanager_local.changeviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void visualise_ms_1(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "", const double& lineWidth = 1.0)
{
    vpmanager_local.addviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void visualise_ms_2(const bso::spatial_design::ms_building& ms, const std::string& type = "spaces", const std::string& title = "", const double& lineWidth = 1.0)
{
    vpmanager_local.addviewport(new bso::visualization::viewport(new bso::visualization::MS_Model(ms, type, title,lineWidth)));
}

void checkGLError(const char* action) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cout << "OpenGL error after " << action << ": " << gluErrorString(err) << std::endl;
    }
}

//Declare a global variable to store the selected button label
std::string selectedButtonLabel = "";

// Function to get the selected button label
std::string getSelectedButtonLabel() {
    return selectedButtonLabel;
}


void buttonClicked(int variable) {
    std::cout << "Button clicked: " << variable << std::endl;

    // Set the selected button label based on the variable
    switch (variable) {
    case 1:
        selectedButtonLabel = "1";
        break;
    case 2:
        selectedButtonLabel = "2";
        break;
    case 3:
        selectedButtonLabel = "3";
        break;
    case 4:
        selectedButtonLabel = "4";
        break;
    case 5:
        selectedButtonLabel = "5";
        break;
    }
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

bso::spatial_design::ms_building MS_base = MS;

std::set<int> removedSpaceIDs;
std::set<int> splitSpaceIDs;
std::set<int> removedSpaceIDs1;
std::set<int> removedSpaceIDs2;
std::set<int> removedSpaceIDs3;

std::set<int> splitSpaceIDs1;
std::set<int> splitSpaceIDs2;
std::set<int> splitSpaceIDs3;


unsigned int initialSpaceCount = MS.getSpacePtrs().size();
double floorArea = MS.getFloorArea();
int nSpacesDelete = 1;
int chosenSDoption = 0;
double sdResult1 = 0;
double sdResult2 = 0;
double sdResult3 = 0;
double sdResult4 = 0;
int chosenMSoption = 0;
bool scalingCompleted = false;
bool splittingConfirmed = false;
bool deletionConfirmed = false;
bool inputFieldDisabled = false;
bool spaceInputError = false;
bool confirmSDmodel = false;
bool confirmMSmodel = false;
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
    imgVilla = loadImageAsTexture("files_SCDP/Villa.png");
    imgElements = loadImageAsTexture("files_SCDP/Elements.png");
    // Load more textures as needed
}

std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::map<int, std::chrono::steady_clock::time_point> screenStartTimes;
int lastScreen = -1;

void createSDmodel() {
    if (currentScreen == 3) {
        bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
        visualise_sd(SD_model);
    }
    else {
        bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
        visualise_sd(SD_model_2);
    }
}
std::vector<std::pair<int, double>> spacePerformancesTable;
std::vector<std::pair<int, double>> spacePerformances;
std::vector<std::pair<int, double>> spacePerformances_option1;
std::vector<std::pair<int, double>> spacePerformances_option2;
std::vector<std::pair<int, double>> spacePerformances_option3;
std::vector<std::pair<int, double>> SDresults;

GrammarConfig config1 = {0, 0.4, 0.2, "312"};
GrammarConfig config2 = {0.8, 0.6, 0, "231"};
GrammarConfig config3 = {1.0, 0.8, 0, "231"};



void analyzeSDmodel() {
    bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
    SD_model.analyze();
	unsigned int strainEnergy = SD_model.getTotalResults().mTotalStrainEnergy;
	unsigned int structuralVolume =SD_model.getTotalResults().mTotalStructuralVolume;

    logAction(iteration_counter, "Total strain energy", strainEnergy);
    logAction(iteration_counter, "Total structural volume", structuralVolume);

    // Get performances per space

     // Vector to hold pairs of ID and performance score

    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_model.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        // Uncomment next line if you need to adjust the performance metric by floor area
        // spacePerformance /= space->getFloorArea();

        int spaceID = space->getID(); // Get the space ID
        spacePerformances.push_back({spaceID, spacePerformance}); // Store ID and performance
    }

    std::cout << "Ranked Performances:\n";
    int rank = 1;
    for (const auto& performance : spacePerformances) {
        int id = performance.first; // First of the pair is ID now
        double score = performance.second; // Second of the pair is performance score
        std::cout << rank << ", Space ID: " << id << ", Performance: " << score << std::endl;
        std::ostringstream oss;
        oss << rank << ", Space ID: " << id << ", Performance: " << score;

        std::string ranking = oss.str(); // Converts the ostringstream to a string
        logAction(iteration_counter, ranking, score); // Assuming iteration_counter is defined elsewhere
        rank++;
    }
}

void analyzeSDmodel1() {
    bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    SD_option_1.analyze();

    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_1.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        int spaceID = space->getID(); // Get the space ID
        spacePerformances_option1.push_back({spaceID, spacePerformance}); // Store ID and performance
    }

    std::cout << "Ranked Performances:\n";
    int rank = 1;
    for (const auto& performance : spacePerformances_option1) {
        int id = performance.first; // First of the pair is ID now
        double score = performance.second; // Second of the pair is performance score
        std::cout << rank << ", Space ID: " << id << ", Performance: " << score << std::endl;
        std::ostringstream oss;
        oss << rank << ", Space ID: " << id << ", Performance: " << score;

        std::string ranking = oss.str(); // Converts the ostringstream to a string
        logAction(iteration_counter, ranking, score); // Assuming iteration_counter is defined elsewhere
        rank++;
    }
}

void analyzeSDmodel2() {
    bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    SD_option_2.analyze();

    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        int spaceID = space->getID(); // Get the space ID
        spacePerformances_option2.push_back({spaceID, spacePerformance}); // Store ID and performance
    }

    std::cout << "Ranked Performances:\n";
    int rank = 1;
    for (const auto& performance : spacePerformances_option2) {
        int id = performance.first; // First of the pair is ID now
        double score = performance.second; // Second of the pair is performance score
        std::cout << rank << ", Space ID: " << id << ", Performance: " << score << std::endl;
        std::ostringstream oss;
        oss << rank << ", Space ID: " << id << ", Performance: " << score;

        std::string ranking = oss.str(); // Converts the ostringstream to a string
        logAction(iteration_counter, ranking, score); // Assuming iteration_counter is defined elsewhere
        rank++;
    }
}

void analyzeSDmodel3() {
    bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
    SD_option_3.analyze();

    for (const auto& space : MS.getSpacePtrs()) {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_3.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        int spaceID = space->getID(); // Get the space ID
        spacePerformances_option3.push_back({spaceID, spacePerformance}); // Store ID and performance
    }

    std::cout << "Ranked Performances:\n";
    int rank = 1;
    for (const auto& performance : spacePerformances_option3) {
        int id = performance.first; // First of the pair is ID now
        double score = performance.second; // Second of the pair is performance score
        std::cout << rank << ", Space ID: " << id << ", Performance: " << score << std::endl;
        std::ostringstream oss;
        oss << rank << ", Space ID: " << id << ", Performance: " << score;

        std::string ranking = oss.str(); // Converts the ostringstream to a string
        logAction(iteration_counter, ranking, score); // Assuming iteration_counter is defined elsewhere
        rank++;
    }
}
double dissimilarity_model;
double dissimilarity_1;
double dissimilarity_2;
double dissimilarity_3;
double relativeDissimilarity1;
double relativeDissimilarity2;
double relativeDissimilarity3;
double baseValue;

std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;

void createMSoptions() {
    bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    SD_option_2.analyze();
    //spacePerformancesMS.clear();
    // create MS model OPTION 1 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_1 = MS_base;
    unsigned int nSpaces = MS_1.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	double floorArea = MS_1.getFloorArea();
    std::cout << floorArea << std::endl;
    std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_1.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
		spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.rbegin(), spacePerformancesMS.rend());
    std::cout << "sort" << std::endl;
    // DELETE n WORST PERFORMING SPACES
	//for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformances.size(); ++j)
    for (unsigned int j = spacePerformancesMS.size() - 1; j >= spacePerformancesMS.size() - nSpacesDelete && j < spacePerformancesMS.size(); --j)
	{
		MS_1.deleteSpace(*(spacePerformancesMS[j].second));
        removedSpaceIDs1.insert((spacePerformancesMS[j].second)->getID());
	}
    MS_1.setZZero();
    std::cout << "Deleted spaces" << std::endl;
    //msDesignsTemp.push_back(MS_1);

    // EXCLUDE ALREADY SPLIT SPACES
    auto removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());
    // SPLIT n BEST PERFORMING SPACES
    //for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesDelete && j < spacePerformances.size(); --j)
    for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformancesMS.size(); ++j)
    {
        auto spaceWithLowScore = spacePerformancesMS[j].second;
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
        MS_1.splitSpace(spaceWithLowScore, {{largestDimensionIndex, 2}});
        splitSpaceIDs1.insert(spaceWithLowScore->getID());
    }
	MS_1.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_1.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_1);

    dissimilarity_1 = calculateSpatialDistanceDissimilarity(MS_base, MS_1);

    // SCALE TO RECOVER INITIAL FLOOR AREA
	double scaleFactor = sqrt(floorArea / MS_1.getFloorArea());
    std::cout << scaleFactor  << std::endl;
	MS_1.scale({{0,scaleFactor},{1,scaleFactor}});
	MS_1.snapOn({{0,1},{1,1}});

    // create MS model OPTION 2 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_2 = MS_base;
    nSpaces = MS_2.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	floorArea = MS_2.getFloorArea();
    std::cout << floorArea << std::endl;
    //std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_2.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
		spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.begin(), spacePerformancesMS.end());
    std::cout << "sort" << std::endl;
    // DELETE n WORST PERFORMING SPACES
	//for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformances.size(); ++j)
    for (unsigned int j = spacePerformancesMS.size() - 1; j >= spacePerformancesMS.size() - nSpacesDelete && j < spacePerformancesMS.size(); --j)
	{
		MS_2.deleteSpace(*(spacePerformancesMS[j].second));
        removedSpaceIDs2.insert((spacePerformancesMS[j].second)->getID());
	}
    MS_2.setZZero();
    std::cout << "Deleted spaces" << std::endl;
    //msDesignsTemp.push_back(MS_1);

    // EXCLUDE ALREADY SPLIT SPACES
    removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());
    // SPLIT n BEST PERFORMING SPACES
    //for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesDelete && j < spacePerformances.size(); --j)
    for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformancesMS.size(); ++j)
    {
        auto spaceWithLowScore = spacePerformancesMS[j].second;
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
        MS_2.splitSpace(spaceWithLowScore, {{largestDimensionIndex, 2}});
        splitSpaceIDs2.insert(spaceWithLowScore->getID());
    }
	MS_2.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_2.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_2);

    dissimilarity_2 = calculateSpatialDistanceDissimilarity(MS_base, MS_2);

    // SCALE TO RECOVER INITIAL FLOOR AREA
	scaleFactor = sqrt(floorArea / MS_2.getFloorArea());
    std::cout << scaleFactor  << std::endl;
	MS_2.scale({{0,scaleFactor},{1,scaleFactor}});
	MS_2.snapOn({{0,1},{1,1}});

    // create MS model OPTION 3 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_3 = MS_base;
    nSpaces = MS_3.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	floorArea = MS_3.getFloorArea();
    std::cout << floorArea << std::endl;
    //std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_3.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.rbegin(), spacePerformancesMS.rend());
    std::cout << "sort" << std::endl;

    // Deleting the 6th worst performing space
    if (spacePerformancesMS.size() > 5) {
        MS_3.deleteSpace(*(spacePerformancesMS[5].second));
        removedSpaceIDs3.insert((spacePerformancesMS[5].second)->getID());
        spacePerformancesMS.erase(spacePerformancesMS.begin() + 5);
    }

    MS_3.setZZero();
    std::cout << "Deleted spaces" << std::endl;

    // Excluding already split spaces
    removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());

    // Splitting the 4th best performing space
    if (spacePerformancesMS.size() > 3) {
        auto spaceToSplit = spacePerformancesMS[3].second;
        double largestDimension = -1.0;
        unsigned int largestDimensionIndex = 0;
        for (unsigned int k = 0; k < 3; ++k) {
            double dimension = spaceToSplit->getDimensions()(k);
            if (dimension > largestDimension) {
                largestDimension = dimension;
                largestDimensionIndex = k;
            }
        }
        MS_3.splitSpace(spaceToSplit, {{largestDimensionIndex, 2}});
        splitSpaceIDs3.insert(spaceToSplit->getID());
    }
	MS_3.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_3.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_3);

    dissimilarity_3 = calculateSpatialDistanceDissimilarity(MS_base, MS_3);

    scaleFactor = sqrt(floorArea / MS_3.getFloorArea());
    std::cout << scaleFactor  << std::endl;
    MS_3.scale({{0,scaleFactor},{1,scaleFactor}});
    MS_3.snapOn({{0,1},{1,1}});


    baseValue = dissimilarity_model;
    relativeDissimilarity1 = (dissimilarity_1 / baseValue) * 100;
    relativeDissimilarity2 = (dissimilarity_2 / baseValue) * 100;
    relativeDissimilarity3 = (dissimilarity_3 / baseValue) * 100;
    logAction(iteration_counter, "Own building spatial design", 100);
    logAction(iteration_counter, "AI generated design 1", relativeDissimilarity1);
    logAction(iteration_counter, "AI generated design 2", relativeDissimilarity2);
    logAction(iteration_counter, "AI generated design 3", relativeDissimilarity3);

    std::cout << "Relative Dissimilarity 1: " << relativeDissimilarity1 << "%" << std::endl;
    std::cout << "Relative Dissimilarity 2: " << relativeDissimilarity2 << "%" << std::endl;
    std::cout << "Relative Dissimilarity 3: " << relativeDissimilarity3 << "%" << std::endl;
}

void createMSoptions2() {
    bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
    SD_option_2.analyze();
    nSpacesDelete = 2;
    //spacePerformancesMS.clear();
    // create MS model OPTION 1 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_1 = MS_base;
    unsigned int nSpaces = MS_1.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	double floorArea = MS_1.getFloorArea();
    std::cout << floorArea << std::endl;
    std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_1.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
		spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.rbegin(), spacePerformancesMS.rend());
    std::cout << "sort" << std::endl;
    // DELETE n WORST PERFORMING SPACES
	//for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformances.size(); ++j)
    for (unsigned int j = spacePerformancesMS.size() - 1; j >= spacePerformancesMS.size() - nSpacesDelete && j < spacePerformancesMS.size(); --j)
	{
		MS_1.deleteSpace(*(spacePerformancesMS[j].second));
        removedSpaceIDs1.insert((spacePerformancesMS[j].second)->getID());
	}
    MS_1.setZZero();
    std::cout << "Deleted spaces" << std::endl;
    //msDesignsTemp.push_back(MS_1);

    // EXCLUDE ALREADY SPLIT SPACES
    auto removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());
    // SPLIT n BEST PERFORMING SPACES
    //for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesDelete && j < spacePerformances.size(); --j)
    for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformancesMS.size(); ++j)
    {
        auto spaceWithLowScore = spacePerformancesMS[j].second;
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
        MS_1.splitSpace(spaceWithLowScore, {{largestDimensionIndex, 2}});
        splitSpaceIDs1.insert(spaceWithLowScore->getID());
    }
	MS_1.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_1.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_1);

    dissimilarity_1 = calculateSpatialDistanceDissimilarity(MS_base, MS_1);

    // SCALE TO RECOVER INITIAL FLOOR AREA
	double scaleFactor = sqrt(floorArea / MS_1.getFloorArea());
    std::cout << scaleFactor  << std::endl;
	MS_1.scale({{0,scaleFactor},{1,scaleFactor}});
	MS_1.snapOn({{0,1},{1,1}});

    // create MS model OPTION 2 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_2 = MS_base;
    nSpaces = MS_2.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	floorArea = MS_2.getFloorArea();
    std::cout << floorArea << std::endl;
    //std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_2.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
		spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.begin(), spacePerformancesMS.end());
    std::cout << "sort" << std::endl;
    // DELETE n WORST PERFORMING SPACES
	//for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformances.size(); ++j)
    for (unsigned int j = spacePerformancesMS.size() - 1; j >= spacePerformancesMS.size() - nSpacesDelete && j < spacePerformancesMS.size(); --j)
	{
		MS_2.deleteSpace(*(spacePerformancesMS[j].second));
        removedSpaceIDs2.insert((spacePerformancesMS[j].second)->getID());
	}
    MS_2.setZZero();
    std::cout << "Deleted spaces" << std::endl;
    //msDesignsTemp.push_back(MS_1);

    // EXCLUDE ALREADY SPLIT SPACES
    removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());
    // SPLIT n BEST PERFORMING SPACES
    //for (unsigned int j = spacePerformances.size() - 1; j >= spacePerformances.size() - nSpacesDelete && j < spacePerformances.size(); --j)
    for (unsigned int j = 0; j < nSpacesDelete && j < spacePerformancesMS.size(); ++j)
    {
        auto spaceWithLowScore = spacePerformancesMS[j].second;
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
        MS_2.splitSpace(spaceWithLowScore, {{largestDimensionIndex, 2}});
        splitSpaceIDs2.insert(spaceWithLowScore->getID());
    }
	MS_2.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_2.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_2);

    dissimilarity_2 = calculateSpatialDistanceDissimilarity(MS_base, MS_2);

    // SCALE TO RECOVER INITIAL FLOOR AREA
	scaleFactor = sqrt(floorArea / MS_2.getFloorArea());
    std::cout << scaleFactor  << std::endl;
	MS_2.scale({{0,scaleFactor},{1,scaleFactor}});
	MS_2.snapOn({{0,1},{1,1}});

    // create MS model OPTION 3 ------------------------------------------------------------------------------------------------------
    bso::spatial_design::ms_building MS_3 = MS_base;
    nSpaces = MS_3.getSpacePtrs().size();
    std::cout << nSpaces << std::endl;
	floorArea = MS_3.getFloorArea();
    std::cout << floorArea << std::endl;
    //std::vector<std::pair<double, bso::spatial_design::ms_space*>> spacePerformancesMS;
    for (const auto& space : MS_3.getSpacePtrs())
    {
        auto spaceGeom = space->getGeometry();
        double spacePerformance = SD_option_2.getPartialResults(&spaceGeom).mTotalStrainEnergy;
        spacePerformance /= space->getFloorArea();
        spacePerformancesMS.push_back({spacePerformance, space});
    }
    std::sort(spacePerformancesMS.rbegin(), spacePerformancesMS.rend()); // Sort from best to worst
    std::cout << "sort" << std::endl;

    // Deleting the 6th and 7th worst performing spaces
    if (spacePerformancesMS.size() > 6) {
        for (int i = 5; i < 7; i++) { // 6th and 7th worst are at index 5 and 6 respectively
            MS_3.deleteSpace(*(spacePerformancesMS[i].second));
            removedSpaceIDs3.insert((spacePerformancesMS[i].second)->getID());
        }
        spacePerformancesMS.erase(spacePerformancesMS.begin() + 5, spacePerformancesMS.begin() + 7); // Erase both spaces
    }

    MS_3.setZZero();
    std::cout << "Deleted spaces" << std::endl;

    // Excluding already split spaces
    removeSpacesIterator = std::remove_if(spacePerformancesMS.begin(), spacePerformancesMS.end(),
        [nSpaces](const auto& spacePerformance) {
            return spacePerformance.second->getID() > nSpaces;
        }
    );
    spacePerformancesMS.erase(removeSpacesIterator, spacePerformancesMS.end());

    // Splitting the 3rd and 4th best performing spaces
    for (int i = 2; i < 4; i++) { // 3rd and 4th best are at index 2 and 3 respectively
        if (i < spacePerformancesMS.size()) {
            auto spaceToSplit = spacePerformancesMS[i].second;
            double largestDimension = -1.0;
            unsigned int largestDimensionIndex = 0;
            for (unsigned int k = 0; k < 3; ++k) {
                double dimension = spaceToSplit->getDimensions()(k);
                if (dimension > largestDimension) {
                    largestDimension = dimension;
                    largestDimensionIndex = k;
                }
            }
            MS_3.splitSpace(spaceToSplit, {{largestDimensionIndex, 2}});
            splitSpaceIDs3.insert(spaceToSplit->getID());
        }
    }
    std::cout << "Splitted spaces" << std::endl;

	MS_3.snapOn({{0,100},{1,100},{2,100}});
    std::cout << "Splitted spaces" << std::endl;
	// IN CASE THE LOWER SPACES WERE REMOVED RESET MINIMUM Z COORDINATE TO Z
	MS_3.setZZero();
    spacePerformancesMS.clear();
    msOptions.push_back(MS_3);

    dissimilarity_3 = calculateSpatialDistanceDissimilarity(MS_base, MS_3);

    scaleFactor = sqrt(floorArea / MS_3.getFloorArea());
    std::cout << scaleFactor  << std::endl;
    MS_3.scale({{0,scaleFactor},{1,scaleFactor}});
    MS_3.snapOn({{0,1},{1,1}});


    baseValue = dissimilarity_model;
    relativeDissimilarity1 = (dissimilarity_1 / baseValue) * 100;
    relativeDissimilarity2 = (dissimilarity_2 / baseValue) * 100;
    relativeDissimilarity3 = (dissimilarity_3 / baseValue) * 100;

    std::cout << "Relative Dissimilarity 1: " << relativeDissimilarity1 << "%" << std::endl;
    std::cout << "Relative Dissimilarity 2: " << relativeDissimilarity2 << "%" << std::endl;
    std::cout << "Relative Dissimilarity 3: " << relativeDissimilarity3 << "%" << std::endl;
}

void chooseMSoptions() {
    if (chosenMSoption == 1) {
        MS = MS;
        msModels.push_back(MS);
    } else if (chosenMSoption == 2) {
        MS = msOptions[0];
        msModels.push_back(MS);
        splitSpaceIDs = splitSpaceIDs1;
        removedSpaceIDs = removedSpaceIDs1;
    } else if (chosenMSoption == 3) {
        MS = msOptions[1];
        msModels.push_back(MS);
        splitSpaceIDs = splitSpaceIDs2;
        removedSpaceIDs = removedSpaceIDs2;
    } else if (chosenMSoption == 4) {
        MS = msOptions[2];
        msModels.push_back(MS);
        splitSpaceIDs = splitSpaceIDs3;
        removedSpaceIDs = removedSpaceIDs3;
    }
}

void changeScreen(int screen) {
    if (lastScreen != -1) {
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - screenStartTimes[lastScreen]).count();
        std::cout << "Time spent on screen " << lastScreen << ": " << duration << " seconds." << std::endl;

        // Log the time spent on the last screen
        logAction(iteration_counter, "Time Spent", duration);  // Assuming duration is acceptable as LogValue
    }

    // Record the start time for the new screen
    screenStartTimes[screen] = std::chrono::steady_clock::now();
    lastScreen = screen;  // Update lastScreen to the current one
    selectedButtonLabel = "";
    // Log entering the new screen
    logAction(iteration_counter, "Entered Screen", screen);

    inputFieldDisabled = false;
    currentScreen = screen;
    std::cout << "Screen changed to: Screen " << screen << std::endl;

    if(screen  <= 2) {
        vpmanager_local.clearviewports();
        visualise(MS);
        visualizationActive = true;
    } else if(screen == 3) {
        vpmanager_local.clearviewports();
        //bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
        std::vector<bso::structural_design::component::structure> structureAssignments(nRectangles);
        bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
        visualise_sd(SD_model);
    } else if(screen == 4) {
        vpmanager_local.clearviewports();
        //bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
        std::vector<bso::structural_design::component::structure> structureAssignments_it2(nRectangles);
        bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
        visualise_sd(SD_model_2);
    } else if (screen == 5) {
        vpmanager_local.clearviewports();
        if (iteration_counter == 1) {
            bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
            SD_model.analyze();
            sdResult1 = std::round(SD_model.getTotalResults().mTotalStrainEnergy);
            visualise_sd_2(SD_model);
        } else {
            bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
            SD_model_2.analyze();
            sdResult1 = std::round(SD_model_2.getTotalResults().mTotalStrainEnergy);
            visualise_sd_2(SD_model_2);
        }
        bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
        bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
        bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);

        SD_option_1.analyze();
        sdResult2 = std::round(SD_option_1.getTotalResults().mTotalStrainEnergy);
        double volume2 = std::round(SD_option_1.getTotalResults().mTotalStructuralVolume);
        logAction(iteration_counter, "AI structural model 1: strain energy", sdResult2);
        logAction(iteration_counter, "AI structural model 1: structural volume", volume2);

        SD_option_2.analyze();
        sdResult3 = std::round(SD_option_2.getTotalResults().mTotalStrainEnergy);
        double volume3 = std::round(SD_option_2.getTotalResults().mTotalStructuralVolume);
        logAction(iteration_counter, "AI structural model 2: strain energy", sdResult3);
        logAction(iteration_counter, "AI structural model 2: structural volume", volume3);

        SD_option_3.analyze();
        sdResult4 = std::round(SD_option_3.getTotalResults().mTotalStrainEnergy);
        double volume4 = std::round(SD_option_3.getTotalResults().mTotalStructuralVolume);
        logAction(iteration_counter, "AI structural model 3: strain energy", sdResult4);
        logAction(iteration_counter, "AI structural model 3: structural volume", volume4);

        visualise_sd_add(SD_option_1);
        visualise_sd_add(SD_option_2);
        visualise_sd_add(SD_option_3);
    }else if(screen > 5 && screen <= 7) {
        vpmanager_local.clearviewports();
        visualise(MS);
        if (iteration_counter == 1) {
            if (chosenSDoption == 1) {
                bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
                visualise_sd_add(SD_model);
                sdModels.push_back(SD_model);
                //createMSoptions();
            }
            else if (chosenSDoption == 2) {
                bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_1);
                sdModels.push_back(SD_option_1);
                //createMSoptions();
            }
            else if (chosenSDoption == 3) {
                bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_2);
                sdModels.push_back(SD_option_2);
                //createMSoptions();
            }
            else if (chosenSDoption == 4) {
                bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_3);
                sdModels.push_back(SD_option_3);
                //createMSoptions();
            }
            //sdModels.push_back(SD_model);
        } else {
            if (chosenSDoption == 1) {
                bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
                visualise_sd_add(SD_model_2);
                sdModels.push_back(SD_model_2);
            }
            else if (chosenSDoption == 2) {
                bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_1);
                sdModels.push_back(SD_option_1);
            }
            else if (chosenSDoption == 3) {
                bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_2);
                sdModels.push_back(SD_option_2);
            }
            else if (chosenSDoption == 4) {
                bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
                visualise_sd_add(SD_option_3);
                sdModels.push_back(SD_option_3);
            }
        }
        visualizationActive = true;
    } else if (screen == 8) {
        vpmanager_local.clearviewports();
        if (iteration_counter == 1) {
            vpmanager_local.clearviewports();
            createMSoptions();
            visualise_ms_0(MS);
            visualise_ms_2(msOptions[0]);
            visualise_ms_2(msOptions[1]);
            visualise_ms_2(msOptions[2]);
        } else {
            vpmanager_local.clearviewports();
            createMSoptions2();
            visualise_ms_0(MS);
            visualise_ms_2(msOptions[3]);
            visualise_ms_2(msOptions[4]);
            visualise_ms_2(msOptions[5]);
        }
    } else if(screen == 9) {
        vpmanager_local.clearviewports();
        chooseMSoptions();
        visualise_ms_0(msModels[0]);
        visualise_sd_add(sdModels[0]);
        visualise_ms_1(msModels[1]);
        visualise_ms_1(msModels[2]);
        visualizationActive = true;
    } else if(screen == 10) {
        vpmanager_local.clearviewports();
        chooseMSoptions();
        visualise_ms_0(msModels[0]);
        visualise_sd_add(sdModels[0]);
        visualise_ms_1(msModels[2]);
        visualise_ms_1(msModels[3]);
        visualise_sd_add(sdModels[1]);
        visualise_ms_1(msModels.back());

        visualizationActive = true;
    } else {
        vpmanager_local.clearviewports();
        visualizationActive = false;
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

bool chooseSDmodel(const std::string& input, int& outNumber, std::string& errorMessage) {
    try {
        outNumber = std::stoi(input);
    } catch (...) {
        errorMessage = "Input is not a valid number.";
        return false;
    }

    if (outNumber <= 0 || outNumber > 4) {
        errorMessage = "Option is out of range.";
        return false;
    }

    return true;
}

bool chooseMSmodel(const std::string& input, int& outNumber, std::string& errorMessage) {
    try {
        outNumber = std::stoi(input);
    } catch (...) {
        errorMessage = "Input is not a valid number.";
        return false;
    }

    if (outNumber <= 0 || outNumber > 4) {
        errorMessage = "Option is out of range.";
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

    logAction(iteration_counter, "Space Removed", spaceID);

    // Reset state as necessary
    awaitingConfirmation = false;
    spaceInputError = false;
    spaceInputErrorMessage = "";
    deletionConfirmed = true;
    inputFieldDisabled = true;

    // Refresh the visualization with the updated MS
    /*
    CF = bso::spatial_design::conformal::cf_building_model(MS);
    grm.~grammar(); // Call the destructor manually
    new (&grm) bso::grammar::grammar(CF);
    */
    visualise(MS);

    if (iteration_counter == 1) {
        if (chosenSDoption == 1) {
            bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
            visualise_sd_add(SD_model);
            //createMSoptions();
        }
        else if (chosenSDoption == 2) {
            bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_1);
            //createMSoptions();
        }
        else if (chosenSDoption == 3) {
            bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_2);
            //createMSoptions();
        }
        else if (chosenSDoption == 4) {
            bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_3);
            //createMSoptions();
        }
        //sdModels.push_back(SD_model);
    } else {
        if (chosenSDoption == 1) {
            bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
            visualise_sd_add(SD_model_2);
        }
        else if (chosenSDoption == 2) {
            bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_1);
        }
        else if (chosenSDoption == 3) {
            bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_2);
        }
        else if (chosenSDoption == 4) {
            bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_3);
        }
    }

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

void scaleModel() {
    // SCALE TO RECOVER INITIAL FLOOR AREA
    std::cout << "Start scaling" << std::endl;
    double scaleFactor = sqrt(floorArea / MS.getFloorArea());
    MS.scale({{0,scaleFactor},{1,scaleFactor}});
    MS.snapOn({{0,1},{1,1}});
    // Update the visualization to reflect the changes

    logAction(iteration_counter, "Scale Factor", scaleFactor);
    std::cout << scaleFactor << std::endl;
    std::cout << "Finished scaling" << std::endl;
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
        /*
        visualise(MS);
        if (iteration_counter == 1) {
            bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
            visualise_sd_add(SD_model);
        } else {
            bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
            visualise_sd_add(SD_model_2);
        }
        */
        splittingConfirmed = true;

    } else {
        std::cout << "Failed to find space for ID: " << spaceID << std::endl;
        spaceInputError = true;
        spaceInputErrorMessage = "Failed to find space for splitting.";
    }

    double dissimilarity = calculateVolumeDissimilarity(MS_base, MS);
    std::cout << "Volume Dissimilarity: " << dissimilarity << std::endl;

    logAction(iteration_counter, "Space Split", spaceID);

    // Reset the UI and state flags regardless of outcome
    awaitingConfirmation = false;
    spaceInputError = false;
    spaceInputErrorMessage = "";
    inputFieldDisabled = true;

    scaleModel();
    msModels.push_back(MS);
    visualise(MS);
    dissimilarity_model = calculateSpatialDistanceDissimilarity(MS_base, MS);

    if (iteration_counter == 1) {
        if (chosenSDoption == 1) {
            bso::structural_design::sd_model SD_model = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments);
            visualise_sd_add(SD_model);
            //createMSoptions();
        }
        else if (chosenSDoption == 2) {
            bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_1);
            //createMSoptions();
        }
        else if (chosenSDoption == 3) {
            bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_2);
            //createMSoptions();
        }
        else if (chosenSDoption == 4) {
            bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_3);
            //createMSoptions();
        }
        //sdModels.push_back(SD_model);
    } else {
        if (chosenSDoption == 1) {
            bso::structural_design::sd_model SD_model_2 = grm.sd_grammar<bso::grammar::DESIGN_INPUT>(std::string("files_SCDP/settings/sd_settings.txt"), etaBend, etaAx, etaShear, etaNoise, etaConverge, checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure, structureAssignments_it2);
            visualise_sd_add(SD_model_2);
        }
        else if (chosenSDoption == 2) {
            bso::structural_design::sd_model SD_option_1 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config1.etaBend, config1.etaAxial, config1.etaShear, 0.025, 1, config1.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_1);
        }
        else if (chosenSDoption == 3) {
            bso::structural_design::sd_model SD_option_2 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config2.etaBend, config2.etaAxial, config2.etaShear, 0.025, 1, config2.checkingOrder, trussStructure, beamStructure, flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_2);
        }
        else if (chosenSDoption == 4) {
            bso::structural_design::sd_model SD_option_3 = grm.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("files_SCDP/settings/sd_settings.txt"), config3.etaBend, config3.etaAxial, config3.etaShear, 0.025, 1, config3.checkingOrder, beamStructure, trussStructure,  flatShellStructure, substituteStructure);
            visualise_sd_add(SD_option_3);
        }
    }

    visualizationActive = true;

    glutPostRedisplay();
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
        case 3: structuralModel1Screen(); break;
        case 4: structuralModel2Screen(); break;
        case 5: chooseSDScreen(); break;
        case 6: removeSpaceScreen(); break;
        case 7: splitSpaceScreen(); break;
        case 8: chooseMSScreen(); break;
        case 9: iteration1CompleteScreen(); break;
        case 10: iteration2CompleteScreen(); break;
        case 11: surveyScreen1(); break;
        case 12: surveyScreen2(); break;
        case 13: surveyScreen3(); break;
        case 14: surveyScreen4(); break;
        case 15: surveyScreen5(); break;
        case 16: surveyScreen6(); break;
        case 17: outroScreen(); break;
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

bool allRowsClicked(const std::vector<int>& tableClicked) {
    for (int clicked : tableClicked) {
        if (clicked == 0) {  // Assuming 0 means no selection
            return false;  // Found a row without a selection
        }
    }
    return true;  // All rows have a selection
}

void keyboard(unsigned char key, int x, int y) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << gluErrorString(err) << std::endl;
    }
    if (currentScreen == 5) {
        if (awaitingConfirmation) {
            if (key == 'y' || key == 'Y') {
                chosenSDoption = std::stoi(chooseSD.text);
                confirmSDmodel = true;
                logAction(iteration_counter, "Chosen SD model", chosenSDoption);
                if (chosenSDoption == 1) {
                    analyzeSDmodel();
                    spacePerformancesTable = spacePerformances;
                } else if (chosenSDoption == 2) {
                    analyzeSDmodel1();
                    spacePerformancesTable = spacePerformances_option1;
                } else if (chosenSDoption == 3) {
                    analyzeSDmodel2();
                    spacePerformancesTable = spacePerformances_option2;
                } else if (chosenSDoption == 4) {
                    analyzeSDmodel3();
                    spacePerformancesTable = spacePerformances_option3;
                }
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                inputFieldDisabled = true;
            } else if (key == 'n' || key == 'N') {
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                chooseSD.text = "";
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
            if (chooseSDmodel(chooseSD.text, spaceID, errorMessage)) { // Validate input
                // If validation succeeds, set up for confirmation
                awaitingConfirmation = true;
                // Call displayConfirmationRequest
                displayConfirmationRequest(spaceID, "Press Y to submit, N to cancel.", removeSpaceConfirmed);
            } else {
                // Handle invalid input based on the specific error message returned by validation
                chooseSD.text = "";
                spaceInputError = true;
                spaceInputErrorMessage = errorMessage;
                glutPostRedisplay();
            }
            return;
        }
        else if (key >= 32 && key <= 126) { // Check if it's a printable ASCII character
            chooseSD.text += key; // Append the character to the input string
        } else if (key == 8 && chooseSD.text != "") { // Backspace key
            chooseSD.text.pop_back(); // Remove the last character from input string
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
                displayConfirmationRequest(spaceID, "Press Y to submit, N to cancel.", removeSpaceConfirmed);
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

    if (currentScreen == 7) {
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
                displayConfirmationRequest(spaceID, "Press Y to submit, N to cancel.", splitSpaceConfirmed);
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

    if (currentScreen == 8) {
        if (awaitingConfirmation) {
            if (key == 'y' || key == 'Y') {
                chosenMSoption = std::stoi(chooseMS.text);
                confirmMSmodel = true;
                logAction(iteration_counter, "Chosen MS model", chosenMSoption);
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                inputFieldDisabled = true;
            } else if (key == 'n' || key == 'N') {
                awaitingConfirmation = false;
                spaceInputError = false;
                spaceInputErrorMessage = "";
                chooseMS.text = "";
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
            if (chooseSDmodel(chooseMS.text, spaceID, errorMessage)) { // Validate input
                // If validation succeeds, set up for confirmation
                awaitingConfirmation = true;
                // Call displayConfirmationRequest
                displayConfirmationRequest(spaceID, "Press Y to submit, N to cancel.", removeSpaceConfirmed);
            } else {
                // Handle invalid input based on the specific error message returned by validation
                chooseMS.text = "";
                spaceInputError = true;
                spaceInputErrorMessage = errorMessage;
                glutPostRedisplay();
            }
            return;
        }
        else if (key >= 32 && key <= 126) { // Check if it's a printable ASCII character
            chooseMS.text += key; // Append the character to the input string
        } else if (key == 8 && chooseMS.text != "") { // Backspace key
            chooseMS.text.pop_back(); // Remove the last character from input string
        }
    }
    if (currentScreen == 11){
        if (key >= 32 && key <= 126) {
            survey1.text += key;
        } else if (key == 8 && survey1.text != "") {
            survey1.text.pop_back();
        }
    }
    if (currentScreen == 12){
        if (key >= 32 && key <= 126) {
            survey2.text += key;
        } else if (key == 8 && survey2.text != "") {
            survey2.text.pop_back();
        }
    }
    if (currentScreen == 13){
        if (key >= 32 && key <= 126) {
            survey3.text += key;
        } else if (key == 8 && survey3.text != "") {
            survey3.text.pop_back();
        }
    }
    if (currentScreen == 14){
        if (key >= 32 && key <= 126) {
            survey4.text += key;
        } else if (key == 8 && survey4.text != "") {
            survey4.text.pop_back();
        }
    }
    if (currentScreen == 15){
        if (key >= 32 && key <= 126) {
            survey5.text += key;
        } else if (key == 8 && survey5.text != "") {
            survey5.text.pop_back();
        }
    }
    if (currentScreen == 16){
        if (key >= 32 && key <= 126) {
            survey6.text += key;
        } else if (key == 8 && survey6.text != "") {
            survey6.text.pop_back();
        }
    }
    if (currentScreen == 17){
        if (key >= 32 && key <= 126) {
            email.text += key;
        } else if (key == 8 && email.text != "") {
            email.text.pop_back();
        }
    }
    glutPostRedisplay();
}

void handleSpecialKeypress(int key, int x, int y) {
    if (key == GLUT_KEY_F11) {  // Check if the key is F11
        static bool isFullScreen = false;
        isFullScreen = !isFullScreen;
        if (isFullScreen) {
            glutFullScreen(); // Go full screen
        } else {
            glutReshapeWindow(screenWidth, screenHeight); // Return to window mode with initial dimensions
            glutPositionWindow(100, 100); // Optionally reposition window
        }
    }
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


    if (getSelectedButtonLabel() == text) {
        glColor4f(0.784, 0.098, 0.098, 0.8); // Light grey color for button background
    }
    else {
        glColor3f(1.0, 1.0, 1.0); // White color for button background
    }

    // Draw button background
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

void handleCellClick(int clickedRow, int clickedColumn) {
    if (currentScreen == 3) {
        if (clickedColumn == 1) {
            structureAssignments[clickedRow] = beamStructure;
            std::cout << "Truss structure added to rectangle " << clickedRow << std::endl;
        }
        else if (clickedColumn == 2) {
            structureAssignments[clickedRow] = trussStructure;
            std::cout << "Beam structure added to rectangle " << clickedRow << std::endl;
        }
        else if (clickedColumn == 3) {
            structureAssignments[clickedRow] = flatShellStructure;
            std::cout << "Flat shell structure added to rectangle " << clickedRow << std::endl;
        }
        if(clickedColumn != 0) {
            tableClicked[clickedRow] = clickedColumn;
        }
    }

    if (currentScreen == 4) {
        if (clickedColumn == 1) {
            structureAssignments_it2[clickedRow] = beamStructure;
            std::cout << "Truss structure added to rectangle " << clickedRow << std::endl;
        }
        else if (clickedColumn == 2) {
            structureAssignments_it2[clickedRow] = trussStructure;
            std::cout << "Beam structure added to rectangle " << clickedRow << std::endl;
        }
        else if (clickedColumn == 3) {
            structureAssignments_it2[clickedRow] = flatShellStructure;
            std::cout << "Flat shell structure added to rectangle " << clickedRow << std::endl;
        }
        if(clickedColumn != 0) {
            tableClicked[clickedRow] = clickedColumn;
        }
    }


    createSDmodel();
}


void onMouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        float mouseY = screenHeight - static_cast<float>(y);
        float mouseX = static_cast<float>(x);

        // Check each text field individually
        checkTextFieldClick(removeSpace, mouseX, mouseY);
        checkTextFieldClick(splitSpace, mouseX, mouseY);
        checkTextFieldClick(chooseSD, mouseX, mouseY);
        checkTextFieldClick(chooseMS, mouseX, mouseY);
        checkTextFieldClick(explanation, mouseX, mouseY);
        checkTextFieldClick(survey1, mouseX, mouseY);
        checkTextFieldClick(survey2, mouseX, mouseY);
        checkTextFieldClick(survey3, mouseX, mouseY);
        checkTextFieldClick(survey4, mouseX, mouseY);
        checkTextFieldClick(survey5, mouseX, mouseY);
        checkTextFieldClick(survey6, mouseX, mouseY);
        checkTextFieldClick(email, mouseX, mouseY);

        // Check for button clicks
        for (const auto& btn : buttons) {
            if (mouseX >= btn.x && mouseX <= btn.x + btn.width &&
                mouseY >= btn.y && mouseY <= btn.y + btn.height) {
                if (btn.callback) {
                    btn.callback(btn.variable);
                }
                break;
            }
        }

        if(currentScreen == 3 || currentScreen == 4) {
            int x = 1550;
            int y = 130;
            int width = 350;
            int cellHeight = 18;
            int numRows = nRectangles;
            int columnWidth = width / 4;

            int clickedRow = (y + numRows * cellHeight - mouseY) / cellHeight;
            int clickedColumn = (mouseX - x) / columnWidth;

            if (clickedRow >= 0 && clickedRow < numRows && clickedColumn >= 0 && clickedColumn < 4) {
                handleCellClick(clickedRow, clickedColumn);
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

void iterationDone() {
    // Update the grammar with new MS
    CF = bso::spatial_design::conformal::cf_building_model(MS);
    grm.~grammar(); // Call the destructor manually
    new (&grm) bso::grammar::grammar(CF); // Reconstruct 'grm' in the existing memory

    // Clear and resize the vector, initializing each element to a default state
    //structureAssignments.clear();
    //structureAssignments.resize(numRectangles, DefaultStructure());  // Assuming DefaultStructure() gives a default-initialized structure
    //std::fill(tableClicked.begin(), tableClicked.end(), 0);  // Reset all clicks to zero


    bso::structural_design::sd_model SD_model_sub = grm.sd_grammar<bso::grammar::DESIGN_HUMAN>(std::string("files_SCDP/settings/sd_settings.txt"), flatShellStructure, substituteStructure);
    nRectangles = SD_model_sub.getSubRectangles().size();
    tableClicked = std::vector<int>(nRectangles, 0);
    spacePerformancesTable.clear();
    spacePerformances.clear();
    spacePerformances_option1.clear();
    spacePerformances_option2.clear();
    spacePerformances_option3.clear();
    //bso::structural_design::sd_model SD_model_sub = grm.sd_grammar<bso::grammar::DESIGN_HUMAN>(std::string("settings/sd_settings.txt"), flatShellStructure, substituteStructure);
    // After updating the grammar and the structural design model
    //auto subRectangle = SD_model_sub.getSubRectangles();  // Get new subrectangles

    // Clear existing data in vectors and resize based on new data
    //structureAssignments.clear();
    //structureAssignments.resize(subRectangle.size(), 0);

    //tableClicked.clear();
    //tableClicked.resize(subRectangle.size(), 0);  // Reset all clicks to zero
}
void drawTwoColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::pair<int, double>>& spacePerformances) {
    int numRows = spacePerformances.size() + 1; // Including header
    int tableHeight = numRows * cellHeight;
    int columnWidth = width / 2;
    int baseY = y + tableHeight; // Top of the table

    // Set the color to black
    glColor3f(0.0, 0.0, 0.0);

    // Draw the perimeter of the table
    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + tableHeight);
    glVertex2f(x, y + tableHeight);
    glEnd();

    // Draw horizontal lines for each row
    for (int i = 0; i <= numRows; ++i) {
        glBegin(GL_LINES);
        glVertex2f(x, y + i * cellHeight);
        glVertex2f(x + width, y + i * cellHeight);
        glEnd();
    }

    // Draw vertical line to separate columns
    glBegin(GL_LINES);
    glVertex2f(x + columnWidth, y);
    glVertex2f(x + columnWidth, y + tableHeight);
    glEnd();

    // Draw headers
    const char* headers[2] = { "Space ID", "Performance Score" };
    for (int col = 0; col < 2; ++col) {
        int columnX = x + col * columnWidth;
        int textWidth = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)headers[col]);
        int textX = columnX + (columnWidth - textWidth) / 2;
        int textY = baseY - cellHeight + (cellHeight - 18) / 2; // 18 is approximately the height of the font

        glRasterPos2f(textX, textY);
        for (const char* c = headers[col]; *c != '\0'; c++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
        }
    }

    // Draw each row of space ID and performance
    for (int row = 0; row < spacePerformances.size(); ++row) {
        int id = spacePerformances[row].first;
        double score = spacePerformances[row].second;
        std::ostringstream oss;
        oss.precision(3);  // Set precision
        oss << std::fixed << score;  // Format the score

        for (int col = 0; col < 2; ++col) {
            std::string text = col == 0 ? std::to_string(id) : oss.str();
            int columnX = x + col * columnWidth;
            int textWidth = glutBitmapLength(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)text.c_str());
            int textX = columnX + (columnWidth - textWidth) / 2;
            int textY = baseY - (row + 2) * cellHeight + (cellHeight - 18) / 2; // Adjust for each subsequent row

            glRasterPos2f(textX, textY);
            for (char c : text) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
            }
        }
    }
}

void drawFourColumnTable(int x, int y, int width, int cellHeight, const std::vector<std::string>& column1, std::vector<int> clickedOption) {
    size_t numRows = column1.size();

    int tableHeight = numRows * cellHeight;

    glColor3f(0.0, 0.0, 0.0);

    int currentY = y + tableHeight;
    int columnWidth = width / 4;

    for (size_t i = 0; i < numRows; ++i) {
        glBegin(GL_LINES);
        glVertex2f(x, currentY);
        glVertex2f(x + width, currentY);
        glEnd();

        for (int col = 0; col < 4; ++col) {
            int columnX = x + col * columnWidth;

            if (col > 0) {
                glBegin(GL_LINES);
                glVertex2f(columnX, currentY);
                glVertex2f(columnX, y);
                glEnd();
            }

            if (i < clickedOption.size() && col == clickedOption[i] && clickedOption[i] > 0 && clickedOption[i] < 4) {
                glColor4f(0.784, 0.098, 0.098, 0.8);
                glBegin(GL_QUADS);
                glVertex2f(columnX, currentY);
                glVertex2f(columnX + columnWidth, currentY);
                glVertex2f(columnX + columnWidth, currentY - cellHeight);
                glVertex2f(columnX, currentY - cellHeight);
                glEnd();
                glColor3f(0.0, 0.0, 0.0); // Reset color to black for text and lines
            }

            // Set text and positioning
            std::string text = (col == 0 && i < column1.size()) ? column1[i] : (col == 1 ? "beam" : (col == 2 ? "truss" : "flat_shell"));
            int textWidth = glutBitmapLength(GLUT_BITMAP_8_BY_13, (const unsigned char*)text.c_str());
            int textX = columnX + (columnWidth - textWidth) / 2; // Center text horizontally
            int textY = currentY - 11; // Set vertical position

            glRasterPos2f(textX, textY);
            for (char c : text) {
                glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
            }
        }

        currentY -= cellHeight;
    }
}




// void setWindowIcon(const char* iconPath) {
//     // Load the icon from the given file path
//     HICON hIcon = (HICON)LoadImage(NULL, iconPath, IMAGE_ICON, 0, 0, LR_LOADFROMFILE | LR_DEFAULTSIZE);

//     // Set the window icon
//     SendMessage(GetActiveWindow(), WM_SETICON, ICON_BIG, (LPARAM)hIcon);
// }

void countSelectionsFromTable(const std::vector<int>& tableClicked) {
    int countBeams = 0, countTrusses = 0, countFlatShells = 0;

    for (int clickedColumn : tableClicked) {
        switch (clickedColumn) {
            case 1:
                countBeams++;
                break;
            case 2:
                countTrusses++;
                break;
            case 3:
                countFlatShells++;
                break;
            default:
                // handle unclicked or improperly clicked rows, if necessary
                break;
        }
    }
    logAction(iteration_counter, "Number of beams", countBeams);
    logAction(iteration_counter, "Number of trusses", countTrusses);
    logAction(iteration_counter, "Number of flat shells", countFlatShells);
    std::cout << "Beams: " << countBeams << ", Trusses: " << countTrusses << ", Flat Shells: " << countFlatShells << std::endl;
}

int nSpaces = 10;

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
    drawText("You are about to start an assignment involving a simulation of co-evolutionary design processes. This is a method to iteratively optimize a building spatial design based on the structural performance of the design. Therefore, you will create a structural model and consequently change the building spatial design. You will in total complete 2 iterations of SCDP. Your assignment starts with a building spatial design from the villa depicted on the left.", startText, 900, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("As part of this assignment, we ask you to 'think aloud.' This means we want you to say what you are thinking about at each step of the process, as if you were explaining your thoughts to someone else. This helps us understand your decision-making process better. Please remember to keep verbalizing your thoughts as you work through each screen.", startText, 550, textWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please read the instructions carefully on each screen. If you have any questions, please raise your hand.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("Start assignment", startText, 80, textWidth, 50, changeScreen, 1);
}

void buildingSpatialScreen() {
    drawText("Building spatial design", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("This screen displays your current building spatial design. Use the mouse to rotate the model and familiarize yourself with each uniquely numbered space. The counter at the top left indicates your current iteration. When you are ready, click 'Continue' to proceed.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);
    drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 2);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModelScreen() {
    drawText("Structural design", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("The first step in this assignment is the creation of a structural model. You will assign one of three possible structural types to each rectangle of the spatial model. The available types are:", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("- Beam", startText, 840, textWidth, 0.0f, 0.0f, 0.0f, false);
    drawText("- Truss", startText, 820, textWidth, 0.0f, 0.0f, 0.0f, false);
    drawText("- Flat shell", startText, 800, textWidth, 0.0f, 0.0f, 0.0f, false);


    GLfloat emissionColor[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // Emit the texture's color
    glMaterialfv(GL_FRONT, GL_EMISSION, emissionColor); // Apply to front face

    float picWidth = textWidth;
    float picHeight = 136;
    displayTexture(imgElements, startText, 600, picWidth, picHeight);

    GLfloat defaultEmission[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, defaultEmission);

    drawText("In the next screen you have to assign a structural type to each rectangle by clicking the corresponding option in the table. Choose the type you believe is best suited for each rectangle. All horizontal rectangles (floors & roof) are assigned a flat shell by default and can not be changed.", startText, 450, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Continue to think aloud as you assign structural types to each rectangle; describe your choices and the reasoning behind them.", startText, 250, textWidth, 0.0f, 0.0f, 0.0f);
    if (iteration_counter == 1) {
        drawButton("Create structural design", startText, 80, textWidth, 50, changeScreen, 3);
    }
    else {
        drawButton("Create structural design", startText, 80, textWidth, 50, changeScreen, 4);
    }

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModel1Screen() {
    std::vector<std::string> nr_rectangles;
    // Populate with numbers corresponding to each rectangle
    for (size_t j = 0; j < nRectangles; ++j) {
        nr_rectangles.push_back(std::to_string(j + 1));  // numbering from 1
    }

    // Initialize or reset the tableClicked vector for the entire list
    if(!tableInitialized){
        tableClicked = std::vector<int>(nRectangles, 0);
        tableInitialized = true;
    }

    // Draw table and other UI elements as before, adjust positioning if needed
    drawFourColumnTable(1550, 130, 350, 18, nr_rectangles, tableClicked);

    // Display a text prompt below the table
    //drawText("Once you are finished, please continue below.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);
    if (allRowsClicked(tableClicked)) {
        drawText("Are you sure this is your final design? Press the button below to continue.", startText, 100, textWidth, 0.784f, 0.098f, 0.098f);
        drawButton("Confirm structural model", startText, 20, textWidth, 40, [](int){analyzeSDmodel(); countSelectionsFromTable(tableClicked); changeScreen(5);}, 0);
    } else {
        drawText("Please think aloud why you choose certain types. You can only continue if you have selected a type for every rectangle.", startText, 100, textWidth, 0.784f, 0.098f, 0.098f);
    }

    // Display the current iteration number at the bottom
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}


void chooseSDScreen() {
    drawText("Choose structural model", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("On the top left, you can see the structural model you just created. Besides this model, three different structural models are shown which are generated by AI. Underneath each model, the option and total strain energy are shown. ", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Strain energy in structural design refers to the energy absorbed by a structure when it deforms under load, essentially representing the work done by the load in causing the deformation.", startText, 800, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Please select one of the models to proceed with. This can be your own model or one of the AI-generated options. As you make your choice, please verbalize your reasoning.", startText, 700, textWidth, 0.0f, 0.0f, 0.0f);

    std::ostringstream oss1;
    oss1 << std::fixed << std::setprecision(0);
    oss1 << "Option 1: your own structural model [" << sdResult1 << " Nmm]";
    std::string option1 = oss1.str();

    std::ostringstream oss2;
    oss2 << std::fixed << std::setprecision(0);
    oss2 << "Option 2: AI generated model [" << sdResult2 << " Nmm]";
    std::string option2 = oss2.str();

    std::ostringstream oss3;
    oss3 << std::fixed << std::setprecision(0);
    oss3 << "Option 3: AI generated model [" << sdResult3 << " Nmm]";
    std::string option3 = oss3.str();

    std::ostringstream oss4;
    oss4 << std::fixed << std::setprecision(0);
    oss4 << "Option 4: AI generated model [" << sdResult4 << " Nmm]";
    std::string option4 = oss4.str();

    //std::string option1 = "Option 1: Your own sturctural model [" + std::to_string(sdResult1) + "]";
    drawText(option1.c_str(), 200, 630, 500, 0.0f, 0.0f, 0.0f);

    //std::string option2 = "Option 2: AI created model [" + std::to_string(sdResult2) + "]";
    drawText(option2.c_str(), 1020, 630, 500, 0.0f, 0.0f, 0.0f);

    //std::string option3 = "Option 3: AI created model [" + std::to_string(sdResult3) + "]";
    drawText(option3.c_str(), 200, 30, 500, 0.0f, 0.0f, 0.0f);

    //std::string option4 = "Option 4: AI created model [" + std::to_string(sdResult4) + "]";
    drawText(option4.c_str(), 1020, 30, 500, 0.0f, 0.0f, 0.0f);

    if (confirmSDmodel) {
        // Draw the message indicating the space has been deleted
        //drawText("Space split successfully. Click 'Continue' to proceed.", startText, 400, textWidth, 0.0f, 0.0f, 0.0f);
        //drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 8);
        changeScreen(6);
    }

    else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Enter the option number of the model you wish to continue with below.", startText, 350, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 300, textWidth, 25, chooseSD);
    }

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to submit, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 250, textWidth, r, g, b);
    }

    // Display the current iteration number at the bottom
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void structuralModel2Screen() {
    std::vector<std::string> nr_rectangles;
    // Populate with numbers corresponding to each rectangle
    for (size_t j = 0; j < nRectangles; ++j) {
        nr_rectangles.push_back(std::to_string(j + 1));  // numbering from 1
    }

    // Initialize or reset the tableClicked vector for the entire list
    if(!tableInitialized){
        tableClicked = std::vector<int>(nRectangles, 0);
        tableInitialized = true;
    }

    // Draw table and other UI elements as before, adjust positioning if needed
    drawFourColumnTable(1550, 130, 350, 18, nr_rectangles, tableClicked);

    // Display a text prompt below the table
    //drawText("Once you are finished, please continue below.", startText, 200, textWidth, 0.0f, 0.0f, 0.0f);

    if (allRowsClicked(tableClicked)) {
        drawText("Are you sure this is your final design? Press the button below to continue.", startText, 100, textWidth, 0.784f, 0.098f, 0.098f);
        drawButton("Confirm structural model", startText, 20, textWidth, 40, [](int){analyzeSDmodel(); countSelectionsFromTable(tableClicked); changeScreen(5);}, 0);
    } else {
        drawText("Please think aloud why you choose certain types. You can only continue if you have selected a type for every rectangle.", startText, 100, textWidth, 0.784f, 0.098f, 0.098f);
    }

    // Display the current iteration number at the bottom
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void removeSpaceScreen() {
    std::vector<std::string> nr_spaces;
    // Populate with numbers corresponding to each rectangle
    for (size_t j = 0; j < nSpaces; ++j) {
        nr_spaces.push_back(std::to_string(j + 1));  // numbering from 1
    }

    // Initialize or reset the tableClicked vector for the entire list
    if(!tableInitialized){
        tableClicked = std::vector<int>(nSpaces, 0);
        tableInitialized = true;
    }

    drawText("Space removal", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to remove a maximum of 1 space. The strain energy for each space is displayed in the table below, which you can use as a basis for your decision. However, you are also free to make your own choice.", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Think aloud about your reasons for choosing a particular space to remove. Enter the space ID below and press 'Enter' to confirm. Please think aloud as you decide which space to remove. Explain your reasoning for the choice you're making.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    if (deletionConfirmed) {
        // Draw the message indicating the space has been deleted
        drawText("Space deleted successfully. Click 'Continue' to proceed.", startText, 250, textWidth, 0.0f, 0.0f, 0.0f);

        // Draw the "Continue" button
        drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 7);
    } else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Please enter the space ID below to remove.", startText, 250, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 200, textWidth, 25, removeSpace);
    }

    drawTwoColumnTable(1530, 300, 350, 30, spacePerformancesTable);

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to submit, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 150, textWidth, r, g, b);
    }

    drawText("Please note that the SD model is not updated after spatial modifications", 800, 50, 800, 1.0f, 0.0f, 0.0f, true);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}


void splitSpaceScreen() {
    std::vector<std::string> nr_spaces;
    // Populate with numbers corresponding to each rectangle
    for (size_t j = 0; j < nSpaces; ++j) {
        nr_spaces.push_back(std::to_string(nSpaces + 1));  // numbering from 1
    }

    // Initialize or reset the tableClicked vector for the entire list
    if(!tableInitialized){
        tableClicked = std::vector<int>(nSpaces, 0);
        tableInitialized = true;
    }

    drawText("Space splitting", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You are asked to split a maximum of 1 space. The strain energy for each space is displayed in the table below, which you can use as a basis for your decision. However, you are also free to make your own choice.", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("As you decide which space to split, please explain your reasoning aloud. Enter the space ID for the space you wish to split and press 'Enter' to confirm.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    if (splittingConfirmed) {
        // Draw the message indicating the space has been deleted
        drawText("Space split successfully. Click 'Continue' to proceed.", startText, 250, textWidth, 0.0f, 0.0f, 0.0f);

        drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 8);
    }

    else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Please enter the space ID below to split.", startText, 250, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 200, textWidth, 25, splitSpace);
    }

    drawTwoColumnTable(1530, 300, 350, 30, spacePerformancesTable);

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to submit, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 150, textWidth, r, g, b);
    }
    scalingCompleted = false;
    drawText("Please note that the SD model is not updated after spatial modifications", 800, 50, 800, 1.0f, 0.0f, 0.0f, true);

    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void chooseMSScreen() {
    drawText("Choose spatial design", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("On the top left, you can see the building spatial design you just created by the spatial modifications. Besides your own design, three different building spatial designs are shown which are generated by AI. For each option, a different space is removed and split. ", startText, 950, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Underneath each design, a diversity score is visualized. Each design is compared to the initial building spatial design. Your own building spatial design serves as base line and therefore the other values indicate how diverse they are compared to your design.  A higher percentage indicates a greater deviation from your original design, whereas a lower percentage indicates a smaller deviation.", startText, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("You are asked to select an option to continue with. This can be your own building spatial model, but also one of the options generated by AI. Please think aloud why you make this decision.", startText, 500, textWidth, 0.0f, 0.0f, 0.0f);

    std::ostringstream oss1;
    oss1 << std::fixed << std::setprecision(0);
    oss1 << "Option 1: Your own building spatial design [" << 100 << "% ]";
    std::string option1 = oss1.str();

    std::ostringstream oss2;
    oss2 << std::fixed << std::setprecision(0);
    oss2 << "Option 2: AI generated design [" << relativeDissimilarity1 << "% ]";
    std::string option2 = oss2.str();

    std::ostringstream oss3;
    oss3 << std::fixed << std::setprecision(0);
    oss3 << "Option 3: AI generated design [" << relativeDissimilarity2 << "% ]";
    std::string option3 = oss3.str();

    std::ostringstream oss4;
    oss4 << std::fixed << std::setprecision(0);
    oss4 << "Option 4: AI generated design [" << relativeDissimilarity3 << "% ]";
    std::string option4 = oss4.str();

    //std::string option1 = "Option 1: Your own sturctural model [" + std::to_string(sdResult1) + "]";
    drawText(option1.c_str(), 200, 630, 500, 0.0f, 0.0f, 0.0f);

    //std::string option2 = "Option 2: AI created model [" + std::to_string(sdResult2) + "]";
    drawText(option2.c_str(), 1020, 630, 500, 0.0f, 0.0f, 0.0f);

    //std::string option3 = "Option 3: AI created model [" + std::to_string(sdResult3) + "]";
    drawText(option3.c_str(), 200, 30, 500, 0.0f, 0.0f, 0.0f);

    //std::string option4 = "Option 4: AI created model [" + std::to_string(sdResult4) + "]";
    drawText(option4.c_str(), 1020, 30, 500, 0.0f, 0.0f, 0.0f);


    //drawText("Option 1: Your own building spatial design", 250, 630, textWidth, 0.0f, 0.0f, 0.0f);
    //drawText("Option 2: AI generated design", 1050, 630, textWidth, 0.0f, 0.0f, 0.0f);
    //drawText("Option 3: AI generated design", 250, 30, textWidth, 0.0f, 0.0f, 0.0f);
    //drawText("Option 4: AI generated design", 1050, 30, textWidth, 0.0f, 0.0f, 0.0f);

    if (confirmMSmodel) {
        // Draw the message indicating the space has been deleted
        //drawText("Space split successfully. Click 'Continue' to proceed.", startText, 400, textWidth, 0.0f, 0.0f, 0.0f);
        //drawButton("Continue", startText, 80, textWidth, 50, changeScreen, 9);
        if (iteration_counter < max_iterations) {
            changeScreen(9);
        }
        else {
            changeScreen(10);
        }
    }

    else if (!inputFieldDisabled) {
        // Render the input field only if it's not disabled
        drawText("Enter the option number of the design you wish to continue with below.", startText, 350, textWidth, 0.0f, 0.0f, 0.0f);
        drawTextField(startText, 300, textWidth, 25, chooseMS);
    }

    // Check and display error message below the text field if there's an error
    if (spaceInputError) {
        bool isConfirmation = spaceInputErrorMessage == "Press Y to submit, N to cancel.";

        float r = isConfirmation ? 0.0f : 1.0f; // Blue for confirmation, red for error
        float g = 0.0f;
        float b = isConfirmation ? 1.0f : 0.0f;

        drawText(spaceInputErrorMessage.c_str(), startText, 250, textWidth, r, g, b);
    }

    // Display the current iteration number at the bottom
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
}

void iteration1CompleteScreen() {
    drawText("Iteration 1 complete", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("First iteration complete! The building spatial model has been adjusted to recover the initial floor area. Review all previous steps on the screen to your left. The updated design will now serve as the starting point for the second iteration.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    std::string nextIterationText = "Start iteration " + std::to_string(iteration_counter + 1);
    drawButton(nextIterationText.c_str(), startText, 80, textWidth, 50, [](int){iteration_counter++; iterationDone(); changeScreen(1);}, 0);

    deletionConfirmed = false;
    splittingConfirmed = false;
    confirmSDmodel = false;
    confirmMSmodel = false;
    removeSpace.text = "";
    splitSpace.text = "";
    chooseMS.text = "";
    chooseSD.text = "";
}

void iteration2CompleteScreen() {
    drawText("Assignment complete", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    std::string iterationText = "Iteration " + std::to_string(iteration_counter);
    drawText(iterationText.c_str(), 80, 1000, 250, 0.0f, 0.0f, 0.0f, true);
    drawText("Congratulations! You have successfully completed the second iteration and the main portion of this assignment. You may now stop verbalizing your thoughts. Please remain in the Teams meeting. On the left, you can review your entire assignment progression, from the initial building spatial design to the final building spatial design. Please proceed to fill out the survey.", startText, 850, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("Start survey", startText, 80, textWidth, 50, changeScreen, 11);

    deletionConfirmed = false;
    splittingConfirmed = false;
    confirmSDmodel = false;
    confirmMSmodel = false;
    removeSpace.text = "";
    splitSpace.text = "";
    chooseMS.text = "";
    chooseSD.text = "";
}

float surveyStart = 100;
float surveyWidth = 800;
float buttonWidth = 50;
float totalSpacingWidth = surveyWidth - (5 * buttonWidth);
float spacing = totalSpacingWidth / 4;

void surveyScreen1() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 1/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("How satisfied were you with the overall design process you experienced?", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawButton("1", surveyStart, 800, buttonWidth, 30, buttonClicked, 1);
    drawButton("2", surveyStart + buttonWidth + spacing, 800, buttonWidth, 30, buttonClicked, 2);
    drawButton("3", surveyStart + 2 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 3);
    drawButton("4", surveyStart + 3 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 4);
    drawButton("5", surveyStart + 4 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 5);

    drawText("Very Unsatisfied", surveyStart - 50, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Unsatisfied", surveyStart + 165, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Neutral", surveyStart + 370, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Satisfied", surveyStart + 550, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Very Satisfied", surveyStart + surveyWidth - 110, 750, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please explain your answer:", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey1);
    if (getSelectedButtonLabel() != "") {
        drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(std::stoi(getSelectedButtonLabel()), "How satisfied were you with the overall design process you experienced?", survey1.text); changeScreen(12);}, 0);
    }

}

void surveyScreen2() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 2/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("How easy was it to use the design tools provided?", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawButton("1", surveyStart, 800, buttonWidth, 30, buttonClicked, 1);
    drawButton("2", surveyStart + buttonWidth + spacing, 800, buttonWidth, 30, buttonClicked, 2);
    drawButton("3", surveyStart + 2 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 3);
    drawButton("4", surveyStart + 3 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 4);
    drawButton("5", surveyStart + 4 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 5);

    drawText("Very Difficult", surveyStart - 40, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Difficult", surveyStart + 180, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Neutral", surveyStart + 370, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Easy", surveyStart + 570, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Very Easy", surveyStart + surveyWidth - 60, 750, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please explain your answer:", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey2);
    if (getSelectedButtonLabel() != "") {
        drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(std::stoi(getSelectedButtonLabel()), "How easy was it to use the design tools provided?", survey2.text); changeScreen(13);}, 0);
    }
}

void surveyScreen3() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 3/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("How often did you find the AI-provided models reliable enough to influence your design decisions?", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawButton("1", surveyStart, 800, buttonWidth, 30, buttonClicked, 1);
    drawButton("2", surveyStart + buttonWidth + spacing, 800, buttonWidth, 30, buttonClicked, 2);
    drawButton("3", surveyStart + 2 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 3);
    drawButton("4", surveyStart + 3 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 4);
    drawButton("5", surveyStart + 4 * (buttonWidth + spacing), 800, buttonWidth, 30, buttonClicked, 5);

    drawText("Never", surveyStart, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Rarely", surveyStart + 165, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Sometimes", surveyStart + 370, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Often", surveyStart + 550, 750, textWidth, 0.0f, 0.0f, 0.0f);
    drawText("Very Often", surveyStart + surveyWidth - 110, 750, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please explain your answer:", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey3);
    if (getSelectedButtonLabel() != "") {
        drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(std::stoi(getSelectedButtonLabel()), "How often did you find the AI-provided models reliable enough to influence your design decisions?", survey3.text); changeScreen(14);}, 0);
    }
}

void surveyScreen4() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 4/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("How do you think your design process would have differed if you had not had AI assistance?", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please explain your answer:", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey4);

    drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(0, "How do you think your design process would have differed if you had not had AI assistance?", survey4.text); changeScreen(15);}, 0);

}

void surveyScreen5() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 5/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Would you choose to use this design process in future projects based on your current experience?", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText(" Please explain why or why not.", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey5);

    drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(0, "Would you choose to use this design process in future projects based on your current experience?", survey5.text); changeScreen(16);}, 0);

}

void surveyScreen6() {
    drawText("Survey", surveyStart, 1000, surveyWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("Question 6/6", surveyStart + surveyWidth - 100, 1000, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please provide any additional comments or suggestions on how we could improve the design process used in this experiment.", surveyStart, 900, surveyWidth, 0.0f, 0.0f, 0.0f);

    drawText("Please explain your answer:", surveyStart, 450, surveyWidth, 0.0f, 0.0f, 0.0f);
    drawTextField(surveyStart, 400, surveyWidth, 300, survey6);

    drawButton("Next", 100, surveyStart, surveyWidth, 50, [](int){logAction(0, "Please provide any additional comments or suggestions on how we could improve the design process used in this experiment.", survey6.text); changeScreen(17);}, 0);
}

void outroScreen() {
    glEnable(GL_LIGHTING); // Enable to show image becomes black
    glEnable(GL_LIGHT0); // Enable to prevent image becomes black

    float picWidth = screenWidth / 1.28f; // Width of the picture as specified.
    float picHeight = screenHeight;
    displayTexture(imgVilla, 0, 0, picWidth, picHeight);

    glDisable(GL_LIGHTING); //Disbale for other GUI elements
    glDisable(GL_LIGHT0); //Disbale for other GUI elements

    // Draw the title
    drawText("Simulation of Co-evolutionary Design Processes", startText, 1000, textWidth, 0.0f, 0.0f, 0.0f, true);
    drawText("You have just completed the assignment. Thank you very much for your participation! If you would like to receive the results of this research and be included in the acknowledgments, please leave your email below.", startText, 900, textWidth, 0.0f, 0.0f, 0.0f);

    drawTextField(startText, 700, textWidth, 25, email);

    drawText("This is the end of the assignment. Don't forget to follow the 'after the assignment' steps in the set-up guide.", startText, 300, textWidth, 0.0f, 0.0f, 0.0f);

    drawButton("Finish assignment", startText, 80, textWidth, 50, [](int){logAction(0, "Email", email.text); writeToProcessFile("assignment_6_SCDP_human_ai.csv"); exit(0);}, 0);
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    initializeModels();
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(screenWidth, screenHeight);
    glutCreateWindow("Assignment 6: SCDP Human-AI Design");
    glutFullScreen(); // Make the window fullscreen right from the start

    initializeTextures();

    // Set window icon
    // setWindowIcon("TUE.ico");

    // Set callback functions
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutMouseFunc(onMouseClick);
    glutMotionFunc(motion);
    glutPassiveMotionFunc(passive_motion);
    glutSpecialFunc(handleSpecialKeypress);

    glShadeModel(GL_SMOOTH);

    // Main loop
    glutMainLoop();
    return 0;
}

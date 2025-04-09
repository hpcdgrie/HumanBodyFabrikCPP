#include <fabrik.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <cassert>
std::vector<osg::Vec3> getPositions(const std::string &filename)
{
    std::vector<osg::Vec3> positions;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return positions;
    }

    float x, y, z;
    while (file >> x >> y >> z)
    {
        positions.emplace_back(x, y, z);
    }
    return positions;
}

std::vector<osg::Quat> getOrientations(const std::string &filename)
{
    std::vector<osg::Quat> orientations;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return orientations;
    }

    float x, y, z, w;
    while (file >> x >> y >> z >> w)
    {
        orientations.emplace_back(x, y, z, w);
    }
    return orientations;
}

std::vector<std::string> splitLineByComma(const std::string &line) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ',')) {
        result.push_back(item);
    }

    return result;
}

void getTarget(const std::string &filename, osg::Vec3 &pos, osg::Quat &orientation)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    auto line = std::string();
    std::getline(file, line);
    auto parts = splitLineByComma(line);
    pos.x() = std::stof(parts[0]);
    pos.y() = std::stof(parts[1]);
    pos.z() = std::stof(parts[2]);
    std::getline(file, line);
    parts = splitLineByComma(line);
    orientation.x() = std::stof(parts[0]);
    orientation.y() = std::stof(parts[1]);
    orientation.z() = std::stof(parts[2]);
    orientation.w() = std::stof(parts[3]);
}
    


std::vector<Constraint::Type> getConstraintTypes(const std::string &filename)
{
    std::vector<Constraint::Type> constraints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return constraints;
    }
    std::array<const char*, 3> constraint_types = {"BALL", "hinge", "endEffector"};

    std::string type;
    while (file >> type)
    {
        bool found = false;
        for(size_t i = 0; i < constraint_types.size(); ++i)
        {
            if (type == constraint_types[i])
            {
                constraints.push_back(static_cast<Constraint::Type>(i));
                found = true;
                break;
            }
        }
        if (!found)
            std::cerr << "Unknown constraint type: " << type << std::endl;
    }
    return constraints;
}

std::vector<float> getBoneTwistConstraints(const std::string &filename)
{
    std::vector<float> constraints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return constraints;
    }

    float constraint;
    while (file >> constraint)
    {
        constraints.push_back(constraint);
    }
    return constraints;
}

int main(){
    std::cout << "Hello, World!" << std::endl;
    // const std::string dir = "C:/Users/Dennis/Apps/FBRIK_Full_Body/FABRIK_Full_Body_C++/test/inputs/";
    const std::string dir = "C:/Users/Dennis/Apps/FBRIK_Full_Body/FABRIK_Full_Body/test/inputs/";
   
    auto joints_position = getPositions(dir + "joints_position.txt");
    auto joints_position_fixed = getPositions(dir + "joints_position_fixed.txt");
    auto orientation = getOrientations(dir + "orientation.txt");
    osg::Vec3 targetPos;
    osg::Quat targetRot;
    getTarget(dir + "target.txt", targetPos, targetRot);
    auto joints_constraint = getOrientations(dir + "joints_constraint.txt");
    auto constraintTypes = getConstraintTypes(dir + "constraint_type.txt");
    auto bone_twist_constraints = getBoneTwistConstraints(dir + "bone_twist_constraints.txt");

    Skeleton skeleton;
    for (size_t i = 0; i < joints_position.size(); i++)
    {
        Joint joint;
        joint.initialPosition = joints_position_fixed[i];
        joint.position = joints_position[i];
        joint.rotation = orientation[i];
        joint.constraint.type = constraintTypes[i];
        joint.constraint.adduction = joints_constraint[i][0];
        joint.constraint.abduction = joints_constraint[i][1];
        joint.constraint.flexion = joints_constraint[i][2];
        joint.constraint.extension = joints_constraint[i][3];
        joint.constraint.twist = bone_twist_constraints[i];
        skeleton[i] = joint;
    }

    Fabrik fabrik(skeleton, targetPos, targetRot);
    fabrik.solve();
    std::cout << "Solved positions:" << std::endl;
    
    auto longestJointName = std::max_element(JointNames.begin(), JointNames.end(), [](const char* a, const char* b) {
        return std::strlen(a) < std::strlen(b);
    });

    for (size_t i = 0; i < skeleton.size(); i++)
    {
        std::cerr << std::left << std::setw(strlen(*longestJointName) + 2) << JointNames[i] << ": ";
        std::cerr << std::fixed << std::setprecision(3) // Set precision to 3 decimal places
                  << std::right << std::setw(10) << skeleton[i].position.x() << " "
                  << std::setw(10) << skeleton[i].position.y() << " "
                  << std::setw(10) << skeleton[i].position.z() << std::endl;
    }
    return 0;
}
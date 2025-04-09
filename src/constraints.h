#pragma once
#include <array>
#include <vector>
#include <osg/Vec3>
#include <osg/Quat>
constexpr size_t numBones = 19;
constexpr size_t defaultTwistConstraint = 40;
// This is the constraints for each human body joints.
// According to the joint reference plane and coordinate, you should enter four number for each joint (Adduction, Abduction, Flexion, Extension).
// Notice that order of these four number is according to joints orientation in its reference plane(in degree).
struct Constraint
{
    float adduction = 0, abduction = 0, flexion = 0, extension = 0, twist = defaultTwistConstraint; // in radian
    enum Type
    {
        Ball,
        Hinge,
        Effector
    } type;
};

struct Joint
{
    osg::Vec3 initialPosition, position, relativePosition;
    osg::Quat rotation;
    Constraint constraint;
};

constexpr std::array<const char *, 19> JointNames = {"hips",          // 1
                                                     "rightSHoulder", // 2
                                                     "rightElbow",    // 3
                                                     "rightWrist",    // 4
                                                     "rightEnd",      // 5
                                                     "leftSHoulder",  // 6
                                                     "leftElbow",     // 7
                                                     "leftWrist",     // 8
                                                     "leftEnd",       // 9
                                                     "rightUpperLeg", // 10
                                                     "leftUpperLeg",  // 11
                                                     "rightKnee",     // 12
                                                     "rightAnkle",    // 13
                                                     "rightFoot",     // 14
                                                     "leftKnee",      // 15
                                                     "leftAnkle",     // 16
                                                     "leftFoot",      // 17
                                                     "neck",          // 18
                                                     "head"};         // 19
typedef std::array<Joint, numBones> Skeleton;

class Constraints
{
public:
    Constraints(const Skeleton &skeleton, const std::vector<size_t> &body_indx, size_t i);
    osg::Vec3 rotational_constraint();

private:
    const Skeleton &m_skeleton;
    std::vector<size_t> m_body_indx;
    size_t m_i;
    Constraint m_theta;
    osg::Quat m_rotation;
    float m_si;
    const Joint &joint();
};
#pragma once

// This is solver of Inverse kinematic of a whole chain of human body with foot on the ground
// For more info about the procedure refer to https://www.sciencedirect.com/science/article/pii/S1524070311000178
// and for the constraints refer to https://www.researchgate.net/publication/271771862_Extending_FABRIK_with_model_constraints


#include "constraints.h"


class Fabrik{
public:
    Fabrik(Skeleton& joints,
           const osg::Vec3& targetPosition,
           const osg::Quat& targetRotation);

    void solve();
private:
    Skeleton &m_skeleton;
    osg::Vec3 m_targetPosition;
    osg::Quat m_targetRotation;

    const float solve_distance_threshold = 0.01;
    const std::vector<size_t> rightArmIndex{17, 1, 2, 3, 4};
    const size_t right_end_effector_index = 4;
    const std::vector<size_t> leftArmIndex{17, 5, 6, 7, 8};
    const size_t left_end_effector_index = 8;
    const std::vector<size_t> upperChain{0, 1, 5, 17, 0};
    const std::vector<size_t> lowerChain{0, 9, 10, 0};
    const std::vector<size_t> rightLeg{13, 12, 11, 9};
    const std::vector<size_t> leftLeg{16, 15, 14, 10};
    const std::vector<size_t> neck{0, 17};
    const std::vector<size_t> head{17, 18};

    const std::vector<size_t> inverseLeftArmIndex{8, 7, 6, 5};
    const std::vector<size_t> inverseRightArmIndex{4, 3, 2, 1};

    enum class Laterality{
        Right, Left
    };
    Laterality laterality() const; // left or right handedness
    void solve_for_distance(size_t i, size_t j, const std::vector<size_t> &chain);
    void solve_for_orientation(size_t outer_joint, size_t inner_joint, const std::vector<size_t> &chain);
    void forward_arm(const std::vector<size_t>& arm_index, const osg::Vec3& target, const osg::Quat& target_orientation);
    void update_upper_chain_f(Laterality target_position);
    void update_upper_chain_b(Laterality target_position);
    void update_lower_chain_f();
    void update_lower_chain_b();
    void forward_leg();
    void backward_arm(const std::vector<size_t>& arm_index);
    void backward_leg();
    float check_reach(const std::vector<size_t>& arm_indicees);






};
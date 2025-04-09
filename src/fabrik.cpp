#include "fabrik.h"
#include "constraints.h"
#include <iostream>
#include <cassert>
Fabrik::Fabrik(Skeleton& Joints,
               const osg::Vec3& targetPosition,
               const osg::Quat& targetRotation)
    : m_skeleton(Joints)
    , m_targetPosition(targetPosition)
    , m_targetRotation(targetRotation)
{
}

Fabrik::Laterality Fabrik::laterality() const
{
    float right_upper_body = (m_skeleton[right_end_effector_index].initialPosition - m_targetPosition).length2();
    float left_upper_body = (m_skeleton[left_end_effector_index].initialPosition - m_targetPosition).length2();
    if (right_upper_body <= left_upper_body)
        return Laterality::Right;
    return Laterality::Left;
}

void Fabrik::solve_for_distance(size_t i, size_t j, const std::vector<size_t> &chain)
{
    auto r = (m_skeleton[chain[i]].position - m_skeleton[chain[j]].position).length();
    auto lamda = (m_skeleton[chain[i]].initialPosition - m_skeleton[chain[j]].initialPosition).length() / r;
    auto pos = m_skeleton[chain[i]].position * (1 - lamda) + m_skeleton[chain[j]].position * lamda;
    assert(pos.x() == pos.x() && pos.y() == pos.y() && pos.z() == pos.z() && "NAN");
    m_skeleton[chain[j]].position = pos;
}
// Finding the rotation rotor between outer joint and inner joint of each FABRIK iteration
osg::Quat find_rotation_quaternion(const osg::Quat& outer_quaternion, const osg::Quat& inner_quaternion)
{
    auto inverse = outer_quaternion.inverse();
    auto r = inner_quaternion * inverse;
    return r / r.length();
}

void Fabrik::solve_for_orientation(size_t outer_joint, size_t inner_joint, const std::vector<size_t> &chain)
{
    auto q1 = m_skeleton[chain[outer_joint]].rotation;
    auto q2 = m_skeleton[chain[inner_joint]].rotation;
    // finding the rotor that express rotation between two orientational frame(between outer and inner joint)
    auto rotor = find_rotation_quaternion(q1, q2);
    auto needed_rotation = acos(rotor.w()) * 2;
    if (needed_rotation <= m_skeleton[chain[outer_joint]].constraint.twist)
    {
        m_skeleton[chain[inner_joint]].rotation = rotor * m_skeleton[chain[outer_joint]].rotation;
    }
    else
    {
        // the maximum allowed rotation angle
        auto theta = (m_skeleton[chain[outer_joint]].constraint.twist);
        // the rotation axis
        auto v1 = rotor.asVec3();
        v1 /= sqrt(1 - rotor.w() * rotor.w()) * sin(theta / 2);
        auto w = cos(theta / 2);
        m_skeleton[chain[inner_joint]].rotation = osg::Quat(w, rotor.asVec3());
    }
}

void Fabrik::forward_arm(const std::vector<size_t>& arm_index, const osg::Vec3& target, const osg::Quat& target_orientation)
{
    auto n = arm_index.size();
    assert(target.x() == target.x() && target.y() == target.y() && target.z() == target.z() && "NAN");

    m_skeleton[arm_index[n - 1]].position = target;
    m_skeleton[arm_index[n - 1]].rotation = target_orientation;
    for (int i = n - 2; i >= 0; --i)
    {
        solve_for_distance(i + 1, i, arm_index);
        solve_for_orientation(i + 1, i, arm_index);
        if (i < n - 2)
        {
            auto mconstraint = Constraints(m_skeleton, arm_index, i + 1);
            auto constraint_return = mconstraint.rotational_constraint();
            if (constraint_return[0] != 0)
                m_skeleton[arm_index[i]].position = constraint_return;
        }
    }
}

void Fabrik::update_upper_chain_f(Laterality target_position)
{
    if (target_position == Laterality::Right)
    {
        solve_for_distance(1, 2, upperChain);
        solve_for_distance(2, 0, upperChain);
        solve_for_distance(1, 0, upperChain);
        solve_for_distance(0, 2, upperChain);
        forward_arm(inverseLeftArmIndex, m_skeleton[leftArmIndex[1]].position, m_skeleton[leftArmIndex[1]].rotation);
    }
    else
    {
        solve_for_distance(2, 0, upperChain);
        solve_for_distance(0, 1, upperChain);
        solve_for_distance(2, 1, upperChain);
        solve_for_distance(1, 0, upperChain);
        forward_arm(inverseRightArmIndex, m_skeleton[rightArmIndex[1]].position, m_skeleton[rightArmIndex[1]].rotation);
    }
}

void Fabrik::update_upper_chain_b(Laterality target_position)
{
    solve_for_distance(0, 2, upperChain);
    solve_for_distance(2, 1, upperChain);
    solve_for_distance(0, 1, upperChain);
    solve_for_distance(1, 2, upperChain);
    if (target_position == Laterality::Right)
        forward_arm(inverseLeftArmIndex, m_skeleton[leftArmIndex[1]].position, m_skeleton[leftArmIndex[1]].rotation);
    else
        forward_arm(inverseRightArmIndex, m_skeleton[rightArmIndex[1]].position, m_skeleton[rightArmIndex[1]].rotation);

    // Neck
    m_skeleton[upperChain[3]].position = (m_skeleton[upperChain[2]].position + m_skeleton[upperChain[1]].position) / 2;
}


void Fabrik::update_lower_chain_f()
{
    solve_for_distance(0, 1, lowerChain);
    solve_for_distance(1, 2, lowerChain);
    solve_for_distance(0, 2, lowerChain);
    solve_for_distance(2, 1, lowerChain);
}


void Fabrik::update_lower_chain_b()
{
    solve_for_distance(1, 0, lowerChain);
    solve_for_distance(2, 0, lowerChain);
}

void Fabrik::forward_leg()
{
    // set end effector of leg as target
    auto n = rightLeg.size();
    for (int i = n - 2; i >= 0; --i)
    {
        solve_for_distance(i + 1, i, rightLeg);
        solve_for_distance(i + 1, i, leftLeg);
        solve_for_orientation(i + 1, i, rightLeg);
        solve_for_orientation(i + 1, i, leftLeg);
        if (i < n - 2)
        {
            auto constraint_return = Constraints(m_skeleton, rightLeg, i + 1).rotational_constraint();
            assert(constraint_return[0] == constraint_return[0] && constraint_return[1] == constraint_return[1] && constraint_return[2] == constraint_return[2] && "NAN");
            if (constraint_return[0] != 0)
                m_skeleton[rightLeg[i]].position = constraint_return;


            constraint_return = Constraints(m_skeleton, leftLeg, i + 1).rotational_constraint();
            assert(constraint_return[0] == constraint_return[0] && constraint_return[1] == constraint_return[1] && constraint_return[2] == constraint_return[2] && "NAN");

            if (constraint_return[0] != 0)
                m_skeleton[leftLeg[i]].position = constraint_return;
        }
    }
}

void Fabrik::backward_arm(const std::vector<size_t>& arm_index)
{
    // set root as initial position
    auto n = arm_index.size();
    for (int i = 2; i < n; ++i)
    {
        solve_for_distance(i - 1, i, arm_index);
        solve_for_orientation(i - 1, i, arm_index);
    }
}

void Fabrik::backward_leg()
{
   // set root as initial position
    m_skeleton[rightLeg[0]].position = m_skeleton[rightLeg[0]].initialPosition;
    m_skeleton[rightLeg[1]].position = m_skeleton[rightLeg[1]].initialPosition;
    m_skeleton[leftLeg[0]].position = m_skeleton[leftLeg[0]].initialPosition;
    m_skeleton[leftLeg[1]].position = m_skeleton[leftLeg[1]].initialPosition;
    auto n = rightLeg.size();
    for (int i = 2; i < n; ++i)
    {
        // for Right leg
        solve_for_distance(i - 1, i, rightLeg);
        // solve_for_orientation(i - 1, i, rightLeg);

        // for left leg
        solve_for_distance(i - 1, i, leftLeg);
        // solve_for_orientation(i - 1, i, leftLeg);
    } 
}

float Fabrik::check_reach(const std::vector<size_t>& arm_indicees) {
        // arm length
        float sum_l = 0;
        for (size_t i = 0; i < arm_indicees.size() - 1; ++i)
        {
            auto p1 = m_skeleton[arm_indicees[i]].initialPosition;
            auto p2 = m_skeleton[arm_indicees[i + 1]].initialPosition;
            sum_l += (m_skeleton[arm_indicees[i]].initialPosition - m_skeleton[arm_indicees[i + 1]].initialPosition).length();
        }
        
        // chain length
        const auto &leg = &arm_indicees == &rightArmIndex ? rightLeg : leftLeg;
        sum_l += (m_skeleton[0].initialPosition - m_skeleton[leg[leg.size() - 1]].position).length();
        
        // Leg length
        for (size_t i = 0; i < leg.size() - 1; ++i)
            sum_l += (m_skeleton[leg[i]].initialPosition - m_skeleton[leg[i + 1]].initialPosition).length();
            
        if (sum_l < (m_skeleton[leg[0]].initialPosition - m_targetPosition).length())
        {
            std::cerr << "target is out of reach!!!!!!!" << std::endl;
            return -1;
        }
        else
            return (m_skeleton[arm_indicees[arm_indicees.size() - 1]].position - m_targetPosition).length();
}

void Fabrik::solve()
{
    auto counter = 0;
    auto sum_l = 0;
    auto target_pos = laterality();
    ////////////////////////////////////////////////////////
    // is target in reach or not
    float dif = 0;
    size_t n;
    const auto & armIndex = target_pos == Laterality::Right ? rightArmIndex : leftArmIndex;
    // is target in reach or not
    n = armIndex.size();
    dif = check_reach(armIndex);
    
    if(dif < 0)
        return;

    // target is in reach
    while (dif > solve_distance_threshold)
    {
        forward_arm(armIndex, m_targetPosition, m_targetRotation);
        update_upper_chain_f(target_pos);
        update_lower_chain_f();
        forward_leg();
        backward_leg();
        update_lower_chain_b();
        update_upper_chain_b(target_pos);
        backward_arm(armIndex);
        dif = (m_skeleton[armIndex[n - 1]].position - m_targetPosition).length();

        counter++;
        if (counter > 10)
            break;
    }
    //calculate the angles


}

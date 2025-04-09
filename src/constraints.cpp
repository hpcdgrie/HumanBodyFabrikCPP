#include "constraints.h"
#include <iostream>
#include <osg/Matrix>
#include <cassert>

osg::Vec2f find_initial_point(float a, float b, size_t sector, float x_t, float y_t)
{
    float x_k1 = x_t * a * b / sqrt(b * b * x_t * x_t + a * a * y_t * y_t);
    float y_k1 = y_t * a * b / sqrt(b * b * x_t * x_t + a * a * y_t * y_t);
    int sign_y = 0, sign_x = 0;
    if (sector == 1 || sector == 2)
        sign_y = 1;
    else if (sector == 3 || sector == 4)
        sign_y = -1;
    if (sector == 1 || sector == 4)
        sign_x = 1;
    else if (sector == 3 || sector == 2)
        sign_x = -1;

    float x_k2 = 0, y_k2 = 0;
    if (abs(x_t) < a)
    {
        x_k2 = x_t;
        y_k2 = sign_y * (b / a) * sqrt(a * a - x_t * x_t);
    }
    else
    {
        x_k2 = a * sign_x * a;
        y_k2 = 0;
    }

    float x0 = 0.5 * (x_k1 + x_k2);
    float y0 = 0.5 * (y_k1 + y_k2);

    return {x0, y0};
}



osg::Vec2f find_delta(float x, float y, float a, float b, float x_t, float y_t)
{
    float f1 = (a * a * y * y + b * b * x * x - a * a * b * b) / 2.0;
    float f2 = b * b * x * (y_t - y) - a * a * y * (x_t - x);

    float q11 = b*b*x;;
    float q12 = a*a *y;
    float q21 = (a*a - b*b) * y + b*b *y_t;
    float q22 = (a * a - b * b) * x - a *a * x_t;


    // inverting 2x2 matrix --> d, -b, -c, a
    auto inv = 1 / (q11 * q22 - q12 * q21);
    auto tmp = q11;
    q11 = q22 * inv;
    q22 = tmp * inv;
    q12 = -q12 * inv;
    q21 = -q21 * inv;

    return {-1 * (q11 * f1 + q12 * f2), -1 * (q21 * f1 + q22 * f2)};
}

osg::Vec2f find_next_point(float x, float y, float a, float b, float x_t, float y_t)
{
    auto delta = find_delta(x, y, a, b, x_t, y_t);
    x = x + delta[0];
    y = y + delta[1];
    assert(x == x && y == y);
    return {x, y};
}

osg::Vec2f find_nearest_point(float a, float b, size_t sector, float x_t, float y_t)
{
    auto initial = find_initial_point(a, b, sector, x_t, y_t);
    float x_initial = initial[0];
    float y_initial = initial[1];
    float x = x_initial;
    float y = y_initial;
    constexpr float thresh = 0.001;
    auto result = find_next_point(x, y, a, b, x_t, y_t);
    while (result[0] > thresh || result[1] > thresh)
    {
        x = result[0];
        y = result[1];
        result = find_next_point(x, y, a, b, x_t, y_t);
    }
    float x_nearest_point = result[0];
    float y_nearest_point = result[1];
    assert(x_nearest_point == x_nearest_point && y_nearest_point == y_nearest_point);
    return {x_nearest_point, y_nearest_point};
}


Constraints::Constraints(const Skeleton& skeleton, const std::vector<size_t> &body_indx, size_t i)
: m_skeleton(skeleton), m_body_indx(body_indx), m_i(i)
{
    m_si = acos(m_skeleton[m_body_indx[m_i]].rotation.w()) * 2;
}

float round(float var, size_t digit)
{
    auto factor = pow(10, digit);
    return std::round(var * factor) / factor;
}

osg::Vec3 Constraints::rotational_constraint()
{
    osg::Vec3 p_i = m_skeleton[m_body_indx[m_i]].position;
    osg::Vec3 p_before = m_skeleton[m_body_indx[m_i - 1]].position;
    osg::Vec3 p_next = m_skeleton[m_body_indx[m_i + 1]].position;

    osg::Vec3 v_i_next = p_next - p_i;
    osg::Vec3 v_before_i = p_i - p_before;

    float dotProd = v_i_next * v_before_i;

    // a unit vector of a line passing  from p(i+1) and P(i)
    osg::Vec3 unit_vec_next_i = -v_i_next;
    unit_vec_next_i.normalize();

    // the center of cone
    osg::Vec3 o = p_i + unit_vec_next_i * dotProd;
    switch (m_skeleton[m_body_indx[m_i]].constraint.type)
    {
    case Constraint::Type::Hinge:
    {
        // normal plane vector of p(next), p(i), p(before)
        osg::Vec3 uv_normal_plane = v_i_next ^ v_before_i;
        uv_normal_plane.normalize();
        // a vector from p_i to o
        osg::Vec3 uv_i_o = o - p_i;
        uv_i_o.normalize();

        // rotating p(i)-o C.C.W to find flexion constraint(left of pi_o)
        osg::Vec3 p_down = osg::Matrix::rotate(joint().constraint.abduction, uv_normal_plane) * uv_i_o;
        // rotating p(i)-o C.W to find extension constraint(right of pi_o )
        osg::Vec3 p_up = osg::Matrix::rotate(-joint().constraint.extension, uv_normal_plane) * uv_i_o;

        float l_before_i = (p_i - p_before).length();
        osg::Vec3 uv_i_before = (p_before - p_i) / l_before_i;
        float theta =  joint().constraint.abduction;// means it is down side of pi_o
        auto p = p_down;
        if ((uv_i_before ^ uv_i_o) * (uv_normal_plane) > 0)
        {
            theta = joint().constraint.extension; // means it is upper side of pi_o
            p = p_up;
        }
        
        if (theta != 0)
        {
            // angle between vec(i_before) and vec(i_o)
            float angle = acos(uv_i_before * uv_i_o);
            if (angle <= theta)
                return osg::Vec3(0, 0, 0);
            else //return nearest point
                return p_i + p * l_before_i; 
        }
        else//return nearest point
           return p_i + uv_i_o * l_before_i;
    }
        break;
    case Constraint::Type::Ball:
    {
        //semi ellipsoidal parameter qi (1,2,3,4)
        const auto &j = joint();
        float q1 = round(dotProd * tan(j.constraint.adduction), 3);
        float q2 = round(dotProd * tan(j.constraint.abduction), 3);
        float q3 = round(dotProd * tan(j.constraint.flexion), 3);
        float q4 = round(dotProd * tan(j.constraint.extension), 3);

        // change the coordinate to cross section of cone and calculating the (i-1)th position in it
        float l_o_next = (o - p_next).length();
        float si = round(m_si);
        size_t sector = 0;
        if(0 <= si && si < osg::PI_2)
            sector = 1;
        else if(osg::PI_2 <= si && si < osg::PI)
            sector = 2;
        else if(osg::PI <= si && si < 3 * osg::PI_2)
            sector = 3;
        else if(3 * osg::PI_2 <= si && si < 2 * osg::PI)
            sector = 4;
        
        float y_t = round(l_o_next * sin(m_si), 2);
        float x_t = round(l_o_next * cos(m_si), 2);
        constexpr float threashold = 0.1;
        // checking that the target point is in ellipsoidal shape
        if ((round((x_t * x_t) / (q3 * q3) + (y_t * y_t) / (q2 * q2)) <= 1 && sector == 1)
         || (round((x_t * x_t) / (q1 * q1) + (y_t * y_t) / (q2 * q2)) <= 1 && sector == 2)
         || (round((x_t * x_t) / (q1 * q1) + (y_t * y_t) / (q4 * q4)) <= 1 && sector == 3)
         || (round((x_t * x_t) / (q3 * q3) + (y_t * y_t) / (q4 * q4)) <= 1 && sector == 4))
        {
            return osg::Vec3(0, 0, 0);
        }
        
        //if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
        float x_nearest_point = 0, y_nearest_point = 0;
        switch (sector)
        {
        case 1:
        {
            if(std::abs(y_t) > threashold)
            {
                auto result = find_nearest_point(q3, q2, sector, x_t, y_t);
                x_nearest_point = result[0];
                y_nearest_point = result[1];
            } 
            else
            {
                x_nearest_point = q3;
                y_nearest_point = 0;
            }
        }
        break;
        case 2:
        {
            if(std::abs(x_t) > threashold)
            {
                auto result = find_nearest_point(q1, q2, sector, x_t, y_t);
                x_nearest_point = result[0];
                y_nearest_point = result[1];
            } 
            else
            {
                x_nearest_point = 0;
                y_nearest_point = q2;
            }
        }
        break;
        case 3:
        {
            if(std::abs(y_t) > threashold)
            {
                auto result = find_nearest_point(q1, q4, sector, x_t, y_t);
                x_nearest_point = result[0];
                y_nearest_point = result[1];
            } 
            else
            {
                x_nearest_point = -q1;
                y_nearest_point = 0;
            }
        }
        break;
        case 4:
        {
            if(std::abs(x_t) > threashold)
            {
                auto result = find_nearest_point(q3, q4, sector, x_t, y_t);
                x_nearest_point = result[0];
                y_nearest_point = result[1];
            } 
            else
            {
                x_nearest_point = 0;
                y_nearest_point = -q4;
            }
        }
        break;
        
        default:
            break;
        }
        // finding nearest point global coordinate in 3d
        float l_o_before = sqrt(x_t * x_t + y_t * y_t);
        float l_o_nearest_point = sqrt(x_nearest_point * x_nearest_point + y_nearest_point * y_nearest_point);
        float l_nearest_before = sqrt((x_t - x_nearest_point) * (x_t - x_nearest_point) + (y_nearest_point - y_t) * (y_nearest_point - y_t));

        // rotation angle between vector from o to p_before and o to p_nearest point
        float rot_angle = acos(std::round((l_nearest_before * l_nearest_before - l_o_before * l_o_before - l_o_nearest_point * l_o_nearest_point) / (-2 * l_o_before * l_o_nearest_point)));

        // rotating the vector from o to p_before around vector from p_next to o

        // a vector from o to p_before
        osg::Vec3 unit_vec_o_before = (p_before - o) / l_o_before;
        osg::Vec3 unit_vec_normal_plane = unit_vec_next_i;

        if (rot_angle == 0) // return nearest point in global coordinate
            return o + unit_vec_o_before * l_o_nearest_point;
        else
        {
            // to find out nearest point is on left or right side of target point
            float orient_near_to_target = x_t * y_nearest_point - x_nearest_point * y_t;
            int sign = -1;
            if (orient_near_to_target * unit_vec_next_i[2] > 0)// to find nearest point should rotate the uv(o-target) in C.C.W
                sign = 1;


            osg::Vec3 uv_o_nearest_point = osg::Matrix::rotate(sign *rot_angle, unit_vec_normal_plane) * unit_vec_o_before;
            osg::Vec3 p_nearest_point_in_global = o + uv_o_nearest_point * l_o_nearest_point;
            return p_nearest_point_in_global;
        }
    }
    default:
        break;
    }
    std::cerr << "Invalid constraint type" << std::endl;
    return osg::Vec3(0, 0, 0);
}

const Joint &Constraints::joint()
{
    return m_skeleton[m_body_indx[m_i]];
}



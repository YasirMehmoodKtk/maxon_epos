/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:42:41
 */

#include "EposProfileVelocityMode.hpp"

EposProfileVelocityMode::~EposProfileVelocityMode()
{}

void EposProfileVelocityMode::init(NodeHandle &node_handle)
{
    ControlModeBase::init(node_handle);
}

void EposProfileVelocityMode::activate()
{}

std::vector<int> EposProfileVelocityMode::read()
{
    return {0, 0, 0};
}

void EposProfileVelocityMode::write(const int position, const int velocity, const int current)
{}

/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:43:21
 */

#include "EposCurrentMode.hpp"


EposCurrentMode::~EposCurrentMode()
{}

void EposCurrentMode::init(NodeHandle &node_handle)
{
    ControlModeBase::init(node_handle);
}

void EposCurrentMode::activate()
{}

std::vector<int> EposCurrentMode::read()
{
    return {0, 0, 0};
}

void EposCurrentMode::write(const int position, const int velocity, const int current)
{}

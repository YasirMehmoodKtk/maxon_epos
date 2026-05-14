/**
 * @file   ControlModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-05 15:28:40
 */

#include "ControlModeBase.hpp"
#include <iostream>
/**
 * @brief Destructor
 */
ControlModeBase::~ControlModeBase()
{}

void ControlModeBase::init(NodeHandle &node_handle)
{
    m_epos_handle = node_handle;
}

std::vector<int> ControlModeBase::read()
{
    return {0, 0, 0};
}

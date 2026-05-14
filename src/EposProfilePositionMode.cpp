/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "EposProfilePositionMode.hpp"

#include <unistd.h>

EposProfilePositionMode::~EposProfilePositionMode()
{}

void EposProfilePositionMode::init(NodeHandle &node_handle)
{
    ControlModeBase::init(node_handle);
}

void EposProfilePositionMode::activate()
{
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
}

void EposProfilePositionMode::write(const int position, const int velocity, const int current)
{
    (void)velocity;
    (void)current;
    int quad_count = static_cast<int>(position);
    std::cout << "[EposProfilePositionMode] MoveToPosition target ticks: "
              << quad_count << std::endl;
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);

    try {
        int is_enabled = 0;
        int target_reached = 0;
        long target_position = 0;
        char operation_mode = 0;

        VCS_NODE_COMMAND(GetEnableState, m_epos_handle, &is_enabled);
        VCS_NODE_COMMAND(GetMovementState, m_epos_handle, &target_reached);
        VCS_NODE_COMMAND(GetTargetPosition, m_epos_handle, &target_position);
        VCS_NODE_COMMAND(GetOperationMode, m_epos_handle, &operation_mode);

        std::cout << "[EposProfilePositionMode] status: enabled=" << is_enabled
                  << ", op_mode=" << static_cast<int>(operation_mode)
                  << ", target_position=" << target_position
                  << ", target_reached=" << target_reached << std::endl;
    } catch (EposException &e) {
        std::cout << "[EposProfilePositionMode] status read failed: "
                  << e.what() << std::endl;
    }

    try {
        usleep(200000);

        int actual_position = 0;
        int actual_velocity = 0;
        short actual_current = 0;
        int target_reached = 0;

        VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &actual_position);
        VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &actual_velocity);
        VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &actual_current);
        VCS_NODE_COMMAND(GetMovementState, m_epos_handle, &target_reached);

        std::cout << "[EposProfilePositionMode] 200ms later: position="
                  << actual_position
                  << ", velocity=" << actual_velocity
                  << ", current=" << actual_current
                  << ", target_reached=" << target_reached
                  << std::endl;
    } catch (EposException &e) {
        std::cout << "[EposProfilePositionMode] delayed status read failed: "
                  << e.what() << std::endl;
    }
}

std::vector<int> EposProfilePositionMode::read()
{
    int positionls = 0, velocityls = 0;
    short currentls = 0;
    try{
        VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &positionls);
        VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &velocityls);
        VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &currentls);
    } catch (EposException &e) {
        std::cout << "Epos GetPositionIs error!" << std::endl;
        std::cout << e.what() << std::endl;
    }

    // if(m_use_ros_unit){
    //     // quad-counts of the encoder -> rad
    //     pos = (positionls / static_cast<double>(m_max_qc_)) * 2. * M_PI;
    //     // rpm -> rad/s
    //     vel = velocityls * M_PI / 30.;
    //     // mA -> A
    //     cur = currentls / 1000.;
    //     return;
    // }
    std::vector<int> output = {positionls, velocityls, currentls};
    return output;
}

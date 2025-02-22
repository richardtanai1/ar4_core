#include "ar3_hardware_drivers/TeensyDriver.h"

#include <thread>
#include <chrono>

#define FW_VERSION "0.0.1"

namespace ar3_hardware_drivers {

void TeensyDriver::init(std::string port, int baudrate, int num_joints, std::vector<double>& enc_steps_per_deg)
{
    // @TODO read version from config
    version_ = FW_VERSION;

    // establish connection with teensy board
    boost::system::error_code ec;
    serial_port_.open(port, ec);

    if (ec)
    {
        ROS_WARN("Failed to connect to serial port %s", port.c_str());
        return;
    }
    else
    {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        ROS_INFO("Successfully connected to serial port %s", port.c_str());
    }
    
    initialised_ = false;
    std::string msg = "STA" + version_ + "\n";

    while (!initialised_)
    {
        ROS_INFO("Waiting for response from Teensy on port %s", port.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        exchange(msg);
    }
    ROS_INFO("Successfully initialised driver on port %s", port.c_str());

    // initialise joint and encoder calibration
    num_joints_ = num_joints;
    enc_commands_.resize(num_joints_);
    enc_steps_.resize(num_joints_);
    enc_steps_per_deg_.resize(num_joints_);
    enc_calibrations_.resize(num_joints_);
    joint_positions_.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        enc_steps_per_deg_[i] = enc_steps_per_deg[i];
    }

    // get current joint positions
    // msg = "JP\n";
    // msg = "RP\n";
    // exchange(msg);
    // for (int i = 0; i < num_joints_; ++i)
    // {
    //     enc_commands_[i] = enc_steps_[i];
    // }
}

TeensyDriver::TeensyDriver()
: serial_port_(io_service_)
{
}

void TeensyDriver::setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel)
{
    std::string outMsg = "SS";
    for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2)
    {
        outMsg += 'A' + charIdx;
        outMsg += std::to_string(max_speed[i]);
        outMsg += 'A' + charIdx + 1;
        outMsg += std::to_string(max_accel[i]);
    }
    outMsg += "\n";
    exchange(outMsg);
}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
{
    // get updated position commands
    //jointPosToEncSteps(pos_commands, enc_commands_);

    // construct update message
    // std::string outMsg = "MT";
    // for (int i = 0; i < num_joints_; ++i)
    // {
    //     outMsg += 'A' + i;
    //     outMsg += std::to_string(enc_commands_[i]);
    // }
    // outMsg += "\n";

    std::string outMsg = "RJ";
    for (int i = 0; i < num_joints_; ++i)
    {
        outMsg += 'A' + i;
        outMsg += std::to_string(pos_commands[i]);
    }
    outMsg += "G0.0Sp25Ac10Dc10Rm50WN";
    outMsg += "\n";


    // run the communication with board
    exchange(outMsg);

    // return updated joint_positions
    //encStepsToJointPos(enc_steps_ , joint_positions);
    copytoJointPos(joint_positions);
}

void TeensyDriver::calibrateJoints()
{
    //std::string outMsg = "JC\n";
    std::string outMsg = "LLA1B1C1D1E1F1G0.0H-2.0I0.0J2.5K-2.0L2.5\n";
    sendCommand(outMsg);
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions)
{
    // get current joint positions
    //std::string msg = "JP\n";
    std::string msg = "RP\n";
    exchange(msg);
    //encStepsToJointPos(enc_steps_, joint_positions);
    copytoJointPos(joint_positions);
}

// Send specific commands
void TeensyDriver::sendCommand(std::string outMsg)
{
    exchange(outMsg);
}

// Send msg to board and collect data
void TeensyDriver::exchange(std::string outMsg)
{
    std::string inMsg;
    std::string errTransmit = "";
    std::string errReceive = "";
    
    if (!transmit(outMsg, errTransmit))
    {
        // print err
    }

    if (!receive(inMsg, errReceive))
    {
        // print err
    }
    // parse msg
    std::string header1 = inMsg.substr(0, 1);
    std::string header2 = inMsg.substr(0, 2);
    if (header1 == "1")
    {
        ROS_ERROR("Joint 1 Calibration Failed");
    }
    else if (header1 == "2")
    {
        ROS_ERROR("Joint 2 Calibration Failed");
    }
    else if (header1 == "3")
    {
        ROS_ERROR("Joint 3 Calibration Failed");
    }
    else if (header1 == "4")
    {
        ROS_ERROR("Joint 4 Calibration Failed");
    }
    else if (header1 == "5")
    {
        ROS_ERROR("Joint 5 Calibration Failed");
    }
    else if (header1 == "6")
    {
        ROS_ERROR("Joint 6 Calibration Failed");
    }
    else if (header2 == "ER")
    {
        // init acknowledgement
        ROS_ERROR("Kinematic Error");
    }
    else if (header2 == "EL")
    {
        // init acknowledgement
        ROS_ERROR("Requested Position Out of Limits");
    }
    else if (header1 == "A")
    {
        // init acknowledgement
        // checkInit(inMsg);
        updateJointAngles(inMsg);
    }
    else if (header2 == "ST")
    {
        // init acknowledgement
        checkInit(inMsg);   
    }
    else if (header2 == "JC")
    {
        // encoder calibration values
        updateEncoderCalibrations(inMsg);
    }
    else if (header2 == "JP")
    {
        // encoder steps
        updateEncoderSteps(inMsg);
    }
    else
    {
        // unknown header
        ROS_WARN("Unknown header %s", header2);
    } 
}

bool TeensyDriver::transmit(std::string msg, std::string& err)
{
    boost::system::error_code ec;
    const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

    boost::asio::write(serial_port_, sendBuffer, ec);

    if(!ec)
    {
        return true;
    }
    else
    {
        err = "Error in transmit";
        return false;
    }    
}

bool TeensyDriver::receive(std::string& inMsg, std::string& err)
{
    char c;
    std::string msg = "";
    bool eol = false;
    while (!eol)
    {
        boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                eol = true;
            default:
                msg += c;
        }
    }
    inMsg = msg;
    return true;
}

void TeensyDriver::checkInit(std::string msg)
{
    std::size_t ack_idx = msg.find("A", 2) + 1;
    std::size_t version_idx = msg.find("B", 2) + 1;
    int ack = std::stoi(msg.substr(ack_idx, version_idx));
    if (ack)
    {
        initialised_ = true;
    }
    else
    {
        std::string version = msg.substr(version_idx);
        ROS_ERROR("Firmware version mismatch %s", version);
    }  
}

void TeensyDriver::updateEncoderCalibrations(std::string msg)
{
    size_t idx1 = msg.find("A", 2) + 1;
    size_t idx2 = msg.find("B", 2) + 1;
    size_t idx3 = msg.find("C", 2) + 1;
    size_t idx4 = msg.find("D", 2) + 1;
    size_t idx5 = msg.find("E", 2) + 1;
    size_t idx6 = msg.find("F", 2) + 1;
    enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
    enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
    enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
    enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
    enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
    enc_calibrations_[5] = std::stoi(msg.substr(idx6));

    // @TODO update config file
    ROS_INFO("Successfully updated encoder calibrations");
}

void TeensyDriver::updateEncoderSteps(std::string msg)
{
    size_t idx1 = msg.find("A", 2) + 1;
    size_t idx2 = msg.find("B", 2) + 1;
    size_t idx3 = msg.find("C", 2) + 1;
    size_t idx4 = msg.find("D", 2) + 1;
    size_t idx5 = msg.find("E", 2) + 1;
    size_t idx6 = msg.find("F", 2) + 1;
    enc_steps_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
    enc_steps_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
    enc_steps_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
    enc_steps_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
    enc_steps_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
    enc_steps_[5] = std::stoi(msg.substr(idx6));
}

void TeensyDriver::updateJointAngles(std::string msg)
{
    size_t idx1 = msg.find("A");
    size_t idx2 = msg.find("B");
    size_t idx3 = msg.find("C");
    size_t idx4 = msg.find("D");
    size_t idx5 = msg.find("E");
    size_t idx6 = msg.find("F");
    size_t idx7 = msg.find("G");
    joint_positions_[0] = std::stod(msg.substr(idx1 +1, idx2 - 1));
    joint_positions_[1] = std::stod(msg.substr(idx2 +1, idx3 - 1));
    joint_positions_[2] = std::stod(msg.substr(idx3 +1, idx4 - 1));
    joint_positions_[3] = std::stod(msg.substr(idx4 +1, idx5 - 1));
    joint_positions_[4] = std::stod(msg.substr(idx5 +1, idx6 - 1));
    joint_positions_[5] = std::stod(msg.substr(idx6 +1, idx7 - 1));
}



void TeensyDriver::encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions)
{
    for (int i = 0; i < enc_steps.size(); ++i)
    {
        // convert enc steps to joint deg
        joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
    }
}

void TeensyDriver::copytoJointPos(std::vector<double>& joint_positions)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        joint_positions[i] = joint_positions_[i];
    }
}

void TeensyDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        // convert joint deg to enc steps
        enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
    }
}

} // namespace ar3_hardware_drivers

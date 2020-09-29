#include <mc_control/ControllerClient.h>

#include <iostream>
#include <sstream>
#include <unordered_map>

struct CLIClient : public mc_control::ControllerClient
{
  CLIClient()
  : mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc")
  {
  }

  void category(const std::vector<std::string> & /*parent*/, const std::string & /*category*/) override {}

  bool set_joint_angle(std::stringstream & args)
  {
    std::string joint;
    double angle = 0;
    args >> joint;
    args >> angle;
    if(!args)
    {
      std::cerr << "Requires two arguments: joint name (string), angle (double)\n";
      return false;
    }
    this->send_request({{"Tasks", "FSM_posture_hrp5_p", "Target"}, joint}, angle);
    return true;
  }

  bool set_posture_weight(std::stringstream & args)
  {
    double weight = 0;
    args >> weight;
    if(!args)
    {
      std::cerr << "Requires one arguments: weight (double)\n";
      return false;
    }
    this->send_request({{"Tasks", "FSM_posture_hrp5_p", "Gains"}, "weight"}, weight);
    return true;
  }

  bool set_posture_stiffness(std::stringstream & args)
  {
    double stiffness = 0;
    args >> stiffness;
    if(!args)
    {
      std::cerr << "Requires one arguments: stiffness (double)\n";
      return false;
    }
    this->send_request({{"Tasks", "FSM_posture_hrp5_p", "Gains"}, "stiffness"}, stiffness);
    return true;
  }

  bool move_com_above(std::stringstream & args)
  {
    std::string target = "";
    args >> target;
    if(!args)
    {
      std::cerr << "Requires one argument: LeftFoot, RightFoot or Center\n";
      return false;
    }
    if(target == "LeftFoot")
    {
      target = "Left foot";
    }
    else if(target == "RightFoot")
    {
      target = "Right foot";
    }
    this->send_request({{"FSM", "Stabilizer::Standing", "Move"}, target});
    return true;
  }

  bool set_com_xyz(std::stringstream & args)
  {
    Eigen::Vector3d target;
    args >> target.x() >> target.y() >> target.z();
    if(!args)
    {
      std::cerr << "Requires three arguments: x y z\n";
      return false;
    }
    this->send_request({{"FSM", "Stabilizer::Standing", "Move"}, "CoM Target"}, target);
    return true;
  }

  void run()
  {
    bool running = true;
    using callback_t = bool(CLIClient::*)(std::stringstream &);
    using fun_t = std::pair<callback_t, std::string>;
    std::unordered_map<std::string, fun_t> callbacks =
    {
      {"set_joint_angle", {&CLIClient::set_joint_angle, "Set joint angle (joint name, angle)"}},
      {"set_posture_weight", {&CLIClient::set_posture_weight, "Set posture weight (weight)"}},
      {"set_posture_stiffness", {&CLIClient::set_posture_stiffness, "Set posture stiffness (stiffness)"}},
      {"move_com_above", {&CLIClient::move_com_above, "Move CoM above a surface (LeftFoot, RightFoot, Center) using the stabilizer state"}},
      {"set_com_xyz", {&CLIClient::set_com_xyz, "Move CoM to a world (x,y,z) position using the stabilizer state"}}
    };
    auto help = [&callbacks]()
    {
      std::cout << "\nsimple_cli_client - Simple tool to demonstrate implementation of a CLI client to set targets/gains of the posture task" << std::endl << std::endl;
        for(const auto & cb : callbacks)
        {
          std::cout << "\t- " << cb.first << " -- " << cb.second.second << "\n";
        }
        std::cout << "\t- help, h -- Display this message\n" << std::endl;
    };
    help();
    while(running)
    {
      std::string ui;
      std::cout << "(command) " << std::flush;
      std::getline(std::cin, ui);
      std::stringstream ss;
      ss << ui;
      std::string token;
      ss >> token;
      if(token == "stop")
      {
        running = false;
      }
      else if(token == "help" || token == "h")
      {
        help();
      }
      else if(callbacks.count(token))
      {
        auto fn = callbacks[token].first;
        bool res = (this->*fn)(ss);
        if(!res)
        {
          mc_rtc::log::error("Invokation of command {} failed", callbacks[token].second);
        }
      }
      else
      {
        std::cerr << "No such functions, type help to get a list of available commands\n";
      }
    }
  }
private:
};

int main(int, char * [])
{
  CLIClient client;
  client.run();
  return 0;
}

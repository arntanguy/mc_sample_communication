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
    this->send_request({{"Tasks", "FSM_posture_jvrc1", "Target"}, joint}, angle);
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
    this->send_request({{"Tasks", "FSM_posture_jvrc1", "Gains"}, "weight"}, weight);
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
    this->send_request({{"Tasks", "FSM_posture_jvrc1", "Gains"}, "stiffness"}, stiffness);
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
      {"set_posture_stiffness", {&CLIClient::set_posture_stiffness, "Set posture stiffness (stiffness)"}}
    };
    auto help = [&callbacks]()
    {
      std::cout << "\nposture_cli_client - Simple tool to demonstrate implementation of a CLI client to set targets/gains of the posture task" << std::endl << std::endl;
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

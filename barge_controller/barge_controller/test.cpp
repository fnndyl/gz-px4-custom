#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/MessageInfo.hh>

using namespace gz;
using namespace sim;

class OdometrySubscriber : public System,
                           public ISystemConfigure
{
public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                 EntityComponentManager & /*_ecm*/,
                 EventManager & /*_eventMgr*/) override
  {
    std::string topic = "/x500/odom";

    // Subscribe to odometry using member function callback
    if (!this->node.Subscribe<msgs::Odometry>(
            topic,
            &OdometrySubscriber::OdometryCallback,
            this))
    {
      gzerr << "Failed to subscribe to topic [" << topic << "]" << std::endl;
    }
    else
    {
      gzdbg << "Subscribed to [" << topic << "]" << std::endl;
    }
  }

private:
  /// \brief Callback for odometry messages
  void OdometryCallback(const msgs::Odometry &_msg,
                        const transport::MessageInfo &/*_info*/)
  {
    gzdbg << "Received odometry:\n" << _msg.DebugString() << std::endl;
  }

  transport::Node node;
};

GZ_ADD_PLUGIN(OdometrySubscriber,
              System,
              ISystemConfigure)
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginMyRobot : public WorldPlugin 
  {
    public : WorldPluginMyRobot() : WorldPlugin()
    {
      printf("Welcome to Federico's World!\n");
    }
    
    public : void Load(physics::WorldPtr _ptr, sdf::ElementPtr _sdf)
    {
    
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}

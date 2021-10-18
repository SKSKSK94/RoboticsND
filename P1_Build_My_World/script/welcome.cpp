#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginMyRobot : public WorldPlugin
  {
    public: WorldPluginMyRobot() : WorldPlugin()
    {
        printf("Welcome to myOffice’s World!\n");
    }

		// This load function is mandatory and should always be included
		// as it receives information from world file
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
    }
  }; 

  // register the plugin with simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
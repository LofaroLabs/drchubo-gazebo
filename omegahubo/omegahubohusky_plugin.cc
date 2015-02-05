#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <time.h>
#include <math.h>

/* Required Hubo Headers */
#include <hubo.h>

// For Ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>

// ach channels
ach_channel_t chan_hubo_state;      // hubo-ach
ach_channel_t chan_hubo_ref;

int i = 0;
//int r  = 0;

namespace gazebo
{   
  class DrcOmegaHuboHuskyPlugin : public ModelPlugin
  {

    // Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
    public: void cb(ConstWorldStatisticsPtr &_msg)
    {
       gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
       //size_t size;
       double ttime = simTime.Double();
       //ach_put(&chan_time, _msg->image().data().c_str() , _msg->image().data().size());

    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {
      // Open Ach channel
     // Open Ach channel
        /* open ach channel */
      int r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
      assert( ACH_OK == r );
      r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
      assert( ACH_OK == r );

      size_t fs;
      hubo_ref_t H_ref;
      memset( &H_ref,   0, sizeof(H_ref));

      H_ref.ref[LEB] = -1.1;
      H_ref.ref[REB] = -1.1;

      ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));



//      r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
//      assert( sizeof(H_ref) == fs);

      // Store the pointer to the model
      this->model = _parent;

      // Get then name of the parent model
//      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
//      std::string worldName = _sdf->GetName();
      this->world = physics::get_world("default");


      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DrcOmegaHuboHuskyPlugin::OnUpdate, this));
      }

      // subscribe to thread
//      gazebo::transport::NodePtr node(new gazebo::transport::Node());
//      node->Init();
//      gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/world_stats", cb);
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) //assigning joints that are defined .sdf file
    {
      if (this->FindJointByParam(_sdf, this->joint_back_left_,  "backLeftJoint") &&
          this->FindJointByParam(_sdf, this->joint_back_right_, "backRightJoint") &&
          this->FindJointByParam(_sdf, this->joint_front_left_, "frontLeftJoint") &&
          this->FindJointByParam(_sdf, this->joint_front_right_,"frontRightJoint"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double maxTorque = 500;
      double iniAngle = 0;
      //double posRSP = 0;


      /* Create initial structures to read and write from */
      struct hubo_state H_state;
      struct hubo_ref H_ref;
      memset( &H_state, 0, sizeof(H_state));
      memset( &H_ref, 0, sizeof(H_ref));

      /* for size check */
      size_t fs;
      int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_COPY );
//      assert( sizeof(H_ref) == fs);

      /* Get the current feed-forward (state) */
//      int r = ach_get(&chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
//      if(ACH_OK != r) {
//          assert( sizeof(H_state) == fs );
//      }

      this->joint_back_left_->SetMaxForce(0, maxTorque);
      this->joint_back_right_->SetMaxForce(0, maxTorque);
      this->joint_front_left_->SetMaxForce(0, maxTorque);
      this->joint_front_right_->SetMaxForce(0, maxTorque);

      this->joint_back_left_->SetVelocity(0, H_ref.ref[LAP]);
      this->joint_back_right_->SetVelocity(0, H_ref.ref[RAP]);
      this->joint_front_left_->SetVelocity(0, H_ref.ref[LAR]);
      this->joint_front_right_->SetVelocity(0, H_ref.ref[RAR]);
/*
      this->joint_back_left_->SetAngle(0, H_ref.ref[LAP]);
      this->joint_back_right_->SetAngle(0, H_ref.ref[RAP]);
      this->joint_front_left_->SetAngle(0, H_ref.ref[LAR]);
      this->joint_front_right_->SetAngle(0, H_ref.ref[RAR]);
*/

    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr joint_back_left_;
    private: physics::JointPtr joint_back_right_;
    private: physics::JointPtr joint_front_left_;
    private: physics::JointPtr joint_front_right_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DrcOmegaHuboHuskyPlugin)
}

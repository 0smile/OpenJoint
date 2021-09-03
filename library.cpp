#include "library.h"
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/common/Profiler.hh>
#include <iostream>
using namespace ignition;
using namespace gazebo;

//To enable the attachment, use ign topic -t "/model/vehicle_blue/open_joint/attach" -m ignition.msgs.Empty -p "unused: true"
//To disable the attachment, use ign topic -t "/model/vehicle_blue/open_joint/detach" -m ignition.msgs.Empty -p "unused: true"
//The contactTopic should be the same as the contact sensor's topic, find it in your SDF.
void
mySystem::OpenJoint::Configure(const ignition::gazebo::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                               ignition::gazebo::EntityComponentManager &_ecm,
                               ignition::gazebo::EventManager &_eventMgr) {
    this->model = Model(_entity);
    auto selfLinkName = _sdf->Get<std::string>("self_link");
    this->selfLinkEntity = this->model.LinkByName(_ecm, selfLinkName);

    if (_sdf->HasElement("detach_topic"))
    {
        this->detachTopic = _sdf->Get<std::string>("detach_topic");
    }
    igndbg << "Model name: " << this->model.Name(_ecm) << std::endl;
    this->detachTopic = "/model/"+this->model.Name(_ecm)+"/open_joint/detach";
    this->contactTopic = _sdf->Get<std::string>("contact_topic");
    this->attachTopic = "/model/"+this->model.Name(_ecm)+"/open_joint/attach";
    ignmsg << "Configure set." << std::endl;
    this->node.Subscribe(
            this->contactTopic, &OpenJoint::OnContact, this);
    this->node.Subscribe(
            this->detachTopic, &OpenJoint::OnDetachRequest, this);
    this->node.Subscribe(
            this->attachTopic, &OpenJoint::OnAttachRequest, this);
}

void mySystem::OpenJoint::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) {
    if (this->collisionEntity != kNullEntity && this->attachRequested)
    {
        if (this->detachableJointEntity == kNullEntity)
        {
            this->collisionLinkEntity = _ecm.ParentEntity(this->collisionEntity);
            this->detachableJointEntity = _ecm.CreateEntity();
            _ecm.CreateComponent(
                    this->detachableJointEntity,
                    components::DetachableJoint({this->selfLinkEntity,
                                                 this->collisionLinkEntity, "fixed"}));
            igndbg << "Creating entity: " << this->detachableJointEntity << std::endl;
        }
    }
    if (this->detachRequested  && (this->detachableJointEntity != kNullEntity))
    {
        igndbg << "Removing entity: " << this->detachableJointEntity << std::endl;
        _ecm.RequestRemoveEntity(this->detachableJointEntity);
        this->detachableJointEntity = kNullEntity;
        this->detachRequested = false;
        this->attachRequested = false;
    }
}

void mySystem::OpenJoint::OnContact(const msgs::Contacts &_msg) {
    auto id = _msg.contact(0).collision2().id();
    this->collisionEntity = id;
}

void mySystem::OpenJoint::OnDetachRequest(const msgs::Empty &_msg) {
    this->detachRequested = true;
}

void mySystem::OpenJoint::OnAttachRequest(const msgs::Empty &_msg) {
    this->attachRequested = true;
}

IGNITION_ADD_PLUGIN(
        mySystem::OpenJoint,
        ignition::gazebo::System,
        mySystem::OpenJoint::ISystemConfigure,
        mySystem::OpenJoint::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(mySystem::OpenJoint,
                          "mySystem::OpenJoint")
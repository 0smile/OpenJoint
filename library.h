#ifndef OPENJOINT_LIBRARY_H
#define OPENJOINT_LIBRARY_H

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/transport/Node.hh>
namespace mySystem
{
    class OpenJoint:
            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPreUpdate
    {
    public:
        OpenJoint() = default;
        void Configure(
                const ignition::gazebo::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm,
                ignition::gazebo::EventManager &_eventMgr
        ) override;
        void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm
        );
    private:
        void OnContact(const ignition::msgs::Contacts &_msg);
        void OnDetachRequest(const ignition::msgs::Empty &_msg);
        void OnAttachRequest(const ignition::msgs::Empty &_msg);
        ignition::gazebo::Model model;
        ignition::msgs::Contacts contactsMsg;
        ignition::transport::Node node;
        std::string selfLinkName;
        std::string contactTopic;
        std::string detachTopic;
        std::string attachTopic;
        ignition::gazebo::Entity collisionEntity{ignition::gazebo::kNullEntity};
        ignition::gazebo::Entity collisionLinkEntity{ignition::gazebo::kNullEntity};
        ignition::gazebo::Entity selfLinkEntity{ignition::gazebo::kNullEntity};
        ignition::gazebo::Entity detachableJointEntity{ignition::gazebo::kNullEntity};
        std::atomic<bool> detachRequested{false};
        std::atomic<bool> attachRequested{false};
    };
}

#endif //OPENJOINT_LIBRARY_H

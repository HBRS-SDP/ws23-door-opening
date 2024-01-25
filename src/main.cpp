#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

#include <InterconnectConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <thread>
#include <iostream>
#include <chrono>

#define PORT 10000

namespace k_api = Kinova::Api;

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

// Data structure to represent a 3D pose
struct Pose {
    double x;
    double y;
    double z;
    double theta_x;
    double theta_y;
    double theta_z;
};


// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;
        }
    };
}

bool move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

        return true;
    }
}


bool move_to_cartesian_position(k_api::Base::BaseClient* base, const Pose& targetPose, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // std::cout << "Moving to Desired position" << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Move to Cartesian Position");
    action.set_application_data("");
    
    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    std::cout << "X value : " << &targetPose << std::endl;
    pose->set_x(targetPose.x);
    pose->set_y(targetPose.y);
    // std::cout << "X value : " << Pose& targetPose << std::endl;
    pose->set_z(targetPose.z);
    pose->set_theta_x(targetPose.theta_x);
    pose->set_theta_y(targetPose.theta_y);
    pose->set_theta_z(targetPose.theta_z);

    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    std::cout << "Cartesian movement completed" << std::endl;
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;

}


// Gripper Commands

 bool Open(k_api::Base::BaseClient* base, bool m_is_init)
    {
        if (m_is_init == false)
        {
            return false;
        }
        std::cout << "Performing gripper test in position..." << std::endl;

        k_api::Base::GripperCommand gripper_command;

        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);

        auto finger = gripper_command.mutable_gripper()->add_finger();
        //finger->set_finger_identifier(1);
        // for (float position = 0.0; position < 1.0; position += 0.9)
        // {
        //     std::cout << "Setting position to " << position << std::endl;
        //     finger->set_value(position);
        //     base->SendGripperCommand(gripper_command);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // }

        std::cout << "Opening gripper using speed command..." << std::endl;
        gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
        finger->set_value(0.3);
        base->SendGripperCommand(gripper_command);
        k_api::Base::Gripper gripper_feedback;
        k_api::Base::GripperRequest gripper_request;
        bool is_motion_completed = false;
        gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
        while(!is_motion_completed)
        {
            float position =0.0f;
            gripper_feedback = base->GetMeasuredGripperMovement(gripper_request);

            if (gripper_feedback.finger_size())
            {
                position = gripper_feedback.finger(0).value();
                cout << "Reported position : " << position << std::endl;
            }

            if (position < 0.01f)
            {
                is_motion_completed = true;
            }
        }
        return true;

    }

bool Close(k_api::Base::BaseClient* base, bool m_is_init)
    {
        if (m_is_init == false)
            {
                return false;
            }

            std::cout << "Closing gripper using speed command..." << std::endl;

            k_api::Base::GripperCommand gripper_command;
            gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);

            auto finger = gripper_command.mutable_gripper()->add_finger();
            finger->set_finger_identifier(1);
            finger->set_value(-0.2);

            base->SendGripperCommand(gripper_command);

            k_api::Base::Gripper gripper_feedback;
            k_api::Base::GripperRequest gripper_request;
            bool is_motion_completed = false;
            std::cout<< "##" << is_motion_completed << "##" << std::endl;

           gripper_request.set_mode(k_api::Base::GRIPPER_SPEED);
        while(!is_motion_completed)
        {

            float position =0.00000f;
            float speed = 0.0;
            gripper_feedback = base->GetMeasuredGripperMovement(gripper_request);
            if (gripper_feedback.finger_size())
            {
                position = gripper_feedback.finger(0).value();
                cout << "Reported position for closing : " << position << std::endl;
                speed = gripper_feedback.finger(0).value();
                cout << "Reported speed : " << speed  << std::endl;
            }

            if (speed ==0.0f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                speed = gripper_feedback.finger(0).value();
                cout << "in the loop " << std::endl;
                cout << "Reported speed : " << speed  << std::endl;
                if (speed==0.0f)
                    {
                        is_motion_completed=true;
                    }
                    
            }

        }

            return true;
        
    }



int main(int argc, char **argv)
{


    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect("192.168.1.12", PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_feedback = new k_api::BaseCyclic::BaseCyclicClient(router);

    // Set the target pose
    // Pose targetPose = {0.458,-0.654, 0.848,-136.63,174.389,176.335 };
    Pose initialPose= {0.612,-0.221,0.406,163.045,140.432,40.025};
    Pose towardsTheHandle={0.395,-0.415,0.699,173.641,115.672,31.338};
    Pose endeffectorClosed={0.395,-0.415,0.699,173.64,115.677,31.341}; //interchanged check
    Pose doorUnlacthed={0.378,-0.418,0.709,175.468,122.039,25.039}; 
    // Pose grabingTheHandle ={0.398,-0.383,0.731,178.561,115.921,38.9};
    Pose doorOpening={0.358,-0.52,0.249,-40.515,63.957,169.502};
    Pose doorUnlatching ={0.383,-0.392,0.715,178.822,116.459,34.002};
    Pose doorOpen1={0.483,-0.408,0.147,131.609,125.552,1.361};
    Pose doorOpen2={0.642,-0.695,0.611,132.096,124.614,72.578};
    
    
    // core
    bool m_is_init=true;
    bool success = true;
    success &= move_to_home_position(base);
    success &= move_to_cartesian_position(base, initialPose, base_feedback);
    std::cout << "Home position is done" << std::endl;
    success &=Open(base,m_is_init);
    // success &= move_to_cartesian_position(base, targetPose, base_feedback);
    std::cout << "move_to_cartesian_position is done" << std::endl;
    success &= move_to_cartesian_position(base, towardsTheHandle, base_feedback);
    std::cout << "Moved towarddds the handle" << std ::endl;
    success &= move_to_cartesian_position(base, endeffectorClosed, base_feedback);
    success &=Close(base,m_is_init);
    std::cout << "endeffector is closed" << std ::endl;
    success &= move_to_cartesian_position(base, doorUnlacthed, base_feedback);
    std::cout << "door is unlatched" << std ::endl;
    success &= move_to_cartesian_position(base, doorUnlatching, base_feedback);
    std ::cout <<"The door has been unlacted"<<std :: endl;
    success &= move_to_cartesian_position(base, doorOpening, base_feedback);
    std ::cout <<"The unlatched door is being opened"<<std :: endl;
    success &=Open(base,m_is_init);
    success &= move_to_cartesian_position(base, doorOpen1, base_feedback);
    std ::cout <<"First half of opening is done"<<std :: endl;
    success &= move_to_cartesian_position(base, doorOpen2, base_feedback);
    std ::cout <<"Opening is completed"<<std :: endl;

    // You can also refer to the 110-Waypoints examples if you want to execute
    // a trajectory defined by a series of waypoints in joint space or in Cartesian space
    
    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success? 0: 1;
}

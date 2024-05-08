/**
 * \file EnvireMotorsPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireMotorsPlugins.hpp"

#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/JointInterface.h>

#include <mars_interfaces/Logging.hpp>

#include <mars_interfaces/MotorData.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/MotorManagerInterface.h>


namespace mars
{
    namespace envire_motors
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireMotorsPlugins::EnvireMotorsPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager},
            envireGraph{ControlCenter::envireGraph},
            graphTreeView{ControlCenter::graphTreeView},
            motors{ControlCenter::motors}
        {
            init();
        }

        EnvireMotorsPlugins::EnvireMotorsPlugins(lib_manager::LibManager *theManager,
                                                 std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                                 std::shared_ptr<envire::core::TreeView> graphTreeView,
                                                 std::shared_ptr<interfaces::MotorManagerInterface> motors) :
            lib_manager::LibInterface{theManager},
            envireGraph{envireGraph},
            graphTreeView{graphTreeView},
            motors{motors}
        {
            init();
        }

        void EnvireMotorsPlugins::init(void)
        {
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::DC>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::PID>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::DirectEffort>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<core::SimMotor>>>::subscribe(envireGraph.get());
        }

        EnvireMotorsPlugins::~EnvireMotorsPlugins()
        {
        }

        // TODO: this should be moved out from here
        std::shared_ptr<SubControlCenter> EnvireMotorsPlugins::getControlCenter(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const auto& vertex = envireGraph->vertex(frame);
                const auto& parentVertex = graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        const auto& it = envireGraph->getItem<SubControlItem>(frame);
                        return it->getData();
                    }
                    catch (...)
                    {
                    }
                }
                else
                {
                    done = true;
                }
            }
            return nullptr;
        }

        // TODO use template method
        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::DC>>& e)
        {
            auto& motor = e.item->getData();
            auto config = motor.getFullConfigMap();
            createMotor(config, e.frame);
        }

        // TODO use template method
        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::PID>>& e)
        {
            auto& motor = e.item->getData();
            auto config = motor.getFullConfigMap();
            createMotor(config, e.frame);
        }

        // TODO use template method
        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::DirectEffort>>& e)
        {
            auto& motor = e.item->getData();
            auto config = motor.getFullConfigMap();
            createMotor(config, e.frame);
        }

        void EnvireMotorsPlugins::itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<std::shared_ptr<core::SimMotor>>>& e)
        {
            const auto& motorID = ControlCenter::motors->getID(e.item->getData()->getName());
            ControlCenter::motors->removeMotor(motorID);
        }

        void EnvireMotorsPlugins::createMotor(configmaps::ConfigMap &config, const std::string &frameId)
        {
            using JointInterfaceItemItr = envire::core::EnvireGraph::ItemIterator<envire::core::Item<JointInterfaceItem>>;

            auto subControl = getControlCenter(frameId);
            if (!subControl)
            {
                return;
            }

            // TODO: we will not use prefix, when we move to base envire types

            LOG_INFO("Add motor item");
            MotorData motorData;
            motorData.fromConfigMap(&config, "");

            // there should be one joint and one motor in the same frame
            const auto num_joints = envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(frameId);
            if (num_joints == 0)
            {
                const auto errmsg = std::string{"Can not create a motor, since the frame " + frameId + " does not contain any joint interface item."};
                LOG_ERROR("%s", errmsg.c_str());
                return;
            } 
            else if (num_joints > 1)
            {
                const auto errmsg = std::string{"Can not create a motor, since there are multiple joint interface items in the frame " + frameId + "."};
                LOG_ERROR("%s", errmsg.c_str());
                return;
            }

            auto jointItemItr = envireGraph->getItem<envire::core::Item<JointInterfaceItem>>(frameId);
            const auto joint = jointItemItr->getData().jointInterface;
            if (!joint)
            {
                const auto errmsg = std::string{"Can not create motor, there is a joint interface item in the frame " + frameId + ", but joint interface is not set."};
                LOG_ERROR("%s", errmsg.c_str());
                return;
            }

            std::string jointName;
            joint->getName(&jointName);
            if (jointName != motorData.jointName)
            {
                const auto errmsg = std::string{"Can not create a motor, since the found joint interface does not correposponded to the motor by its name. Joint name required by motor: " + motorData.jointName + ". Found joint name: " + jointName};
                LOG_ERROR("%s", errmsg.c_str());
                return;
            }
            motorData.jointIndex = ControlCenter::jointIDManager->getID(jointName);

            // TODO: add MotorInterface and store it in the graph instead of SimMotor
            const auto& motorId = motors->addMotor(&motorData);
            // TODO: we should replace SimMotor by MotorInterface how it was done for joints
            const std::shared_ptr<core::SimMotor> motor{motors->getSimMotor(motorId)};
            envire::core::Item<std::shared_ptr<mars::core::SimMotor>>::Ptr motorItemPtr{new envire::core::Item<std::shared_ptr<mars::core::SimMotor>>(motor)};
            envireGraph->addItemToFrame(frameId, motorItemPtr);
        }
    } // end of namespace envire_motors
} // end of namespace mars


DESTROY_LIB(mars::envire_motors::EnvireMotorsPlugins);
CREATE_LIB(mars::envire_motors::EnvireMotorsPlugins);

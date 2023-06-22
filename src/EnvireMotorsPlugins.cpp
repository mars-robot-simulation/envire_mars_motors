/**
 * \file EnvireMotorsPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireMotorsPlugins.hpp"

//#include "PhysicsMapper.h"

#include <mars/utils/mathUtils.h>
#include <mars/utils/misc.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/JointInterface.h>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_interfaces/Logging.hpp>

#include <mars_interfaces/MotorData.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/MotorManagerInterface.h>

#define SIM_CENTER_FRAME_NAME "world"
typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace envire_motors
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireMotorsPlugins::EnvireMotorsPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface(theManager)
        {
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::DC>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::PID>>::subscribe(ControlCenter::envireGraph.get());
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
                const envire::core::GraphTraits::vertex_descriptor vertex = ControlCenter::envireGraph->vertex(frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = ControlCenter::envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        envire::core::EnvireGraph::ItemIterator<SubControlItem> it = ControlCenter::envireGraph->getItem<SubControlItem>(frame);
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

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::DC>>& e)
        {
            envire::base_types::motors::DC& motor = e.item->getData();
            ConfigMap config = motor.getFullConfigMap();
            createMotor(config, e.frame);
        }

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::PID>>& e)
        {
            envire::base_types::motors::PID& motor = e.item->getData();
            ConfigMap config = motor.getFullConfigMap();
            createMotor(config, e.frame);
        }

        void EnvireMotorsPlugins::createMotor(configmaps::ConfigMap &config, const std::string &frameId)
        {
            using JointInterfaceItemItr = envire::core::EnvireGraph::ItemIterator<envire::core::Item<JointInterfaceItem>>;

            std::shared_ptr<interfaces::SubControlCenter> subControl = getControlCenter(frameId);
            if (subControl == nullptr)
                return;

            // TODO: we will not use prefix, when we move to base envire types

            LOG_INFO("Add motor item");
            MotorData motorData;
            motorData.fromConfigMap(&config, "");

            // there should be one joint and one motor in the same frame
            size_t jointNumb = ControlCenter::envireGraph->getItemCount<envire::core::Item<JointInterfaceItem>>(frameId);
            if (jointNumb == 0)
            {
                LOG_ERROR_S << "Can not create a motor, since the frame " << frameId << " does not contain any joint interface item.";
                return;
            } else if (jointNumb > 1)
            {
                LOG_ERROR_S << "Can not create a motor, since there are multiple joint interface items in the frame " << frameId << ".";
                return;
            }

            JointInterfaceItemItr jointItemItr = ControlCenter::envireGraph->getItem<envire::core::Item<JointInterfaceItem>>(frameId);
            std::shared_ptr<JointInterface> joint = jointItemItr->getData().jointInterface;

            if (!joint)
            {
                LOG_ERROR_S << "Can not create motor, there is a joint interface item in the frame " << frameId << ", but joint interface is not set.";
                return;
            }

            std::string jointName;
            joint->getName(&jointName);
            if (jointName != motorData.jointName)
            {
                LOG_ERROR_S << "Can not create a motor, since the found joint interface does not correposponded to the motor by its name.";
                LOG_ERROR_S << "Joint name required by motor: " << motorData.jointName;
                LOG_ERROR_S << "Found joint name: " << jointName;
                return;
            }

            // todo: use shared_ptr in motor
            // TODO: add MotorInterface and store it in the graph instead of SimMotor
            unsigned long motorId = ControlCenter::motors->addMotor(&motorData, joint.get());
            // TODO: we should replace SimMotor by MotorInterface how it was done for joints
            //std::shared_ptr<SimMotor> motor;
            //motor.reset(control->motors->getSimMotor(motorId));
            //envire::core::Item<std::shared_ptr<SimMotor>>::Ptr motorItemPtr(new envire::core::Item<std::shared_ptr<SimMotor>>(motor));
            //ControlCenter::envireGraph->addItemToFrame(frameId, motorItemPtr);
        }
    } // end of namespace envire_motors

} // end of namespace mars

DESTROY_LIB(mars::envire_motors::EnvireMotorsPlugins);
CREATE_LIB(mars::envire_motors::EnvireMotorsPlugins);

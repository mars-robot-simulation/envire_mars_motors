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
#include <mars_interfaces/sim/JointManagerInterface.h>

#include <mars_core/MotorManager.hpp>
#include <mars_core/JointManager.hpp>


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
            motors{ControlCenter::motors},
            joints{ControlCenter::joints}
        {
            init();
        }

        EnvireMotorsPlugins::EnvireMotorsPlugins(lib_manager::LibManager *theManager,
                                                 std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                                 std::shared_ptr<envire::core::TreeView> graphTreeView,
                                                 std::shared_ptr<interfaces::MotorManagerInterface> motors,
                                                 std::shared_ptr<interfaces::JointManagerInterface> joints) :
            lib_manager::LibInterface{theManager},
            envireGraph{envireGraph},
            graphTreeView{graphTreeView},
            motors{motors},
            joints{joints}
        {
            init();
        }

        void EnvireMotorsPlugins::init(void)
        {
            if(libManager) libManager->acquireLibrary("mars_core");
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DC>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::PID>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DirectEffort>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::FeedForwardEffort>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<core::SimMotor>>>::subscribe(envireGraph.get());

        }

        EnvireMotorsPlugins::~EnvireMotorsPlugins()
        {
            // unsubscribe from envire graph
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DC>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::PID>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DirectEffort>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::FeedForwardEffort>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<core::SimMotor>>>::unsubscribe();

            if(libManager) libManager->releaseLibrary("mars_core");
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

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::DC>>& e)
        {
            handleAddedMotor(e);
        }

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::PID>>& e)
        {
            handleAddedMotor(e);
        }

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::DirectEffort>>& e)
        {
            handleAddedMotor(e);
        }

        void EnvireMotorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::FeedForwardEffort>>& e)
        {
            handleAddedMotor(e);
        }

        void EnvireMotorsPlugins::itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<std::shared_ptr<core::SimMotor>>>& e)
        {
            const auto& motorID = motors->getID(e.item->getData()->getName());
            motors->removeMotor(motorID);
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

            LOG_INFO(std::string{"Add motor item to frame \"" + frameId + "\"."}.c_str());
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
                const auto errmsg = std::string{"Motor is configured to be correspond to joint named \""} + motorData.jointName + "\" but the joint contained in frame \"" + frameId + "\" is named \"" + jointName + "\". The motor will be actuating on this existing joint!";
                LOG_WARN(errmsg.c_str());
                motorData.jointName = jointName;
            }

            // todo: can we get the joint id from the joint itself if we need it?
            if(joints)
            {
                motorData.jointIndex = joints->getID(jointName);
            }
            if(motors)
            {
                motorData.index = std::dynamic_pointer_cast<core::MotorManager>(motors)->registerMotorID(motorData.name);
            }
            // TODO: we should replace SimMotor by MotorInterface how it was done for joints (https://github.com/mars-robot-simulation/mars_interfaces/issues/1)
            auto motor = createSimMotor(motorData, joint, subControl->control);
            envire::core::Item<std::shared_ptr<mars::core::SimMotor>>::Ptr motorItemPtr{new envire::core::Item<std::shared_ptr<mars::core::SimMotor>>(motor)};
            envireGraph->addItemToFrame(frameId, motorItemPtr);

            std::dynamic_pointer_cast<core::MotorManager>(motors)->addSimMotor(motor);
        }


        std::shared_ptr<core::SimMotor> EnvireMotorsPlugins::createSimMotor(const interfaces::MotorData& motorData, std::weak_ptr<interfaces::JointInterface> joint, ControlCenter *c) const
        {
            auto newMotor = std::make_shared<core::SimMotor>(c, motorData, joint);

            newMotor->setSMotor(motorData);

            // TODO: Use "const auto& version" as soon as configmaps::ConfigMap is const-correct.
            //const auto& config = motorData.config;
            configmaps::ConfigMap config = motorData.config;

            // set approximation functions
            if(config.find("maxeffort_approximation") != config.end())
            {
                std::vector<sReal>* maxeffort_coefficients = new std::vector<sReal>;
                ConfigVector::iterator vIt = config["maxeffort_coefficients"].begin();
                for(; vIt != config["maxeffort_coefficients"].end(); ++vIt)
                {
                    maxeffort_coefficients->push_back((double)(*vIt));
                    newMotor->setMaxEffortApproximation(
                        utils::getApproximationFunctionFromString(config["maxeffort_approximation"].toString()),
                        maxeffort_coefficients);
                }
            }
            if(config.find("maxspeed_approximation") != config.end())
            {
                std::vector<sReal>* maxspeed_coefficients = new std::vector<sReal>;
                ConfigVector::iterator vIt = config["maxspeed_coefficients"].begin();
                for(; vIt != config["maxspeed_coefficients"].end(); ++vIt)
                {
                    maxspeed_coefficients->push_back((double)(*vIt));
                    newMotor->setMaxSpeedApproximation(
                        utils::getApproximationFunctionFromString(config["maxspeed_approximation"].toString()),
                        maxspeed_coefficients);
                }
            }
            if(config.find("current_approximation") != config.end())
            {
                std::vector<sReal>* current_coefficients = new std::vector<sReal>;
                ConfigVector::iterator vIt = config["current_coefficients"].begin();
                for(; vIt != config["current_coefficients"].end(); ++vIt)
                {
                    current_coefficients->push_back((double)(*vIt));
                    newMotor->setCurrentApproximation(
                        utils::getApproximationFunction2DFromString(config["current_approximation"].toString()),
                        current_coefficients);
                }
            }

            return newMotor;
        }
    } // end of namespace envire_motors
} // end of namespace mars


DESTROY_LIB(mars::envire_motors::EnvireMotorsPlugins);
CREATE_LIB(mars::envire_motors::EnvireMotorsPlugins);

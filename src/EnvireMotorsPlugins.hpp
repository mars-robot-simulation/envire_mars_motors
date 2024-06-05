/**
 * \file EnvireMotorsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load physics representation based on envire items
 *
 */

#pragma once

#include <lib_manager/LibInterface.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_types/motors/DC.hpp>
#include <envire_types/motors/PID.hpp>
#include <envire_types/motors/DirectEffort.hpp>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/Vector.h>
// TODO: Remove when SimMotor is no longer stored in envireGraph
#include <mars_core/SimMotor.hpp>


namespace mars
{
    namespace envire_motors
    {
        // move the typedef to separate file
        class EnvireMotorsPlugins : public lib_manager::LibInterface,
                                    public envire::core::GraphEventDispatcher,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DC>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::PID>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::motors::DirectEffort>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<core::SimMotor>>>
        {

        public:
            EnvireMotorsPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            EnvireMotorsPlugins(lib_manager::LibManager *theManager,
                                std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                std::shared_ptr<envire::core::TreeView> graphTreeView,
                                std::shared_ptr<mars::interfaces::JointManagerInterface> joints,
                                std::shared_ptr<mars::interfaces::MotorManagerInterface> motors,
                                std::shared_ptr<mars::interfaces::IDManager> JointIDManager,
                                std::shared_ptr<mars::interfaces::IDManager> motorIDManager);
            virtual ~EnvireMotorsPlugins();

            void init(void);

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string{"envire_mars_motors"};
            }

            CREATE_MODULE_INFO();

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::DC>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::PID>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::motors::DirectEffort>>& e) override;
            virtual void itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<std::shared_ptr<core::SimMotor>>>& e) override;

        private:
            // TODO: Move to central location
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            interfaces::SimulatorInterface* sim{nullptr};
            std::shared_ptr<envire::core::EnvireGraph> envireGraph;
            std::shared_ptr<envire::core::TreeView> graphTreeView;
            std::shared_ptr<mars::interfaces::JointManagerInterface> joints;
            std::shared_ptr<mars::interfaces::MotorManagerInterface> motors;
            std::shared_ptr<mars::interfaces::IDManager> jointIDManager;
            std::shared_ptr<mars::interfaces::IDManager> motorIDManager;

            void createMotor(configmaps::ConfigMap &config, const std::string &frameId);
            // TODO: Create "MotorInterfaceItem" instead
            std::shared_ptr<core::SimMotor> createSimMotor(const interfaces::MotorData& motorData) const;
        };

    } // end of namespace envire_motors
} // end of namespace mars

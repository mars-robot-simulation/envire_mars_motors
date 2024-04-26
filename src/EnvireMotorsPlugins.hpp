/**
 * \file EnvireOdePhysicsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load physics representation based on envire items
 *
 */

#pragma once

// TODO: check and clean the header includes

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/Vector.h>

#include <lib_manager/LibInterface.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_base_types/motors/DC.hpp>
#include <envire_base_types/motors/PID.hpp>
#include <envire_base_types/motors/DirectEffort.hpp>

#include <mars_core/SimMotor.hpp>

#include <iostream>

namespace mars
{
    namespace envire_motors
    {
        // move the typedef to separate file
        class EnvireMotorsPlugins : public lib_manager::LibInterface,
                                    public envire::core::GraphEventDispatcher,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::DC>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::PID>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::motors::DirectEffort>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<core::SimMotor>>>
        {

        public:
            EnvireMotorsPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            EnvireMotorsPlugins(lib_manager::LibManager *theManager,
                                std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                std::shared_ptr<envire::core::TreeView> graphTreeView,
                                std::shared_ptr<mars::interfaces::MotorManagerInterface> motors);
            virtual ~EnvireMotorsPlugins();

            void init(void);

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("envire_mars_motors");
            }

            CREATE_MODULE_INFO();

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::DC>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::PID>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::motors::DirectEffort>>& e) override;
            virtual void itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<std::shared_ptr<core::SimMotor>>>& e) override;

        private:
            std::shared_ptr<envire::core::EnvireGraph> envireGraph;
            std::shared_ptr<envire::core::TreeView> graphTreeView;
            std::shared_ptr<mars::interfaces::MotorManagerInterface> motors;
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void createMotor(configmaps::ConfigMap &config, const std::string &frameId);
        };

    } // end of namespace envire_motors
} // end of namespace mars

 /**
  ******************************************************************************
  * @file    plugin_registry.hpp
  * @author  UrbanIzzy
  * @date    Feb 10, 2026
  * @brief   Plugin registry for sensor plugin management
  ******************************************************************************
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "sensor_plugin.hpp"

#include <map>
#include <mutex>
#include <iostream>
#include <vector>

namespace robotics {
namespace sensors {

class PluginRegistry {
    public:
        static PluginRegistry& getInstance(){
            static PluginRegistry instance;
            return instance;
        }
        
        bool registerPlugin(
            const std::string& name,
            const std::string& description,
            SensorPluginFactory factory
        ) {
            std::lock_guard<std::mutex>lock(_mutex);

            if(_factories.find(name) != _factories.end()){
                std::cerr << "PluginRegistry: plugin '" << name
                          <<"' already registered!" << std::endl;
                return false;
            }

            _factories[name] = factory;
            _descriptions[name] = description;

            std::cerr << "PluginRegistry: plugin '" << name
                      <<"' - " << description << " is now registered!" << std::endl;
            return true;
        }

        std::unique_ptr<SensorPlugin> createPlugin(
            const std::string& name,
            const std::string& config = ""
        ){
            std::lock_guard<std::mutex> lock(_mutex);

            auto it = _factories.find(name);
            if(it == _factories.end()) {
                std::cerr << "PluginRegistry: plugin '" << name
                          <<"' not found!" << std::endl;
                return nullptr;
            }

            try {
                auto plugin = it->second(config);
                if(plugin){
                    std::cout << "PluginRegistry: Created Plugin '" << name << "'" << std::endl;
                }
                else{
                    std::cout << "PluginRegistry: Failed to Created Plugin '" << name << "'" << std::endl;
                }
                return plugin;
            }
            catch(const std::exception& e) {
                std::cerr << "Plugin Registry: exception creation plugin: '" << name 
                << "': " << e.what() << std::endl;
            }
        }
        
        bool hasPlugin(const std::string& name) const {
            std::lock_guard<std::mutex> lock(_mutex);

            return _factories.find(name) != _factories.end();
        }

        std::string getPluginDescription(const std::string& name) const {
            std::lock_guard<std::mutex> lock(_mutex);

            auto it = _descriptions.find(name);
            if(it != _descriptions.end()) {
                return it->second;
            }
            return "";
        }

        std::vector<std::string> listPlugin() const {
            std::lock_guard<std::mutex> lock(_mutex);

            std::vector<std::string> names;
            names.reserve(_factories.size());

            for(const auto& pair : _factories) {
                names.push_back(pair.first);
            }
            return names;
        }

        size_t getPluginCount() const {
            std::lock_guard<std::mutex> lock(_mutex);
            return _factories.size();
        }

        void printPlugins() const {
            std::lock_guard<std::mutex> lock(_mutex);
            
            std::cout << "\n=== Register Sensor Plugins (" << _factories.size() << ")===" << std::endl;
            for(const auto& pair : _factories) {
                std::cout << " - " << pair.first;

                auto descIt = _descriptions.find(pair.first);
                if(descIt != _descriptions.end()){
                    std::cout << ": " << descIt->second;
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

    private:
        PluginRegistry() = default;
        ~PluginRegistry() = default;

        PluginRegistry(const PluginRegistry&) = delete;
        PluginRegistry& operator=(const PluginRegistry&) = delete;

        std::map<std::string, SensorPluginFactory> _factories;
        std::map<std::string, std::string> _descriptions;
        mutable std::mutex _mutex;
};

template<typename PluginCalss>
class PluginRegistrar {
    public:
        PluginRegistrar(const std::string& name, const std::string& description){
            PluginRegistry::getInstance().registerPlugin(
            name,
            description,
            [](const std::string& config)->std::unique_ptr<SensorPlugin> {
                return PluginCalss::create(config);
            }
        );
    }
};

#define REGISTER_SENSOR_PULGIN(PluginCalss, name, descriptor) \
    namespace { \
        static ::robotics::sensors::PluginRegistrar<PluginCalss> \
            g_##PluginCalss##_register(name, descriptor); \
    }
} 
}

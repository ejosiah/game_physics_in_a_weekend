#include <iostream>

#include <vulkan_util/ImGuiPlugin.hpp>
#include "GameWorld.hpp"

int main(){
    fs::current_path("../../resources");
    try{
        std::unique_ptr<Plugin> plugin = std::make_unique<ImGuiPlugin>();
        Settings settings;
        settings.depthTest = true;

        auto app = GameWorld{ settings };
        app.addPlugin(plugin);
        app.run();
    }catch(std::runtime_error& err){
        spdlog::error(err.what());
        return 500;
    }
    return 0;
}

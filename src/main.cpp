#include <iostream>

#include <vulkan_util/ImGuiPlugin.hpp>
#include "GameWorld.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>

int main(){
    fs::current_path("../../resources");
//    spdlog::set_level(spdlog::level::off);
    try{
        std::unique_ptr<Plugin> plugin = std::make_unique<ImGuiPlugin>();
        Settings settings;
        settings.enabledFeatures.wideLines = true;
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
